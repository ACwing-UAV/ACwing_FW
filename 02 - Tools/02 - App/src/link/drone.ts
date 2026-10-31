/**
 * Drone session: owns the transport, parses the stream, keeps the latest value
 * of every frame, sends commands and waits for their ACK.
 *
 * Rules shared with the firmware (HANDOVER_FW.md):
 *  - every command but RC/HEARTBEAT carries a sequence number and gets an ACK;
 *  - a command without ACK after 600 ms is re-sent with the SAME sequence
 *    number (max 2 retries): the firmware answers again without executing twice;
 *  - while connected the app always sends either RC (25 Hz, cockpit open) or
 *    HEARTBEAT (5 Hz): silence for > 1 s is the app-link failsafe.
 */
import { StreamParser } from "../protocol/frame";
import { decodeFrame, quatToEuler, type AckMsg, type CapsMsg, type FlightMsg, type PowerMsg, type NavMsg, type MotorsMsg, type SystemMsg, type StatusMsg, type ChecksMsg, type CalibMsg, type EventMsg } from "../protocol/messages";
import { Cmd, type RcInput } from "../protocol/commands";
import { FT, OP, AckResult, SENSOR, PARAM_BY_ID, EventCode } from "../protocol/ids";
import type { Transport } from "./transport";
import { appSettings } from "../settings/appSettings";

export type LinkState = "idle" | "connecting" | "connected" | "lost";

export interface EventEntry extends EventMsg {
  at: number; // Date.now()
  id: number;
}

export interface Attitude {
  rollRight: number; // deg, + = right side down
  pitchUp: number; // deg, + = nose up
  yaw: number; // deg NWU, + = counter-clockwise
}

export interface DroneState {
  link: LinkState;
  deviceName: string;
  transport: Transport["kind"] | null;
  rssi: number | null;
  mtu: number | null;
  lastRx: number;
  rxBytesPerS: number;
  crcErrors: number;
  caps: CapsMsg | null;
  flight: FlightMsg | null;
  power: PowerMsg | null;
  nav: NavMsg | null;
  motors: MotorsMsg | null;
  system: SystemMsg | null;
  status: StatusMsg | null;
  checks: ChecksMsg | null;
  attitude: Attitude | null;
  calib: CalibMsg | null;
  params: Record<number, { value: number; dirty: boolean }>;
  events: EventEntry[];
  /** EVENT 33 (firmware 0.5.0): the battery task stopped, battery unknown, ARM refused. Cleared when POWER comes back. */
  battMonitorLostAt: number | null;
  /** history for the charts: one sample every 500 ms, last 2 min */
  history: { t: number; alt: number; vz: number; vbat: number; current: number }[];
}

const initial = (): DroneState => ({
  link: "idle", deviceName: "", transport: null, rssi: null, mtu: null, lastRx: 0, rxBytesPerS: 0, crcErrors: 0,
  caps: null, flight: null, power: null, nav: null, motors: null, system: null, status: null, checks: null,
  attitude: null, calib: null, params: {}, events: [], history: [], battMonitorLostAt: null,
});

export class CommandError extends Error {
  constructor(public result: number | "timeout" | "offline", public reason: number) {
    super(typeof result === "string" ? result : `result ${result}, reason ${reason}`);
  }
}

type Waiter = { resolve: (a: AckMsg) => void; reject: (e: CommandError) => void };

export class Drone {
  private s: DroneState = initial();
  private listeners = new Set<() => void>();
  private notifyQueued = false;
  private transport: Transport | null = null;
  private parser = new StreamParser();
  private seq = 1;
  private waiters = new Map<string, Waiter>();
  private rc: RcInput | null = null;
  private timers: ReturnType<typeof setInterval>[] = [];
  private rxBytes = 0;
  private eventId = 0;
  private eventListeners = new Set<(e: EventEntry) => void>();
  private frameListeners = new Set<(type: number, payload: Uint8Array) => void>();

  /* ---- store (useSyncExternalStore) ---- */
  subscribe = (fn: () => void) => {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  };
  getSnapshot = () => this.s;
  onEvent(fn: (e: EventEntry) => void) {
    this.eventListeners.add(fn);
    return () => this.eventListeners.delete(fn);
  }

  /** Every received frame, raw (the flight-log client uses LOG_ENTRY, LOG_DATA and ACK). */
  onFrame(fn: (type: number, payload: Uint8Array) => void) {
    this.frameListeners.add(fn);
    return () => this.frameListeners.delete(fn);
  }

  private set(patch: Partial<DroneState>) {
    this.s = { ...this.s, ...patch };
    if (!this.notifyQueued) {
      this.notifyQueued = true;
      setTimeout(() => {
        this.notifyQueued = false;
        this.listeners.forEach((l) => l());
      }, 40); // ≤ 25 UI updates per second whatever the frame rate
    }
  }

  has(sensor: number) {
    return !!(this.s.caps && this.s.caps.sensors & sensor);
  }
  get hasGps() {
    return this.has(SENSOR.GNSS);
  }

  /* ---- connection ---- */
  async connect(t: Transport) {
    await this.disconnect();
    this.transport = t;
    this.parser = new StreamParser();
    this.s = { ...initial(), events: this.s.events };
    this.set({ link: "connecting", deviceName: t.deviceName, transport: t.kind });
    await t.open((chunk) => this.onData(chunk), () => this.onClosed());
    this.set({ link: "connected", lastRx: Date.now(), mtu: await t.mtu() });
    // ask for everything the screens need, then keep the link alive
    this.fire(Cmd.request(this.nextSeq(), FT.CAPS));
    this.fire(Cmd.request(this.nextSeq(), FT.CHECKS));
    this.fire(Cmd.paramList(this.nextSeq()));
    this.setStateRate(appSettings.get().telemetryRate);
    this.timers.push(setInterval(() => this.keepAlive(), 40));
    this.timers.push(setInterval(() => this.housekeeping(), 500));
  }

  async disconnect() {
    this.timers.forEach(clearInterval);
    this.timers = [];
    this.waiters.forEach((w) => w.reject(new CommandError("offline", 0)));
    this.waiters.clear();
    const t = this.transport;
    this.transport = null;
    if (t) await t.close();
    if (this.s.link !== "idle") this.set({ ...initial(), events: this.s.events });
  }

  private onClosed() {
    this.set({ link: "lost" });
  }

  /* ---- receive ---- */
  private onData(chunk: Uint8Array) {
    this.rxBytes += chunk.length;
    const now = Date.now();
    const patch: Partial<DroneState> = { lastRx: now };
    if (this.s.link === "lost") patch.link = "connected";
    for (const f of this.parser.push(chunk)) {
      if (this.frameListeners.size) this.frameListeners.forEach((l) => l(f.type, f.payload));
      const d = decodeFrame(f.type, f.payload);
      if (!d) continue;
      switch (d.type) {
        case FT.STATE: {
          const e = quatToEuler(d.msg.q);
          patch.attitude = { rollRight: e.roll, pitchUp: -e.pitch, yaw: e.yaw };
          break;
        }
        case FT.STATUS: patch.status = d.msg; break;
        case FT.CAPS: patch.caps = d.msg; break;
        case FT.FLIGHT: patch.flight = d.msg; break;
        case FT.POWER:
          patch.power = d.msg;
          if (this.s.battMonitorLostAt && now - this.s.battMonitorLostAt > 3000) patch.battMonitorLostAt = null;
          break;
        case FT.NAV: patch.nav = d.msg; break;
        case FT.MOTORS: patch.motors = d.msg; break;
        case FT.SYSTEM: patch.system = d.msg; break;
        case FT.CHECKS: patch.checks = d.msg; break;
        case FT.CALIB: patch.calib = d.msg; break;
        case FT.PARAM:
          patch.params = { ...(patch.params ?? this.s.params), [d.msg.id]: { value: d.msg.value, dirty: !!(d.msg.flags & 1) } };
          break;
        case FT.EVENT: {
          const e: EventEntry = { ...d.msg, at: now, id: ++this.eventId };
          if (e.code === EventCode.BATTERY_MONITOR_LOST) patch.battMonitorLostAt = now;
          // the firmware repeats important events 3 times: keep one
          const dup = this.s.events.find((x) => x.code === e.code && x.arg === e.arg && now - x.at < 600);
          if (!dup) {
            patch.events = [e, ...(patch.events ?? this.s.events)].slice(0, 100);
            this.eventListeners.forEach((l) => l(e));
          }
          break;
        }
        case FT.ACK: {
          const w = this.waiters.get(`${d.msg.op}:${d.msg.seq}`);
          if (w) {
            this.waiters.delete(`${d.msg.op}:${d.msg.seq}`);
            if (d.msg.result === AckResult.OK || d.msg.result === AckResult.IN_PROGRESS) w.resolve(d.msg);
            else w.reject(new CommandError(d.msg.result, d.msg.reason));
          }
          break;
        }
      }
    }
    patch.crcErrors = this.parser.crcErrors;
    this.set(patch);
  }

  private housekeeping() {
    const now = Date.now();
    const p: Partial<DroneState> = { rxBytesPerS: this.rxBytes * 2 };
    this.rxBytes = 0;
    if (this.s.link === "connected" && now - this.s.lastRx > 1500) p.link = "lost";
    if (this.s.flight && this.s.power) {
      const h = [...this.s.history, { t: now, alt: this.s.flight.altRelM, vz: this.s.flight.vzMs, vbat: this.s.power.vbatV, current: this.s.power.currentA }];
      p.history = h.filter((x) => now - x.t <= 120_000);
    }
    this.set(p);
    this.transport?.rssi().then((r) => r !== null && this.set({ rssi: r }));
  }

  /* ---- send ---- */
  private nextSeq() {
    this.seq = (this.seq % 255) + 1;
    return this.seq;
  }

  private fire(bytes: Uint8Array, withResponse = true) {
    this.transport?.write(bytes, withResponse).catch(() => undefined);
  }

  private tick = 0;
  private keepAlive() {
    if (!this.transport || this.s.link === "idle") return;
    this.tick++;
    if (this.rc) this.fire(Cmd.rc(0, this.rc), false); // every 40 ms = 25 Hz
    else if (this.tick % 5 === 0) this.fire(Cmd.heartbeat(0), false); // 5 Hz
  }

  /** STATE (attitude) rate over BLE, through the phase-2 decimation command (no ACK). */
  setStateRate(hz: number) {
    this.fire(Cmd.stateDecim(Math.round(240 / Math.max(1, hz))));
  }

  /** Cockpit open: stream the sticks. null = stop (heartbeat only). */
  setRc(rc: RcInput | null) {
    this.rc = rc;
  }

  /** Writes a command without waiting for its ACK (the caller listens with onFrame). Returns the seq used. */
  sendRaw(build: (seq: number) => Uint8Array): number {
    if (!this.transport || this.s.link === "idle") throw new CommandError("offline", 0);
    const seq = this.nextSeq();
    this.fire(build(seq), true);
    return seq;
  }

  /** Sends a command and resolves with its ACK. `build` receives the sequence number. */
  send(build: (seq: number) => Uint8Array): Promise<AckMsg> {
    if (!this.transport || this.s.link === "idle") return Promise.reject(new CommandError("offline", 0));
    const seq = this.nextSeq();
    const bytes = build(seq);
    const key = `${bytes[0]}:${seq}`;
    return new Promise<AckMsg>((resolve, reject) => {
      let tries = 0;
      const attempt = () => {
        if (!this.waiters.has(key)) return;
        if (tries++ > 2) {
          this.waiters.delete(key);
          return reject(new CommandError("timeout", 0));
        }
        this.fire(bytes, true);
        setTimeout(attempt, 600);
      };
      this.waiters.set(key, { resolve, reject });
      attempt();
    });
  }

  /* ---- convenience ---- */
  arm = () => this.send((s) => Cmd.arm(s));
  disarm = () => this.send((s) => Cmd.disarm(s));
  takeoff = (altM: number) => this.send((s) => Cmd.takeoff(s, altM));
  land = () => this.send((s) => Cmd.land(s));
  rth = () => this.send((s) => Cmd.rth(s));
  hold = () => this.send((s) => Cmd.hold(s));
  kill = () => this.send((s) => Cmd.kill(s));
  setMode = (m: number) => this.send((s) => Cmd.setMode(s, m));
  setParam = (key: string, v: number) => {
    const def = Object.values(PARAM_BY_ID).find((p) => p.key === key);
    if (!def) return Promise.reject(new CommandError(AckResult.INVALID, 0));
    return this.send((s) => Cmd.paramSet(s, def.id, v));
  };
  saveParams = () => this.send((s) => Cmd.paramSave(s));
  paramDefaults = () => this.send((s) => Cmd.paramDefaults(s));
  calStart = (type: number, option = 0) => this.send((s) => Cmd.calStart(s, type, option));
  calCapture = () => this.send((s) => Cmd.calCapture(s));
  calAbort = () => this.send((s) => Cmd.calAbort(s)).catch(() => undefined);
  calSave = () => this.send((s) => Cmd.calSave(s));
  calRestore = (type: number) => this.send((s) => Cmd.calRestore(s, type));
  calSetRef = (type: number, idx: number, v: number) => this.send((s) => Cmd.calSetRef(s, type, idx, v));
  motorTest = (m: number, pct = 8, ms = 2000) => this.send((s) => Cmd.motorTest(s, m, pct, ms));
  motorReverse = (m: number) => this.send((s) => Cmd.motorReverse(s, m));
  setName = (n: string) => this.send((s) => Cmd.setName(s, n));

  /** Value of a drone parameter: last value received, else the default. */
  param(key: string): number {
    const def = Object.values(PARAM_BY_ID).find((p) => p.key === key)!;
    return this.s.params[def.id]?.value ?? def.def;
  }

  /** For tests: raw access to OP constants of pending waiters. */
  get pending() {
    return [...this.waiters.keys()].map((k) => Number(k.split(":")[0])).filter((op) => op !== OP.HEARTBEAT);
  }
}

export const drone = new Drone();

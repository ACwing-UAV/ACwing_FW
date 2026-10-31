/**
 * Drone simulator: behaves like the firmware described in HANDOVER_FW.md, so
 * the whole app (and every screen) can be developed and demonstrated without
 * hardware. It speaks the real protocol: it parses CMD bytes and produces
 * encoded frames.
 *
 * Frames: NWU world, FLU body (as the firmware). Flight model: kinematic, with
 * first-order responses – enough for the UI, not for control design.
 */
import { encodeFrame, Reader } from "../protocol/frame";
import * as M from "../protocol/messages";
import { parseCommand } from "../protocol/commands";
import {
  FT, OP, AckResult, Reason, FlightMode, ArmState, AutoAction, FS, FF, InputSource, SENSOR, Model, FrameType,
  BattWarn, EventLevel, EventCode, CalType, CalState, CheckId, CheckState, PARAMS, PARAM_BY_ID, ARM_MAGIC, KILL_MAGIC,
  PROPS_OFF_MAGIC,
} from "../protocol/ids";

export type SimModel = "outdoor" | "indoor";

const D2R = Math.PI / 180;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const wrap360 = (a: number) => ((a % 360) + 360) % 360;
const approach = (v: number, target: number, rate: number, dt: number) => v + (target - v) * Math.min(1, rate * dt);
const noise = (s: number) => (Math.random() * 2 - 1) * s;

interface Cal {
  type: number;
  state: number;
  step: number;
  progress: number;
  quality: number;
  reason: number;
  t: number; // time in the current step, s
  data: M.CalibData;
}

export class SimDrone {
  // ---- identity ----
  readonly model: SimModel;
  readonly outdoor: boolean;
  name: string;
  private sensors: number;
  private cells: number;
  private nMotors = 4;

  // ---- flight state ----
  private t = 0;
  private armed = false;
  private mode: number = FlightMode.POS_HOLD;
  private auto: number = AutoAction.NONE;
  private failsafe = 0;
  private inAir = false;
  private landedT = 0;
  private flightTime = 0;
  private posN = 0;
  private posE = 0;
  private alt = 0;
  private vN = 0;
  private vE = 0;
  private vz = 0;
  private heading = 274;
  private roll = 0;
  private pitch = 0; // FLU: + = nose down
  private homeSet = false;
  private killedFall = false;

  // ---- power ----
  private usedMah = 0;
  private current = 0;
  private scaleV = 110.0;
  private scaleI = 1.0;
  private lastFlightScaleI = 1.0; // scaleI the last flight's mAh were counted with (as the firmware stores it)
  private battLevel = 0;          // BattWarn latched as in the firmware (left only when disarmed)
  private battLock = false;       // EMERGENCY: landing locked, only KILL overrides

  // ---- inputs ----
  private rc = { throttle: -1000, yaw: 0, pitch: 0, roll: 0 };
  private lastAppMsg = 0; // s, last RC or heartbeat
  private appInput = false;
  private elrsLinked: boolean;
  private elrsActive = false; // radio sticks moved → radio overrides the app

  // ---- params, calibration, motors ----
  private params = new Map<number, number>();
  private saved = new Map<number, number>();
  private cal: Cal | null = null;
  private compassCalibrated = false;
  private motorTest: { motor: number; until: number; pct: number } | null = null;
  private motorRev = 0;
  private motorOk = 0;

  // ---- output ----
  private out: Uint8Array[] = [];
  private acc: Record<string, number> = {};
  private seq = 0;
  private decim = 4;
  private ackCache = new Map<string, Uint8Array>();
  private rates = { flight: 10, power: 5, nav: 5, motors: 5, system: 2, checks: 2, caps: 0.2, status: 10, state: 60 };

  constructor(model: SimModel) {
    this.model = model;
    this.outdoor = model === "outdoor";
    this.name = this.outdoor ? "ACwing #1" : "ACwing Indoor";
    this.cells = this.outdoor ? 4 : 1;
    this.sensors = SENSOR.IMU | SENSOR.BARO | SENSOR.MAG_FC | SENSOR.FLOW | SENSOR.TOF | SENSOR.CURRENT | SENSOR.ELRS | SENSOR.LEDS | SENSOR.FLASH_LOG;
    if (this.outdoor) this.sensors |= SENSOR.GNSS | SENSOR.MAG_GPS | SENSOR.ESC_TLM;
    this.elrsLinked = this.outdoor;
    PARAMS.forEach((p) => {
      this.params.set(p.id, p.def);
      this.saved.set(p.id, p.def);
    });
    this.homeSet = this.outdoor;
  }

  /** Called when a client subscribes: CAPS + INIT immediately, as the firmware does. */
  onSubscribe() {
    this.emitCaps();
    this.emit(FT.INIT, new Uint8Array(137));
  }

  /** Frames produced since the last call. */
  drain(): Uint8Array[] {
    const o = this.out;
    this.out = [];
    return o;
  }

  private emit(type: number, payload: Uint8Array) {
    this.out.push(encodeFrame(type, payload));
  }

  private event(level: number, code: number, text: string, arg = 0) {
    this.emit(FT.EVENT, M.encodeEvent({ level, code, arg, text }));
  }

  private p(key: string) {
    const def = PARAMS.find((x) => x.key === key)!;
    return this.params.get(def.id) ?? def.def;
  }

  /* ================================================================ */
  /* Commands                                                          */
  /* ================================================================ */

  command(bytes: Uint8Array) {
    const c = parseCommand(bytes);
    if (!c) return;
    if (c.op === OP.SEND_INIT) return this.emit(FT.INIT, new Uint8Array(137));
    if (c.op === OP.STATE_DECIM && "n" in c) {
      this.decim = c.n;
      return;
    }
    if (!("seq" in c)) return;
    const { op, seq, r } = c;
    if (op === OP.RC) {
      this.rc = { throttle: r.i16(), yaw: r.i16(), pitch: r.i16(), roll: r.i16() };
      this.lastAppMsg = this.t;
      this.appInput = true;
      return;
    }
    if (op === OP.HEARTBEAT) {
      this.lastAppMsg = this.t;
      return;
    }
    // A repeated sequence number gets the same ACK again and is NOT executed twice.
    const key = `${op}:${seq}`;
    const cached = this.ackCache.get(key);
    if (cached) return this.out.push(cached);
    const [result, reason] = this.exec(op, r);
    const ack = encodeFrame(FT.ACK, M.encodeAck({ op, seq, result, reason }));
    this.ackCache.set(key, ack);
    if (this.ackCache.size > 64) this.ackCache.delete(this.ackCache.keys().next().value!);
    this.out.push(ack);
  }

  private exec(op: number, r: Reader): [number, number] {
    const ok: [number, number] = [AckResult.OK, Reason.NONE];
    const rej = (why: number): [number, number] => [AckResult.REJECTED, why];
    // battery EMERGENCY (firmware 0.5.0): the landing cannot be changed, only KILL
    if (this.battLock && (op === OP.SET_MODE || op === OP.HOLD || op === OP.RTH || op === OP.TAKEOFF)) return rej(Reason.BATTERY_LOW);
    switch (op) {
      case OP.ARM: {
        if (r.u8_() !== ARM_MAGIC[0] || r.u8_() !== ARM_MAGIC[1]) return [AckResult.INVALID, Reason.NONE];
        if (this.armed) return rej(Reason.ARMED);
        if (this.cal) return rej(Reason.CALIBRATING);
        if (this.inAir) return rej(Reason.NOT_LANDED);
        // throttle "low": bottom of the stick in ACRO/ANGLE, not above centre in the hold modes (centre = hold altitude)
        const holdMode = this.mode === FlightMode.ALT_HOLD || this.mode === FlightMode.POS_HOLD;
        if (this.appInput && this.t - this.lastAppMsg < 0.5 && this.rc.throttle > (holdMode ? 100 : -900)) return rej(Reason.THROTTLE_NOT_LOW);
        if (this.battLevel >= BattWarn.CRITICAL) return rej(Reason.BATTERY_LOW);
        if (this.p("armRequireChecks") && !this.checks().ready) return rej(Reason.CHECKS_FAILING);
        this.armed = true;
        this.landedT = 0;
        if (this.outdoor && !this.homeSet) this.homeSet = true;
        this.event(EventLevel.INFO, EventCode.ARMED, "Armed");
        return ok;
      }
      case OP.DISARM:
        if (!this.armed) return ok;
        if (this.inAir) return rej(Reason.NOT_LANDED);
        this.disarm();
        return ok;
      case OP.TAKEOFF: {
        const alt = r.u16() / 100 || this.p("takeoffAlt");
        if (!this.armed) return rej(Reason.NOT_ARMED);
        if (this.inAir) return rej(Reason.NOT_LANDED);
        this.auto = AutoAction.TAKEOFF;
        this.takeoffTarget = clamp(alt, 0.5, 3);
        this.inAir = true;
        this.event(EventLevel.INFO, EventCode.TAKEOFF_STARTED, "Takeoff");
        return ok;
      }
      case OP.LAND:
        if (!this.armed || !this.inAir) return rej(Reason.NOT_ARMED);
        this.auto = AutoAction.LAND;
        return ok;
      case OP.RTH:
        if (!(this.sensors & SENSOR.GNSS)) return rej(Reason.NO_GPS);
        if (!this.homeSet) return rej(Reason.NO_HOME);
        if (!this.armed || !this.inAir) return rej(Reason.NOT_ARMED);
        this.auto = AutoAction.RTH;
        this.event(EventLevel.INFO, EventCode.RTH_STARTED, "Return to home");
        return ok;
      case OP.HOLD:
        this.auto = this.inAir ? AutoAction.HOLD : AutoAction.NONE;
        return ok;
      case OP.KILL: {
        const m = [r.u8_(), r.u8_(), r.u8_(), r.u8_()];
        if (m.some((v, i) => v !== KILL_MAGIC[i])) return [AckResult.INVALID, Reason.NONE];
        if (this.inAir) this.killedFall = true;
        this.disarm();
        this.event(EventLevel.CRITICAL, EventCode.MOTOR_KILLED, "Motors stopped");
        return ok;
      }
      case OP.SET_MODE: {
        const m = r.u8_();
        if (m > FlightMode.POS_HOLD) return [AckResult.INVALID, Reason.NONE];
        if (m === FlightMode.POS_HOLD && !(this.sensors & (SENSOR.GNSS | SENSOR.FLOW))) return rej(Reason.SENSOR_MISSING);
        if (m === FlightMode.ACRO && this.p("beginner")) return rej(Reason.NONE);
        this.mode = m;
        this.event(EventLevel.INFO, EventCode.MODE_CHANGED, `Mode ${m}`, m);
        return ok;
      }
      case OP.PARAM_GET: {
        const id = r.u16();
        if (!PARAM_BY_ID[id]) return [AckResult.INVALID, Reason.UNKNOWN_PARAM];
        this.emitParam(id);
        return ok;
      }
      case OP.PARAM_SET: {
        const id = r.u16(), v = r.f32();
        const d = PARAM_BY_ID[id];
        if (!d) return [AckResult.INVALID, Reason.UNKNOWN_PARAM];
        if (!(v >= d.min - 1e-6 && v <= d.max + 1e-6)) return [AckResult.INVALID, Reason.OUT_OF_RANGE];
        this.params.set(id, v);
        this.emitParam(id);
        return ok;
      }
      case OP.PARAM_SAVE:
        this.params.forEach((v, k) => this.saved.set(k, v));
        this.event(EventLevel.INFO, EventCode.PARAMS_SAVED, "Settings saved");
        PARAMS.forEach((p) => this.emitParam(p.id));
        return ok;
      case OP.PARAM_DEFAULTS:
        if (this.armed) return rej(Reason.ARMED);
        PARAMS.forEach((p) => this.params.set(p.id, p.def));
        PARAMS.forEach((p) => this.emitParam(p.id));
        return ok;
      case OP.PARAM_LIST:
        PARAMS.forEach((p) => this.emitParam(p.id));
        return ok;
      case OP.CAL_START:
        return this.calStart(r.u8_(), r.u8_());
      case OP.CAL_CAPTURE:
        return this.calCapture();
      case OP.CAL_ABORT:
        if (!this.cal) return rej(Reason.NOT_CALIBRATING);
        this.cal = null;
        this.motorTest = null;
        return ok;
      case OP.CAL_SAVE:
        if (!this.cal || this.cal.state !== CalState.DONE && this.cal.type !== CalType.MOTORS) return rej(Reason.NOT_CALIBRATING);
        if (this.cal.type === CalType.COMPASS) this.compassCalibrated = true;
        this.cal.state = CalState.SAVED;
        this.emitCal();
        this.event(EventLevel.INFO, EventCode.CALIB_DONE, "Calibration saved", this.cal.type);
        this.cal = null;
        return ok;
      case OP.CAL_RESTORE:
        if (this.armed) return rej(Reason.ARMED);
        return ok;
      case OP.CAL_SET_REF: {
        const type = r.u8_(), idx = r.u8_(), v = r.f32();
        return this.calSetRef(type, idx, v);
      }
      case OP.MOTOR_TEST: {
        const motor = r.u8_(), pct = r.u8_(), dur = r.u16(), magic = r.u8_();
        if (magic !== PROPS_OFF_MAGIC) return rej(Reason.PROPS_CONFIRM_MISSING);
        if (this.armed) return rej(Reason.ARMED);
        if (motor < 1 || motor > this.nMotors || pct > 20) return [AckResult.INVALID, Reason.OUT_OF_RANGE];
        this.motorTest = { motor, until: this.t + Math.min(dur, 3000) / 1000, pct };
        this.motorOk |= 1 << (motor - 1);
        return ok;
      }
      case OP.MOTOR_REVERSE: {
        const m = r.u8_();
        if (this.armed) return rej(Reason.ARMED);
        if (m < 1 || m > this.nMotors) return [AckResult.INVALID, Reason.OUT_OF_RANGE];
        this.motorRev ^= 1 << (m - 1);
        return ok;
      }
      case OP.MOTOR_REMAP:
        return this.armed ? rej(Reason.ARMED) : ok;
      case OP.STREAM_RATES:
        this.rates.flight = clamp(r.u8_(), 1, 20);
        this.rates.power = clamp(r.u8_(), 1, 10);
        this.rates.nav = clamp(r.u8_(), 1, 10);
        this.rates.motors = clamp(r.u8_(), 1, 10);
        this.rates.system = clamp(r.u8_(), 1, 5);
        return ok;
      case OP.REQUEST: {
        const ft = r.u8_();
        if (ft === FT.CAPS) this.emitCaps();
        else if (ft === FT.CHECKS) this.emit(FT.CHECKS, M.encodeChecks(this.checks()));
        else if (ft === FT.INIT) this.emit(FT.INIT, new Uint8Array(137));
        else return [AckResult.UNSUPPORTED, Reason.NONE];
        return ok;
      }
      case OP.SET_NAME:
        this.name = r.str(16) || this.name;
        this.emitCaps();
        return ok;
      default:
        return [AckResult.UNSUPPORTED, Reason.NONE];
    }
  }

  private takeoffTarget = 1;
  private fsLong = false;

  private disarm() {
    this.armed = false;
    this.auto = AutoAction.NONE;
    this.failsafe = 0;
    this.battLock = false;
    this.event(EventLevel.INFO, EventCode.DISARMED, "Disarmed");
  }

  /* ================================================================ */
  /* Calibration                                                       */
  /* ================================================================ */

  private calStart(type: number, option: number): [number, number] {
    if (this.armed) return [AckResult.REJECTED, Reason.ARMED];
    if (this.cal && this.cal.state !== CalState.DONE) return [AckResult.BUSY, Reason.CALIBRATING];
    const base = { type, state: CalState.RUNNING, step: 1, progress: 0, quality: 0xff, reason: 0, t: 0 };
    switch (type) {
      case CalType.ACCEL:
        this.cal = { ...base, state: CalState.WAITING_USER, data: { kind: "accel", doneMask: 0, current: 0, stillPct: 0, offsetMg: [0, 0, 0], scale: [1, 1, 1], fitErrPct: 0 } };
        break;
      case CalType.LEVEL:
        this.cal = { ...base, data: { kind: "level", rollDeg: -0.3, pitchDeg: 0.5, remainingMs: 5000, trimRollDeg: 0, trimPitchDeg: 0 } };
        break;
      case CalType.GYRO:
        this.cal = { ...base, data: { kind: "gyro", remainingMs: 5000, biasDps: [0, 0, 0], noiseDps: 0, tempC: 31.2 } };
        break;
      case CalType.COMPASS:
        this.cal = { ...base, step: 2, data: {
          kind: "compass", coverage: Array(36).fill(false), interfFc: 18,
          interfGps: this.sensors & SENSOR.MAG_GPS ? 6 : null, qualityFc: null, qualityGps: null,
        } };
        this.calOption = option;
        break;
      case CalType.FLOW_TOF:
        if (!(this.sensors & SENSOR.FLOW)) return [AckResult.REJECTED, Reason.SENSOR_MISSING];
        this.cal = { ...base, state: CalState.WAITING_USER, data: { kind: "flow", tofMm: 12, heightMm: 0, flowQ: 0, matchPct: 0, axesDone: 0, scaleX: 1, scaleY: 1, tofOffMm: 0 } };
        break;
      case CalType.BATTERY:
        this.cal = { ...base, state: CalState.WAITING_USER, data: { kind: "battery", vbatV: this.vbat(), scaleV: this.scaleV, scaleI: this.scaleI, usedMah: 1240 } };
        break;
      case CalType.MOTORS:
        this.cal = { ...base, state: CalState.WAITING_USER, data: { kind: "motors", motor: 0, spinning: false, revMask: this.motorRev, okMask: 0 } };
        this.motorOk = 0;
        break;
      case CalType.STICKS:
        if (!this.elrsLinked) return [AckResult.REJECTED, Reason.SENSOR_MISSING];
        this.cal = { ...base, state: CalState.WAITING_USER, data: { kind: "sticks", centre: [988, 1501, 1498, 1502], min: [1500, 1500, 1500, 1500], max: [1500, 1500, 1500, 1500], auxPos: [0, 0, 0, 0] } };
        break;
      default:
        return [AckResult.INVALID, Reason.NONE];
    }
    this.emitCal();
    return [AckResult.OK, Reason.NONE];
  }

  private calOption = 0;

  private calCapture(): [number, number] {
    const c = this.cal;
    if (!c) return [AckResult.REJECTED, Reason.NOT_CALIBRATING];
    const d = c.data;
    if (d.kind === "accel") this.accelCaptured();
    else if (d.kind === "flow" && c.step === 1) {
      d.tofOffMm = d.tofMm;
      c.step = 2;
      c.state = CalState.RUNNING;
      c.t = 0;
    } else if (d.kind === "battery") {
      c.state = CalState.DONE;
      c.step = 3;
    } else if (d.kind === "sticks") {
      if (c.step === 1) { c.step = 2; c.state = CalState.RUNNING; c.t = 0; }
      else { c.step = 3; c.state = CalState.DONE; d.auxPos = [2, 3, 2, 0]; c.quality = 100; }
    } else return [AckResult.INVALID, Reason.NONE];
    this.emitCal();
    return [AckResult.OK, Reason.NONE];
  }

  private accelCaptured() {
    const c = this.cal!;
    const d = c.data as Extract<M.CalibData, { kind: "accel" }>;
    d.doneMask |= 1 << d.current;
    d.stillPct = 0;
    c.t = 0;
    if (d.doneMask === 0x3f) {
      c.state = CalState.DONE;
      c.step = 3;
      c.progress = 100;
      c.quality = 96;
      d.offsetMg = [12, -8, 21];
      d.scale = [1.002, 0.998, 1.004];
      d.fitErrPct = 0.4;
    } else {
      d.current = [0, 1, 2, 3, 4, 5].find((i) => !(d.doneMask & (1 << i)))!;
      c.progress = Math.round((100 * popcount(d.doneMask)) / 6);
    }
  }

  private calSetRef(type: number, idx: number, v: number): [number, number] {
    const c = this.cal;
    if (!c || c.type !== type) return [AckResult.REJECTED, Reason.NOT_CALIBRATING];
    const d = c.data;
    if (d.kind === "battery" && idx === 0) {
      if (v < 2.5 || v > 30) return [AckResult.INVALID, Reason.OUT_OF_RANGE];
      this.scaleV = +(this.scaleV * (v / this.vbat())).toFixed(1);
      d.scaleV = this.scaleV;
      d.vbatV = v;
      c.step = 2;
    } else if (d.kind === "battery" && idx === 1) {
      if (v < 50 || v > 20000) return [AckResult.INVALID, Reason.OUT_OF_RANGE];
      // the firmware scales the current with the scale the flight was counted with
      this.scaleI = +((this.lastFlightScaleI * v) / d.usedMah).toFixed(4);
      d.scaleI = this.scaleI;
      c.step = 3;
      c.state = CalState.DONE;
    } else if (d.kind === "level") {
      if (Math.abs(v) > 5) return [AckResult.INVALID, Reason.OUT_OF_RANGE];
      if (idx === 0) d.trimRollDeg = v;
      else d.trimPitchDeg = v;
    } else return [AckResult.INVALID, Reason.NONE];
    this.emitCal();
    return [AckResult.OK, Reason.NONE];
  }

  private stepCal(dt: number) {
    const c = this.cal;
    if (!c) return;
    c.t += dt;
    const d = c.data;
    switch (d.kind) {
      case "accel":
        if (c.state === CalState.WAITING_USER) {
          d.stillPct = clamp(Math.round((c.t / 2) * 100), 0, 100);
          if (c.t >= 2.2) this.accelCaptured();
        }
        break;
      case "level":
        if (c.state === CalState.RUNNING) {
          d.rollDeg = -0.3 + noise(0.04);
          d.pitchDeg = 0.5 + noise(0.04);
          d.remainingMs = Math.max(0, Math.round(5000 - c.t * 1000));
          c.progress = clamp(Math.round((c.t / 5) * 100), 0, 100);
          if (c.t >= 5) {
            c.state = CalState.DONE;
            c.step = 3;
            c.quality = 95;
            d.trimRollDeg = -0.3;
            d.trimPitchDeg = 0.5;
          }
        }
        break;
      case "gyro":
        if (c.state === CalState.RUNNING) {
          d.remainingMs = Math.max(0, Math.round(5000 - c.t * 1000));
          c.progress = clamp(Math.round((c.t / 5) * 100), 0, 100);
          if (c.t >= 5) {
            c.state = CalState.DONE;
            c.step = 2;
            c.quality = 97;
            d.biasDps = [0.02, 0.01, -0.03];
            d.noiseDps = 0.04;
          }
        }
        break;
      case "compass":
        if (c.state === CalState.RUNNING) {
          const n = Math.min(36, Math.floor(c.t * 3.2));
          const order = [0, 12, 24, 1, 13, 25, 2, 14, 26, 3, 15, 27, 4, 16, 28, 5, 17, 29, 6, 18, 30, 7, 19, 31, 8, 20, 32, 9, 21, 33, 10, 22, 34, 11, 23, 35];
          d.coverage = d.coverage.map((_, i) => order.indexOf(i) < n);
          c.progress = Math.round((n / 36) * 100);
          if (n >= 36) {
            c.state = CalState.DONE;
            c.step = 3;
            d.qualityFc = this.calOption === 2 ? null : 88;
            d.qualityGps = d.interfGps === null || this.calOption === 1 ? null : 96;
            d.interfFc = 11;
            d.interfGps = d.interfGps === null ? null : 4;
            c.quality = Math.max(d.qualityFc ?? 0, d.qualityGps ?? 0);
          }
        }
        break;
      case "flow":
        if (c.step === 1) d.tofMm = 12 + Math.round(noise(1));
        else if (c.state === CalState.RUNNING) {
          d.heightMm = 960 + Math.round(noise(15));
          d.flowQ = 84 + Math.round(noise(3));
          d.matchPct = clamp(Math.round(60 + c.t * 6), 0, 93);
          if (c.t > 4) d.axesDone |= 1;
          if (c.t > 8) d.axesDone |= 2;
          c.progress = clamp(Math.round((c.t / 8) * 100), 0, 100);
          if (c.t > 8) {
            c.state = CalState.DONE;
            c.step = 3;
            c.quality = 90;
            d.scaleX = 1.03;
            d.scaleY = 1.01;
          }
        }
        break;
      case "battery":
        if (c.step === 1) d.vbatV = this.vbat();
        break;
      case "motors":
        d.motor = this.motorTest ? this.motorTest.motor : 0;
        d.spinning = !!this.motorTest;
        d.revMask = this.motorRev;
        d.okMask = this.motorOk;
        break;
      case "sticks":
        if (c.step === 2) {
          const k = clamp(c.t / 6, 0, 1);
          const span = (lo: number, hi: number) => [Math.round(1500 - (1500 - lo) * k), Math.round(1500 + (hi - 1500) * k)];
          const lims = [[988, 2012], [990, 2010], [989, 2011], [987, 2013]].map(([lo, hi], i) => (i === 1 ? span(lo, Math.min(hi, 1946 + 64 * clamp(c.t - 6, 0, 1))) : span(lo, hi)));
          d.min = lims.map((x) => x[0]);
          d.max = lims.map((x) => x[1]);
          c.progress = Math.round(k * 100);
        }
        break;
    }
  }

  /* ================================================================ */
  /* Simulation step                                                   */
  /* ================================================================ */

  tick(dt: number) {
    this.t += dt;
    this.stepFlight(dt);
    this.stepCal(dt);
    if (this.motorTest && this.t > this.motorTest.until) this.motorTest = null;
    this.schedule(dt);
  }

  private vbat() {
    const full = 4.2, empty = 3.3;
    const soc = clamp(1 - this.usedMah / (this.outdoor ? 3000 : 450), 0, 1);
    const ocv = empty + (full - empty) * (0.15 + 0.85 * soc) - 0.1 * (1 - soc) ** 3;
    const sag = this.current * (this.outdoor ? 0.012 : 0.05) / this.cells;
    return (ocv - sag) * this.cells * (this.scaleV / 110);
  }

  private stepFlight(dt: number) {
    // ---- app link failsafe (radio in control: the app link does not matter) ----
    const silent = this.t - this.lastAppMsg;
    // a battery failsafe that is landing the drone is not replaced by the app-link failsafe (firmware 0.5.0)
    const battOwns = !!(this.failsafe & FS.BATTERY) && (this.auto === AutoAction.LAND || this.auto === AutoAction.RTH);
    if (this.armed && this.inAir && !this.elrsActive && !battOwns) {
      if (silent > 3 && !this.fsLong) {
        const a = this.p("fsAppLong");
        this.auto = a === 2 && this.sensors & SENSOR.GNSS ? AutoAction.RTH : a === 0 ? AutoAction.HOLD : AutoAction.LAND;
        this.failsafe |= FS.APP_LINK;
        this.fsLong = true;
        this.event(EventLevel.ERROR, EventCode.FAILSAFE_APP_LINK, "App link lost");
      } else if (silent > 1 && !(this.failsafe & FS.APP_LINK)) {
        this.auto = this.p("fsAppShort") === 1 ? AutoAction.LAND : AutoAction.HOLD;
        this.failsafe |= FS.APP_LINK;
      } else if (silent < 0.5 && this.failsafe & FS.APP_LINK) {
        this.failsafe &= ~FS.APP_LINK;
        if (this.auto === AutoAction.HOLD && !this.fsLong) this.auto = AutoAction.NONE;
        this.fsLong = false;
      }
    }

    // ---- battery warnings ----
    // as the firmware (HANDOVER §6 + firmware 0.5.0): one event per level crossed, the level is
    // latched while armed; CRITICAL in air -> RTH (GNSS + home) or LAND once, EVENT 31 then 22;
    // EMERGENCY -> LAND locked. The firmware also filters the cell voltage for 2 s first.
    const cell = this.vbat() / this.cells;
    const lvl = cell < this.p("battEmerg") ? BattWarn.EMERGENCY : cell < this.p("battCrit") ? BattWarn.CRITICAL : cell < this.p("battWarn") ? BattWarn.LOW : BattWarn.OK;
    if (lvl > this.battLevel) {
      const mv = Math.round(cell * 1000);
      if (lvl === BattWarn.LOW) this.event(EventLevel.WARN, EventCode.BATTERY_LOW, "Battery low", mv);
      if (lvl === BattWarn.CRITICAL) this.event(EventLevel.ERROR, EventCode.BATTERY_CRITICAL, "Battery critical", mv);
      if (lvl === BattWarn.EMERGENCY) this.event(EventLevel.CRITICAL, EventCode.BATTERY_EMERGENCY, "Battery emergency: landing", mv);
      this.battLevel = lvl;
    } else if (!this.armed && lvl < this.battLevel) this.battLevel = lvl;
    if (this.armed && this.inAir) {
      if (this.battLevel >= BattWarn.CRITICAL && !(this.failsafe & FS.BATTERY)) {
        this.failsafe |= FS.BATTERY;
        if (this.auto !== AutoAction.LAND && this.auto !== AutoAction.RTH)
          this.auto = this.sensors & SENSOR.GNSS && this.homeSet ? AutoAction.RTH : AutoAction.LAND;
        this.event(EventLevel.ERROR, EventCode.FAILSAFE_BATTERY, "Battery failsafe", this.auto);
      }
      if (this.battLevel >= BattWarn.EMERGENCY) {
        this.battLock = true;
        this.failsafe |= FS.BATTERY;
        this.auto = AutoAction.LAND;
      }
    }

    // ---- killed: falls ----
    if (this.killedFall) {
      this.vz -= 9.81 * dt;
      this.alt += this.vz * dt;
      if (this.alt <= 0) {
        this.alt = 0;
        this.vz = 0;
        this.killedFall = false;
        this.inAir = false;
      }
    }

    const sticks = this.rc;
    const thr = sticks.throttle / 1000, yawIn = sticks.yaw / 1000, pIn = sticks.pitch / 1000, rIn = sticks.roll / 1000;
    const manual = this.auto === AutoAction.NONE && this.armed && this.inAir && silent < 1;
    const maxH = this.outdoor ? this.p("maxHSpeed") : Math.min(2, this.p("maxHSpeed"));
    const beginner = this.p("beginner") ? 0.5 : 1;
    let tgtVN = 0, tgtVE = 0, tgtVz = 0;
    const h = this.heading * D2R;

    // HOLD (pause) ends as soon as the pilot moves the sticks
    if (this.auto === AutoAction.HOLD && silent < 0.5 && !(this.failsafe & FS.APP_LINK) &&
        (Math.abs(pIn) > 0.3 || Math.abs(rIn) > 0.3 || Math.abs(yawIn) > 0.3 || Math.abs(thr) > 0.3)) this.auto = AutoAction.NONE;

    if (this.armed && !this.killedFall) {
      if (this.auto === AutoAction.TAKEOFF) {
        tgtVz = this.alt < this.takeoffTarget - 0.05 ? 0.6 : 0;
        if (this.alt >= this.takeoffTarget - 0.05) {
          this.auto = AutoAction.NONE;
          this.mode = this.sensors & (SENSOR.GNSS | SENSOR.FLOW) ? FlightMode.POS_HOLD : FlightMode.ALT_HOLD;
        }
      } else if (this.auto === AutoAction.LAND) {
        tgtVz = -(this.alt < 1 ? 0.3 : this.p("landSpeed"));
      } else if (this.auto === AutoAction.RTH) {
        const rthAlt = this.p("rthAlt");
        const dist = Math.hypot(this.posN, this.posE);
        if (dist > 1 && this.alt < rthAlt - 0.5) tgtVz = 2;
        else if (dist > 1) {
          const sp = Math.min(5, dist);
          tgtVN = (-this.posN / dist) * sp;
          tgtVE = (-this.posE / dist) * sp;
          this.heading = approach(this.heading, wrap360(Math.atan2(-this.posE, -this.posN) / D2R), 1, dt);
        } else tgtVz = -(this.alt < 1 ? 0.3 : this.p("landSpeed"));
        if (manual && this.p("rthStickCancel") && (Math.abs(pIn) > 0.3 || Math.abs(rIn) > 0.3)) this.auto = AutoAction.NONE;
      } else if (manual) {
        const fwd = pIn * maxH * beginner, right = rIn * maxH * beginner;
        tgtVN = fwd * Math.cos(h) - right * Math.sin(h);
        tgtVE = fwd * Math.sin(h) + right * Math.cos(h);
        tgtVz = Math.abs(thr) < 0.05 ? 0 : thr * this.p("maxClimb") * beginner;
        this.heading = wrap360(this.heading + yawIn * this.p("maxYawRate") * beginner * dt);
        if (this.mode === FlightMode.ALT_HOLD || this.mode === FlightMode.ANGLE || this.mode === FlightMode.ACRO) {
          // no position hold: the drone keeps drifting with the wind
          tgtVN += 0.4;
        }
      }
      // geofence
      if (this.outdoor && Math.hypot(this.posN, this.posE) > this.p("gfMaxDist") && this.auto === AutoAction.NONE) {
        const d = Math.hypot(this.posN, this.posE);
        tgtVN = (-this.posN / d) * 2;
        tgtVE = (-this.posE / d) * 2;
        if (!(this.failsafe & FS.GEOFENCE)) this.event(EventLevel.WARN, EventCode.GEOFENCE_BREACH, "Geofence reached");
        this.failsafe |= FS.GEOFENCE;
      } else this.failsafe &= ~FS.GEOFENCE;
      const ceiling = this.outdoor ? this.p("gfMaxAlt") : Math.min(this.p("gfMaxAlt"), this.p("gfCeiling"));
      if (this.alt >= ceiling && tgtVz > 0) tgtVz = 0;
    }

    if (!this.killedFall) {
      this.vN = approach(this.vN, tgtVN, 2, dt);
      this.vE = approach(this.vE, tgtVE, 2, dt);
      this.vz = approach(this.vz, this.inAir ? tgtVz : 0, 3, dt);
      this.posN += this.vN * dt;
      this.posE += this.vE * dt;
      if (this.inAir) this.alt = Math.max(0, this.alt + this.vz * dt);
    }

    // touchdown
    if (this.inAir && this.alt <= 0.02 && this.vz <= 0 && this.auto !== AutoAction.TAKEOFF) {
      this.inAir = false;
      this.alt = 0;
      this.vz = 0;
      if (this.auto === AutoAction.LAND || this.auto === AutoAction.RTH) this.event(EventLevel.INFO, EventCode.LANDED, "Landed");
      this.auto = AutoAction.NONE;
      this.landedT = 0;
    }
    if (this.armed && !this.inAir) {
      this.landedT += dt;
      if (this.landedT > this.p("autoDisarm") && this.flightTime > 0) {
        this.disarm();
        this.flightTime = 0;
      }
    }
    if (this.armed && this.inAir) this.flightTime += dt;

    // attitude follows the horizontal acceleration
    const aN = (tgtVN - this.vN) * 2, aE = (tgtVE - this.vE) * 2;
    const aF = aN * Math.cos(h) + aE * Math.sin(h), aR = -aN * Math.sin(h) + aE * Math.cos(h);
    this.pitch = approach(this.pitch, this.inAir ? clamp(Math.atan2(aF, 9.81) / D2R, -this.p("maxTilt"), this.p("maxTilt")) : 0, 6, dt);
    this.roll = approach(this.roll, this.inAir ? clamp(Math.atan2(aR, 9.81) / D2R, -this.p("maxTilt"), this.p("maxTilt")) : 0, 6, dt);

    // power
    const hover = this.outdoor ? 10 : 3.2;
    const spin = this.armed ? (this.inAir ? hover * (1 + 0.6 * Math.max(0, this.vz) + 0.05 * Math.hypot(this.vN, this.vE)) : 0.6) : 0.12;
    this.current = approach(this.current, spin + (this.motorTest ? 0.8 : 0), 4, dt);
    this.usedMah += (this.current * this.scaleI * 1000 * dt) / 3600;
  }

  /* ================================================================ */
  /* Telemetry                                                         */
  /* ================================================================ */

  private schedule(dt: number) {
    const every = (key: string, hz: number, fn: () => void) => {
      this.acc[key] = (this.acc[key] ?? 0) + dt * hz;
      while (this.acc[key] >= 1) {
        this.acc[key] -= 1;
        fn();
      }
    };
    every("state", 240 / Math.max(1, this.decim), () => this.emitState());
    every("status", this.rates.status, () => this.emitStatus());
    every("flight", this.rates.flight, () => this.emitFlight());
    every("power", this.rates.power, () => this.emitPower());
    every("nav", this.rates.nav, () => this.emitNav());
    every("motors", this.rates.motors, () => this.emitMotors());
    every("system", this.rates.system, () => this.emitSystem());
    every("checks", this.armed ? 0.5 : this.rates.checks, () => this.emit(FT.CHECKS, M.encodeChecks(this.checks())));
    every("caps", this.rates.caps, () => this.emitCaps());
    every("init", 1, () => this.emit(FT.INIT, new Uint8Array(137)));
    if (this.cal) every("cal", 10, () => this.emitCal());
    else this.acc.cal = 0;
  }

  private emitCaps() {
    this.emit(FT.CAPS, M.encodeCaps({
      proto: 2, fw: [0, 5, 0], hwRev: "H", model: this.outdoor ? Model.OUTDOOR : Model.INDOOR, sensors: this.sensors,
      nMotors: this.nMotors, frame: FrameType.QUAD_X, cells: this.cells, name: this.name, serial: this.outdoor ? "ACW-0001" : "ACW-0002",
    }));
  }

  private emitParam(id: number) {
    const idx = PARAMS.findIndex((p) => p.id === id);
    const v = this.params.get(id)!;
    this.emit(FT.PARAM, M.encodeParam({ id, value: v, flags: Math.abs(v - (this.saved.get(id) ?? v)) > 1e-6 ? 1 : 0, index: idx, count: PARAMS.length }));
  }

  private emitCal() {
    const c = this.cal;
    if (!c) return;
    this.emit(FT.CALIB, M.encodeCalib({ type: c.type, state: c.state, step: c.step, progress: c.progress, quality: c.quality, reason: c.reason, data: c.data }));
  }

  private emitState() {
    const yaw = -this.heading * D2R; // NWU: + = counter-clockwise from north
    const q = eulerToQuat(this.roll * D2R + noise(0.002), this.pitch * D2R + noise(0.002), yaw);
    this.emit(FT.STATE, M.encodeState({
      seq: (this.seq = (this.seq + this.decim) & 0xffff), tUs: Math.round(this.t * 1e6) >>> 0, q,
      bias: [0.0003, 0.0002, -0.0005], acc: [noise(0.02), noise(0.02), 1 + noise(0.02)], gyro: [noise(0.01), noise(0.01), noise(0.01)],
      mag: [22 + noise(0.5), -3 + noise(0.5), 39 + noise(0.5)], flags: 0x01 | (this.inAir ? 0 : 0x20),
    }));
  }

  private emitStatus() {
    this.emit(FT.STATUS, M.encodeStatus({
      uptimeMs: Math.round(this.t * 1000), irqCount: Math.round(this.t * 480), busErr: 0, fifoOvr: 0, tlmDrop: 0, uartDrop: 0,
      procCycMax: 18000, nGyro: Math.round(this.t * 960), nAcc: Math.round(this.t * 960), nMag: Math.round(this.t * 100), nMagDup: 0,
      nShNack: 0, wordsMax: 6, calibCount: 480, imuInit: 0, running: 1, bleDrop: 0, bleMtu: 247, bleState: 3, bleErr: 0,
    }));
  }

  private emitFlight() {
    const dist = Math.hypot(this.posN, this.posE);
    const gnss = !!(this.sensors & SENSOR.GNSS);
    const flags = (this.inAir ? FF.IN_AIR : FF.LANDED) | (this.homeSet && gnss ? FF.HOME_SET : 0) | FF.GEOFENCE_ON | (this.p("beginner") ? FF.BEGINNER : 0) |
      (!this.armed && this.p("armRequireChecks") && !this.checks().ready ? FF.ARM_BLOCKED : 0);
    this.emit(FT.FLIGHT, M.encodeFlight({
      arm: this.armed ? ArmState.ARMED : ArmState.DISARMED, mode: this.mode, auto: this.auto, failsafe: this.failsafe, flags,
      input: this.elrsActive ? InputSource.ELRS : this.t - this.lastAppMsg < 1 ? InputSource.APP : InputSource.NONE,
      flightTimeS: Math.round(this.flightTime), altRelM: this.alt, vzMs: this.vz, groundSpeedMs: Math.hypot(this.vN, this.vE),
      headingDeg: this.heading, homeDistM: gnss && this.homeSet ? Math.round(dist) : null,
      homeBearingDeg: wrap360(Math.atan2(-this.posE, -this.posN) / D2R), rthEtaS: gnss && this.homeSet ? Math.round(dist / 5 + this.alt / 0.7) : null,
    }));
  }

  private emitPower() {
    const v = this.vbat();
    const cell = v / this.cells;
    const pct = clamp(Math.round(((cell - 3.3) / 0.9) * 100), 0, 100);
    const warn = this.battLevel;
    const cap = this.outdoor ? 3000 : 450;
    this.emit(FT.POWER, M.encodePower({
      vbatV: v, currentA: this.current, usedMah: Math.round(this.usedMah), cells: this.cells, cellV: cell, pct,
      remainS: this.armed ? Math.round(((cap * 0.8 - this.usedMah) / Math.max(0.1, this.current * 1000)) * 3600) : null, warn,
    }));
  }

  private emitNav() {
    const gnss = !!(this.sensors & SENSOR.GNSS);
    const lat0 = 40.837, lon0 = 14.248;
    this.emit(FT.NAV, M.encodeNav({
      fix: gnss ? 3 : 0xff, sats: gnss ? 14 : 0, hdop: gnss ? 0.8 : 0,
      lat: gnss ? lat0 + this.posN / 111320 : 0, lon: gnss ? lon0 + this.posE / (111320 * Math.cos(lat0 * D2R)) : 0,
      altMslM: gnss ? 36.7 + this.alt : 0, velNed: [this.vN, this.vE, -this.vz],
      tofM: this.alt < 4 ? Math.max(0.012, this.alt + 0.012) : null, flowQ: this.alt < 4 ? clamp(Math.round(84 - this.alt * 8 + noise(2)), 0, 100) : 42,
      baroHpa: 1013.25 - this.alt * 0.12 - 6.4, windMs: gnss ? 3.1 : null,
    }));
  }

  private emitMotors() {
    const esc = !!(this.sensors & SENSOR.ESC_TLM);
    const base = this.armed ? (this.inAir ? 55 + this.vz * 8 : 8) : 0;
    const motors = Array.from({ length: this.nMotors }, (_, i) => {
      const test = this.motorTest && this.motorTest.motor === i + 1 ? this.motorTest.pct : 0;
      const out = clamp(Math.round(base + (this.inAir ? noise(2) : 0) + test), 0, 100);
      return { rpm: esc ? Math.round(out * 200 + (out ? noise(80) : 0)) : null, tempC: esc ? Math.round(30 + out / 5) : null, outPct: out };
    });
    this.emit(FT.MOTORS, M.encodeMotors({ motors }));
  }

  private emitSystem() {
    this.emit(FT.SYSTEM, M.encodeSystem({
      lq: this.elrsLinked ? 100 : null, rssi: -68, snr: 9, pktRateHz: this.elrsLinked ? 250 : 0, txPowerMw: this.elrsLinked ? 100 : 0, cpuPct: 46,
      loopHz: 960, jitterUs: 6, logPct: 12, imuTempC: 38.4, vibrationG: this.inAir ? 0.21 : 0.02, accClip: 0, attUncDeg: 0.6,
      estFlags: 0x01 | (this.compassCalibrated ? 0 : 0x08), magInterfPct: this.compassCalibrated ? 4 : 18,
    }));
  }

  checks(): M.ChecksMsg {
    const cell = this.vbat() / this.cells;
    const gnss = !!(this.sensors & SENSOR.GNSS);
    const items: M.CheckItem[] = [
      { id: CheckId.BATTERY, state: cell < this.p("battCrit") || this.battLevel >= BattWarn.CRITICAL ? CheckState.FAIL : cell < this.p("battWarn") || this.battLevel >= BattWarn.LOW ? CheckState.WARN : CheckState.OK, value: Math.round(cell * 1000) },
      { id: CheckId.IMU, state: CheckState.OK, value: 0 },
      { id: CheckId.COMPASS, state: this.compassCalibrated ? CheckState.OK : CheckState.WARN, value: this.compassCalibrated ? 4 : 18 },
      { id: CheckId.GNSS, state: gnss ? CheckState.OK : CheckState.NA, value: gnss ? 14 : 0 },
      { id: CheckId.HOME, state: gnss ? (this.homeSet ? CheckState.OK : CheckState.WARN) : CheckState.NA, value: 12 },
      { id: CheckId.FLOW, state: this.sensors & SENSOR.FLOW ? CheckState.OK : CheckState.NA, value: 82 },
      { id: CheckId.RADIO, state: this.elrsLinked ? CheckState.OK : CheckState.NA, value: this.elrsLinked ? 100 : 0 },
      { id: CheckId.EKF, state: CheckState.OK, value: 0 },
      { id: CheckId.FAILSAFE, state: CheckState.OK, value: Math.round(this.p("gfMaxDist")) },
      // value: low byte = calibrations required (bit = CalType - 1), high byte = recommended
      { id: CheckId.CALIBRATION, state: this.compassCalibrated ? CheckState.OK : CheckState.WARN, value: this.compassCalibrated ? 0 : (1 << (CalType.COMPASS - 1)) << 8 },
    ];
    const ready = items.every((i) => i.state !== CheckState.FAIL);
    return { ready, items };
  }
}

function popcount(x: number) {
  let n = 0;
  while (x) {
    n += x & 1;
    x >>= 1;
  }
  return n;
}

/** ZYX Euler (roll φ about x, pitch θ about y, yaw ψ about z) → quaternion [w x y z]. */
export function eulerToQuat(r: number, p: number, y: number): [number, number, number, number] {
  const cr = Math.cos(r / 2), sr = Math.sin(r / 2), cp = Math.cos(p / 2), sp = Math.sin(p / 2), cy = Math.cos(y / 2), sy = Math.sin(y / 2);
  return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
}

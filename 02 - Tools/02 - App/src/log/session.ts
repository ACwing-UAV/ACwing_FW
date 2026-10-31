/**
 * State of the Flight logs screen, kept outside React so a download goes on
 * when the user leaves the screen: the last flight over BLE, the USB link
 * with every log, the running download, the erase.
 */
import { drone } from "../link/drone";
import { grantedFcPort, pickFcPort, usbAvailable } from "../link/serialTransport";
import type { Transport } from "../link/transport";
import { ArmState } from "../protocol/ids";
import type { LogEntryMsg } from "../protocol/messages";
import { SimUsbTransport } from "../sim/simUsbTransport";
import { LogClient, logErrorKey } from "./client";
import { BleLogPort, UsbLogPort } from "./ports";
import { logStore, type StoredLogMeta } from "./store";

export interface Job {
  source: "ble" | "usb";
  logId: number;
  done: number;
  total: number;
  rate: number;
  /** position in a "download all" batch */
  n: number;
  of: number;
  state: "running" | "done" | "failed" | "cancelled";
  error?: string;
  saved?: StoredLogMeta;
}

export interface LogsState {
  ble: { entry: LogEntryMsg | null; listed: boolean; busy: boolean; error?: string };
  usb: { open: boolean; name: string; sim: boolean; entries: LogEntryMsg[]; busy: boolean; erasing: boolean; error?: string };
  job: Job | null;
}

let s: LogsState = {
  ble: { entry: null, listed: false, busy: false },
  usb: { open: false, name: "", sim: false, entries: [], busy: false, erasing: false },
  job: null,
};
const listeners = new Set<() => void>();
const set = (patch: Partial<LogsState>) => {
  s = { ...s, ...patch };
  listeners.forEach((l) => l());
};
let usbPort: UsbLogPort | null = null;
let abort: AbortController | null = null;

export const logsSession = {
  subscribe(fn: () => void) {
    listeners.add(fn);
    return () => void listeners.delete(fn);
  },
  get: () => s,

  /** Typical BLE download speed: a few kB/s to ~20 kB/s depending on the phone (HANDOVER_APP §4). */
  bleEstimateS: (bytes: number) => bytes / 10_000,
  /** Flight time of a log from its size: ~3.4 kB/s of records (HANDOVER_APP §7). */
  durationEstimateS: (bytes: number) => bytes / 3400,

  resetBle() {
    if (s.ble.listed || s.ble.entry) set({ ble: { entry: null, listed: false, busy: false } });
  },

  /** BLE: the last flight (drone connected and disarmed). */
  async listBle() {
    if (s.ble.busy) return;
    set({ ble: { ...s.ble, busy: true, error: undefined } });
    try {
      const [entry] = await new LogClient(new BleLogPort()).list();
      set({ ble: { entry: entry ?? null, listed: true, busy: false } });
    } catch (e) {
      set({ ble: { ...s.ble, busy: false, listed: true, error: logErrorKey(e) } });
    }
  },

  async downloadBle() {
    const e = s.ble.entry;
    if (!e || s.job?.state === "running") return;
    await run([e], "ble", new LogClient(new BleLogPort()));
  },

  usbSupported: usbAvailable,

  /** Opens the USB link: an FC port granted earlier, else the browser's picker; `sim` = simulated FC. */
  async connectUsb(opts: { sim?: boolean; silent?: boolean } = {}) {
    if (s.usb.open) return;
    let t: Transport | null = null;
    try {
      t = opts.sim ? new SimUsbTransport() : (await grantedFcPort()) ?? (opts.silent ? null : await pickFcPort());
    } catch (e) {
      if (!opts.silent) set({ usb: { ...s.usb, error: (e as { code?: string }).code === "notFound" ? "logs.err.noPort" : "logs.err.usbOpen" } });
      return;
    }
    if (!t) return;
    const port = new UsbLogPort(t);
    try {
      await port.open(() => {
        usbPort = null;
        abort?.abort();
        set({ usb: { ...s.usb, open: false, entries: [], error: "logs.err.unplugged" } });
      });
    } catch {
      set({ usb: { ...s.usb, error: "logs.err.usbOpen" } });
      return;
    }
    usbPort = port;
    set({ usb: { open: true, name: t.deviceName, sim: !!opts.sim, entries: [], busy: false, erasing: false } });
    await logsSession.listUsb();
  },

  async disconnectUsb() {
    abort?.abort();
    const p = usbPort;
    usbPort = null;
    set({ usb: { open: false, name: "", sim: false, entries: [], busy: false, erasing: false } });
    await p?.close();
  },

  async listUsb() {
    if (!usbPort) return;
    set({ usb: { ...s.usb, busy: true, error: undefined } });
    try {
      const entries = await new LogClient(usbPort).list();
      set({ usb: { ...s.usb, entries, busy: false } });
    } catch (e) {
      set({ usb: { ...s.usb, busy: false, error: logErrorKey(e) } });
    }
  },

  async downloadUsb(ids: number[]) {
    if (!usbPort || s.job?.state === "running") return;
    const list = s.usb.entries.filter((e) => ids.includes(e.id));
    await run(list, "usb", new LogClient(usbPort));
  },

  async eraseUsb() {
    if (!usbPort || s.usb.erasing) return;
    set({ usb: { ...s.usb, erasing: true, error: undefined } });
    try {
      await new LogClient(usbPort).erase();
      set({ usb: { ...s.usb, erasing: false } });
      await logsSession.listUsb();
    } catch (e) {
      set({ usb: { ...s.usb, erasing: false, error: logErrorKey(e) } });
    }
  },

  cancel() {
    abort?.abort();
  },

  dismissJob() {
    if (s.job?.state !== "running") set({ job: null });
  },
};

/** Downloads the logs one after the other and stores them. */
async function run(list: LogEntryMsg[], source: "ble" | "usb", client: LogClient) {
  abort = new AbortController();
  const signal = abort.signal;
  for (let i = 0; i < list.length; i++) {
    const e = list[i];
    set({ job: { source, logId: e.id, done: 0, total: e.bytes, rate: 0, n: i + 1, of: list.length, state: "running" } });
    try {
      const data = await client.read(e.id, e.bytes, {
        signal,
        onProgress: (p) => set({ job: { ...s.job!, done: p.done, rate: p.rate } }),
      });
      // keyed by the drone name in the log's INFO record: the same log from BLE or USB is kept once
      const saved = await logStore.put(data, { logId: e.id, t0Ms: e.t0Ms, source, truncated: e.truncated });
      set({ job: { ...s.job!, done: e.bytes, state: "done", saved } });
    } catch (err) {
      const key = logErrorKey(err);
      set({ job: { ...s.job!, state: key === "logs.err.cancelled" ? "cancelled" : "failed", error: key } });
      return;
    } finally {
      if (i === list.length - 1 || signal.aborted) abort = null;
    }
  }
}

/** Already on this device? (same log id and start time; the drone name comes from the log itself) */
export const storedAs = (metas: StoredLogMeta[], e: LogEntryMsg) => metas.find((m) => m.logId === e.id && m.t0Ms === e.t0Ms) ?? null;

/** The drone can serve the last flight over BLE now. */
export const bleReady = () => {
  const d = drone.getSnapshot();
  return d.link === "connected" && !!d.caps && d.flight?.arm !== ArmState.ARMED;
};

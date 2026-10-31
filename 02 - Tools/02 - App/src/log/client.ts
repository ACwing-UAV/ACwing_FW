/**
 * Flight-log client: LOG_LIST, LOG_READ (in chunks), LOG_ERASE over any
 * LogPort (BLE through the flight session, USB through its own serial link).
 * Port of `FcLink` in `02 - Tools/01 - Python/acwing_tools/flightlog.py`.
 *
 * Rules (HANDOVER_APP.md §5):
 *  - the ACK comes AFTER the data: it means "complete";
 *  - 2 s without any log frame while a command is pending → retry with a new seq;
 *  - ERASE answers IN_PROGRESS first and may take minutes;
 *  - reads go in chunks (256 KB USB, 32 KB BLE): a lost frame costs one chunk.
 */
import { AckResult, FT, OP, Reason, WIPE_MAGIC } from "../protocol/ids";
import { decodeLogData, decodeLogEntry, decodeAck, type LogEntryMsg } from "../protocol/messages";

export interface LogPort {
  readonly kind: "ble" | "usb";
  /** writes `op, seq, args` (wrapped in a CMD frame on USB) and returns the seq used */
  send(op: number, args: Uint8Array): number;
  /** every received frame (the client keeps LOG_ENTRY, LOG_DATA and ACK) */
  listen(fn: (type: number, payload: Uint8Array) => void): () => void;
  /** a long transfer starts / ends (BLE: lower the STATE rate meanwhile) */
  begin?(): void;
  end?(): void;
}

export class LogError extends Error {
  constructor(public kind: "rejected" | "timeout" | "cancelled" | "data" | "offline", public result = 0, public reason = 0, message?: string) {
    super(message ?? (kind === "rejected" ? `result ${result}, reason ${reason}` : kind));
  }
  /** a refusal that a retry cannot fix (BUSY can: another read is still finishing) */
  get final() {
    return this.kind === "cancelled" || this.kind === "offline" || (this.kind === "rejected" && this.result !== AckResult.BUSY);
  }
}

export interface ReadProgress {
  done: number;
  total: number;
  /** bytes/s over the last chunk */
  rate: number;
}

const CHUNK = { usb: 256 * 1024, ble: 32 * 1024 } as const;

export class LogClient {
  timeoutMs = 2000;
  eraseTimeoutMs = 300_000;
  retries = 2;

  constructor(readonly port: LogPort) {}

  /** Sends one command and collects the frames before its final ACK. */
  private run(op: number, args: Uint8Array, onFrame?: (type: number, p: Uint8Array) => void, signal?: AbortSignal): Promise<void> {
    return new Promise((resolve, reject) => {
      let deadline = Date.now() + this.timeoutMs;
      let seq = -1;
      let tries = 0;
      const done = (e?: LogError) => {
        clearInterval(timer);
        unlisten();
        signal?.removeEventListener("abort", onAbort);
        if (e) reject(e);
        else resolve();
      };
      const onAbort = () => done(new LogError("cancelled"));
      const unlisten = this.port.listen((type, p) => {
        if (type === FT.ACK && p.length >= 4) {
          const a = decodeAck(p);
          if (a.op !== op || a.seq !== seq) return;
          if (a.result === AckResult.IN_PROGRESS) {
            deadline = Date.now() + this.eraseTimeoutMs;
            return;
          }
          if (a.result === AckResult.OK) done();
          else done(new LogError("rejected", a.result, a.reason));
          return;
        }
        if (type === FT.LOG_ENTRY || type === FT.LOG_DATA) {
          deadline = Math.max(deadline, Date.now() + this.timeoutMs);
          onFrame?.(type, p);
        }
      });
      const fire = () => {
        try {
          seq = this.port.send(op, args);
        } catch {
          return done(new LogError("offline"));
        }
        deadline = Date.now() + this.timeoutMs;
      };
      const timer = setInterval(() => {
        if (Date.now() < deadline) return;
        if (tries++ >= this.retries) return done(new LogError("timeout"));
        fire();
      }, 100);
      if (signal?.aborted) return onAbort();
      signal?.addEventListener("abort", onAbort);
      fire();
    });
  }

  /** LOG_LIST: every log over USB, the last flight only over BLE. */
  async list(signal?: AbortSignal): Promise<LogEntryMsg[]> {
    let out: LogEntryMsg[] = [];
    await this.run(OP.LOG_LIST, new Uint8Array(0), (type, p) => {
      if (type !== FT.LOG_ENTRY || p.length < 15) return;
      const e = decodeLogEntry(p);
      out = [...out.filter((x) => x.id !== e.id), e]; // a retried list sends the entries again
    }, signal);
    return out.sort((a, b) => a.index - b.index);
  }

  /** Reads a whole log in chunks; a chunk with a gap or a timeout is read again (3 attempts). */
  async read(id: number, size: number, opts: { chunk?: number; onProgress?: (p: ReadProgress) => void; signal?: AbortSignal } = {}): Promise<Uint8Array> {
    const chunk = opts.chunk ?? CHUNK[this.port.kind];
    const buf = new Uint8Array(size).fill(0xff);
    this.port.begin?.();
    try {
      for (let off = 0; off < size;) {
        const n = Math.min(chunk, size - off);
        let attempt = 0;
        for (;;) {
          const t0 = Date.now();
          let got = 0, next = off, gap = false;
          try {
            await this.run(OP.LOG_READ, logReadArgs(id, off, n), (type, p) => {
              if (type !== FT.LOG_DATA || p.length < 6) return;
              const d = decodeLogData(p);
              if (d.id !== id || d.offset < off || d.offset + d.data.length > off + n) return;
              if (d.offset !== next) gap = true;
              buf.set(d.data, d.offset);
              got += d.data.length;
              next = d.offset + d.data.length;
              opts.onProgress?.({ done: Math.min(size, off + got), total: size, rate: (got * 1000) / Math.max(1, Date.now() - t0) });
            }, opts.signal);
            if (!gap && got === n) break;
            throw new LogError("data", 0, 0, `log ${id}: ${got} of ${n} bytes at offset ${off}`);
          } catch (e) {
            const err = e as LogError;
            if (err.final || ++attempt >= 3) throw err;
            if (err.result === AckResult.BUSY) await new Promise((r) => setTimeout(r, 500));
          }
        }
        off += n;
      }
      return buf;
    } finally {
      this.port.end?.();
    }
  }

  /** LOG_ERASE (USB only): IN_PROGRESS, then OK when the flash is blank. */
  erase(signal?: AbortSignal) {
    return this.run(OP.LOG_ERASE, Uint8Array.from(WIPE_MAGIC), undefined, signal);
  }
}

export function logReadArgs(id: number, offset: number, length: number) {
  const a = new Uint8Array(10);
  const dv = new DataView(a.buffer);
  dv.setUint16(0, id, true);
  dv.setUint32(2, offset, true);
  dv.setUint32(6, length, true);
  return a;
}

/** i18n key for a log refusal (HANDOVER_APP.md §5 table). */
export function logErrorKey(e: unknown): string {
  if (!(e instanceof LogError)) return "logs.err.failed";
  if (e.kind === "timeout") return "logs.err.timeout";
  if (e.kind === "cancelled") return "logs.err.cancelled";
  if (e.kind === "offline") return "cmdError.offline";
  if (e.kind === "data") return "logs.err.data";
  if (e.result === AckResult.REJECTED && e.reason === Reason.ARMED) return "logs.err.armed";
  if (e.result === AckResult.REJECTED && e.reason === Reason.OUT_OF_RANGE) return "logs.err.notLast";
  if (e.result === AckResult.REJECTED && e.reason === Reason.SENSOR_MISSING) return "logs.err.noFlash";
  if (e.result === AckResult.INVALID && e.reason === Reason.OUT_OF_RANGE) return "logs.err.unknownLog";
  if (e.result === AckResult.BUSY) return "logs.err.busy";
  if (e.result === AckResult.UNSUPPORTED) return "logs.err.unsupported";
  return "logs.err.failed";
}

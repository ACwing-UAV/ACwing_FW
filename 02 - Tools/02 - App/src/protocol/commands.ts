/**
 * Commands app → drone, written on the CMD characteristic (≤ 20 bytes).
 * Format: [op u8][seq u8][payload…]. Legacy ops 0x01/0x02 keep their phase-2 format.
 * Every command with a sequence number is answered by an ACK frame, except RC and HEARTBEAT.
 */
import { OP, ARM_MAGIC, KILL_MAGIC, PROPS_OFF_MAGIC, BLE } from "./ids";
import { Reader, Writer } from "./frame";

export interface RcInput {
  throttle: number; // -1000..1000 (bottom..top of the stick)
  yaw: number; // -1000 left .. 1000 right
  pitch: number; // -1000 back .. 1000 forward
  roll: number; // -1000 left .. 1000 right
  buttons: number; // bit0 = haptic/feedback request (reserved)
}

const cmd = (op: number, seq: number, fill?: (w: Writer) => void) => {
  const w = new Writer().u8(op).u8(seq & 0xff);
  fill?.(w);
  const out = w.done();
  if (out.length > BLE.cmdMaxLen) throw new Error(`command 0x${op.toString(16)} longer than ${BLE.cmdMaxLen} bytes`);
  return out;
};

export const Cmd = {
  sendInit: () => Uint8Array.of(OP.SEND_INIT),
  stateDecim: (n: number) => Uint8Array.of(OP.STATE_DECIM, Math.max(1, Math.min(240, n))),
  heartbeat: (seq: number) => cmd(OP.HEARTBEAT, seq),
  rc: (seq: number, rc: RcInput) =>
    cmd(OP.RC, seq, (w) => w.i16(rc.throttle).i16(rc.yaw).i16(rc.pitch).i16(rc.roll).u8(rc.buttons)),
  arm: (seq: number) => cmd(OP.ARM, seq, (w) => w.raw(ARM_MAGIC)),
  disarm: (seq: number) => cmd(OP.DISARM, seq),
  takeoff: (seq: number, altM: number) => cmd(OP.TAKEOFF, seq, (w) => w.u16(altM * 100)),
  land: (seq: number) => cmd(OP.LAND, seq),
  rth: (seq: number) => cmd(OP.RTH, seq),
  hold: (seq: number) => cmd(OP.HOLD, seq),
  kill: (seq: number) => cmd(OP.KILL, seq, (w) => w.raw(KILL_MAGIC)),
  setMode: (seq: number, mode: number) => cmd(OP.SET_MODE, seq, (w) => w.u8(mode)),
  paramGet: (seq: number, id: number) => cmd(OP.PARAM_GET, seq, (w) => w.u16(id)),
  paramSet: (seq: number, id: number, value: number) => cmd(OP.PARAM_SET, seq, (w) => w.u16(id).f32(value)),
  paramSave: (seq: number) => cmd(OP.PARAM_SAVE, seq),
  paramDefaults: (seq: number) => cmd(OP.PARAM_DEFAULTS, seq),
  paramList: (seq: number) => cmd(OP.PARAM_LIST, seq),
  calStart: (seq: number, type: number, option = 0) => cmd(OP.CAL_START, seq, (w) => w.u8(type).u8(option)),
  calCapture: (seq: number) => cmd(OP.CAL_CAPTURE, seq),
  calAbort: (seq: number) => cmd(OP.CAL_ABORT, seq),
  calSave: (seq: number) => cmd(OP.CAL_SAVE, seq),
  calRestore: (seq: number, type: number) => cmd(OP.CAL_RESTORE, seq, (w) => w.u8(type)),
  calSetRef: (seq: number, type: number, index: number, value: number) =>
    cmd(OP.CAL_SET_REF, seq, (w) => w.u8(type).u8(index).f32(value)),
  motorTest: (seq: number, motor: number, throttlePct: number, durationMs: number) =>
    cmd(OP.MOTOR_TEST, seq, (w) => w.u8(motor).u8(throttlePct).u16(durationMs).u8(PROPS_OFF_MAGIC)),
  motorReverse: (seq: number, motor: number) => cmd(OP.MOTOR_REVERSE, seq, (w) => w.u8(motor)),
  motorRemap: (seq: number, map: number[]) => cmd(OP.MOTOR_REMAP, seq, (w) => w.raw(map.slice(0, 8))),
  streamRates: (seq: number, r: { flight: number; power: number; nav: number; motors: number; system: number }) =>
    cmd(OP.STREAM_RATES, seq, (w) => w.u8(r.flight).u8(r.power).u8(r.nav).u8(r.motors).u8(r.system)),
  request: (seq: number, frameType: number) => cmd(OP.REQUEST, seq, (w) => w.u8(frameType)),
  setName: (seq: number, name: string) => cmd(OP.SET_NAME, seq, (w) => w.str(name, 16)),
};

/* ------------------------------------------------------------------ */
/* Decoder (simulator and tests; the firmware does the same in C)      */
/* ------------------------------------------------------------------ */

export type ParsedCmd =
  | { op: typeof OP.SEND_INIT }
  | { op: typeof OP.STATE_DECIM; n: number }
  | { op: number; seq: number; r: Reader };

export function parseCommand(b: Uint8Array): ParsedCmd | null {
  if (b.length < 1) return null;
  if (b[0] === OP.SEND_INIT) return { op: OP.SEND_INIT };
  if (b[0] === OP.STATE_DECIM) return { op: OP.STATE_DECIM, n: b[1] ?? 1 };
  if (b.length < 2) return null;
  const r = new Reader(b);
  const op = r.u8_();
  const seq = r.u8_();
  return { op, seq, r };
}

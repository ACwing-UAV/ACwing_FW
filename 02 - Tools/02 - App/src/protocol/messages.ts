/**
 * Payload layouts of every frame, drone → app, with decoders (used by the app)
 * and encoders (used by the simulator and the tests, and as an executable
 * reference for the firmware). All little-endian, packed, no padding.
 */
import { FT, CalType } from "./ids";
import { Reader, Writer } from "./frame";

/* ------------------------------------------------------------------ */
/* Existing frames (phase 1/2)                                         */
/* ------------------------------------------------------------------ */

export interface StateMsg {
  seq: number;
  tUs: number;
  q: [number, number, number, number]; // body→world, [w x y z]
  bias: number[]; // rad/s
  acc: number[]; // g
  gyro: number[]; // rad/s
  mag: number[]; // µT
  flags: number;
}

export function decodeState(p: Uint8Array): StateMsg {
  const r = new Reader(p);
  return {
    seq: r.u16(),
    tUs: r.u32(),
    q: r.f32s(4) as StateMsg["q"],
    bias: r.f32s(3),
    acc: r.f32s(3),
    gyro: r.f32s(3),
    mag: r.f32s(3),
    flags: r.u8_(),
  };
}

export function encodeState(m: StateMsg): Uint8Array {
  const w = new Writer().u16(m.seq).u32(m.tUs);
  [...m.q, ...m.bias, ...m.acc, ...m.gyro, ...m.mag].forEach((v) => w.f32(v));
  return w.u8(m.flags).done();
}

export interface StatusMsg {
  uptimeMs: number;
  irqCount: number;
  busErr: number;
  fifoOvr: number;
  tlmDrop: number;
  uartDrop: number;
  procCycMax: number;
  nGyro: number;
  nAcc: number;
  nMag: number;
  nMagDup: number;
  nShNack: number;
  wordsMax: number;
  calibCount: number;
  imuInit: number;
  running: number;
  bleDrop: number;
  bleMtu: number;
  bleState: number;
  bleErr: number;
}

export function decodeStatus(p: Uint8Array): StatusMsg {
  const r = new Reader(p);
  return {
    uptimeMs: r.u32(),
    irqCount: r.u32(),
    busErr: r.u32(),
    fifoOvr: r.u32(),
    tlmDrop: r.u32(),
    uartDrop: r.u32(),
    procCycMax: r.u32(),
    nGyro: r.u32(),
    nAcc: r.u32(),
    nMag: r.u32(),
    nMagDup: r.u32(),
    nShNack: r.u32(),
    wordsMax: r.u16(),
    calibCount: r.u16(),
    imuInit: r.i8(),
    running: r.u8_(),
    bleDrop: r.u32(),
    bleMtu: r.u16(),
    bleState: r.u8_(),
    bleErr: r.u8_(),
  };
}

export function encodeStatus(m: StatusMsg): Uint8Array {
  return new Writer()
    .u32(m.uptimeMs).u32(m.irqCount).u32(m.busErr).u32(m.fifoOvr).u32(m.tlmDrop).u32(m.uartDrop).u32(m.procCycMax)
    .u32(m.nGyro).u32(m.nAcc).u32(m.nMag).u32(m.nMagDup).u32(m.nShNack)
    .u16(m.wordsMax).u16(m.calibCount).i8(m.imuInit).u8(m.running)
    .u32(m.bleDrop).u16(m.bleMtu).u8(m.bleState).u8(m.bleErr)
    .done();
}

/* ------------------------------------------------------------------ */
/* New frames (protocol v2)                                            */
/* ------------------------------------------------------------------ */

/** 0x05 CAPS – 39 B. On subscribe, on REQUEST, every 5 s. */
export interface CapsMsg {
  proto: number;
  fw: [number, number, number];
  hwRev: string; // one ASCII letter, e.g. "H"
  model: number; // Model
  sensors: number; // SENSOR bits
  nMotors: number;
  frame: number; // FrameType
  cells: number; // 0 = not known yet
  name: string; // ≤ 16 B UTF-8
  serial: string; // ≤ 12 B ASCII
}

export function decodeCaps(p: Uint8Array): CapsMsg {
  const r = new Reader(p);
  return {
    proto: r.u8_(),
    fw: [r.u8_(), r.u8_(), r.u8_()],
    hwRev: String.fromCharCode(r.u8_() || 63),
    model: r.u8_(),
    sensors: r.u16(),
    nMotors: r.u8_(),
    frame: r.u8_(),
    cells: r.u8_(),
    name: r.str(16),
    serial: r.str(12),
  };
}

export function encodeCaps(m: CapsMsg): Uint8Array {
  return new Writer()
    .u8(m.proto).u8(m.fw[0]).u8(m.fw[1]).u8(m.fw[2]).u8(m.hwRev.charCodeAt(0))
    .u8(m.model).u16(m.sensors).u8(m.nMotors).u8(m.frame).u8(m.cells)
    .str(m.name, 16).str(m.serial, 12)
    .done();
}

/** 0x06 FLIGHT – 25 B, 10 Hz. */
export interface FlightMsg {
  arm: number; // ArmState
  mode: number; // FlightMode
  auto: number; // AutoAction
  failsafe: number; // FS bits
  flags: number; // FF bits
  input: number; // InputSource
  flightTimeS: number;
  altRelM: number; // wire: cm i32
  vzMs: number; // wire: cm/s i16, + up
  groundSpeedMs: number; // wire: cm/s u16
  headingDeg: number; // wire: cdeg u16, 0..359.99, 0 = north
  homeDistM: number | null; // wire: m u16, 0xFFFF = n/a
  homeBearingDeg: number; // wire: cdeg u16, direction FROM the drone TO home
  rthEtaS: number | null; // wire: s u16, 0xFFFF = n/a
}

export function decodeFlight(p: Uint8Array): FlightMsg {
  const r = new Reader(p);
  const arm = r.u8_(), mode = r.u8_(), auto = r.u8_(), failsafe = r.u8_(), flags = r.u16(), input = r.u8_();
  const flightTimeS = r.u16(), alt = r.i32(), vz = r.i16(), gs = r.u16(), hdg = r.u16();
  const hd = r.u16(), hb = r.u16(), eta = r.u16();
  return {
    arm, mode, auto, failsafe, flags, input, flightTimeS,
    altRelM: alt / 100, vzMs: vz / 100, groundSpeedMs: gs / 100, headingDeg: hdg / 100,
    homeDistM: hd === 0xffff ? null : hd, homeBearingDeg: hb / 100, rthEtaS: eta === 0xffff ? null : eta,
  };
}

export function encodeFlight(m: FlightMsg): Uint8Array {
  return new Writer()
    .u8(m.arm).u8(m.mode).u8(m.auto).u8(m.failsafe).u16(m.flags).u8(m.input).u16(m.flightTimeS)
    .i32(m.altRelM * 100).i16(m.vzMs * 100).u16(m.groundSpeedMs * 100).u16(((m.headingDeg % 360) + 360) % 360 * 100)
    .u16(m.homeDistM ?? 0xffff).u16(((m.homeBearingDeg % 360) + 360) % 360 * 100).u16(m.rthEtaS ?? 0xffff)
    .done();
}

/** 0x07 POWER – 13 B, 5 Hz. */
export interface PowerMsg {
  vbatV: number; // wire mV u16
  currentA: number; // wire cA i16
  usedMah: number; // u16
  cells: number; // u8
  cellV: number; // wire mV u16, average
  pct: number; // u8 0..100
  remainS: number | null; // u16, 0xFFFF = n/a
  warn: number; // BattWarn
}

export function decodePower(p: Uint8Array): PowerMsg {
  const r = new Reader(p);
  const vb = r.u16(), cur = r.i16(), used = r.u16(), cells = r.u8_(), cell = r.u16(), pct = r.u8_(), rem = r.u16(), warn = r.u8_();
  return { vbatV: vb / 1000, currentA: cur / 100, usedMah: used, cells, cellV: cell / 1000, pct, remainS: rem === 0xffff ? null : rem, warn };
}

export function encodePower(m: PowerMsg): Uint8Array {
  return new Writer()
    .u16(m.vbatV * 1000).i16(m.currentA * 100).u16(m.usedMah).u8(m.cells).u16(m.cellV * 1000).u8(m.pct)
    .u16(m.remainS ?? 0xffff).u8(m.warn)
    .done();
}

/** 0x08 NAV – 31 B, 5 Hz. Fields of absent sensors carry their n/a value. */
export interface NavMsg {
  fix: number; // 0 none, 2 = 2D, 3 = 3D, 0xFF = no GNSS hardware
  sats: number;
  hdop: number; // wire ×100 u16
  lat: number; // wire 1e-7 deg i32
  lon: number;
  altMslM: number; // wire cm i32
  velNed: [number, number, number]; // wire cm/s i16 ×3
  tofM: number | null; // wire mm u16, 0xFFFF = n/a or out of range
  flowQ: number | null; // u8 0..100, 0xFF = n/a
  baroHpa: number; // wire Pa u32
  windMs: number | null; // wire cm/s u16, 0xFFFF = n/a
}

export function decodeNav(p: Uint8Array): NavMsg {
  const r = new Reader(p);
  const fix = r.u8_(), sats = r.u8_(), hdop = r.u16() / 100, lat = r.i32() / 1e7, lon = r.i32() / 1e7, alt = r.i32() / 100;
  const v: [number, number, number] = [r.i16() / 100, r.i16() / 100, r.i16() / 100];
  const tof = r.u16(), fq = r.u8_(), baro = r.u32(), wind = r.u16();
  return {
    fix, sats, hdop, lat, lon, altMslM: alt, velNed: v,
    tofM: tof === 0xffff ? null : tof / 1000, flowQ: fq === 0xff ? null : fq, baroHpa: baro / 100,
    windMs: wind === 0xffff ? null : wind / 100,
  };
}

export function encodeNav(m: NavMsg): Uint8Array {
  return new Writer()
    .u8(m.fix).u8(m.sats).u16(m.hdop * 100).i32(m.lat * 1e7).i32(m.lon * 1e7).i32(m.altMslM * 100)
    .i16(m.velNed[0] * 100).i16(m.velNed[1] * 100).i16(m.velNed[2] * 100)
    .u16(m.tofM === null ? 0xffff : m.tofM * 1000).u8(m.flowQ ?? 0xff).u32(m.baroHpa * 100)
    .u16(m.windMs === null ? 0xffff : m.windMs * 100)
    .done();
}

/** 0x09 MOTORS – 1 + 4·n B, 5 Hz. */
export interface MotorMsg {
  rpm: number | null; // u16, 0xFFFF = n/a (brushed motors without telemetry)
  tempC: number | null; // i8, -128 = n/a
  outPct: number; // u8, commanded output 0..100
}
export interface MotorsMsg {
  motors: MotorMsg[];
}

export function decodeMotors(p: Uint8Array): MotorsMsg {
  const r = new Reader(p);
  const n = r.u8_();
  const motors: MotorMsg[] = [];
  for (let i = 0; i < n && r.has(4); i++) {
    const rpm = r.u16(), t = r.i8(), out = r.u8_();
    motors.push({ rpm: rpm === 0xffff ? null : rpm, tempC: t === -128 ? null : t, outPct: out });
  }
  return { motors };
}

export function encodeMotors(m: MotorsMsg): Uint8Array {
  const w = new Writer().u8(m.motors.length);
  m.motors.forEach((x) => w.u16(x.rpm ?? 0xffff).i8(x.tempC ?? -128).u8(x.outPct));
  return w.done();
}

/** 0x0A SYSTEM – 23 B, 2 Hz. */
export interface SystemMsg {
  lq: number | null; // ELRS link quality %, 0xFF = no receiver / not linked
  rssi: number; // dBm i8
  snr: number; // dB i8
  pktRateHz: number; // u16
  txPowerMw: number; // u16
  cpuPct: number; // u8
  loopHz: number; // u16, measured control/estimation loop
  jitterUs: number; // u16
  logPct: number; // u8, flash log used
  imuTempC: number; // wire cC i16
  vibrationG: number; // wire mg u16 (RMS)
  accClip: number; // u16 count
  attUncDeg: number; // wire cdeg u16 (1σ of the attitude estimate)
  estFlags: number; // bit0 EKF ok, bit1 acc gated, bit2 mag gated, bit3 mag interference
  magInterfPct: number; // u8
}

export function decodeSystem(p: Uint8Array): SystemMsg {
  const r = new Reader(p);
  const lq = r.u8_();
  return {
    lq: lq === 0xff ? null : lq, rssi: r.i8(), snr: r.i8(), pktRateHz: r.u16(), txPowerMw: r.u16(), cpuPct: r.u8_(),
    loopHz: r.u16(), jitterUs: r.u16(), logPct: r.u8_(), imuTempC: r.i16() / 100, vibrationG: r.u16() / 1000,
    accClip: r.u16(), attUncDeg: r.u16() / 100, estFlags: r.u8_(), magInterfPct: r.u8_(),
  };
}

export function encodeSystem(m: SystemMsg): Uint8Array {
  return new Writer()
    .u8(m.lq ?? 0xff).i8(m.rssi).i8(m.snr).u16(m.pktRateHz).u16(m.txPowerMw).u8(m.cpuPct).u16(m.loopHz).u16(m.jitterUs)
    .u8(m.logPct).i16(m.imuTempC * 100).u16(m.vibrationG * 1000).u16(m.accClip).u16(m.attUncDeg * 100).u8(m.estFlags)
    .u8(m.magInterfPct)
    .done();
}

/** 0x0B EVENT – 8 + text B, on event (repeat ×3, 100 ms apart, for level ≥ ERROR). */
export interface EventMsg {
  level: number; // EventLevel
  code: number; // EventCode
  arg: number; // i32, code-specific
  text: string; // ≤ 40 B ASCII, English fallback
}

export function decodeEvent(p: Uint8Array): EventMsg {
  const r = new Reader(p);
  const level = r.u8_(), code = r.u16(), arg = r.i32(), n = r.u8_();
  return { level, code, arg, text: r.str(n) };
}

export function encodeEvent(m: EventMsg): Uint8Array {
  const t = new TextEncoder().encode(m.text).subarray(0, 40);
  return new Writer().u8(m.level).u16(m.code).i32(m.arg).u8(t.length).raw(t).done();
}

/** 0x0C PARAM – 11 B, answer to PARAM_GET/SET/LIST. */
export interface ParamMsg {
  id: number;
  value: number;
  flags: number; // bit0 = differs from the value saved in flash, bit1 = read-only
  index: number; // position in the table (PARAM_LIST)
  count: number; // table size
}

export function decodeParam(p: Uint8Array): ParamMsg {
  const r = new Reader(p);
  return { id: r.u16(), value: r.f32(), flags: r.u8_(), index: r.u16(), count: r.u16() };
}

export function encodeParam(m: ParamMsg): Uint8Array {
  return new Writer().u16(m.id).f32(m.value).u8(m.flags).u16(m.index).u16(m.count).done();
}

/** 0x0D CALIB – 6 + data B, 10 Hz while a calibration is active, once on every state change. */
export interface CalibMsg {
  type: number; // CalType
  state: number; // CalState
  step: number;
  progress: number; // 0..100
  quality: number; // 0..100, 0xFF = n/a
  reason: number; // Reason, when FAILED
  data: CalibData;
}

export type CalibData =
  | { kind: "accel"; doneMask: number; current: number; stillPct: number; offsetMg: number[]; scale: number[]; fitErrPct: number }
  | { kind: "level"; rollDeg: number; pitchDeg: number; remainingMs: number; trimRollDeg: number; trimPitchDeg: number }
  | { kind: "gyro"; remainingMs: number; biasDps: number[]; noiseDps: number; tempC: number }
  | { kind: "compass"; coverage: boolean[]; interfFc: number; interfGps: number | null; qualityFc: number | null; qualityGps: number | null }
  | { kind: "flow"; tofMm: number; heightMm: number; flowQ: number; matchPct: number; axesDone: number; scaleX: number; scaleY: number; tofOffMm: number }
  | { kind: "battery"; vbatV: number; scaleV: number; scaleI: number; usedMah: number }
  | { kind: "motors"; motor: number; spinning: boolean; revMask: number; okMask: number }
  | { kind: "sticks"; centre: number[]; min: number[]; max: number[]; auxPos: number[] }
  | { kind: "none" };

export function decodeCalib(p: Uint8Array): CalibMsg {
  const r = new Reader(p);
  const type = r.u8_(), state = r.u8_(), step = r.u8_(), progress = r.u8_(), quality = r.u8_(), reason = r.u8_();
  let data: CalibData = { kind: "none" };
  const n = (f: () => number, k: number) => Array.from({ length: k }, f);
  switch (type) {
    case CalType.ACCEL:
      data = { kind: "accel", doneMask: r.u8_(), current: r.u8_(), stillPct: r.u8_(), offsetMg: n(() => r.i16(), 3), scale: n(() => r.u16() / 1e4, 3), fitErrPct: r.u16() / 100 };
      break;
    case CalType.LEVEL:
      data = { kind: "level", rollDeg: r.i16() / 100, pitchDeg: r.i16() / 100, remainingMs: r.u16(), trimRollDeg: r.i16() / 100, trimPitchDeg: r.i16() / 100 };
      break;
    case CalType.GYRO:
      data = { kind: "gyro", remainingMs: r.u16(), biasDps: n(() => r.i16() / 1000, 3), noiseDps: r.u16() / 1000, tempC: r.i16() / 100 };
      break;
    case CalType.COMPASS: {
      const cov = r.bytes(5);
      const coverage = Array.from({ length: 36 }, (_, i) => ((cov[i >> 3] ?? 0) >> (i & 7)) & 1 ? true : false);
      const iF = r.u8_(), iG = r.u8_(), qF = r.u8_(), qG = r.u8_();
      data = { kind: "compass", coverage, interfFc: iF, interfGps: iG === 0xff ? null : iG, qualityFc: qF === 0xff ? null : qF, qualityGps: qG === 0xff ? null : qG };
      break;
    }
    case CalType.FLOW_TOF:
      data = { kind: "flow", tofMm: r.u16(), heightMm: r.u16(), flowQ: r.u8_(), matchPct: r.u8_(), axesDone: r.u8_(), scaleX: r.u16() / 1e4, scaleY: r.u16() / 1e4, tofOffMm: r.i16() };
      break;
    case CalType.BATTERY:
      data = { kind: "battery", vbatV: r.u16() / 1000, scaleV: r.u16() / 10, scaleI: r.u16() / 1e4, usedMah: r.u16() };
      break;
    case CalType.MOTORS:
      data = { kind: "motors", motor: r.u8_(), spinning: r.u8_() === 1, revMask: r.u8_(), okMask: r.u8_() };
      break;
    case CalType.STICKS:
      data = { kind: "sticks", centre: n(() => r.u16(), 4), min: n(() => r.u16(), 4), max: n(() => r.u16(), 4), auxPos: n(() => r.u8_(), 4) };
      break;
  }
  return { type, state, step, progress, quality, reason, data };
}

export function encodeCalib(m: CalibMsg): Uint8Array {
  const w = new Writer().u8(m.type).u8(m.state).u8(m.step).u8(m.progress).u8(m.quality).u8(m.reason);
  const d = m.data;
  switch (d.kind) {
    case "accel":
      w.u8(d.doneMask).u8(d.current).u8(d.stillPct);
      d.offsetMg.forEach((v) => w.i16(v));
      d.scale.forEach((v) => w.u16(v * 1e4));
      w.u16(d.fitErrPct * 100);
      break;
    case "level":
      w.i16(d.rollDeg * 100).i16(d.pitchDeg * 100).u16(d.remainingMs).i16(d.trimRollDeg * 100).i16(d.trimPitchDeg * 100);
      break;
    case "gyro":
      w.u16(d.remainingMs);
      d.biasDps.forEach((v) => w.i16(v * 1000));
      w.u16(d.noiseDps * 1000).i16(d.tempC * 100);
      break;
    case "compass": {
      const cov = [0, 0, 0, 0, 0];
      d.coverage.forEach((on, i) => { if (on) cov[i >> 3] |= 1 << (i & 7); });
      w.raw(cov).u8(d.interfFc).u8(d.interfGps ?? 0xff).u8(d.qualityFc ?? 0xff).u8(d.qualityGps ?? 0xff);
      break;
    }
    case "flow":
      w.u16(d.tofMm).u16(d.heightMm).u8(d.flowQ).u8(d.matchPct).u8(d.axesDone).u16(d.scaleX * 1e4).u16(d.scaleY * 1e4).i16(d.tofOffMm);
      break;
    case "battery":
      w.u16(d.vbatV * 1000).u16(d.scaleV * 10).u16(d.scaleI * 1e4).u16(d.usedMah);
      break;
    case "motors":
      w.u8(d.motor).u8(d.spinning ? 1 : 0).u8(d.revMask).u8(d.okMask);
      break;
    case "sticks":
      [...d.centre, ...d.min, ...d.max].forEach((v) => w.u16(v));
      d.auxPos.forEach((v) => w.u8(v));
      break;
  }
  return w.done();
}

/** 0x0E ACK – 4 B, answer to every command with a sequence number (not RC/HEARTBEAT). */
export interface AckMsg {
  op: number;
  seq: number;
  result: number; // AckResult
  reason: number; // Reason
}

export function decodeAck(p: Uint8Array): AckMsg {
  const r = new Reader(p);
  return { op: r.u8_(), seq: r.u8_(), result: r.u8_(), reason: r.u8_() };
}

export function encodeAck(m: AckMsg): Uint8Array {
  return new Writer().u8(m.op).u8(m.seq).u8(m.result).u8(m.reason).done();
}

/** 0x0F CHECKS – 2 + 4·n B, 2 Hz disarmed, 0.5 Hz armed. */
export interface CheckItem {
  id: number; // CheckId
  state: number; // CheckState
  value: number; // i16, meaning per id (see HANDOVER_FW.md)
}
export interface ChecksMsg {
  ready: boolean;
  items: CheckItem[];
}

export function decodeChecks(p: Uint8Array): ChecksMsg {
  const r = new Reader(p);
  const ready = r.u8_() === 1, n = r.u8_();
  const items: CheckItem[] = [];
  for (let i = 0; i < n && r.has(4); i++) items.push({ id: r.u8_(), state: r.u8_(), value: r.i16() });
  return { ready, items };
}

export function encodeChecks(m: ChecksMsg): Uint8Array {
  const w = new Writer().u8(m.ready ? 1 : 0).u8(m.items.length);
  m.items.forEach((i) => w.u8(i.id).u8(i.state).i16(i.value));
  return w.done();
}

/** 0x10 LOG_ENTRY – 15 B, one per log, answer to LOG_LIST (then the ACK). */
export interface LogEntryMsg {
  id: number;
  index: number;
  count: number;
  /** its beginning was overwritten by newer logs */
  truncated: boolean;
  /** being written (closed by the first log command) */
  active: boolean;
  sectors: number;
  /** ms after the drone's power-on */
  t0Ms: number;
  bytes: number;
}

export function decodeLogEntry(p: Uint8Array): LogEntryMsg {
  const r = new Reader(p);
  const id = r.u16(), index = r.u8_(), count = r.u8_(), flags = r.u8_(), sectors = r.u16(), t0Ms = r.u32(), bytes = r.u32();
  return { id, index, count, truncated: !!(flags & 1), active: !!(flags & 2), sectors, t0Ms, bytes };
}

export function encodeLogEntry(m: LogEntryMsg): Uint8Array {
  return new Writer().u16(m.id).u8(m.index).u8(m.count).u8((m.truncated ? 1 : 0) | (m.active ? 2 : 0))
    .u16(m.sectors).u32(m.t0Ms).u32(m.bytes).done();
}

/** 0x11 LOG_DATA – 6 + n B (n ≤ 240), answer to LOG_READ (then the ACK). */
export interface LogDataMsg {
  id: number;
  offset: number;
  data: Uint8Array;
}
export const LOG_DATA_MAX = 240;

export function decodeLogData(p: Uint8Array): LogDataMsg {
  const r = new Reader(p);
  const id = r.u16(), offset = r.u32();
  return { id, offset, data: p.slice(6) };
}

export function encodeLogData(m: LogDataMsg): Uint8Array {
  return new Writer().u16(m.id).u32(m.offset).raw(m.data).done();
}

export type Decoded =
  | { type: typeof FT.STATE; msg: StateMsg }
  | { type: typeof FT.STATUS; msg: StatusMsg }
  | { type: typeof FT.INIT; msg: Uint8Array }
  | { type: typeof FT.CAPS; msg: CapsMsg }
  | { type: typeof FT.FLIGHT; msg: FlightMsg }
  | { type: typeof FT.POWER; msg: PowerMsg }
  | { type: typeof FT.NAV; msg: NavMsg }
  | { type: typeof FT.MOTORS; msg: MotorsMsg }
  | { type: typeof FT.SYSTEM; msg: SystemMsg }
  | { type: typeof FT.EVENT; msg: EventMsg }
  | { type: typeof FT.PARAM; msg: ParamMsg }
  | { type: typeof FT.CALIB; msg: CalibMsg }
  | { type: typeof FT.ACK; msg: AckMsg }
  | { type: typeof FT.CHECKS; msg: ChecksMsg }
  | { type: typeof FT.LOG_ENTRY; msg: LogEntryMsg }
  | { type: typeof FT.LOG_DATA; msg: LogDataMsg };

/** Decodes a frame; unknown types (and REPLAY) return null and are ignored. */
export function decodeFrame(type: number, p: Uint8Array): Decoded | null {
  switch (type) {
    case FT.STATE: return { type, msg: decodeState(p) };
    case FT.STATUS: return { type, msg: decodeStatus(p) };
    case FT.INIT: return { type, msg: p };
    case FT.CAPS: return { type, msg: decodeCaps(p) };
    case FT.FLIGHT: return { type, msg: decodeFlight(p) };
    case FT.POWER: return { type, msg: decodePower(p) };
    case FT.NAV: return { type, msg: decodeNav(p) };
    case FT.MOTORS: return { type, msg: decodeMotors(p) };
    case FT.SYSTEM: return { type, msg: decodeSystem(p) };
    case FT.EVENT: return { type, msg: decodeEvent(p) };
    case FT.PARAM: return { type, msg: decodeParam(p) };
    case FT.CALIB: return { type, msg: decodeCalib(p) };
    case FT.ACK: return { type, msg: decodeAck(p) };
    case FT.CHECKS: return { type, msg: decodeChecks(p) };
    case FT.LOG_ENTRY: return { type, msg: decodeLogEntry(p) };
    case FT.LOG_DATA: return { type, msg: decodeLogData(p) };
    default: return null;
  }
}

/** Quaternion [w x y z] body→world → roll, pitch, yaw in degrees (ZYX, display only). */
export function quatToEuler(q: number[]): { roll: number; pitch: number; yaw: number } {
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const s = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  const pitch = Math.asin(s);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const d = 180 / Math.PI;
  return { roll: roll * d, pitch: pitch * d, yaw: yaw * d };
}

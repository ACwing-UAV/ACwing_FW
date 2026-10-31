/**
 * Flight log, on-flash format 1 (firmware `00 - FC_FW/App/log/log_format.h`):
 * decoder (the app's viewer), encoder (the simulator) and CSV export.
 *
 * A downloaded log is `sectors × 4096` bytes. Every sector starts with a
 * 20-byte header, then records `[type][len][payload][crc8]` until the end of
 * the sector or the first 0xFF byte. Same rules as the Python `decode_log()`
 * and MATLAB `acwingLogDecode.m`:
 *  - a bad header → the sector is skipped;
 *  - a bad record CRC → the rest of the sector is skipped (one cut record after a power loss);
 *  - an unknown type, or a known type with another length → skipped and counted.
 */
import { crc16 } from "../protocol/frame";

export const LOG_MAGIC = 0x474c4341; // "ACLG"
export const LOG_FORMAT = 1;
export const SECTOR = 4096;
export const HDR_SIZE = 20;

export const LR = { INFO: 0x01, ATT: 0x10, VERT: 0x11, FLIGHT: 0x12, EVENT: 0x13, EST: 0x14, POWER: 0x20, DROP: 0x7f } as const;

/** CRC-8/SMBUS (poly 0x07, init 0), as `log_crc8()`. */
export function crc8(d: Uint8Array, start = 0, end = d.length) {
  let c = 0;
  for (let i = start; i < end; i++) {
    c ^= d[i];
    for (let k = 0; k < 8; k++) c = c & 0x80 ? ((c << 1) ^ 0x07) & 0xff : (c << 1) & 0xff;
  }
  return c;
}

/* ------------------------------------------------------------------ */
/* Record layouts                                                       */
/* ------------------------------------------------------------------ */

type Kind = "u8" | "i8" | "u16" | "i16" | "u32" | "i32" | "c16";
const SIZE: Record<Kind, number> = { u8: 1, i8: 1, u16: 2, i16: 2, u32: 4, i32: 4, c16: 16 };

/** Raw record fields, in wire order (names as in log_format.h). */
export const RECORDS: Record<number, { name: string; fields: [string, Kind][] }> = {
  [LR.INFO]: { name: "info", fields: [["fw0", "u8"], ["fw1", "u8"], ["fw2", "u8"], ["hw_rev", "u8"], ["format", "u8"], ["t0_ms", "u32"], ["sensors", "u16"], ["name", "c16"]] },
  [LR.ATT]: { name: "att", fields: [["t_ms", "u32"], ["qw", "i16"], ["qx", "i16"], ["qy", "i16"], ["qz", "i16"], ["gx", "i16"], ["gy", "i16"], ["gz", "i16"], ["ax", "i16"], ["ay", "i16"], ["az", "i16"]] },
  [LR.VERT]: { name: "vert", fields: [["t_ms", "u32"], ["h_mm", "i32"], ["vz_mms", "i16"], ["h_sp_mm", "i32"], ["vz_sp_mms", "i16"], ["thrust", "u8"], ["tof_mm", "u16"], ["flow_q", "u8"], ["baro_pa", "u32"], ["flags", "u8"]] },
  [LR.FLIGHT]: { name: "flight", fields: [["t_ms", "u32"], ["armed", "u8"], ["mode", "u8"], ["autom", "u8"], ["failsafe", "u8"], ["flags", "u8"], ["thr", "i8"], ["yaw", "i8"], ["pitch", "i8"], ["roll", "i8"], ["silence_ms", "u16"]] },
  [LR.EVENT]: { name: "event", fields: [["t_ms", "u32"], ["level", "u8"], ["code", "u16"], ["arg", "i32"]] },
  [LR.EST]: { name: "est", fields: [["t_ms", "u32"], ["bx", "i16"], ["by", "i16"], ["bz", "i16"], ["att_unc_cdeg", "u16"], ["vib_mg", "u16"], ["loop_hz", "u16"], ["jitter_us", "u16"], ["cpu_pct", "u8"], ["mag_interf_pct", "u8"]] },
  [LR.POWER]: { name: "power", fields: [["t_ms", "u32"], ["vbat_mv", "u16"], ["current_ca", "i16"], ["used_mah", "u16"], ["cell_mv", "u16"], ["cell_rest_mv", "u16"], ["pct", "u8"], ["level", "u8"], ["r_cell_mohm", "u16"], ["cells", "u8"]] },
  [LR.DROP]: { name: "drop", fields: [["t_ms", "u32"], ["lost", "u16"], ["source", "u8"]] },
};
export const RECORD_SIZE: Record<number, number> = Object.fromEntries(
  Object.entries(RECORDS).map(([t, r]) => [t, r.fields.reduce((n, [, k]) => n + SIZE[k], 0)]),
);

/** Event names as the PC tools write them in the CSV (`EVENT_NAMES` in flightlog.py). */
export const EVENT_NAMES: Record<number, string> = {
  1: "ARMED", 2: "DISARMED", 3: "TAKEOFF_STARTED", 4: "LANDED", 5: "MODE_CHANGED", 6: "RTH_STARTED", 7: "HOME_SET", 8: "TAKEOFF_ABORTED",
  20: "FAILSAFE_APP_LINK", 21: "FAILSAFE_RADIO", 22: "FAILSAFE_BATTERY", 23: "GEOFENCE_BREACH", 24: "GPS_LOST", 25: "FLOW_LOST",
  30: "BATTERY_LOW", 31: "BATTERY_CRITICAL", 32: "BATTERY_EMERGENCY", 33: "BATTERY_MONITOR_LOST", 40: "MAG_INTERFERENCE",
  41: "HIGH_VIBRATION", 42: "MOTOR_HOT", 43: "RADIO_OVERRIDE", 50: "CALIB_DONE", 51: "CALIB_FAILED", 60: "PARAMS_SAVED", 70: "MOTOR_KILLED",
};

type Raw = Record<string, number | string>;

function readRecord(dv: DataView, off: number, fields: [string, Kind][]): Raw {
  const o: Raw = {};
  for (const [k, kind] of fields) {
    switch (kind) {
      case "u8": o[k] = dv.getUint8(off); break;
      case "i8": o[k] = dv.getInt8(off); break;
      case "u16": o[k] = dv.getUint16(off, true); break;
      case "i16": o[k] = dv.getInt16(off, true); break;
      case "u32": o[k] = dv.getUint32(off, true); break;
      case "i32": o[k] = dv.getInt32(off, true); break;
      case "c16": {
        const b = new Uint8Array(dv.buffer, dv.byteOffset + off, 16);
        const z = b.indexOf(0);
        o[k] = new TextDecoder().decode(z < 0 ? b : b.subarray(0, z));
        break;
      }
    }
    off += SIZE[kind];
  }
  return o;
}

/** Encodes one record payload from raw field values (missing fields = 0). */
export function encodeRecord(type: number, v: Raw): Uint8Array {
  const def = RECORDS[type];
  const out = new Uint8Array(RECORD_SIZE[type]);
  const dv = new DataView(out.buffer);
  let off = 0;
  for (const [k, kind] of def.fields) {
    const x = v[k] ?? 0;
    const n = typeof x === "number" ? Math.round(x) : 0;
    switch (kind) {
      case "u8": dv.setUint8(off, n & 0xff); break;
      case "i8": dv.setInt8(off, Math.max(-128, Math.min(127, n))); break;
      case "u16": dv.setUint16(off, n & 0xffff, true); break;
      case "i16": dv.setInt16(off, Math.max(-32768, Math.min(32767, n)), true); break;
      case "u32": dv.setUint32(off, n >>> 0, true); break;
      case "i32": dv.setInt32(off, n | 0, true); break;
      case "c16": out.set(new TextEncoder().encode(String(x)).subarray(0, 16), off); break;
    }
    off += SIZE[kind];
  }
  return out;
}

/* ------------------------------------------------------------------ */
/* Decoder                                                              */
/* ------------------------------------------------------------------ */

export interface LogInfo { fw: string; hwRev: string; format: number; t0Ms: number; sensors: number; name: string }

type Col = number[];
export interface DecodedLog {
  sectors: number;
  /** sectors with a bad header + sectors cut by a bad record */
  bad: number;
  /** records of an unknown type (or unexpected length), skipped */
  unknown: number;
  info: LogInfo | null;
  /** time in s after the drone's power-on (as the PC tools) */
  att: { n: number; t: Col; q: [Col, Col, Col, Col]; gyro: [Col, Col, Col]; acc: [Col, Col, Col] };
  vert: { n: number; t: Col; h: Col; vz: Col; hSp: Col; vzSp: Col; thrust: Col; tof: Col; flowQ: Col; baroPa: Col; flags: Col };
  flight: { n: number; t: Col; armed: Col; mode: Col; autom: Col; failsafe: Col; flags: Col; silenceMs: Col; sticks: [Col, Col, Col, Col] };
  event: { n: number; t: Col; level: Col; code: Col; arg: Col; name: string[] };
  est: { n: number; t: Col; gyroBias: [Col, Col, Col]; attUncDeg: Col; vibMg: Col; loopHz: Col; jitterUs: Col; cpuPct: Col; magInterfPct: Col };
  power: { n: number; t: Col; vbat: Col; current: Col; usedMah: Col; cell: Col; cellRest: Col; pct: Col; level: Col; rCell: Col; cells: Col; present: Col };
  drop: { n: number; t: Col; lost: Col; source: Col };
}

export function decodeLog(data: Uint8Array): DecodedLog {
  const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const rows: Record<number, Raw[]> = Object.fromEntries(Object.keys(RECORDS).map((t) => [t, []]));
  let bad = 0, unknown = 0, sectors = 0;
  for (let s = 0; s + SECTOR <= data.length; s += SECTOR) {
    const magic = dv.getUint32(s, true), format = dv.getUint8(s + 16), crc = dv.getUint16(s + 18, true);
    if (magic !== LOG_MAGIC || crc !== crc16(data, s, s + 18) || format !== LOG_FORMAT) {
      bad++;
      continue;
    }
    sectors++;
    let i = s + HDR_SIZE;
    const end = s + SECTOR;
    while (i + 3 <= end && data[i] !== 0xff) {
      const t = data[i], n = data[i + 1];
      if (i + 3 + n > end || crc8(data, i, i + 2 + n) !== data[i + 2 + n]) {
        bad++; // cut record (power loss): rest of the sector skipped
        break;
      }
      if (RECORDS[t] && RECORD_SIZE[t] === n) rows[t].push(readRecord(dv, i + 2, RECORDS[t].fields));
      else unknown++;
      i += 3 + n;
    }
  }

  const col = (r: Raw[], k: string) => r.map((x) => x[k] as number);
  const tOf = (r: Raw[]) => r.map((x) => (x.t_ms as number) * 1e-3);
  const info0 = rows[LR.INFO][0];
  const a = rows[LR.ATT], v = rows[LR.VERT], f = rows[LR.FLIGHT], e = rows[LR.EVENT], es = rows[LR.EST], p = rows[LR.POWER], d = rows[LR.DROP];
  const na = (r: Raw[], k: string, nil: number, scale = 1) => r.map((x) => ((x[k] as number) === nil ? NaN : (x[k] as number) * scale));
  return {
    sectors, bad, unknown,
    info: info0 ? {
      fw: `${info0.fw0}.${info0.fw1}.${info0.fw2}`, hwRev: String.fromCharCode(info0.hw_rev as number), format: info0.format as number,
      t0Ms: info0.t0_ms as number, sensors: info0.sensors as number, name: info0.name as string,
    } : null,
    att: {
      n: a.length, t: tOf(a),
      q: [a.map((x) => (x.qw as number) / 32767.0), a.map((x) => (x.qx as number) / 32767.0), a.map((x) => (x.qy as number) / 32767.0), a.map((x) => (x.qz as number) / 32767.0)],
      gyro: [a.map((x) => (x.gx as number) * 1e-3), a.map((x) => (x.gy as number) * 1e-3), a.map((x) => (x.gz as number) * 1e-3)],
      acc: [a.map((x) => (x.ax as number) * 1e-3), a.map((x) => (x.ay as number) * 1e-3), a.map((x) => (x.az as number) * 1e-3)],
    },
    vert: {
      n: v.length, t: tOf(v), h: v.map((x) => (x.h_mm as number) * 1e-3), vz: v.map((x) => (x.vz_mms as number) * 1e-3),
      hSp: v.map((x) => (x.h_sp_mm as number) * 1e-3), vzSp: v.map((x) => (x.vz_sp_mms as number) * 1e-3), thrust: v.map((x) => (x.thrust as number) / 255.0),
      tof: na(v, "tof_mm", 0xffff, 1e-3), flowQ: na(v, "flow_q", 0xff), baroPa: na(v, "baro_pa", 0), flags: col(v, "flags"),
    },
    flight: {
      n: f.length, t: tOf(f), armed: col(f, "armed"), mode: col(f, "mode"), autom: col(f, "autom"), failsafe: col(f, "failsafe"), flags: col(f, "flags"),
      silenceMs: col(f, "silence_ms"),
      sticks: [f.map((x) => (x.thr as number) / 100.0), f.map((x) => (x.yaw as number) / 100.0), f.map((x) => (x.pitch as number) / 100.0), f.map((x) => (x.roll as number) / 100.0)],
    },
    event: { n: e.length, t: tOf(e), level: col(e, "level"), code: col(e, "code"), arg: col(e, "arg"), name: e.map((x) => EVENT_NAMES[x.code as number] ?? `EVENT_${x.code}`) },
    est: {
      n: es.length, t: tOf(es), gyroBias: [es.map((x) => (x.bx as number) * 1e-4), es.map((x) => (x.by as number) * 1e-4), es.map((x) => (x.bz as number) * 1e-4)],
      attUncDeg: es.map((x) => (x.att_unc_cdeg as number) / 100.0), vibMg: col(es, "vib_mg"), loopHz: col(es, "loop_hz"), jitterUs: col(es, "jitter_us"),
      cpuPct: col(es, "cpu_pct"), magInterfPct: col(es, "mag_interf_pct"),
    },
    power: {
      n: p.length, t: tOf(p), vbat: p.map((x) => (x.vbat_mv as number) * 1e-3), current: p.map((x) => (x.current_ca as number) * 1e-2), usedMah: col(p, "used_mah"),
      cell: p.map((x) => (x.cell_mv as number) * 1e-3), cellRest: p.map((x) => (x.cell_rest_mv as number) * 1e-3), pct: col(p, "pct"), level: col(p, "level"),
      rCell: p.map((x) => (x.r_cell_mohm as number) * 1e-3), cells: p.map((x) => (x.cells as number) & 0x7f), present: p.map((x) => ((x.cells as number) & 0x80 ? 1 : 0)),
    },
    drop: { n: d.length, t: tOf(d), lost: col(d, "lost"), source: col(d, "source") },
  };
}

/* ------------------------------------------------------------------ */
/* Summary                                                              */
/* ------------------------------------------------------------------ */

export interface LogSummary {
  durationS: number;
  armedS: number;
  minCell: number | null;
  usedMah: number | null;
  maxCurrent: number | null;
  maxHeight: number | null;
  events: number;
  dropped: number;
}

/** min/max without spreading (a long "always" log has hundreds of thousands of samples) */
function extreme(c: number[], max: boolean, keep: (x: number) => boolean = Number.isFinite): number | null {
  let r: number | null = null;
  for (const x of c) if (keep(x) && (r === null || (max ? x > r : x < r))) r = x;
  return r;
}

export function summarize(L: DecodedLog): LogSummary {
  const ts = [L.att.t, L.vert.t, L.flight.t, L.power.t].filter((t) => t.length);
  const t0 = extreme(ts.map((t) => t[0]), false), t1 = extreme(ts.map((t) => t[t.length - 1]), true);
  let armedS = 0;
  for (let i = 1; i < L.flight.n; i++) if (L.flight.armed[i - 1]) armedS += L.flight.t[i] - L.flight.t[i - 1];
  return {
    durationS: t0 !== null && t1 !== null ? t1 - t0 : 0,
    armedS,
    minCell: extreme(L.power.cell, false, (x) => Number.isFinite(x) && x > 0),
    usedMah: L.power.n ? L.power.usedMah[L.power.n - 1] - L.power.usedMah[0] : null,
    maxCurrent: extreme(L.power.current, true),
    maxHeight: extreme(L.vert.h, true),
    events: L.event.n,
    dropped: L.drop.lost.reduce((a, b) => a + b, 0),
  };
}

/* ------------------------------------------------------------------ */
/* CSV, interchangeable with `save_csv()` of the Python tools           */
/* ------------------------------------------------------------------ */

/** Python's `f"{x:.6g}"`. */
export function g6(x: number): string {
  if (Number.isNaN(x)) return "nan";
  if (!Number.isFinite(x)) return x > 0 ? "inf" : "-inf";
  if (x === 0) return Object.is(x, -0) ? "-0" : "0";
  const [m, e] = x.toExponential(5).split("e");
  const exp = parseInt(e, 10);
  const strip = (s: string) => (s.includes(".") ? s.replace(/0+$/, "").replace(/\.$/, "") : s);
  if (exp < -4 || exp >= 6) return `${strip(m)}e${exp < 0 ? "-" : "+"}${String(Math.abs(exp)).padStart(2, "0")}`;
  return strip(x.toFixed(5 - exp));
}

/** Columns of every CSV, in the order of the Python tools. */
function csvColumns(L: DecodedLog): Record<string, [string, number[]][]> {
  const vec = (k: string, v: number[][]) => v.map((c, j) => [`${k}${j}`, c] as [string, number[]]);
  return {
    att: [["t_s", L.att.t], ...vec("q", L.att.q), ...vec("gyro", L.att.gyro), ...vec("acc", L.att.acc)],
    vert: [["t_s", L.vert.t], ["h", L.vert.h], ["vz", L.vert.vz], ["h_sp", L.vert.hSp], ["vz_sp", L.vert.vzSp], ["thrust", L.vert.thrust],
      ["tof", L.vert.tof], ["flow_q", L.vert.flowQ], ["baro_pa", L.vert.baroPa], ["flags", L.vert.flags]],
    flight: [["t_s", L.flight.t], ["armed", L.flight.armed], ["mode", L.flight.mode], ["autom", L.flight.autom], ["failsafe", L.flight.failsafe],
      ["flags", L.flight.flags], ["silence_ms", L.flight.silenceMs], ...vec("sticks", L.flight.sticks)],
    event: [["t_s", L.event.t], ["level", L.event.level], ["code", L.event.code], ["arg", L.event.arg]],
    est: [["t_s", L.est.t], ...vec("gyro_bias", L.est.gyroBias), ["att_unc_deg", L.est.attUncDeg], ["vib_mg", L.est.vibMg], ["loop_hz", L.est.loopHz],
      ["jitter_us", L.est.jitterUs], ["cpu_pct", L.est.cpuPct], ["mag_interf_pct", L.est.magInterfPct]],
    power: [["t_s", L.power.t], ["vbat", L.power.vbat], ["current", L.power.current], ["used_mah", L.power.usedMah], ["cell", L.power.cell],
      ["cell_rest", L.power.cellRest], ["pct", L.power.pct], ["level", L.power.level], ["r_cell", L.power.rCell], ["cells", L.power.cells], ["present", L.power.present]],
    drop: [["t_s", L.drop.t], ["lost", L.drop.lost], ["source", L.drop.source]],
  };
}

/** One CSV per record type that has data: `{ "<stem>_att.csv": "…", … }`. */
export function toCsv(L: DecodedLog, stem: string): Record<string, string> {
  const out: Record<string, string> = {};
  for (const [name, cols] of Object.entries(csvColumns(L))) {
    const n = cols[0][1].length;
    if (!n) continue;
    const ev = name === "event";
    const lines = [[...cols.map(([h]) => h), ...(ev ? ["name"] : [])].join(",")];
    for (let i = 0; i < n; i++) lines.push([...cols.map(([, c]) => g6(c[i])), ...(ev ? [L.event.name[i]] : [])].join(","));
    out[`${stem}_${name}.csv`] = lines.join("\n") + "\n";
  }
  return out;
}

/* ------------------------------------------------------------------ */
/* Encoder (simulator, tests)                                           */
/* ------------------------------------------------------------------ */

/** Writes a log the way the firmware does: sectors with headers, records never across a sector. */
export class LogWriter {
  private sectors: Uint8Array[] = [];
  private pos = SECTOR;

  constructor(readonly logId: number, readonly t0Ms: number, private serial = 1) {}

  get bytes() {
    return this.sectors.length * SECTOR;
  }

  append(type: number, payload: Uint8Array) {
    const n = payload.length;
    if (this.pos + 3 + n > SECTOR) this.newSector();
    const s = this.sectors[this.sectors.length - 1];
    s[this.pos] = type;
    s[this.pos + 1] = n;
    s.set(payload, this.pos + 2);
    s[this.pos + 2 + n] = crc8(s, this.pos, this.pos + 2 + n);
    this.pos += 3 + n;
  }

  record(type: number, v: Raw) {
    this.append(type, encodeRecord(type, v));
  }

  private newSector() {
    const s = new Uint8Array(SECTOR).fill(0xff);
    const dv = new DataView(s.buffer);
    dv.setUint32(0, LOG_MAGIC, true);
    dv.setUint32(4, this.serial++ >>> 0, true);
    dv.setUint16(8, this.logId, true);
    dv.setUint16(10, this.sectors.length, true);
    dv.setUint32(12, this.t0Ms >>> 0, true);
    s[16] = LOG_FORMAT;
    s[17] = 0;
    dv.setUint16(18, crc16(s, 0, 18), true);
    this.sectors.push(s);
    this.pos = HDR_SIZE;
  }

  /** The log as the drone would send it (whole sectors). */
  data(): Uint8Array {
    const out = new Uint8Array(this.bytes);
    this.sectors.forEach((s, i) => out.set(s, i * SECTOR));
    return out;
  }
}

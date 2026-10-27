/**
 * Frame layer: CRC, encoder and stream parser.
 * Same algorithm as tlm_crc16()/tlm_encode() in App/telemetry/telemetry.c and
 * parse_telemetry() in the Python tools.
 */
export const SYNC0 = 0xa5;
export const SYNC1 = 0x5a;
export const OVERHEAD = 6;

/** CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). crc16("123456789") = 0x29B1 */
export function crc16(data: Uint8Array, start = 0, end = data.length): number {
  let crc = 0xffff;
  for (let i = start; i < end; i++) {
    crc ^= data[i] << 8;
    for (let b = 0; b < 8; b++) crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
  }
  return crc;
}

export function encodeFrame(type: number, payload: Uint8Array): Uint8Array {
  if (payload.length > 255) throw new Error("payload longer than 255 bytes");
  const out = new Uint8Array(payload.length + OVERHEAD);
  out[0] = SYNC0;
  out[1] = SYNC1;
  out[2] = type;
  out[3] = payload.length;
  out.set(payload, 4);
  const crc = crc16(out, 2, 4 + payload.length);
  out[4 + payload.length] = crc & 0xff;
  out[5 + payload.length] = crc >> 8;
  return out;
}

export interface Frame {
  type: number;
  payload: Uint8Array;
}

/**
 * Incremental parser for the BLE byte stream: notifications may cut frames
 * anywhere. A wrong CRC shifts by one byte and re-synchronises.
 */
export class StreamParser {
  private buf = new Uint8Array(1024);
  private len = 0;
  crcErrors = 0;
  bytes = 0;

  push(chunk: Uint8Array): Frame[] {
    this.bytes += chunk.length;
    if (this.len + chunk.length > this.buf.length) {
      const nb = new Uint8Array(Math.max(this.buf.length * 2, this.len + chunk.length));
      nb.set(this.buf.subarray(0, this.len));
      this.buf = nb;
    }
    this.buf.set(chunk, this.len);
    this.len += chunk.length;

    const out: Frame[] = [];
    const b = this.buf;
    let i = 0;
    while (i + OVERHEAD <= this.len) {
      if (b[i] !== SYNC0 || b[i + 1] !== SYNC1) {
        i++;
        continue;
      }
      const plen = b[i + 3];
      const flen = plen + OVERHEAD;
      if (i + flen > this.len) break;
      const crc = b[i + 4 + plen] | (b[i + 5 + plen] << 8);
      if (crc16(b, i + 2, i + 4 + plen) === crc) {
        out.push({ type: b[i + 2], payload: b.slice(i + 4, i + 4 + plen) });
        i += flen;
      } else {
        this.crcErrors++;
        i++;
      }
    }
    // keep the unconsumed tail (a lone 0xA5 may start the next frame)
    this.buf.copyWithin(0, i, this.len);
    this.len -= i;
    return out;
  }

  reset() {
    this.len = 0;
  }
}

/** Little-endian reader with bounds that return NaN-free defaults (short frames from older firmware). */
export class Reader {
  private dv: DataView;
  off = 0;
  constructor(private u8: Uint8Array) {
    this.dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  }
  get left() {
    return this.u8.length - this.off;
  }
  has(n: number) {
    return this.off + n <= this.u8.length;
  }
  u8_() {
    return this.has(1) ? this.dv.getUint8(this.off++) : 0;
  }
  i8() {
    if (!this.has(1)) return 0;
    const v = this.dv.getInt8(this.off);
    this.off += 1;
    return v;
  }
  u16() {
    if (!this.has(2)) return 0;
    const v = this.dv.getUint16(this.off, true);
    this.off += 2;
    return v;
  }
  i16() {
    if (!this.has(2)) return 0;
    const v = this.dv.getInt16(this.off, true);
    this.off += 2;
    return v;
  }
  u32() {
    if (!this.has(4)) return 0;
    const v = this.dv.getUint32(this.off, true);
    this.off += 4;
    return v;
  }
  i32() {
    if (!this.has(4)) return 0;
    const v = this.dv.getInt32(this.off, true);
    this.off += 4;
    return v;
  }
  f32() {
    if (!this.has(4)) return 0;
    const v = this.dv.getFloat32(this.off, true);
    this.off += 4;
    return v;
  }
  f32s(n: number) {
    return Array.from({ length: n }, () => this.f32());
  }
  str(n: number) {
    const s = this.u8.subarray(this.off, Math.min(this.off + n, this.u8.length));
    this.off += n;
    const z = s.indexOf(0);
    return new TextDecoder().decode(z >= 0 ? s.subarray(0, z) : s);
  }
  bytes(n: number) {
    const s = this.u8.slice(this.off, Math.min(this.off + n, this.u8.length));
    this.off += n;
    return s;
  }
}

/** Little-endian writer */
export class Writer {
  private b: number[] = [];
  private dv = new DataView(new ArrayBuffer(8));
  u8(v: number) {
    this.b.push(v & 0xff);
    return this;
  }
  i8(v: number) {
    return this.u8(v < 0 ? v + 256 : v);
  }
  u16(v: number) {
    v = Math.round(v);
    this.b.push(v & 0xff, (v >> 8) & 0xff);
    return this;
  }
  i16(v: number) {
    return this.u16(Math.max(-32768, Math.min(32767, Math.round(v))) & 0xffff);
  }
  u32(v: number) {
    this.dv.setUint32(0, Math.round(v) >>> 0, true);
    for (let i = 0; i < 4; i++) this.b.push(this.dv.getUint8(i));
    return this;
  }
  i32(v: number) {
    this.dv.setInt32(0, Math.round(v), true);
    for (let i = 0; i < 4; i++) this.b.push(this.dv.getUint8(i));
    return this;
  }
  f32(v: number) {
    this.dv.setFloat32(0, v, true);
    for (let i = 0; i < 4; i++) this.b.push(this.dv.getUint8(i));
    return this;
  }
  str(s: string, n: number) {
    const e = new TextEncoder().encode(s).subarray(0, n);
    for (let i = 0; i < n; i++) this.b.push(i < e.length ? e[i] : 0);
    return this;
  }
  raw(a: ArrayLike<number>) {
    for (let i = 0; i < a.length; i++) this.b.push(a[i] & 0xff);
    return this;
  }
  get length() {
    return this.b.length;
  }
  done() {
    return Uint8Array.from(this.b);
  }
}

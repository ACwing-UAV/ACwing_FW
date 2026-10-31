import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { crc8, decodeLog, g6, LogWriter, LR, SECTOR, summarize, toCsv } from "./format";

const DIR = join(__dirname, "testdata");
const sample = () => new Uint8Array(readFileSync(join(DIR, "flightlog_sample.bin")));

describe("flight log format 1", () => {
  it("CRC-8/SMBUS check value", () => {
    expect(crc8(new TextEncoder().encode("123456789"))).toBe(0xf4);
  });

  it("decodes the firmware's test vector (HANDOVER_APP §6)", () => {
    const L = decodeLog(sample());
    expect(L.sectors).toBe(6);
    expect(L.bad).toBe(0);
    expect(L.unknown).toBe(0);
    expect(L.info).toMatchObject({ fw: "0.5.0", hwRev: "N", format: 1 });
    expect(L.att.n).toBe(350);
    expect(L.vert.n).toBe(350);
    for (let i = 1; i < L.att.n; i++) expect(L.att.t[i] - L.att.t[i - 1]).toBeCloseTo(0.02, 6);
    expect(L.flight.n).toBe(70);
    expect(L.power.n).toBe(70);
    expect(L.power.vbat.every((v) => Math.abs(v - 3.9) < 1e-9)).toBe(true);
    expect(L.power.cells.every((c) => c === 1)).toBe(true);
    expect(L.power.present.every((p) => p === 1)).toBe(true);
    expect(L.event.name).toEqual(["ARMED", "DISARMED"]);
    expect(L.vert.tof.every((x) => Number.isNaN(x))).toBe(true);
    expect(L.drop.n).toBe(0);
  });

  it("a damaged record skips only the rest of its sector, a damaged header only its sector", () => {
    const d = sample();
    d[SECTOR + 100] ^= 0x55;
    d[3 * SECTOR + 2] ^= 0x01;
    const L = decodeLog(d), L0 = decodeLog(sample());
    expect(L.bad).toBe(2);
    expect(L.sectors).toBe(L0.sectors - 1);
    expect(L.att.n).toBeGreaterThan(0);
    expect(L.att.n).toBeLessThan(L0.att.n);
    for (let i = 1; i < L.att.n; i++) expect(L.att.t[i]).toBeGreaterThan(L.att.t[i - 1]);
  });

  it("unknown record types are skipped and counted", () => {
    const w = new LogWriter(9, 1000);
    w.record(LR.INFO, { fw0: 0, fw1: 9, fw2: 1, hw_rev: 65, format: 1, t0_ms: 1000, name: "X" });
    w.append(0x55, new Uint8Array([1, 2, 3]));
    w.record(LR.EVENT, { t_ms: 1200, level: 0, code: 1, arg: 0 });
    const L = decodeLog(w.data());
    expect(L.unknown).toBe(1);
    expect(L.event.n).toBe(1);
    expect(L.info?.fw).toBe("0.9.1");
  });

  it("CSV export is byte-identical to the Python tools (save_csv)", () => {
    const L = decodeLog(sample());
    const csv = toCsv(L, "flightlog_sample");
    expect(Object.keys(csv).sort()).toEqual(["att", "event", "flight", "power", "vert"].map((n) => `flightlog_sample_${n}.csv`).sort());
    for (const [file, text] of Object.entries(csv)) expect(text).toBe(readFileSync(join(DIR, file), "utf8"));
  });

  it("%.6g formatting", () => {
    expect([0.02, 1, 101000, 1234567, 0.0000123, -0.5, 3.9, 12.004, NaN, 1e21].map(g6)).toEqual(
      ["0.02", "1", "101000", "1.23457e+06", "1.23e-05", "-0.5", "3.9", "12.004", "nan", "1e+21"],
    );
  });

  it("writer → decoder round trip across several sectors", () => {
    const w = new LogWriter(4, 5000);
    w.record(LR.INFO, { fw0: 0, fw1: 5, fw2: 0, hw_rev: 83, format: 1, t0_ms: 5000, sensors: 0x61, name: "ACwing Indoor" });
    for (let i = 0; i < 600; i++) {
      w.record(LR.ATT, { t_ms: 5000 + i * 20, qw: 32767, gx: i % 100, az: 1000 });
      w.record(LR.VERT, { t_ms: 5000 + i * 20, h_mm: i, tof_mm: 0xffff, flow_q: 80, baro_pa: 101300, flags: 1 });
      if (i % 5 === 0) w.record(LR.POWER, { t_ms: 5000 + i * 20, vbat_mv: 4100 - i, cell_mv: 4100 - i, used_mah: i / 10, cells: 0x81, current_ca: 350 });
    }
    w.record(LR.EVENT, { t_ms: 17000, level: 0, code: 2, arg: 0 });
    const L = decodeLog(w.data());
    expect(L.bad).toBe(0);
    expect(L.sectors).toBe(w.bytes / SECTOR);
    expect(L.sectors).toBeGreaterThan(5);
    expect(L.att.n).toBe(600);
    expect(L.vert.h[599]).toBeCloseTo(0.599, 9);
    expect(L.info?.name).toBe("ACwing Indoor");
    const s = summarize(L);
    expect(s.minCell).toBeCloseTo(4.1 - 0.595, 6);
    expect(s.maxCurrent).toBeCloseTo(3.5, 9);
    expect(s.durationS).toBeCloseTo(11.98, 6);
  });
});

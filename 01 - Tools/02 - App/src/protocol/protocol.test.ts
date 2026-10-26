import { describe, expect, it } from "vitest";
import { crc16, encodeFrame, StreamParser } from "./frame";
import * as M from "./messages";
import { Cmd, parseCommand } from "./commands";
import { FT, OP, CalType, BLE } from "./ids";

const near = (a: number, b: number, eps = 1e-6) => expect(Math.abs(a - b)).toBeLessThanOrEqual(eps);

describe("frame layer", () => {
  it("CRC-16/CCITT-FALSE check value", () => {
    expect(crc16(new TextEncoder().encode("123456789"))).toBe(0x29b1);
  });

  it("reassembles frames cut into 20-byte notifications, with garbage and a corrupt frame", () => {
    const good1 = encodeFrame(FT.ACK, M.encodeAck({ op: 0x12, seq: 7, result: 0, reason: 0 }));
    const bad = encodeFrame(FT.ACK, M.encodeAck({ op: 1, seq: 2, result: 3, reason: 4 }));
    bad[bad.length - 1] ^= 0xff;
    const good2 = encodeFrame(FT.POWER, M.encodePower({ vbatV: 3.912, currentA: 4.5, usedMah: 120, cells: 1, cellV: 3.912, pct: 71, remainS: 300, warn: 0 }));
    const stream = Uint8Array.from([0x00, 0xa5, 0x13, ...good1, ...bad, 0xa5, ...good2]);
    const p = new StreamParser();
    const frames = [];
    for (let i = 0; i < stream.length; i += 20) frames.push(...p.push(stream.subarray(i, i + 20)));
    expect(frames.map((f) => f.type)).toEqual([FT.ACK, FT.POWER]);
    expect(p.crcErrors).toBeGreaterThan(0);
    expect(M.decodeAck(frames[0].payload).seq).toBe(7);
  });
});

describe("payload sizes match the handover", () => {
  it("fixed sizes", () => {
    expect(M.encodeCaps({ proto: 2, fw: [0, 5, 0], hwRev: "H", model: 1, sensors: 0xffff, nMotors: 4, frame: 0, cells: 4, name: "ACwing #1", serial: "X" }).length).toBe(39);
    expect(M.encodeFlight({ arm: 1, mode: 3, auto: 0, failsafe: 0, flags: 0, input: 1, flightTimeS: 1, altRelM: 1, vzMs: 0, groundSpeedMs: 0, headingDeg: 0, homeDistM: null, homeBearingDeg: 0, rthEtaS: null }).length).toBe(25);
    expect(M.encodePower({ vbatV: 15.6, currentA: 11.4, usedMah: 812, cells: 4, cellV: 3.9, pct: 78, remainS: null, warn: 0 }).length).toBe(13);
    expect(M.encodeNav({ fix: 3, sats: 14, hdop: 0.8, lat: 40.85, lon: 14.27, altMslM: 61.3, velNed: [0, 0, 0], tofM: null, flowQ: null, baroHpa: 1006.8, windMs: null }).length).toBe(31);
    expect(M.encodeSystem({ lq: 100, rssi: -68, snr: 9, pktRateHz: 250, txPowerMw: 100, cpuPct: 46, loopHz: 960, jitterUs: 6, logPct: 12, imuTempC: 38.4, vibrationG: 0.21, accClip: 0, attUncDeg: 0.6, estFlags: 1, magInterfPct: 18 }).length).toBe(23);
    expect(M.encodeParam({ id: 1, value: 1, flags: 0, index: 0, count: 1 }).length).toBe(11);
    expect(M.encodeAck({ op: 0, seq: 0, result: 0, reason: 0 }).length).toBe(4);
  });
});

describe("round trips", () => {
  it("FLIGHT keeps units and n/a values", () => {
    const m: M.FlightMsg = { arm: 1, mode: 3, auto: 3, failsafe: 1, flags: 5, input: 2, flightTimeS: 402, altRelM: -1.25, vzMs: 0.4, groundSpeedMs: 5.8, headingDeg: 274, homeDistM: 132, homeBearingDeg: 32, rthEtaS: 45 };
    const d = M.decodeFlight(M.encodeFlight(m));
    expect(d).toEqual(m);
    const n = M.decodeFlight(M.encodeFlight({ ...m, homeDistM: null, rthEtaS: null }));
    expect(n.homeDistM).toBeNull();
    expect(n.rthEtaS).toBeNull();
  });

  it("NAV with and without GNSS", () => {
    const m: M.NavMsg = { fix: 0xff, sats: 0, hdop: 0, lat: 0, lon: 0, altMslM: 0, velNed: [0.3, -5.8, -0.4], tofM: 1.42, flowQ: 82, baroHpa: 1006.8, windMs: null };
    const d = M.decodeNav(M.encodeNav(m));
    expect(d.fix).toBe(0xff);
    near(d.tofM!, 1.42, 1e-3);
    expect(d.flowQ).toBe(82);
    expect(d.windMs).toBeNull();
    near(d.velNed[1], -5.8, 1e-9);
  });

  it("MOTORS for brushed motors without telemetry", () => {
    const m = { motors: [{ rpm: null, tempC: null, outPct: 55 }, { rpm: 11240, tempC: 41, outPct: 56 }] };
    expect(M.decodeMotors(M.encodeMotors(m))).toEqual(m);
  });

  it("CALIB compass coverage bits", () => {
    const coverage = Array.from({ length: 36 }, (_, i) => i % 3 !== 0);
    const m: M.CalibMsg = { type: CalType.COMPASS, state: 1, step: 2, progress: 72, quality: 0xff, reason: 0, data: { kind: "compass", coverage, interfFc: 11, interfGps: 4, qualityFc: null, qualityGps: 96 } };
    const d = M.decodeCalib(M.encodeCalib(m));
    expect(d).toEqual(m);
  });

  it("CALIB of every type decodes to its kind", () => {
    const samples: M.CalibData[] = [
      { kind: "accel", doneMask: 3, current: 2, stillPct: 60, offsetMg: [12, -8, 21], scale: [1.002, 0.998, 1.004], fitErrPct: 0.4 },
      { kind: "level", rollDeg: -0.3, pitchDeg: 0.5, remainingMs: 3000, trimRollDeg: -0.3, trimPitchDeg: 0.5 },
      { kind: "gyro", remainingMs: 2000, biasDps: [0.02, 0.01, -0.03], noiseDps: 0.04, tempC: 31.2 },
      { kind: "flow", tofMm: 12, heightMm: 960, flowQ: 84, matchPct: 91, axesDone: 1, scaleX: 1.03, scaleY: 1.01, tofOffMm: 12 },
      { kind: "battery", vbatV: 15.58, scaleV: 110.4, scaleI: 1.04, usedMah: 1240 },
      { kind: "motors", motor: 2, spinning: true, revMask: 0, okMask: 3 },
      { kind: "sticks", centre: [988, 1501, 1498, 1502], min: [988, 990, 989, 987], max: [2012, 1946, 2011, 2013], auxPos: [2, 3, 2, 0] },
    ];
    const types = [CalType.ACCEL, CalType.LEVEL, CalType.GYRO, CalType.FLOW_TOF, CalType.BATTERY, CalType.MOTORS, CalType.STICKS];
    samples.forEach((data, i) => {
      const d = M.decodeCalib(M.encodeCalib({ type: types[i], state: 1, step: 1, progress: 50, quality: 90, reason: 0, data }));
      expect(d.data.kind).toBe(data.kind);
      expect(JSON.stringify(d.data)).toBe(JSON.stringify(data));
    });
  });

  it("STATUS decodes both the 54-byte and 62-byte layouts", () => {
    const s: M.StatusMsg = { uptimeMs: 1, irqCount: 2, busErr: 3, fifoOvr: 4, tlmDrop: 5, uartDrop: 6, procCycMax: 7, nGyro: 8, nAcc: 9, nMag: 10, nMagDup: 11, nShNack: 12, wordsMax: 13, calibCount: 14, imuInit: -2, running: 1, bleDrop: 15, bleMtu: 247, bleState: 3, bleErr: 0 };
    const full = M.encodeStatus(s);
    expect(full.length).toBe(62);
    expect(M.decodeStatus(full)).toEqual(s);
    const short = M.decodeStatus(full.subarray(0, 54));
    expect(short.running).toBe(1);
    expect(short.bleMtu).toBe(0);
  });

  it("EVENT text is truncated to 40 bytes", () => {
    const d = M.decodeEvent(M.encodeEvent({ level: 1, code: 40, arg: 18, text: "x".repeat(60) }));
    expect(d.text.length).toBe(40);
  });
});

describe("commands", () => {
  it("every command fits the 20-byte CMD characteristic", () => {
    const all = [
      Cmd.heartbeat(1), Cmd.rc(1, { throttle: 0, yaw: 0, pitch: 0, roll: 0, buttons: 0 }), Cmd.arm(1), Cmd.disarm(1), Cmd.takeoff(1, 1),
      Cmd.land(1), Cmd.rth(1), Cmd.hold(1), Cmd.kill(1), Cmd.setMode(1, 3), Cmd.paramGet(1, 1), Cmd.paramSet(1, 1, 1.5), Cmd.paramSave(1),
      Cmd.paramDefaults(1), Cmd.paramList(1), Cmd.calStart(1, 4, 0), Cmd.calCapture(1), Cmd.calAbort(1), Cmd.calSave(1), Cmd.calRestore(1, 4),
      Cmd.calSetRef(1, 6, 0, 15.64), Cmd.motorTest(1, 2, 8, 2000), Cmd.motorReverse(1, 3), Cmd.motorRemap(1, [1, 2, 3, 4, 5, 6, 7, 8]),
      Cmd.streamRates(1, { flight: 10, power: 5, nav: 5, motors: 5, system: 2 }), Cmd.request(1, FT.CAPS), Cmd.setName(1, "ACwing #1 with a long name"),
    ];
    all.forEach((c) => expect(c.length).toBeLessThanOrEqual(BLE.cmdMaxLen));
  });

  it("RC and PARAM_SET decode back", () => {
    const p = parseCommand(Cmd.rc(9, { throttle: -1000, yaw: 250, pitch: 1000, roll: -3, buttons: 0 }));
    expect(p && "seq" in p && p.op).toBe(OP.RC);
    if (p && "seq" in p) {
      expect(p.seq).toBe(9);
      expect([p.r.i16(), p.r.i16(), p.r.i16(), p.r.i16()]).toEqual([-1000, 250, 1000, -3]);
    }
    const q = parseCommand(Cmd.paramSet(3, 21, 3.55));
    if (q && "seq" in q) {
      expect(q.r.u16()).toBe(21);
      near(q.r.f32(), 3.55, 1e-6);
    }
  });

  it("legacy commands keep the phase-2 format", () => {
    expect(Array.from(Cmd.sendInit())).toEqual([0x01]);
    expect(Array.from(Cmd.stateDecim(4))).toEqual([0x02, 4]);
  });
});

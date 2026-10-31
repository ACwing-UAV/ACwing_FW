import { describe, expect, it } from "vitest";
import { Drone } from "../link/drone";
import { SimTransport } from "../sim/simTransport";
import { SimUsbTransport } from "../sim/simUsbTransport";
import { SimDrone } from "../sim/simDrone";
import { AckResult, ArmState, Reason } from "../protocol/ids";
import { LogClient, LogError, logErrorKey } from "./client";
import { BleLogPort, UsbLogPort } from "./ports";
import { decodeLog } from "./format";

const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));
async function until(fn: () => boolean, ms = 8000) {
  const t0 = Date.now();
  while (!fn()) {
    if (Date.now() - t0 > ms) throw new Error("timeout");
    await wait(20);
  }
}

async function usb(drone?: SimDrone) {
  const port = new UsbLogPort(new SimUsbTransport(drone));
  await port.open();
  return { port, client: new LogClient(port) };
}

describe("flight log over the simulated USB port (HANDOVER_APP §9)", () => {
  it("lists every log, reads one in chunks byte-exact, erases", async () => {
    const { port, client } = await usb(new SimDrone("indoor"));
    const logs = await client.list();
    expect(logs.length).toBe(2);
    expect(logs.map((l) => l.index)).toEqual([0, 1]);
    const e = logs[1];
    const whole = await client.read(e.id, e.bytes, { chunk: e.bytes });
    let calls = 0;
    const chunked = await client.read(e.id, e.bytes, { chunk: 10_000, onProgress: () => calls++ });
    expect(chunked).toEqual(whole);
    expect(calls).toBeGreaterThan(5);
    const L = decodeLog(whole);
    expect(L.bad).toBe(0);
    expect(L.event.name[0]).toBe("ARMED");
    expect(L.event.name.at(-1)).toBe("DISARMED");
    expect(L.att.n).toBeGreaterThan(2000);
    await client.erase();
    expect(await client.list()).toEqual([]);
    await port.close();
  }, 30000);

  it("refusals: unknown log, armed, other opcodes", async () => {
    const sim = new SimDrone("indoor");
    const { port, client } = await usb(sim);
    const err = (await client.read(99, 4096).catch((e) => e)) as LogError;
    expect(err).toBeInstanceOf(LogError);
    expect([err.result, err.reason]).toEqual([AckResult.INVALID, Reason.OUT_OF_RANGE]);
    expect(logErrorKey(err)).toBe("logs.err.unknownLog");
    (sim as unknown as { armed: boolean }).armed = true;
    const logs = await client.list(); // allowed over USB while armed
    const e2 = await client.read(logs[0].id, logs[0].bytes).catch((e) => e as LogError);
    expect(logErrorKey(e2)).toBe("logs.err.armed");
    expect(await client.erase().catch((e) => logErrorKey(e))).toBe("logs.err.armed");
    await port.close();
  }, 20000);

  it("times out when nothing answers", async () => {
    const silent = { kind: "usb" as const, send: () => 1, listen: () => () => undefined };
    const c = new LogClient(silent);
    c.timeoutMs = 150;
    await expect(c.list()).rejects.toMatchObject({ kind: "timeout" });
  });
});

describe("flight log over BLE through the flight session (last flight only)", () => {
  it("records a flight, lists only the newest log, reads it, refuses older ids and erase", async () => {
    const t = new SimTransport("indoor");
    t.drone.logFramesPerTick.ble = 20; // faster than a phone, to keep the test short
    const d = new Drone();
    await d.connect(t);
    await until(() => !!d.getSnapshot().caps && !!d.getSnapshot().params[70]);
    await d.arm();
    await d.takeoff(1);
    await until(() => (d.getSnapshot().flight?.altRelM ?? 0) > 0.5, 6000);
    await d.land();
    await until(() => d.getSnapshot().flight?.arm === ArmState.DISARMED, 12000);
    await wait(1200); // post-roll

    const client = new LogClient(new BleLogPort(d));
    const logs = await client.list();
    expect(logs.length).toBe(1);
    expect(logs[0].id).toBe(3); // two seeded logs, then this flight
    const bytes = await client.read(logs[0].id, logs[0].bytes);
    const L = decodeLog(bytes);
    expect(L.bad).toBe(0);
    const names = L.event.name;
    expect(names).toContain("ARMED");
    expect(names).toContain("TAKEOFF_STARTED");
    expect(names).toContain("DISARMED");
    expect(Math.max(...L.vert.h)).toBeGreaterThan(0.5);

    expect(await client.read(1, 4096).catch((e) => logErrorKey(e))).toBe("logs.err.notLast");
    expect(await client.erase().catch((e) => logErrorKey(e))).toBe("logs.err.unsupported");
    // the flight controls still work afterwards
    await d.arm();
    await until(() => d.getSnapshot().flight?.arm === ArmState.ARMED);
    await d.disarm();
    await d.disconnect();
  }, 60000);

  it("arming during a download stops it with ARMED", async () => {
    const t = new SimTransport("indoor");
    t.drone.logFramesPerTick.ble = 1;
    const d = new Drone();
    await d.connect(t);
    await until(() => !!d.getSnapshot().caps);
    const client = new LogClient(new BleLogPort(d));
    const [e] = await client.list();
    const p = client.read(e.id, e.bytes, { chunk: e.bytes }).catch((x) => x as LogError);
    await wait(300);
    (t.drone as unknown as { armed: boolean }).armed = true;
    expect(logErrorKey(await p)).toBe("logs.err.armed");
    await d.disconnect();
  }, 20000);

  it("the USB simulator sees the same logs as the BLE simulator", async () => {
    const t = new SimTransport("outdoor");
    const d = new Drone();
    await d.connect(t);
    const { port, client } = await usb();
    expect((port.transport as SimUsbTransport).drone).toBe(t.drone);
    expect((await client.list()).length).toBe(2);
    await port.close();
    await d.disconnect();
  }, 20000);
});

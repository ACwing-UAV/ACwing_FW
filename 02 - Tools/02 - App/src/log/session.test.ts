import { afterEach, describe, expect, it } from "vitest";
import { drone } from "../link/drone";
import { ArmState } from "../protocol/ids";
import { SimTransport } from "../sim/simTransport";
import { BLE_HOLD_MS, logsSession } from "./session";

const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));
async function until(fn: () => boolean, ms = 8000) {
  const t0 = Date.now();
  while (!fn()) {
    if (Date.now() - t0 > ms) throw new Error("timeout");
    await wait(20);
  }
}

async function connect() {
  const t = new SimTransport("indoor");
  await drone.connect(t);
  await until(() => !!drone.getSnapshot().caps && drone.getSnapshot().params[70] !== undefined);
  return t;
}

afterEach(async () => {
  await drone.disconnect();
  logsSession.resetBle();
});

describe("Flight logs session, last flight over BLE (bench #95)", () => {
  it("a new arming clears the last-flight entry listed before", async () => {
    await connect();
    await logsSession.listBle();
    expect(logsSession.get().ble.listed).toBe(true);
    expect(logsSession.get().ble.entry).not.toBeNull();
    await drone.arm();
    await until(() => drone.getSnapshot().flight?.arm === ArmState.ARMED);
    await until(() => !logsSession.get().ble.listed, 1000);   // the drone notifies its listeners on the next tick
    expect(logsSession.get().ble.listed).toBe(false);
    expect(logsSession.get().ble.entry).toBeNull();
    await drone.disarm();
  }, 20000);

  it("#97 logMode 2: without the hold a new log opens 5 s after the list; with it the entry stays", async () => {
    await connect();
    drone.setParam("logMode", 2);
    await until(() => drone.param("logMode") === 2);
    await wait(1500);                                       // recording since the change

    // the defect: Check, then 7 s before Download → a new short log is "the last flight"
    await logsSession.listBle();
    const first = logsSession.get().ble.entry!;
    await wait(7000);
    await logsSession.listBle();
    const later = logsSession.get().ble.entry!;
    expect(later.id).toBeGreaterThan(first.id);

    // the fix: the screen holds the pause; 8 s later the entry is the same log
    const held = later;
    const tm = setInterval(() => void logsSession.holdBle(), BLE_HOLD_MS);
    await wait(8000);
    clearInterval(tm);
    await logsSession.listBle();
    expect(logsSession.get().ble.entry!.id).toBe(held.id);
    expect(logsSession.get().ble.entry!.bytes).toBe(held.bytes);
  }, 40000);

  it("logMode 1: the hold sends nothing", async () => {
    await connect();
    await logsSession.listBle();
    let sent = 0;
    const orig = drone.sendRaw.bind(drone);
    (drone as unknown as { sendRaw: typeof drone.sendRaw }).sendRaw = (...a: Parameters<typeof drone.sendRaw>) => {
      sent++;
      return orig(...a);
    };
    await logsSession.holdBle();
    expect(sent).toBe(0);
    (drone as unknown as { sendRaw: typeof drone.sendRaw }).sendRaw = orig;
  }, 20000);
});

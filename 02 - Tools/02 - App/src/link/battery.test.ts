import { describe, expect, it } from "vitest";
import { Drone } from "./drone";
import { SimTransport } from "../sim/simTransport";
import { ArmState, CalType, CheckId, CheckState, Reason, AckResult, EventCode } from "../protocol/ids";

const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));
async function until(fn: () => boolean, ms = 8000) {
  const t0 = Date.now();
  while (!fn()) {
    if (Date.now() - t0 > ms) throw new Error("timeout");
    await wait(20);
  }
}

describe("battery, firmware 0.5.0 (HANDOVER_APP §2-§3)", () => {
  it("remaining time only with a capacity; EVENT 33 blocks arming and is cleared when POWER returns", async () => {
    const t = new SimTransport("indoor");
    const d = new Drone();
    await d.connect(t);
    await until(() => !!d.getSnapshot().caps && !!d.getSnapshot().params[23]);
    await d.arm();
    await until(() => d.getSnapshot().flight?.arm === ArmState.ARMED && !!d.getSnapshot().power);
    await wait(300);
    expect(d.getSnapshot().power?.remainS).toBeNull(); // battCapacity 0
    await d.setParam("battCapacity", 450);
    await wait(400);
    expect(d.getSnapshot().power?.remainS).not.toBeNull();
    await d.disarm();

    t.drone.battMonitorLost = true;
    await until(() => d.getSnapshot().battMonitorLostAt !== null);
    expect(d.getSnapshot().events.some((e) => e.code === EventCode.BATTERY_MONITOR_LOST)).toBe(true);
    await until(() => d.getSnapshot().checks?.items.find((i) => i.id === CheckId.BATTERY)?.state === CheckState.FAIL);
    expect(d.getSnapshot().checks?.items.find((i) => i.id === CheckId.BATTERY)?.value).toBe(0);
    await expect(d.arm()).rejects.toMatchObject({ reason: Reason.BATTERY_LOW });
    t.drone.battMonitorLost = false;
    await until(() => d.getSnapshot().battMonitorLostAt === null, 6000);
    await d.disconnect();
  }, 30000);

  it("BATTERY wizard: no recorded flight → SET_REF(6,1) refused; out-of-range voltage refused", async () => {
    const t = new SimTransport("indoor");
    (t.drone as unknown as { lastFlightMah: number }).lastFlightMah = 0;
    const d = new Drone();
    await d.connect(t);
    await until(() => !!d.getSnapshot().caps);
    await d.calStart(CalType.BATTERY);
    await until(() => d.getSnapshot().calib?.type === CalType.BATTERY);
    const v = d.getSnapshot().calib!.data.kind === "battery" ? (d.getSnapshot().calib!.data as { vbatV: number }).vbatV : 4;
    await expect(d.calSetRef(CalType.BATTERY, 0, v * 1.5)).rejects.toMatchObject({ result: AckResult.INVALID, reason: Reason.OUT_OF_RANGE });
    await d.calSetRef(CalType.BATTERY, 0, v * 1.02);
    await expect(d.calSetRef(CalType.BATTERY, 1, 400)).rejects.toMatchObject({ result: AckResult.REJECTED, reason: Reason.OUT_OF_RANGE });
    await d.calAbort();
    await d.disconnect();
  }, 20000);
});

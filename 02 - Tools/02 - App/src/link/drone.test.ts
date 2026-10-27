import { describe, expect, it } from "vitest";
import { Drone, CommandError } from "./drone";
import { SimTransport } from "../sim/simTransport";
import { CalType, CalState, Reason, AckResult, ArmState, SENSOR } from "../protocol/ids";

const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));
async function until(fn: () => boolean, ms = 8000) {
  const t0 = Date.now();
  while (!fn()) {
    if (Date.now() - t0 > ms) throw new Error("timeout");
    await wait(20);
  }
}

describe("app session against the simulator (real bytes, 20-byte notifications)", () => {
  it("outdoor: caps, params, arm, takeoff to 1 m, RTH, land", async () => {
    const d = new Drone();
    await d.connect(new SimTransport("outdoor"));
    await until(() => !!d.getSnapshot().caps && !!d.getSnapshot().flight && Object.keys(d.getSnapshot().params).length > 10);
    expect(d.has(SENSOR.GNSS)).toBe(true);
    expect(d.param("takeoffAlt")).toBeCloseTo(1, 5);

    await expect(d.setParam("takeoffAlt", 9)).rejects.toMatchObject({ result: AckResult.INVALID, reason: Reason.OUT_OF_RANGE });
    await expect(d.takeoff(1)).rejects.toMatchObject({ reason: Reason.NOT_ARMED });

    await d.arm();
    await until(() => d.getSnapshot().flight?.arm === ArmState.ARMED);
    await d.takeoff(1);
    await until(() => (d.getSnapshot().flight?.altRelM ?? 0) > 0.9, 6000);
    await d.rth();
    await d.land();
    await until(() => (d.getSnapshot().flight?.altRelM ?? 1) < 0.05, 8000);
    await d.disconnect();
  }, 30000);

  it("indoor: no GPS, RTH rejected, gyro calibration completes", async () => {
    const d = new Drone();
    await d.connect(new SimTransport("indoor"));
    await until(() => !!d.getSnapshot().caps);
    expect(d.hasGps).toBe(false);
    expect(d.getSnapshot().nav?.fix ?? 0xff).toBe(0xff);
    await d.arm();
    await d.takeoff(1);
    const err = await d.rth().catch((e: CommandError) => e);
    expect(err).toBeInstanceOf(CommandError);
    expect((err as CommandError).reason).toBe(Reason.NO_GPS);
    await d.land();
    await until(() => d.getSnapshot().flight?.arm === ArmState.DISARMED, 12000);
    await d.calStart(CalType.GYRO);
    await until(() => d.getSnapshot().calib?.state === CalState.DONE, 8000);
    await d.calSave();
    await d.disconnect();
  }, 40000);
});

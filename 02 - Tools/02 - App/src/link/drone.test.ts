import { describe, expect, it } from "vitest";
import { Drone, CommandError, RECONNECT_MS } from "./drone";
import { SimTransport } from "../sim/simTransport";
import { CalType, CalState, Reason, AckResult, ArmState, SENSOR, AutoAction, BattWarn, EventCode, FS, FlightMode, PARAM_BY_KEY, CheckId, CheckState } from "../protocol/ids";
import { checkValue } from "../screens/ConnectScreen";

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

  it("Discard after a finished calibration leaves the result screen (CAL_ABORT clears the wizard state)", async () => {
    const d = new Drone();
    await d.connect(new SimTransport("indoor"));
    await until(() => !!d.getSnapshot().caps);
    await d.calStart(CalType.GYRO);
    await until(() => d.getSnapshot().calib?.state === CalState.DONE, 8000);
    await d.calAbort();
    // the drone sends no CALIB frame once IDLE: before the fix the DONE frame stayed and the wizard showed the result
    expect(d.getSnapshot().calib).toBeNull();
    await wait(300);
    expect(d.getSnapshot().calib).toBeNull();
    // a second Discard (nothing running): NOT_CALIBRATING, still no stale state, no exception
    await d.calAbort();
    expect(d.getSnapshot().calib).toBeNull();
    await d.disconnect();
  }, 20000);

  it("motors wizard: the test is followed in CALIB, MOTOR_REVERSE says brushed motors turn one way (FIXED_DIRECTION)", async () => {
    const d = new Drone();
    await d.connect(new SimTransport("indoor"));
    await until(() => !!d.getSnapshot().caps);
    await d.calStart(CalType.MOTORS);
    await until(() => d.getSnapshot().calib?.type === CalType.MOTORS);
    await d.motorTest(2);
    await until(() => { const c = d.getSnapshot().calib?.data; return !!c && c.kind === "motors" && (c.okMask & 0x02) !== 0; });
    const e = await d.motorReverse(2).then(() => null, (x: unknown) => x);
    expect(e).toBeInstanceOf(CommandError);
    expect((e as CommandError).result).toBe(AckResult.REJECTED);
    expect((e as CommandError).reason).toBe(Reason.FIXED_DIRECTION);
    await d.calAbort();
    await d.disconnect();
  }, 20000);

  it("link closed and back within RECONNECT_MS: reconnecting, commands offline meanwhile, then a fresh session", async () => {
    const d = new Drone();
    const t = new SimTransport("indoor");
    await d.connect(t);
    await until(() => !!d.getSnapshot().caps);
    const pending = d.calStart(CalType.GYRO).then(() => null, (e: unknown) => e);
    t.outage(2000);                                                     // drone off for 2 s
    expect(d.getSnapshot().link).toBe("reconnecting");
    const e = await pending;
    expect(e).toBeInstanceOf(CommandError);                             // the waiting command is refused at once
    expect((e as CommandError).result).toBe("offline");
    await expect(d.arm()).rejects.toMatchObject({ result: "offline" }); // nothing is sent into a closed link
    await until(() => d.getSnapshot().link === "connected", 4000);
    expect(d.getSnapshot().dropped).toBeNull();
    await until(() => !!d.getSnapshot().flight);                        // telemetry flows again
    await d.disconnect();
  }, 20000);

  it("link closed and not back: the session ends after RECONNECT_MS, old data cleared, `dropped` names the drone; Reconnect works", async () => {
    const d = new Drone();
    const t = new SimTransport("indoor");
    await d.connect(t);
    await until(() => !!d.getSnapshot().caps && !!d.getSnapshot().flight);
    const name = d.getSnapshot().caps!.name;
    const t0 = Date.now();
    t.outage(RECONNECT_MS + 3000);
    await until(() => d.getSnapshot().link === "idle", RECONNECT_MS + 3000);
    const dt = Date.now() - t0;
    expect(dt).toBeGreaterThanOrEqual(RECONNECT_MS - 50);
    expect(dt).toBeLessThan(RECONNECT_MS + 1500);
    const s = d.getSnapshot();
    expect(s.dropped?.name).toBe(name);
    expect(s.caps).toBeNull();                                          // no stale drone left on the screens
    expect(s.flight).toBeNull();
    await wait(3100);                                                   // the drone is back on
    await d.reconnect();
    await until(() => !!d.getSnapshot().caps);
    expect(d.getSnapshot().link).toBe("connected");
    expect(d.getSnapshot().dropped).toBeNull();
    await d.disconnect();
  }, 30000);

  it("Disconnect while reconnecting stops the attempts, no `dropped`", async () => {
    const d = new Drone();
    const t = new SimTransport("indoor");
    await d.connect(t);
    await until(() => !!d.getSnapshot().caps);
    t.outage(RECONNECT_MS * 2);
    await d.disconnect();
    await wait(RECONNECT_MS + 500);
    expect(d.getSnapshot().link).toBe("idle");
    expect(d.getSnapshot().dropped).toBeNull();
  }, 20000);

  it("firmware 0.5.0 extensions: battery params, logMode, battery EMERGENCY locks the landing", async () => {
    const t = new SimTransport("indoor");
    const d = new Drone();
    await d.connect(t);
    await until(() => !!d.getSnapshot().caps && !!d.getSnapshot().params[70]);
    expect(d.param("battCapacity")).toBe(0);
    expect(d.param("battCells")).toBe(0);
    expect(d.param("logMode")).toBe(1);
    await d.setParam("battCapacity", 450);
    await d.setParam("logMode", 2);
    await expect(d.setParam("battCells", 7)).rejects.toMatchObject({ reason: Reason.OUT_OF_RANGE });
    expect(PARAM_BY_KEY.battCapacity.needs).toBe(SENSOR.CURRENT);

    await d.arm();
    await d.takeoff(1);
    await until(() => (d.getSnapshot().flight?.altRelM ?? 0) > 0.5, 6000);
    // drain the simulated battery: below battEmerg (3.3 V/cell) at once
    (t.drone as unknown as { usedMah: number }).usedMah = 449;
    await until(() => d.getSnapshot().power?.warn === BattWarn.EMERGENCY, 4000);
    const codes = d.getSnapshot().events.map((e) => e.code);
    expect(codes).toContain(EventCode.BATTERY_EMERGENCY);
    expect(codes).toContain(EventCode.FAILSAFE_BATTERY);
    const f = d.getSnapshot().flight!;
    expect(f.auto).toBe(AutoAction.LAND);
    expect(f.failsafe & FS.BATTERY).toBeTruthy();
    await expect(d.setMode(FlightMode.ANGLE)).rejects.toMatchObject({ reason: Reason.BATTERY_LOW });
    await expect(d.hold()).rejects.toMatchObject({ reason: Reason.BATTERY_LOW });
    await d.kill();
    await until(() => d.getSnapshot().flight?.arm === ArmState.DISARMED);
    await d.disconnect();
  }, 30000);

  it("CHECKS battery value 0: USB power or monitor stopped", () => {
    const tt = (k: string) => k;
    expect(checkValue({ id: CheckId.BATTERY, state: CheckState.WARN, value: 0 }, tt, 1)).toBe("checks.noBattery");
    expect(checkValue({ id: CheckId.BATTERY, state: CheckState.FAIL, value: 0 }, tt, 1)).toBe("checks.battMonitor");
    expect(checkValue({ id: CheckId.BATTERY, state: CheckState.OK, value: 3910 }, tt, 3)).toBe("3S · 3.91 V/cell");
  });
});

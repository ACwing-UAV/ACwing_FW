import { describe, expect, it } from "vitest";
import { SimDrone } from "./simDrone";
import { Cmd } from "../protocol/commands";
import { AutoAction, FlightMode } from "../protocol/ids";

/** private state of the simulator, read by the tests */
type Peek = { auto: number; armed: boolean; inAir: boolean; alt: number; vz: number; vN: number; vE: number; pitch: number; roll: number; heading: number; mode: number };

/** runs the simulator for `s` seconds with the given sticks (RC at 25 Hz, as the cockpit) */
function fly(d: SimDrone, s: number, rc: { throttle: number; yaw?: number; pitch?: number; roll?: number }) {
  const dt = 0.01;
  for (let k = 0; k < s / dt; k++) {
    if (k % 4 === 0) d.command(Cmd.rc(0, { throttle: rc.throttle, yaw: rc.yaw ?? 0, pitch: rc.pitch ?? 0, roll: rc.roll ?? 0, buttons: 0 }));
    d.tick(dt);
    d.drain();
  }
}

function armIn(mode: number) {
  const d = new SimDrone("indoor");
  d.onSubscribe();
  let seq = 1;
  d.command(Cmd.setMode(seq++, mode));
  fly(d, 0.2, { throttle: -1000 });
  d.command(Cmd.arm(seq++));
  const p = d as unknown as Peek;
  expect(p.mode).toBe(mode);
  expect(p.armed).toBe(true);
  return { d, p };
}

describe("simulator: manual collective in ANGLE and ACRO", () => {
  it("ANGLE: lifts off with the throttle, hovers at centre stick, flies forward, lands and disarms", () => {
    const { d, p } = armIn(FlightMode.ANGLE);
    fly(d, 1, { throttle: -1000 });
    expect(p.inAir).toBe(false);                         // idle on the ground

    fly(d, 1.2, { throttle: 400 });                      // above hover: climbs
    expect(p.inAir).toBe(true);
    expect(p.alt).toBeGreaterThan(0.3);

    const h0 = p.alt;
    fly(d, 2, { throttle: 0 });                          // centre = hover: the climb dies out
    expect(Math.abs(p.vz)).toBeLessThan(0.3);
    expect(p.alt).toBeGreaterThan(h0);

    fly(d, 1, { throttle: 0, pitch: 500 });              // half stick forward: half of maxTilt, nose down
    expect(p.pitch).toBeGreaterThan(10);
    expect(p.pitch).toBeLessThanOrEqual(15.01);
    const vFwd = p.vN * Math.cos(p.heading * Math.PI / 180) + p.vE * Math.sin(p.heading * Math.PI / 180);
    expect(vFwd).toBeGreaterThan(1);

    fly(d, 1, { throttle: 0 });                          // stick released: level again
    expect(Math.abs(p.pitch)).toBeLessThan(2);

    fly(d, 6, { throttle: -1000 });                      // bottom: falls, touches down, auto-disarm (2 s)
    expect(p.inAir).toBe(false);
    expect(p.armed).toBe(false);
  });

  it("ACRO: sticks command a rate, the attitude stays when they are released", () => {
    const { d, p } = armIn(FlightMode.ACRO);
    fly(d, 1.2, { throttle: 400 });
    expect(p.inAir).toBe(true);
    fly(d, 0.05, { throttle: 300, roll: 100 });          // 10 % roll stick: 48 deg/s
    const r1 = p.roll;
    expect(r1).toBeGreaterThan(1);
    fly(d, 0.5, { throttle: 300 });                      // released: no self-levelling in ACRO
    expect(p.roll).toBeCloseTo(r1, 5);
  });

  it("a stale RC on the ground never lifts the drone off (slide-to-arm after the cockpit was left with the stick up)", () => {
    const d = new SimDrone("indoor");
    d.onSubscribe();
    const p = d as unknown as Peek;
    d.command(Cmd.setMode(1, FlightMode.ANGLE));
    fly(d, 0.2, { throttle: 800 });                      // cockpit closed with the throttle high
    const beat = (s: number) => { for (let k = 0; k < s / 0.01; k++) { if (k % 20 === 0) d.command(Cmd.heartbeat(0)); d.tick(0.01); d.drain(); } };
    beat(1);                                             // RC older than 0.5 s: the throttle rule does not apply
    d.command(Cmd.arm(2));
    expect(p.armed).toBe(true);
    beat(2);
    expect(p.inAir).toBe(false);
  });

  it("HOLD in ANGLE is not cancelled by a collective off the centre; a 30 % throttle change cancels it", () => {
    const { d, p } = armIn(FlightMode.ANGLE);
    fly(d, 1.5, { throttle: 300 });
    expect(p.inAir).toBe(true);
    d.command(Cmd.hold(9));
    fly(d, 1, { throttle: 300 });
    expect(p.auto).toBe(AutoAction.HOLD);
    fly(d, 0.3, { throttle: 700 });
    expect(p.auto).toBe(AutoAction.NONE);
  });

  it("after a LAND in ANGLE the motors stay at idle: a stick above the centre does not lift off again", () => {
    const { d, p } = armIn(FlightMode.ANGLE);
    fly(d, 1.5, { throttle: 300 });
    d.command(Cmd.land(9));
    fly(d, 15, { throttle: 300 });                       // stick left above hover during and after the landing
    expect(p.inAir).toBe(false);
    expect(p.alt).toBe(0);
    expect(p.armed).toBe(false);                         // auto-disarm, no second lift-off
  });
});


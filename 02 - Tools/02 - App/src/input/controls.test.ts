import { describe, expect, it } from "vitest";
import { KEY_THR_RATE, ThrottleArbiter, toRc } from "./controls";

const S = { deadband: 0, expo: 0, sensitivity: 70 };
const dt = 0.02;

describe("throttle arbitration (issue #32)", () => {
  it("ANGLE on the ground: W raises the collective although the touch stick rests at the bottom", () => {
    const a = new ThrottleArbiter();
    a.reset(-1);
    let v = a.step(-1, null, 0, false, dt);
    expect(v).toBe(-1);
    for (let i = 0; i < 50; i++) v = a.step(-1, null, 1, false, dt);   // W held 1 s
    expect(a.source).toBe("key");
    expect(v).toBeCloseTo(-1 + KEY_THR_RATE * 1.0, 5);
    // released: the collective stays where it was left (no spring to the centre, no fall to the touch stick)
    const kept = v;
    for (let i = 0; i < 20; i++) v = a.step(-1, null, 0, false, dt);
    expect(v).toBeCloseTo(kept, 5);
    // S lowers it, clamped at the bottom
    for (let i = 0; i < 300; i++) v = a.step(-1, null, -1, false, dt);
    expect(v).toBe(-1);
    // the old merge would have kept the touch stick: largest magnitude, first wins
    expect(toRc([{ throttle: -1, yaw: 0, pitch: 0, roll: 0 }, { throttle: 1, yaw: 0, pitch: 0, roll: 0 }], S).throttle).toBe(-1000);
    expect(toRc([{ throttle: -1, yaw: 0, pitch: 0, roll: 0 }], S, 0.25).throttle).toBe(250);
  });

  it("the keyboard takes over from the value in use (no jump)", () => {
    const a = new ThrottleArbiter();
    a.reset(0);
    a.step(0.3, null, 0, false, dt);
    a.step(0.4, null, 0, false, dt);                // touch moved: touch drives, 0.4
    const v = a.step(0.4, null, 1, false, dt);      // W: from 0.4 up
    expect(v).toBeCloseTo(0.4 + KEY_THR_RATE * dt, 5);
  });

  it("hold modes: W / S are momentary climb / descend, released = centre", () => {
    const a = new ThrottleArbiter();
    a.reset(0);
    expect(a.step(0, null, 1, true, dt)).toBe(1);
    expect(a.step(0, null, -1, true, dt)).toBe(-1);
    expect(a.step(0, null, 0, true, dt)).toBe(0);
  });

  it("gamepad: takes over when its stick moves, gives back to the touch stick when the touch moves", () => {
    const a = new ThrottleArbiter();
    a.reset(-1);
    a.step(-1, 0, 0, false, dt);
    expect(a.step(-1, 0.01, 0, false, dt)).toBe(-1);     // noise at rest: still the touch stick
    expect(a.step(-1, 0.3, 0, false, dt)).toBe(0.3);     // moved: the gamepad drives
    expect(a.source).toBe("pad");
    expect(a.step(-0.5, 0.3, 0, false, dt)).toBe(-0.5);  // touch moved: touch drives again
    expect(a.step(-0.5, null, 0, false, dt)).toBe(-0.5); // gamepad unplugged
  });

  it("mode change in the air (ALT_HOLD -> ANGLE): every source starts from the centre", () => {
    const a = new ThrottleArbiter();
    a.reset(0);
    a.step(0, 0, 0, true, dt);
    a.reset(0);                                       // the screen puts the touch stick at the centre
    expect(a.step(0, 0, 0, false, dt)).toBe(0);
    const v = a.step(0, 0, 1, false, dt);             // W from the centre, not from the bottom
    expect(v).toBeCloseTo(KEY_THR_RATE * dt, 5);
  });

  it("a long frame (tab in the background) does not jump the keyboard collective", () => {
    const a = new ThrottleArbiter();
    a.reset(-1);
    a.step(-1, null, 1, false, 0);
    expect(a.step(-1, null, 1, false, 5)).toBeCloseTo(-1 + KEY_THR_RATE * 0.1, 5);
  });
});

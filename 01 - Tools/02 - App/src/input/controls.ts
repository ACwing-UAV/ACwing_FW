/**
 * Pilot inputs: touch sticks, keyboard and gamepad, merged into one RC
 * command. Stick mode, deadband, expo and sensitivity come from the app settings.
 */
import type { RcInput } from "../protocol/commands";
import type { AppSettings } from "../settings/appSettings";

export interface Axes {
  throttle: number; // -1..1
  yaw: number;
  pitch: number; // + forward
  roll: number; // + right
}

export const ZERO: Axes = { throttle: 0, yaw: 0, pitch: 0, roll: 0 };

/** Physical stick positions (x right, y up, both -1..1) → axes, for Mode 1 or Mode 2. */
export function sticksToAxes(left: { x: number; y: number }, right: { x: number; y: number }, mode: 1 | 2): Axes {
  return mode === 2
    ? { throttle: left.y, yaw: left.x, pitch: right.y, roll: right.x }
    : { throttle: right.y, yaw: left.x, pitch: left.y, roll: right.x };
}

export function shape(v: number, deadbandPct: number, expoPct: number, gain = 1) {
  const db = deadbandPct / 100;
  const a = Math.abs(v);
  if (a <= db) return 0;
  const x = (a - db) / (1 - db);
  const e = expoPct / 100;
  return Math.sign(v) * Math.min(1, ((1 - e) * x + e * x * x * x) * gain);
}

/** Merges sources (largest magnitude wins per axis) and applies the stick feel. Throttle keeps its full range. */
export function toRc(sources: Axes[], s: Pick<AppSettings, "deadband" | "expo" | "sensitivity">): RcInput {
  const pick = (k: keyof Axes) => sources.reduce((best, src) => (Math.abs(src[k]) > Math.abs(best) ? src[k] : best), 0);
  const g = s.sensitivity / 100 / 0.7; // 70 % = nominal gain 1
  return {
    throttle: Math.round(shape(pick("throttle"), s.deadband, 0) * 1000),
    yaw: Math.round(shape(pick("yaw"), s.deadband, s.expo, g) * 1000),
    pitch: Math.round(shape(pick("pitch"), s.deadband, s.expo, g) * 1000),
    roll: Math.round(shape(pick("roll"), s.deadband, s.expo, g) * 1000),
    buttons: 0,
  };
}

/* ---------- keyboard ---------- */

const down = new Set<string>();
let keyHandlers: Partial<Record<string, () => void>> = {};

function onKey(e: KeyboardEvent) {
  const tag = (e.target as HTMLElement)?.tagName;
  if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
  if (e.type === "keydown") {
    if (!down.has(e.code) && keyHandlers[e.code]) keyHandlers[e.code]!();
    down.add(e.code);
    if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "Space"].includes(e.code)) e.preventDefault();
  } else down.delete(e.code);
}

export function attachKeyboard(handlers: Partial<Record<string, () => void>>) {
  keyHandlers = handlers;
  addEventListener("keydown", onKey);
  addEventListener("keyup", onKey);
  const blur = () => down.clear();
  addEventListener("blur", blur);
  return () => {
    removeEventListener("keydown", onKey);
    removeEventListener("keyup", onKey);
    removeEventListener("blur", blur);
    down.clear();
    keyHandlers = {};
  };
}

/** Keyboard state: W/S throttle, A/D yaw, arrows pitch/roll (always Mode-independent). */
export function keyboardAxes(): Axes {
  const k = (c: string) => (down.has(c) ? 1 : 0);
  return {
    throttle: k("KeyW") - k("KeyS"),
    yaw: k("KeyD") - k("KeyA"),
    pitch: k("ArrowUp") - k("ArrowDown"),
    roll: k("ArrowRight") - k("ArrowLeft"),
  };
}

/* ---------- gamepad (standard mapping: axes 0/1 left stick, 2/3 right stick, y down) ---------- */

export function gamepad(): { name: string; left: { x: number; y: number }; right: { x: number; y: number } } | null {
  const pads = typeof navigator !== "undefined" && navigator.getGamepads ? navigator.getGamepads() : [];
  const p = Array.from(pads).find((x) => x && x.connected && x.axes.length >= 4);
  if (!p) return null;
  return { name: p.id.replace(/\(.*\)/, "").trim(), left: { x: p.axes[0], y: -p.axes[1] }, right: { x: p.axes[2], y: -p.axes[3] } };
}

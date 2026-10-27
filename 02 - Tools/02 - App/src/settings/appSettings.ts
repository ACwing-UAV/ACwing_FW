/**
 * Settings that live in the app (this device), not on the drone: language,
 * units, theme, stick feel, voice and alerts. Drone-side settings are
 * parameters (PARAM_GET/SET) and live on the flight controller.
 */
import type { Lang } from "../i18n";

export interface AppSettings {
  lang: Lang;
  units: "metric" | "imperial";
  theme: "dark" | "day";
  stickMode: 1 | 2;
  sensitivity: number; // 10..100 %
  expo: number; // 0..100 %
  deadband: number; // 0..15 %
  haptics: boolean;
  voice: boolean;
  volume: number; // 0..100
  vibrate: boolean;
  callAlt: 0 | 5 | 10;
  callBatt: 0 | 10 | 25;
  callTimer: "off" | "min" | "5";
  warn: Record<"battLow" | "link" | "gps" | "flow" | "fence" | "compass" | "vibration" | "hot", boolean>;
  telemetryRate: 5 | 10 | 20 | 50;
  recordLog: boolean;
  killConfirm: boolean;
  lastLink: "ble" | "sim-outdoor" | "sim-indoor" | null;
}

export const DEFAULTS: AppSettings = {
  lang: "en",
  units: "metric",
  theme: "dark",
  stickMode: 2,
  sensitivity: 70,
  expo: 30,
  deadband: 5,
  haptics: true,
  voice: true,
  volume: 80,
  vibrate: true,
  callAlt: 10,
  callBatt: 10,
  callTimer: "min",
  warn: { battLow: true, link: true, gps: true, flow: true, fence: true, compass: true, vibration: false, hot: true },
  telemetryRate: 20,
  recordLog: true,
  killConfirm: true,
  lastLink: null,
};

const KEY = "acwing.pilot.settings.v1";

function load(): AppSettings {
  try {
    const raw = typeof localStorage !== "undefined" ? localStorage.getItem(KEY) : null;
    if (raw) {
      const v = JSON.parse(raw);
      return { ...DEFAULTS, ...v, warn: { ...DEFAULTS.warn, ...(v.warn ?? {}) } };
    }
  } catch {
    /* private mode, blocked storage: defaults */
  }
  return { ...DEFAULTS };
}

let current = load();
const listeners = new Set<() => void>();

export const appSettings = {
  get: () => current,
  subscribe: (fn: () => void) => {
    listeners.add(fn);
    return () => listeners.delete(fn);
  },
  set(patch: Partial<AppSettings>) {
    current = { ...current, ...patch };
    try {
      localStorage.setItem(KEY, JSON.stringify(current));
    } catch {
      /* not persisted: still applied for this session */
    }
    listeners.forEach((l) => l());
  },
  reset() {
    appSettings.set({ ...DEFAULTS });
  },
};

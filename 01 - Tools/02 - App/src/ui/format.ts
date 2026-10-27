import { appSettings } from "../settings/appSettings";

const imp = () => appSettings.get().units === "imperial";
const fx = (v: number, d: number) => (Number.isFinite(v) ? v.toFixed(d) : "–");

export const fmt = {
  alt: (m: number | null | undefined, d = 1) => (m == null ? "–" : imp() ? `${fx(m * 3.28084, d)} ft` : `${fx(m, d)} m`),
  altNum: (m: number) => (imp() ? m * 3.28084 : m),
  altUnit: () => (imp() ? "ft" : "m"),
  dist: (m: number | null | undefined) => (m == null ? "–" : imp() ? `${Math.round(m * 3.28084)} ft` : `${Math.round(m)} m`),
  speed: (ms: number | null | undefined, d = 1) => (ms == null ? "–" : imp() ? `${fx(ms * 2.23694, d)} mph` : `${fx(ms, d)} m/s`),
  speedNum: (ms: number) => (imp() ? ms * 2.23694 : ms),
  speedUnit: () => (imp() ? "mph" : "m/s"),
  vz: (ms: number | null | undefined) => (ms == null ? "–" : `${ms >= 0 ? "+" : ""}${fmt.speed(ms)}`),
  mmss: (s: number | null | undefined) => {
    if (s == null) return "–";
    const m = Math.floor(s / 60);
    return `${m}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
  },
  deg: (d: number | null | undefined, digits = 0) => (d == null ? "–" : `${fx(d, digits)}°`),
  signed: (v: number, d = 1) => `${v >= 0 ? "+" : "−"}${fx(Math.abs(v), d)}`,
  /** parameter value with its unit, honouring imperial units for lengths and speeds */
  param: (v: number, unit?: string, step = 1) => {
    const digits = step < 0.1 ? 2 : step < 1 ? 1 : 0;
    if (unit === "m") return fmt.alt(v, digits);
    if (unit === "m/s") return fmt.speed(v, digits);
    if (unit === "V") return `${v.toFixed(2)} V`;
    return `${fx(v, digits)}${unit ? (unit === "°" ? "°" : ` ${unit}`) : ""}`;
  },
};

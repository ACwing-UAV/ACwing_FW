/**
 * Flight log on this device: one row every 500 ms while connected (setting
 * "Record flight log"), exported as CSV. The full-rate logs stay on the drone.
 */
import { drone } from "./drone";
import { appSettings } from "../settings/appSettings";

const COLS = [
  "time_iso", "armed", "mode", "auto", "failsafe", "alt_rel_m", "vz_ms", "gs_ms", "heading_deg", "home_dist_m", "roll_deg", "pitch_deg",
  "vbat_v", "current_a", "used_mah", "cell_v", "batt_pct", "gnss_fix", "sats", "lat", "lon", "tof_m", "flow_q", "lq", "cpu_pct", "vib_g",
] as const;

let rows: string[] = [];
let timer: ReturnType<typeof setInterval> | null = null;

function sample() {
  const s = drone.getSnapshot();
  if (s.link === "idle" || !s.flight || !appSettings.get().recordLog) return;
  const f = s.flight, p = s.power, n = s.nav, a = s.attitude, y = s.system;
  const v = [
    new Date().toISOString(), f.arm, f.mode, f.auto, f.failsafe, f.altRelM.toFixed(2), f.vzMs.toFixed(2), f.groundSpeedMs.toFixed(2), f.headingDeg.toFixed(1),
    f.homeDistM ?? "", a?.rollRight.toFixed(1) ?? "", a?.pitchUp.toFixed(1) ?? "", p?.vbatV.toFixed(2) ?? "", p?.currentA.toFixed(2) ?? "", p?.usedMah ?? "",
    p?.cellV.toFixed(3) ?? "", p?.pct ?? "", n?.fix ?? "", n?.sats ?? "", n && n.fix !== 0xff ? n.lat.toFixed(7) : "", n && n.fix !== 0xff ? n.lon.toFixed(7) : "",
    n?.tofM?.toFixed(3) ?? "", n?.flowQ ?? "", y?.lq ?? "", y?.cpuPct ?? "", y?.vibrationG.toFixed(3) ?? "",
  ];
  rows.push(v.join(","));
  if (rows.length > 20000) rows = rows.slice(-20000); // ~2.7 h
}

export const flightLog = {
  start() {
    if (!timer) timer = setInterval(sample, 500);
  },
  get count() {
    return rows.length;
  },
  clear() {
    rows = [];
  },
  csv() {
    return [COLS.join(","), ...rows].join("\n");
  },
  async export(name = "ACwing") {
    const file = `${name.replace(/[^\w-]+/g, "_")}_${new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19)}.csv`;
    const blob = new Blob([flightLog.csv()], { type: "text/csv" });
    const nav = navigator as Navigator & { canShare?: (d: unknown) => boolean };
    const f = typeof File !== "undefined" ? new File([blob], file, { type: "text/csv" }) : null;
    if (f && nav.canShare?.({ files: [f] })) {
      try {
        await navigator.share({ files: [f], title: file });
        return;
      } catch {
        /* cancelled: fall back to a download */
      }
    }
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = file;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  },
};

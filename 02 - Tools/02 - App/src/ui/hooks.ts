import { useEffect, useState, useSyncExternalStore } from "react";
import { drone, type DroneState } from "../link/drone";
import { appSettings } from "../settings/appSettings";

export function useDrone(): DroneState {
  return useSyncExternalStore(drone.subscribe, drone.getSnapshot);
}

export function useSettings() {
  return useSyncExternalStore(appSettings.subscribe, appSettings.get);
}

/** Hash routes: #/connect, #/cockpit, #/telemetry, #/settings/<section>, #/calib/<type>, #/help, #/logs[/last|/view/<key>] */
export function useRoute(): string[] {
  const [hash, setHash] = useState(() => location.hash);
  useEffect(() => {
    const on = () => setHash(location.hash);
    addEventListener("hashchange", on);
    return () => removeEventListener("hashchange", on);
  }, []);
  return hash.replace(/^#\/?/, "").split("/").filter(Boolean);
}

export const go = (path: string) => {
  location.hash = `#/${path}`;
};

/** Window size class: phone (< 700 px wide), wide otherwise; `short` for phones in landscape. */
export function useViewport() {
  const read = () => ({ w: innerWidth, h: innerHeight });
  const [v, setV] = useState(read);
  useEffect(() => {
    const on = () => setV(read());
    addEventListener("resize", on);
    addEventListener("orientationchange", on);
    return () => {
      removeEventListener("resize", on);
      removeEventListener("orientationchange", on);
    };
  }, []);
  return { ...v, phone: v.w < 700 || v.h < 500, wide: v.w >= 900 && v.h >= 500, compactCockpit: v.w < 1100 || v.h < 700 };
}

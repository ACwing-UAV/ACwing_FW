import { Capacitor } from "@capacitor/core";
import { Haptics, ImpactStyle } from "@capacitor/haptics";
import { appSettings } from "../settings/appSettings";

/** Short tick (stick centre / end stop) or a longer buzz (warnings). */
export function haptic(kind: "tick" | "warn" = "tick") {
  const s = appSettings.get();
  if (kind === "tick" && !s.haptics) return;
  if (kind === "warn" && !s.vibrate) return;
  if (Capacitor.isNativePlatform()) {
    if (kind === "tick") Haptics.impact({ style: ImpactStyle.Light }).catch(() => undefined);
    else Haptics.vibrate({ duration: 300 }).catch(() => undefined);
  } else if ("vibrate" in navigator) {
    navigator.vibrate(kind === "tick" ? 10 : [120, 60, 120]);
  }
}

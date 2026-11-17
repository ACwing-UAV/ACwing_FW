/**
 * The link closed and did not come back (Drone.dropped, after RECONNECT_MS): say it once and go back to the connect
 * screen, where "Reconnect to <name>" is waiting. The log screen (it also works over USB without a session) and the
 * help stay where they are. While the drone is being reconnected the screens stay too: the cockpit says so.
 */
import { useEffect, useRef } from "react";
import { useT } from "../i18n";
import { go, useDrone, useRoute } from "../ui/hooks";
import { toast } from "../ui/kit";

/** pages that do not need a drone session: a drop does not leave them */
export const STAY_ON_DROP = new Set(["connect", "logs", "help"]);

export function LinkWatcher() {
  const { t } = useT();
  const s = useDrone();
  const [page] = useRoute();
  const seen = useRef<number | null>(null);
  useEffect(() => {
    const d = s.dropped;
    if (!d || d.at === seen.current) return;
    seen.current = d.at;
    toast(t("link.dropped", { name: d.name }), "error");
    if (!STAY_ON_DROP.has(page || "connect")) go("connect");
  }, [s.dropped, page, t]);
  return null;
}

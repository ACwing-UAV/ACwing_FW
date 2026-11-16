/** The toast for a refused or failed drone command (useCommand in kit.tsx). Pure: the text and the toast level. */
import type { CommandError } from "../link/drone";

type T = (key: string, vars?: Record<string, string | number>) => string;

export function commandErrorToast(e: CommandError, t: T): { text: string; level: "warn" | "error" } {
  if (e.result === "timeout") return { text: t("cmdError.timeout"), level: "error" };
  if (e.result === "offline") return { text: t("cmdError.offline"), level: "warn" };
  // any refusal that carries a reason says it: REJECTED, INVALID, BUSY and UNSUPPORTED too
  // (MOTOR_TEST answers UNSUPPORTED / MOTORS_DISABLED on a firmware without motor outputs, issue #61)
  const k = `reasons.${e.reason}`;
  const why = e.reason ? t(k) : "";
  return why && why !== k ? { text: t("cmdError.rejected", { why }), level: "warn" } : { text: t("cmdError.other"), level: "warn" };
}

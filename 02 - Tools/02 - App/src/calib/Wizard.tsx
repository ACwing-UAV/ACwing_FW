import { useEffect, type ReactNode } from "react";
import { useT } from "../i18n";
import { drone } from "../link/drone";
import { CalState } from "../protocol/ids";
import type { CalibMsg } from "../protocol/messages";
import { useDrone, useViewport } from "../ui/hooks";
import { Icon, Mark } from "../ui/icons";

export const BACK = "#/settings/calibration";

/** Layout shared by every calibration: step list (wide) or progress bars (phone), visual + panel. */
export function WizardShell({ title, steps, current, visual, panel }: {
  title: string; steps: string[]; current: number; visual: ReactNode; panel: ReactNode;
}) {
  const { t } = useT();
  const vp = useViewport();
  const s = useDrone();
  return (
    <div className="page">
      <header className="topbar">
        {vp.wide ? (
          <a className="btn" href={BACK}><Icon name="back" />{t("calib.title")}</a>
        ) : (
          <a className="btn icon" href={BACK} aria-label={t("common.close")}><Icon name="close" /></a>
        )}
        {vp.wide && <Mark />}
        <div style={{ display: "flex", flexDirection: "column", gap: 2, flex: 1, minWidth: 0 }}>
          <h1 style={{ fontSize: vp.wide ? 22 : 19 }}>{title}</h1>
          {!vp.wide && <span className="small muted">{t("common.step", { n: current + 1, total: steps.length })} · {steps[current]}</span>}
        </div>
        {vp.wide && <span className="small muted">{s.caps?.name ?? ""} · {s.transport === "sim" ? t("connect.simulator") : "Bluetooth"} · {s.flight?.arm ? t("cockpit.armed") : t("cockpit.disarmed")}</span>}
      </header>
      {!vp.wide && (
        <div style={{ padding: "12px 14px 0" }}>
          <div className="bars" aria-hidden="true">{steps.map((_, i) => <span key={i} className={i <= current ? "on" : ""} />)}</div>
        </div>
      )}
      <div className="wizard">
        <aside aria-label="Steps">
          <ol>
            {steps.map((n, i) => (
              <li key={n} className={i < current ? "done" : i === current ? "cur" : ""} aria-current={i === current ? "step" : undefined}>
                <span className="n">{i < current ? "✓" : i + 1}</span>{n}
              </li>
            ))}
          </ol>
        </aside>
        <main className="body">
          <div className="visual">{visual}</div>
          <div className="panel">{panel}</div>
        </main>
      </div>
    </div>
  );
}

/** Calibration frame for this type, if the drone is running it. */
export function useCal(type: number): CalibMsg | null {
  const s = useDrone();
  return s.calib && s.calib.type === type ? s.calib : null;
}

/** Leaving the page aborts an unsaved calibration of this type. */
export function useAbortOnLeave(type: number) {
  useEffect(() => () => {
    const c = drone.getSnapshot().calib;
    if (c && c.type === type && c.state !== CalState.SAVED && c.state !== CalState.IDLE) drone.calAbort();
  }, [type]);
}

export function Success({ text }: { text: string }) {
  return (
    <div className="card okline" style={{ flexDirection: "row", alignItems: "center", gap: 12, width: "100%" }}>
      <span style={{ color: "var(--ok)", display: "flex" }}><Icon name="check" size={28} stroke={2.2} /></span>
      <b>{text}</b>
    </div>
  );
}

export function Failed({ reason, onRetry }: { reason: number; onRetry: () => void }) {
  const { t } = useT();
  return (
    <div className="banner crit" style={{ alignItems: "center" }}>
      <span style={{ flex: 1 }}>{t("calib.failed", { why: t(`reasons.${reason}`) || "–" })}</span>
      <button className="btn small" onClick={onRetry}>{t("common.restart")}</button>
    </div>
  );
}

export function Bullets({ items }: { items: string[] }) {
  return (
    <ul style={{ margin: 0, padding: 0, listStyle: "none", display: "flex", flexDirection: "column", gap: 10, lineHeight: 1.45 }}>
      {items.map((x) => (
        <li key={x} style={{ display: "flex", gap: 10, color: "var(--text-2)" }}>
          <span style={{ width: 8, height: 8, marginTop: 7, borderRadius: "50%", background: "var(--accent)", flex: "none" }} />{x}
        </li>
      ))}
    </ul>
  );
}

export function Facts({ rows }: { rows: [string, ReactNode][] }) {
  return (
    <dl className="facts card" style={{ width: "100%" }}>
      {rows.map(([k, v]) => <div key={k}><dt>{k}</dt><dd>{v}</dd></div>)}
    </dl>
  );
}

/** The small side-view drone used by the accelerometer positions. */
export function DroneSide({ transform, size = 160, label }: { transform: string; size?: number; label?: string }) {
  return (
    <svg width={size} height={size * 0.75} viewBox="0 0 80 60" role={label ? "img" : undefined} aria-label={label} aria-hidden={label ? undefined : true}>
      <g transform={transform}>
        <rect x="20" y="27" width="40" height="6" rx="2" fill="var(--text)" />
        <rect x="22" y="21" width="4" height="6" fill="var(--text-2)" />
        <rect x="54" y="21" width="4" height="6" fill="var(--text-2)" />
        <line x1="12" y1="20" x2="36" y2="20" stroke="var(--text-2)" strokeWidth="2" />
        <line x1="44" y1="20" x2="68" y2="20" stroke="var(--text-2)" strokeWidth="2" />
        <path d="M60 26 L68 30 L60 34 Z" fill="var(--accent)" />
      </g>
    </svg>
  );
}

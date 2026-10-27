import { LANGS, useT } from "../../i18n";
import { drone } from "../../link/drone";
import { Model } from "../../protocol/ids";
import { useDrone, useSettings, useViewport } from "../../ui/hooks";
import { Icon, Mark } from "../../ui/icons";
import { Dot } from "../../ui/kit";
import { SaveBar } from "./params";
import { About, Alerts, Calibration, Connection, Controls, General, Safety, calibStatus, calibTypes } from "./sections";

const SECTIONS = [
  { id: "general", icon: "globe", C: General },
  { id: "controls", icon: "sticks", C: Controls },
  { id: "safety", icon: "shield", C: Safety },
  { id: "calibration", icon: "target", C: Calibration },
  { id: "connection", icon: "bt", C: Connection },
  { id: "alerts", icon: "bell", C: Alerts },
  { id: "about", icon: "chip", C: About },
] as const;

export function SettingsScreen({ section }: { section?: string }) {
  const { t } = useT();
  const s = useDrone();
  const st = useSettings();
  const vp = useViewport();
  const cur = SECTIONS.find((x) => x.id === section);
  const calWarn = calibTypes(s).filter((ct) => calibStatus(s, ct) !== "ok").length;
  const back = s.link !== "idle" ? "#/cockpit" : "#/connect";

  const deviceLine = s.caps
    ? `${s.caps.name} · ${s.caps.model === Model.OUTDOOR ? t("about.outdoor") + " (GPS)" : t("about.indoor")} · ${s.link === "lost" ? t("link.lost") : t("link.connected")}`
    : t("link.notConnected");

  const summary: Record<string, string> = {
    general: t("settings.generalSummary", { lang: LANGS.find((l) => l.code === st.lang)!.native, units: t(`general.${st.units}`), theme: st.theme === "dark" ? t("general.dark") : t("general.daylight") }),
    controls: t("settings.controlsSummary", { mode: st.stickMode, alt: drone.param("takeoffAlt").toFixed(1) }),
    safety: t("settings.safetySummary"),
    calibration: calWarn ? t("settings.calibrationSummaryWarn", { n: calWarn }) : t("settings.calibrationSummaryOk"),
    connection: s.transport === "sim" ? t("connect.simulator") : s.transport === "ble" ? "Bluetooth" : t("link.notConnected"),
    alerts: t("settings.alertsSummary", { v: st.voice ? t("common.on") : t("common.off") }),
    about: s.caps ? t("settings.aboutSummary", { fw: s.caps.fw.join(".") }) : "–",
  };

  // ---------- phone: menu, then one section per page ----------
  if (!vp.wide) {
    if (!cur) {
      return (
        <div className="page">
          <header className="topbar">
            <a className="btn icon" href={back} aria-label={t("settings.backToFlight")}><Icon name="back" /></a>
            <h1 style={{ flex: 1 }}>{t("settings.title")}</h1>
            <Mark />
          </header>
          <main className="content">
            <div className="card" style={{ flexDirection: "row", alignItems: "center", gap: 12 }}>
              <Dot state={s.link === "connected" ? "ok" : s.link === "lost" ? "warn" : "na"} />
              <span style={{ display: "flex", flexDirection: "column", gap: 2 }}><b>{s.caps?.name ?? t("link.notConnected")}</b><span className="small muted">{deviceLine}</span></span>
            </div>
            <nav aria-label={t("settings.sections")} className="card tight menu-list">
              {SECTIONS.map((x, i) => (
                <a key={x.id} className={`row${i === 0 ? " first" : ""}`} href={`#/settings/${x.id}`}>
                  <span style={{ color: x.id === "calibration" && calWarn ? "var(--warn)" : "var(--accent)", display: "flex" }}><Icon name={x.icon} size={22} /></span>
                  <span className="grow"><span className="label">{t(`settings.${x.id}`)}</span>
                    <span className="hint" style={x.id === "calibration" && calWarn ? { color: "var(--warn)" } : undefined}>{summary[x.id]}</span></span>
                  <Icon name="chevron" size={16} />
                </a>
              ))}
            </nav>
            <span className="small muted" style={{ textAlign: "center" }}>ACwing Pilot v0.1.0</span>
          </main>
        </div>
      );
    }
    const C = cur.C;
    return (
      <div className="page">
        <header className="topbar">
          <a className="btn icon" href="#/settings" aria-label={t("common.back")}><Icon name="back" /></a>
          <h1 style={{ flex: 1 }}>{t(`settings.${cur.id}`)}</h1>
        </header>
        <main className="content"><C /><SaveBar /></main>
      </div>
    );
  }

  // ---------- tablet / PC: sidebar + section ----------
  const active = cur ?? SECTIONS[0];
  const C = active.C;
  return (
    <div className="page">
      <header className="topbar">
        <a className="btn" href={back}><Icon name="back" />{t("settings.backToFlight")}</a>
        <Mark />
        <h1>{t("settings.title")}</h1>
        <span className="spacer" />
        <span className="small muted">{deviceLine}</span>
      </header>
      <div className="settings">
        <nav className="side" aria-label={t("settings.sections")}>
          {SECTIONS.map((x) => (
            <a key={x.id} href={`#/settings/${x.id}`} aria-current={x.id === active.id ? "page" : undefined}>
              {t(`settings.${x.id}`)}
              {x.id === "calibration" && calWarn > 0 && <span className="dot warn" aria-label={t("calib.recommended")} />}
            </a>
          ))}
        </nav>
        <main className="main"><C /><SaveBar /></main>
      </div>
    </div>
  );
}

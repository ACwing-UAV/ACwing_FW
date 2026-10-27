import { useT } from "../i18n";
import { drone } from "../link/drone";
import { useDrone, useSettings, useViewport } from "../ui/hooks";
import { Icon, Mark } from "../ui/icons";

/** Flight controls reference (#/help): stick layout for the chosen mode, automatic actions and input sources. */
export function HelpScreen() {
  const { t } = useT();
  const s = useDrone();
  const st = useSettings();
  const vp = useViewport();
  const back = s.link !== "idle" ? "#/cockpit" : "#/settings/controls";
  const m1 = st.stickMode === 1;
  const gps = drone.hasGps || !s.caps;
  const fx = (n: number, d = 1) => n.toFixed(d);
  const stick = (title: string, axes: string) => (
    <div className="card" style={{ alignItems: "center", gap: 10 }}>
      <svg width="120" height="120" viewBox="0 0 120 120" aria-hidden="true">
        <circle cx="60" cy="60" r="54" fill="var(--surface-2)" stroke="var(--line-2)" strokeWidth="2" />
        <line x1="60" y1="12" x2="60" y2="108" stroke="var(--line)" /><line x1="12" y1="60" x2="108" y2="60" stroke="var(--line)" />
        <circle cx="60" cy="60" r="18" fill="var(--accent)" fillOpacity="0.85" />
      </svg>
      <b>{title}</b>
      <span className="small muted">{axes}</span>
    </div>
  );
  return (
    <div className="page">
      <header className="topbar">
        <a className={vp.wide ? "btn" : "btn icon"} href={back} aria-label={t("common.back")}><Icon name="back" />{vp.wide && t("common.back")}</a>
        {vp.wide && <Mark />}
        <h1 style={{ flex: 1 }}>{t("controls.helpTitle")}</h1>
      </header>
      <main className="content" style={{ maxWidth: 1000, margin: "0 auto", width: "100%" }}>
        <p className="muted" style={{ margin: 0 }}>{t("controls.helpIntro")}</p>
        <h2>{t("help.sticks")} · {m1 ? t("controls.mode1") : t("controls.mode2")}</h2>
        <div className="grid two" style={{ gridTemplateColumns: "repeat(2, minmax(0,1fr))" }}>
          {stick(m1 ? t("cockpit.leftStickM1") : t("cockpit.leftStick"), m1 ? t("cockpit.pitchYaw") : t("cockpit.throttleYaw"))}
          {stick(m1 ? t("cockpit.rightStickM1") : t("cockpit.rightStick"), m1 ? t("cockpit.throttleRoll") : t("cockpit.pitchRoll"))}
        </div>
        <h2>{t("help.actions")}</h2>
        <div className="card tight">
          {[
            [t("cockpit.armed"), t("help.arm")],
            [t("cockpit.autoTakeoff"), t("help.takeoff", { alt: fx(drone.param("takeoffAlt")) })],
            [t("cockpit.autoLand"), t("help.land", { v: fx(drone.param("landSpeed")), s: fx(drone.param("autoDisarm"), 0) })],
            [t("cockpit.rthLong"), gps ? t("help.rth", { alt: fx(drone.param("rthAlt"), 0) }) : t("help.indoorRth")],
            [t("cockpit.pause"), t("help.hold")],
            [t("cockpit.kill"), t("help.kill")],
          ].map(([k, v], i) => (
            <div key={k} className={`row${i === 0 ? " first" : ""}`}><span className="label" style={{ width: vp.phone ? 110 : 180, flex: "none" }}>{k}</span><span className="grow muted">{v}</span></div>
          ))}
        </div>
        <h2>{t("help.inputs")}</h2>
        <div className="card tight">
          {[
            [t("controls.elrs"), t("controls.elrsHelp")],
            [t("controls.gamepad"), t("controls.gamepadHelp")],
            [t("controls.keyboard"), t("controls.keyboardHelp")],
            [t("controls.touch"), t("controls.touchHelp")],
          ].map(([k, v], i) => (
            <div key={k} className={`row${i === 0 ? " first" : ""}`}><span className="label" style={{ width: vp.phone ? 110 : 180, flex: "none" }}>{k}</span><span className="grow muted">{v}</span></div>
          ))}
        </div>
      </main>
    </div>
  );
}

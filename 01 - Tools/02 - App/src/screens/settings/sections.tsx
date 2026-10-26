import { useState } from "react";
import { LANGS, translate, useT, type Lang } from "../../i18n";
import { drone } from "../../link/drone";
import { appSettings } from "../../settings/appSettings";
import { CalType, CheckId, SENSOR, Model } from "../../protocol/ids";
import { useDrone, useSettings, useViewport } from "../../ui/hooks";
import { Icon, Mark } from "../../ui/icons";
import { Confirm, Dot, GpsBadge, Row, Seg, SliderRow, Switch, useCommand } from "../../ui/kit";
import { gamepad } from "../../input/controls";
import { ParamNumber, ParamSelect, ParamSlider, ParamSwitch, useVisible } from "./params";

/* ---------------- General ---------------- */

export function General() {
  const { t } = useT();
  const st = useSettings();
  const s = useDrone();
  const run = useCommand();
  const cur = LANGS.find((l) => l.code === st.lang)!;
  return (
    <div className="grid two" style={{ alignItems: "start" }}>
      <section className="card" aria-labelledby="lang-h">
        <h2 id="lang-h">{t("general.language")}</h2>
        <p className="small muted" style={{ margin: 0 }}>{t("general.languageHint")}</p>
        <div role="radiogroup" aria-labelledby="lang-h" style={{ display: "flex", flexDirection: "column", gap: 6 }}>
          {LANGS.map((l) => (
            <label key={l.code} className={`choice${l.code === st.lang ? " sel" : ""}`}>
              <input type="radio" name="lang" checked={l.code === st.lang} onChange={() => appSettings.set({ lang: l.code as Lang })} />
              <span style={{ fontWeight: 600, width: 110 }} lang={l.code}>{l.native}</span>
              <span className="small muted" style={{ flex: 1 }}>{l.english}</span>
              {l.code === "en" && <span className="badge plain">{t("common.default")}</span>}
            </label>
          ))}
        </div>
      </section>
      <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
        <section className="card accent">
          <h2>{t("general.preview", { lang: cur.native })}</h2>
          <div className="grid two" style={{ gap: 8 }}>
            {["cockpit.autoTakeoff", "cockpit.autoLand", "cockpit.rthLong", "connect.slideToArm"].map((k) => (
              <span key={k} className="btn" style={{ cursor: "default", whiteSpace: "normal", textAlign: "center" }} lang={st.lang}>{translate(st.lang, k)}</span>
            ))}
          </div>
          <span className="small muted">{t("general.previewHint")}</span>
        </section>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 44 }}><h2>{t("general.display")}</h2></div>
          <Row label={t("general.units")}>
            <Seg label={t("general.units")} value={st.units} onChange={(v) => appSettings.set({ units: v })} options={[{ v: "metric", label: t("general.metric") }, { v: "imperial", label: t("general.imperial") }]} />
          </Row>
          <Row label={t("general.theme")}>
            <Seg label={t("general.theme")} value={st.theme} onChange={(v) => appSettings.set({ theme: v })} options={[{ v: "dark", label: t("general.dark") }, { v: "day", label: t("general.daylight") }]} />
          </Row>
          {s.caps && (
            <Row label={<label htmlFor="dname">{t("general.droneName")}</label>}>
              <input id="dname" className="input" style={{ width: 180 }} maxLength={16} defaultValue={s.caps.name} key={s.caps.name}
                onBlur={(e) => e.target.value.trim() && e.target.value !== s.caps!.name && run(drone.setName(e.target.value.trim()))} />
            </Row>
          )}
        </section>
      </div>
    </div>
  );
}

/* ---------------- Controls ---------------- */

export function Controls() {
  const { t } = useT();
  const st = useSettings();
  const s = useDrone();
  const pad = gamepad();
  const elrs = s.system?.lq != null;
  return (
    <div className="grid two" style={{ alignItems: "start" }}>
      <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48 }}><h2>{t("controls.sticks")}</h2></div>
          <Row label={t("controls.stickMode")}>
            <Seg label={t("controls.stickMode")} value={st.stickMode} onChange={(v) => appSettings.set({ stickMode: v })} options={[{ v: 1, label: t("controls.mode1") }, { v: 2, label: t("controls.mode2") }]} />
          </Row>
          <SliderRow label={t("controls.sensitivity")} value={st.sensitivity} min={10} max={100} step={5} format={(v) => `${v}%`} onChange={(v) => appSettings.set({ sensitivity: v })} />
          <SliderRow label={t("controls.expo")} value={st.expo} min={0} max={100} step={5} format={(v) => `${v}%`} onChange={(v) => appSettings.set({ expo: v })} />
          <SliderRow label={t("controls.deadband")} value={st.deadband} min={0} max={15} step={1} format={(v) => `${v}%`} onChange={(v) => appSettings.set({ deadband: v })} />
          <Row label={t("controls.haptics")} hint={t("controls.hapticsHint")}><Switch label={t("controls.haptics")} checked={st.haptics} onChange={(v) => appSettings.set({ haptics: v })} /></Row>
        </section>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48, flexDirection: "column", alignItems: "flex-start", justifyContent: "center", gap: 2, padding: "10px 0" }}>
            <h2>{t("controls.inputs")}</h2><span className="small muted">{t("controls.inputsHint")}</span>
          </div>
          <Row label={t("controls.elrs")} badge={<b className="num muted" style={{ width: 18 }}>1</b>}><span className="small" style={{ color: elrs ? "var(--ok)" : "var(--muted)" }}>{elrs ? t("controls.linked") : t("controls.notLinked")}</span></Row>
          <Row label={`${t("controls.gamepad")}${pad ? ` · ${pad.name}` : ""}`} badge={<b className="num muted" style={{ width: 18 }}>2</b>}>
            {!pad && <span className="small muted">{t("controls.gamepadNone")}</span>}
          </Row>
          <Row label={t("controls.touch")} badge={<b className="num muted" style={{ width: 18 }}>3</b>}><a href="#/help" className="small">{t("controls.shortcuts")}</a></Row>
        </section>
      </div>
      <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48 }}><h2>{t("controls.limits")}</h2></div>
          <ParamSlider k="maxTilt" />
          <ParamSlider k="maxYawRate" />
          <ParamSlider k="maxClimb" />
          <ParamSlider k="maxHSpeed" hint={t("controls.indoorCap")} />
          <ParamSwitch k="beginner" hint={t("controls.beginnerHint")} />
        </section>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48 }}><h2>{t("controls.automatic")}</h2></div>
          <ParamSlider k="takeoffAlt" />
          <ParamSlider k="landSpeed" />
          <ParamSlider k="autoDisarm" />
        </section>
      </div>
    </div>
  );
}

/* ---------------- Safety ---------------- */

export function Safety() {
  const { t } = useT();
  const st = useSettings();
  const s = useDrone();
  const visible = useVisible();
  const gps = drone.hasGps || !s.caps;
  const cells = s.power?.cells ?? s.caps?.cells ?? 0;
  return (
    <>
      <div className="banner"><GpsBadge /><span>{t("safety.banner")}</span></div>
      <div className="grid two" style={{ alignItems: "start" }}>
        <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
          <section className="card tight">
            <div className="row first" style={{ minHeight: 48 }}><h2>{t("safety.failsafe")}</h2></div>
            <ParamSelect k="fsAppShort" />
            <ParamSelect k="fsAppLong" />
            <ParamSelect k="fsRadio" />
            <ParamSelect k="fsGpsLost" />
            <ParamSelect k="fsFlowLost" />
          </section>
          <section className="card tight">
            <div className="row first" style={{ minHeight: 48, justifyContent: "space-between" }}><h2>{t("safety.battery")}</h2>{cells > 0 && <span className="small muted">{t("safety.cellsDetected", { n: cells })}</span>}</div>
            <ParamNumber k="battWarn" hint={t("safety.alert")} suffix={t("safety.perCell")} />
            <ParamNumber k="battCrit" hint={gps ? t("safety.rthOrLand") : t("actions.land")} suffix={t("safety.perCell")} />
            <ParamNumber k="battEmerg" hint={t("safety.landNow")} suffix={t("safety.perCell")} />
          </section>
        </div>
        <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
          <section className="card tight">
            <div className="row first" style={{ minHeight: 48 }}><h2>{t("safety.geofence")}</h2></div>
            <ParamSlider k="gfMaxAlt" hint={t("safety.altAll")} />
            <ParamSlider k="gfCeiling" hint={t("safety.tofHint")} />
            <ParamSlider k="gfMaxDist" />
            <ParamSelect k="gfAction" />
          </section>
          {visible("rthAlt") && (
            <section className="card tight">
              <div className="row first" style={{ minHeight: 48, gap: 10 }}><h2>{t("safety.rth")}</h2><GpsBadge /></div>
              <ParamSlider k="rthAlt" />
              <ParamSwitch k="rthStickCancel" />
            </section>
          )}
          <section className="card tight">
            <div className="row first" style={{ minHeight: 48 }}><h2>{t("safety.arming")}</h2></div>
            <ParamSwitch k="armRequireChecks" hint={t("safety.armHint")} />
            <Row label={t("safety.killConfirm")} hint={t("settings.savedLocal")}><Switch label={t("safety.killConfirm")} checked={st.killConfirm} onChange={(v) => appSettings.set({ killConfirm: v })} /></Row>
          </section>
        </div>
      </div>
    </>
  );
}

/* ---------------- Calibration overview ---------------- */

export function calibStatus(s: ReturnType<typeof drone.getSnapshot>, type: number): "ok" | "warn" | "fail" {
  const c = s.checks?.items.find((i) => i.id === CheckId.CALIBRATION);
  if (!c) return "ok";
  const bit = 1 << (type - 1);
  if (c.value & bit) return "fail";
  if ((c.value >> 8) & bit) return "warn";
  return "ok";
}

export function calibTypes(s: ReturnType<typeof drone.getSnapshot>) {
  const has = (b: number) => !s.caps || !!(s.caps.sensors & b);
  return [CalType.ACCEL, CalType.LEVEL, CalType.GYRO, CalType.COMPASS, CalType.FLOW_TOF, CalType.BATTERY, CalType.MOTORS, CalType.STICKS].filter((ct) =>
    ct === CalType.COMPASS ? has(SENSOR.MAG_FC) || has(SENSOR.MAG_GPS) : ct === CalType.FLOW_TOF ? has(SENSOR.FLOW) : ct === CalType.STICKS ? has(SENSOR.ELRS) : true,
  );
}

export function Calibration() {
  const { t } = useT();
  const s = useDrone();
  const run = useCommand();
  const [ask, setAsk] = useState(false);
  const vp = useViewport();
  const types = calibTypes(s);
  const rec = types.filter((ct) => calibStatus(s, ct) !== "ok").length;
  const btn = (ct: number) => (ct === CalType.GYRO ? t("calib.rerun") : ct === CalType.BATTERY ? t("calib.adjust") : ct === CalType.MOTORS ? t("common.open") : ct === CalType.STICKS ? t("calib.check") : t("calib.calibrate"));
  const detail = (ct: number) => (ct === CalType.COMPASS ? t("calib.details.4", { i: s.system?.magInterfPct ?? 0 }) : t(`calib.details.${ct}`));
  return (
    <div className="grid" style={{ gridTemplateColumns: vp.wide ? "minmax(0,1.75fr) minmax(0,1fr)" : "minmax(0,1fr)", alignItems: "start" }}>
      <section className="card tight">
        <div className="row first" style={{ minHeight: 48, justifyContent: "space-between" }}>
          <h2>{t("calib.onDrone")}</h2>
          {rec > 0 && <span className="small" style={{ color: "var(--warn)" }}>{t("calib.recommendedN", { n: rec })}</span>}
        </div>
        {types.map((ct) => {
          const st = calibStatus(s, ct);
          return (
            <a key={ct} className="row" href={`#/calib/${ct}`} style={st !== "ok" ? { background: "color-mix(in srgb, var(--warn) 6%, transparent)" } : undefined}>
              <Dot state={st} />
              <div className="grow"><span className="label" style={{ fontWeight: 600 }}>{t(`calib.names.${ct}`)}</span><span className="hint">{detail(ct)}</span></div>
              <span className="small" style={{ color: st === "ok" ? "var(--muted)" : st === "warn" ? "var(--warn)" : "var(--crit)", minWidth: 96 }}>
                {st === "ok" ? t("calib.ok") : st === "warn" ? t("calib.recommended") : t("calib.required")}
              </span>
              <span className={`btn small${st !== "ok" ? " warn" : ""}`} aria-hidden="true">{btn(ct)}</span>
            </a>
          );
        })}
      </section>
      <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
        <section className="card">
          <h2>{t("calib.beforeStart")}</h2>
          <ul style={{ margin: 0, paddingLeft: 18, display: "flex", flexDirection: "column", gap: 8, color: "var(--text-2)", fontSize: 13, lineHeight: 1.45 }}>
            <li>{t("calib.rule1")}</li><li>{t("calib.rule2")}</li><li>{t("calib.rule3")}</li><li>{t("calib.rule4")}</li>
          </ul>
        </section>
        <section className="card">
          <h2>{t("calib.statusMeaning")}</h2>
          <span className="small" style={{ display: "flex", gap: 10, alignItems: "center" }}><Dot state="ok" /><b>{t("calib.ok")}</b> · {t("calib.okMeaning")}</span>
          <span className="small" style={{ display: "flex", gap: 10, alignItems: "center" }}><Dot state="warn" /><b>{t("calib.recommended")}</b> · {t("calib.recMeaning")}</span>
          <span className="small" style={{ display: "flex", gap: 10, alignItems: "center" }}><Dot state="fail" /><b>{t("calib.required")}</b> · {t("calib.reqMeaning")}</span>
        </section>
        <section className="card">
          <h2>{t("calib.undo")}</h2>
          <p className="small muted" style={{ margin: 0 }}>{t("calib.undoHint")}</p>
          <button className="btn" style={{ alignSelf: "flex-start" }} onClick={() => setAsk(true)} disabled={s.link === "idle"}>{t("calib.restore")}</button>
        </section>
      </div>
      {ask && <Confirm title={t("calib.restore")} yes={t("calib.restore")} onNo={() => setAsk(false)}
        onYes={async () => { setAsk(false); for (const ct of types) await run(drone.calRestore(ct)); }} />}
    </div>
  );
}

/* ---------------- Connection ---------------- */

export function Connection() {
  const { t } = useT();
  const s = useDrone();
  const st = useSettings();
  const y = s.system;
  return (
    <div className="grid two" style={{ alignItems: "start" }}>
      <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
        <section className="card" aria-labelledby="ln-h">
          <h2 id="ln-h">{t("connection.link")}</h2>
          <div role="radiogroup" aria-labelledby="ln-h" style={{ display: "flex", flexDirection: "column", gap: 8 }}>
            <label className="choice sel"><input type="radio" name="link" defaultChecked />
              <span style={{ flex: 1, display: "flex", flexDirection: "column", gap: 2 }}><b>{t("connection.ble")}</b><span className="small muted">{t("connection.bleHint")}</span></span>
              {s.rssi !== null && <span className="small" style={{ color: "var(--ok)" }}>{s.rssi} dBm</span>}
            </label>
            <label className="choice" style={{ opacity: 0.6 }}><input type="radio" name="link" disabled />
              <span style={{ flex: 1, display: "flex", flexDirection: "column", gap: 2 }}><b>{t("connection.usb")}</b><span className="small muted">{t("connect.usbSoon")}</span></span>
            </label>
            <label className="choice" style={{ opacity: 0.6 }}><input type="radio" name="link" disabled />
              <span style={{ flex: 1, display: "flex", flexDirection: "column", gap: 2 }}><b>{t("connection.wifi")}</b><span className="small muted">{t("connect.wifiSoon")}</span></span>
            </label>
          </div>
        </section>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48 }}><h2>{t("connection.telemetry")}</h2></div>
          <Row label={<label htmlFor="trate">{t("connection.rate")}</label>}>
            <select id="trate" className="select" value={st.telemetryRate} onChange={(e) => {
              const hz = +e.target.value as 5 | 10 | 20 | 50;
              appSettings.set({ telemetryRate: hz });
              drone.setStateRate(hz);
            }}>
              {[5, 10, 20, 50].map((v) => <option key={v} value={v}>{v} Hz</option>)}
            </select>
          </Row>
          <Row label={t("connection.record")} hint={t("connection.recordHint")}><Switch label={t("connection.record")} checked={st.recordLog} onChange={(v) => appSettings.set({ recordLog: v })} /></Row>
          <Row label={t("connection.current")}><span className="value">{s.transport === "sim" ? t("connect.simulator") : s.transport === "ble" ? "Bluetooth" : t("link.notConnected")}{s.mtu ? ` · ${t("connection.mtu")} ${s.mtu}` : ""}</span></Row>
        </section>
      </div>
      <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
        <section className="card">
          <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
            <h2>{t("connection.elrs")}</h2>
            <span className="small" style={{ color: y?.lq != null ? "var(--ok)" : "var(--muted)" }}>{y?.lq != null ? t("controls.linked") : t("controls.notLinked")}</span>
          </div>
          <dl className="facts">
            <div><dt>{t("connection.lq")}</dt><dd>{y?.lq != null ? `${y.lq}%` : "–"}</dd></div>
            <div><dt>{t("connection.rssiSnr")}</dt><dd>{y?.lq != null ? `${y.rssi} dBm / ${y.snr} dB` : "–"}</dd></div>
            <div><dt>{t("connection.packetRate")}</dt><dd>{y?.lq != null ? `${y.pktRateHz} Hz` : "–"}</dd></div>
            <div><dt>{t("connection.txPower")}</dt><dd>{y?.lq != null ? `${y.txPowerMw} mW` : "–"}</dd></div>
          </dl>
        </section>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48 }}><h2>{t("connection.paired")}</h2></div>
          {s.caps && (
            <div className="row">
              <Mark className="brand-mark" alt="" />
              <div className="grow"><b>{s.caps.name}</b><span className="hint">{s.caps.model === Model.OUTDOOR ? t("about.outdoor") : t("about.indoor")} · {s.power?.cells ?? s.caps.cells}S</span></div>
              <span className="small" style={{ color: "var(--ok)" }}>{t("connection.inUse")}</span>
            </div>
          )}
          <div className="row">
            {s.link !== "idle" ? <button className="btn" onClick={() => drone.disconnect()}>{t("link.disconnect")}</button>
              : <a className="btn" href="#/connect" style={{ borderColor: "var(--accent)", color: "var(--accent)" }}>{t("connection.pairNew")}</a>}
          </div>
        </section>
      </div>
    </div>
  );
}

/* ---------------- Alerts ---------------- */

export function Alerts() {
  const { t } = useT();
  const st = useSettings();
  const s = useDrone();
  const gps = drone.hasGps || !s.caps;
  const flow = !s.caps || !!(s.caps.sensors & SENSOR.FLOW);
  const w = st.warn;
  const set = (k: keyof typeof w, v: boolean) => appSettings.set({ warn: { ...w, [k]: v } });
  const items: [keyof typeof w, boolean][] = [["battLow", true], ["link", true], ["gps", gps], ["flow", flow], ["fence", true], ["compass", true], ["vibration", true], ["hot", true]];
  return (
    <div className="grid two" style={{ alignItems: "start" }}>
      <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48 }}><h2>{t("alerts.voice")}</h2></div>
          <Row label={t("alerts.voiceAlerts")} hint={t("alerts.voiceHint")}><Switch label={t("alerts.voiceAlerts")} checked={st.voice} onChange={(v) => appSettings.set({ voice: v })} /></Row>
          <SliderRow label={t("alerts.volume")} value={st.volume} min={0} max={100} step={5} format={(v) => `${v}%`} onChange={(v) => appSettings.set({ volume: v })} />
          <Row label={t("alerts.vibrate")} hint={t("alerts.vibrateHint")}><Switch label={t("alerts.vibrate")} checked={st.vibrate} onChange={(v) => appSettings.set({ vibrate: v })} /></Row>
        </section>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48 }}><h2>{t("alerts.callouts")}</h2></div>
          <Row label={<label htmlFor="c1">{t("alerts.altitude")}</label>}>
            <select id="c1" className="select" value={st.callAlt} onChange={(e) => appSettings.set({ callAlt: +e.target.value as 0 | 5 | 10 })}>
              <option value={0}>{t("common.off")}</option><option value={5}>{t("alerts.every5m")}</option><option value={10}>{t("alerts.every10m")}</option>
            </select>
          </Row>
          <Row label={<label htmlFor="c2">{t("alerts.battery")}</label>}>
            <select id="c2" className="select" value={st.callBatt} onChange={(e) => appSettings.set({ callBatt: +e.target.value as 0 | 10 | 25 })}>
              <option value={0}>{t("common.off")}</option><option value={10}>{t("alerts.every10pct")}</option><option value={25}>{t("alerts.every25pct")}</option>
            </select>
          </Row>
          <Row label={<label htmlFor="c3">{t("alerts.timer")}</label>}>
            <select id="c3" className="select" value={st.callTimer} onChange={(e) => appSettings.set({ callTimer: e.target.value as "off" | "min" | "5" })}>
              <option value="off">{t("common.off")}</option><option value="min">{t("alerts.everyMin")}</option><option value="5">{t("alerts.at5min")}</option>
            </select>
          </Row>
        </section>
      </div>
      <section className="card tight">
        <div className="row first" style={{ minHeight: 48, flexDirection: "column", alignItems: "flex-start", justifyContent: "center", gap: 2, padding: "10px 0" }}>
          <h2>{t("alerts.warnings")}</h2><span className="small muted">{t("alerts.warningsHint")}</span>
        </div>
        {items.filter(([, show]) => show).map(([k]) => (
          <Row key={k} label={t(`alerts.w.${k}`)} badge={k === "gps" ? <GpsBadge /> : undefined}>
            <Switch label={t(`alerts.w.${k}`)} checked={w[k]} onChange={(v) => set(k, v)} />
          </Row>
        ))}
      </section>
    </div>
  );
}

/* ---------------- Drone & firmware ---------------- */

export function About() {
  const { t } = useT();
  const s = useDrone();
  const run = useCommand();
  const [ask, setAsk] = useState(false);
  const c = s.caps;
  const has = (b: number) => !!(c && c.sensors & b);
  const hw: [string, boolean, string?][] = c ? [
    [t("about.imu"), has(SENSOR.IMU)],
    [t("about.baro"), has(SENSOR.BARO)],
    [t("about.compass"), has(SENSOR.MAG_FC) || has(SENSOR.MAG_GPS)],
    [t("about.gps"), has(SENSOR.GNSS), has(SENSOR.GNSS) ? t("about.outdoor") : t("about.indoor")],
    [t("about.flow"), has(SENSOR.FLOW) && has(SENSOR.TOF)],
    [t("about.esc"), has(SENSOR.ESC_TLM), t("about.motorsFrame", { n: c.nMotors, frame: t(`about.frames.${c.frame}`) })],
    [t("about.battery"), true, `${s.power?.cells ?? c.cells}S`],
  ] : [];
  const calWarn = calibTypes(s).filter((ct) => calibStatus(s, ct) !== "ok").length;
  return (
    <div className="grid two" style={{ alignItems: "start" }}>
      <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48, flexDirection: "column", alignItems: "flex-start", justifyContent: "center", gap: 2, padding: "10px 0" }}>
            <h2>{t("about.hardware")}</h2><span className="small muted">{t("about.hardwareHint")}</span>
          </div>
          {!c && <Row label={t("link.notConnected")} />}
          {hw.map(([name, present, note]) => (
            <Row key={name} label={name} badge={<Dot state={present ? "ok" : "na"} />}>
              <span className="small" style={{ color: name === t("about.gps") && present ? "var(--accent)" : "var(--muted)" }}>{present ? note ?? "OK" : note ?? t("about.absent")}</span>
            </Row>
          ))}
        </section>
        <a className="card" href="#/settings/calibration" style={{ flexDirection: "row", alignItems: "center", gap: 14, color: "inherit", textDecoration: "none" }}>
          <Dot state={calWarn ? "warn" : "ok"} />
          <span style={{ flex: 1, display: "flex", flexDirection: "column", gap: 2 }}><b>{t("calib.title")}</b><span className="small muted">{calWarn ? t("calib.recommendedN", { n: calWarn }) : t("settings.calibrationSummaryOk")}</span></span>
          <Icon name="chevron" size={16} />
        </a>
      </div>
      <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
        <section className="card">
          <h2>{t("about.firmware")}</h2>
          <dl className="facts">
            <div><dt>{t("about.fc")}</dt><dd>{c ? `FC rev ${c.hwRev}` : "–"}</dd></div>
            <div><dt>{t("about.serial")}</dt><dd>{c?.serial || "–"}</dd></div>
            <div><dt>{t("about.installed")}</dt><dd>{c ? c.fw.join(".") : "–"}</dd></div>
            <div><dt>Protocol</dt><dd>{c ? `v${c.proto}` : "–"}</dd></div>
          </dl>
          <span className="small muted">{t("about.updateHint")}</span>
        </section>
        <section className="card tight">
          <div className="row first" style={{ minHeight: 48 }}><h2>{t("about.logs")}</h2></div>
          <Row label={t("about.app")}><span className="value">v0.1.0</span></Row>
          <Row label={t("about.reset")}><button className="btn small" style={{ borderColor: "var(--crit)", color: "var(--crit-text)" }} disabled={!c} onClick={() => setAsk(true)}>{t("about.resetBtn")}</button></Row>
        </section>
      </div>
      {ask && <Confirm danger title={t("about.reset")} body={t("about.resetConfirm")} yes={t("about.resetBtn")} onNo={() => setAsk(false)}
        onYes={() => { setAsk(false); run(drone.paramDefaults()); }} />}
    </div>
  );
}

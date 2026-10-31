import { useState } from "react";
import { useT } from "../i18n";
import { drone } from "../link/drone";
import { bleAvailable, pickBleDrone } from "../link/bleTransport";
import { LinkError } from "../link/transport";
import { SimTransport } from "../sim/simTransport";
import { appSettings } from "../settings/appSettings";
import { CheckId, CheckState, ArmState, Model, SENSOR } from "../protocol/ids";
import type { CheckItem } from "../protocol/messages";
import { go, useDrone, useViewport } from "../ui/hooks";
import { Icon, Logo, Mark } from "../ui/icons";
import { Dot, Seg, SlideToConfirm, toast, useCommand } from "../ui/kit";

export function checkValue(c: CheckItem, t: (k: string, v?: Record<string, string | number>) => string, cells: number) {
  switch (c.id) {
    case CheckId.BATTERY:
      if (c.state === CheckState.NA) return t("common.na");
      // value 0: no battery measured -- USB power only (WARN) or the battery monitor stopped (FAIL)
      if (c.value === 0) return c.state === CheckState.FAIL ? t("checks.battMonitor") : t("checks.noBattery");
      return `${cells || "?"}S · ${(c.value / 1000).toFixed(2)} V/cell`;
    case CheckId.COMPASS: return t("calib.compass.interference", { i: c.value });
    case CheckId.GNSS: return c.state === CheckState.NA ? t("common.na") : `3D · ${c.value} sat`;
    case CheckId.HOME: return c.state === CheckState.NA ? t("common.na") : `± ${(c.value / 10).toFixed(1)} m`;
    case CheckId.RADIO: return c.state === CheckState.NA ? t("common.na") : `LQ ${c.value}%`;
    case CheckId.FLOW: return c.state === CheckState.NA ? t("common.na") : t("telemetry.quality", { q: c.value });
    case CheckId.FAILSAFE: return `${c.value} m`;
    default: return t(`checks.state.${c.state}`);
  }
}

export function ConnectScreen() {
  const { t } = useT();
  const s = useDrone();
  const vp = useViewport();
  const run = useCommand();
  const [tab, setTab] = useState<"ble" | "usb" | "wifi">("ble");
  const [busy, setBusy] = useState(false);
  const ble = bleAvailable();
  const connected = s.link === "connected" || s.link === "lost";
  const armed = s.flight?.arm === ArmState.ARMED;

  const connectBle = async () => {
    setBusy(true);
    try {
      const tr = await pickBleDrone();
      await drone.connect(tr);
      appSettings.set({ lastLink: "ble" });
    } catch (e) {
      if (e instanceof LinkError && e.code === "denied") toast(t("connect.cancelled"));
      else toast(t("connect.failed", { msg: e instanceof Error ? e.message : String(e) }), "error");
    } finally {
      setBusy(false);
    }
  };
  const connectSim = async (m: "outdoor" | "indoor") => {
    setBusy(true);
    await drone.connect(new SimTransport(m));
    appSettings.set({ lastLink: m === "outdoor" ? "sim-outdoor" : "sim-indoor" });
    setBusy(false);
  };

  const items = (s.checks?.items ?? []).filter((c) => c.state !== CheckState.NA || c.id === CheckId.GNSS);
  const shown = items.filter((c) => c.state !== CheckState.NA);
  const okN = shown.filter((c) => c.state === CheckState.OK).length;
  const caps = s.caps;

  const connectCard = (
    <section aria-labelledby="c-h" style={{ display: "flex", flexDirection: "column", gap: 14 }}>
      <h1 id="c-h" className="display" style={{ margin: 0, fontSize: vp.phone ? 24 : 28 }}>{t("connect.title")}</h1>
      <Seg full label={t("connect.type")} value={tab} onChange={setTab}
        options={[{ v: "ble", label: t("connect.bluetooth") }, { v: "usb", label: t("connect.usb") }, { v: "wifi", label: t("connect.wifi") }]} />
      {connected && caps ? (
        <div className="devcard on">
          <Mark className="" alt="" />
          <div style={{ flex: 1, display: "flex", flexDirection: "column", gap: 2, minWidth: 0 }}>
            <b>{caps.name || s.deviceName}</b>
            <span className="small muted">
              {caps.model === Model.OUTDOOR ? t("about.outdoor") : t("about.indoor")} · fw {caps.fw.join(".")}
              {s.rssi !== null ? ` · ${s.rssi} dBm` : ""}{s.transport === "sim" ? ` · ${t("connect.simulator")}` : ""}
            </span>
          </div>
          <span className="small" style={{ color: s.link === "lost" ? "var(--warn)" : "var(--ok)", fontWeight: 600 }}>
            {s.link === "lost" ? t("link.lost") : t("link.connected")}
          </span>
        </div>
      ) : tab === "ble" ? (
        <>
          {!ble.ok && <div className="banner warn">{t("connect.noWebBluetooth")}</div>}
          <button className="btn primary big" disabled={busy || !ble.ok} onClick={connectBle}>
            <Icon name="bt" /> {busy ? t("connect.searching") : t("connect.search")}
          </button>
          <p className="small muted" style={{ margin: 0, lineHeight: 1.45 }}>{t("connect.bleHint")}</p>
        </>
      ) : (
        <div className="banner" style={{ alignItems: "center" }}>
          <span style={{ flex: 1 }}>{tab === "usb" ? t("connect.usbSoon") : t("connect.wifiSoon")}</span>
          {tab === "usb" && <a className="btn small" href="#/logs">{t("logs.title")}</a>}
        </div>
      )}
      {!connected && (
        <div className="card">
          <span className="section-label" style={{ margin: 0 }}>{t("connect.simulator")}</span>
          <div className="btn-row">
            <button className="btn" disabled={busy} onClick={() => connectSim("outdoor")}><Icon name="sim" /> {t("connect.simOutdoor")}</button>
            <button className="btn" disabled={busy} onClick={() => connectSim("indoor")}><Icon name="sim" /> {t("connect.simIndoor")}</button>
          </div>
        </div>
      )}
      {connected && <button className="btn" onClick={() => drone.disconnect()}>{t("link.disconnect")}</button>}
    </section>
  );

  const preflight = connected && (
    <section aria-labelledby="p-h" style={{ display: "flex", flexDirection: "column", gap: 12 }}>
      <div style={{ display: "flex", alignItems: "baseline", justifyContent: "space-between" }}>
        <h2 id="p-h" className="display" style={{ margin: 0, fontSize: 20, fontWeight: 600 }}>{t("connect.preflight")}</h2>
        <span className="small muted">{t("connect.okCount", { ok: okN, total: shown.length })}</span>
      </div>
      <ul className="checklist" style={vp.wide ? { display: "grid", gridTemplateColumns: "repeat(2, minmax(0,1fr))" } : undefined}>
        {shown.map((c) => (
          <li key={c.id} style={vp.wide ? { borderTop: "none", borderBottom: "1px solid var(--line)" } : undefined}>
            <Dot state={c.state === CheckState.OK ? "ok" : c.state === CheckState.WARN ? "warn" : "fail"} />
            <span style={{ flex: 1 }}>{t(`checks.${c.id}`)}</span>
            {c.id === CheckId.COMPASS && c.state !== CheckState.OK ? (
              <a href="#/calib/4" style={{ color: "var(--warn)", fontSize: 13 }}>{t("connect.calibrate")}</a>
            ) : (
              <span className="v">{checkValue(c, t, s.power?.cells || caps?.cells || 0)}</span>
            )}
          </li>
        ))}
      </ul>
      {s.battMonitorLostAt !== null && <div className="banner crit" role="alert">{t("connect.battMonitorLost")}</div>}
      {caps && !(caps.sensors & SENSOR.GNSS) && <span className="small muted">{t("connect.indoorNote")}</span>}
      <span style={{ flex: 1 }} />
      {armed ? (
        <SlideToConfirm armed label={t("cockpit.slideToDisarm")} aria={t("cockpit.slideToDisarmAria")} onConfirm={() => run(drone.disarm())} />
      ) : (
        <SlideToConfirm label={t("connect.slideToArm")} aria={t("connect.slideToArmAria")}
          onConfirm={async () => { if (await run(drone.arm())) go("cockpit"); }} />
      )}
      <div style={{ display: "flex", alignItems: "center", gap: 12, justifyContent: "space-between", flexWrap: "wrap" }}>
        <span className="small muted" style={{ flex: "1 1 220px" }}>{t("connect.armHint")}</span>
        <a className="btn" href="#/cockpit">{t("connect.openCockpit")}</a>
      </div>
    </section>
  );

  return (
    <div className="page">
      <header className="topbar">
        <Logo />
        <span className="display" style={{ fontWeight: 600, fontSize: 18 }}>Pilot</span>
        <span className="spacer" />
        <span className="small muted">v0.1.0</span>
        <a className="btn icon" href="#/settings" aria-label={t("common.settings")}><Icon name="gear" /></a>
      </header>
      <main className="content" style={{ maxWidth: 1280 }}>
        <div className="grid" style={{ gridTemplateColumns: vp.wide && connected ? "minmax(0,1fr) minmax(0,1.15fr)" : "minmax(0,1fr)", gap: 28, maxWidth: vp.wide && connected ? undefined : 560, width: "100%", margin: "0 auto" }}>
          {connectCard}
          {preflight}
        </div>
      </main>
    </div>
  );
}

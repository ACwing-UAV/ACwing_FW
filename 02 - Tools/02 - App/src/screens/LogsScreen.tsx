/**
 * Flight logs (HANDOVER_APP.md §7): the last flight over Bluetooth on every
 * platform, every log over USB (web app on PC/Mac and Android Chrome), the
 * logs already on this device, and the viewer.
 *   #/logs            list
 *   #/logs/last       list, and download the last flight at once (from the cockpit)
 *   #/logs/view/<key> viewer of a stored log
 */
import { useEffect, useMemo, useState, useSyncExternalStore } from "react";
import { Capacitor } from "@capacitor/core";
import { useT } from "../i18n";
import { drone } from "../link/drone";
import { ArmState, SENSOR, EventLevel } from "../protocol/ids";
import type { LogEntryMsg } from "../protocol/messages";
import { useDrone, useViewport } from "../ui/hooks";
import { Icon, Mark } from "../ui/icons";
import { Confirm, Row, toast } from "../ui/kit";
import { fmt } from "../ui/format";
import { TimeChart } from "../ui/timeChart";
import { ParamSelect } from "./settings/params";
import { logsSession, storedAs, type Job } from "../log/session";
import { logStore, type StoredLogMeta } from "../log/store";
import { decodeLog, toCsv, type DecodedLog } from "../log/format";
import { shareFiles } from "../log/share";

const useLogs = () => useSyncExternalStore(logsSession.subscribe, logsSession.get);

function useStored(): StoredLogMeta[] {
  const v = useSyncExternalStore(logStore.subscribe, logStore.getVersion);
  const [list, setList] = useState<StoredLogMeta[]>([]);
  useEffect(() => {
    let on = true;
    logStore.list().then((l) => on && setList(l)).catch(() => undefined);
    return () => {
      on = false;
    };
  }, [v]);
  return list;
}

const kb = (b: number) => (b >= 1024 * 1024 ? `${(b / 1024 / 1024).toFixed(1)} MB` : `${Math.max(1, Math.round(b / 1024))} kB`);
const dur = (s: number) => (s < 60 ? `${Math.round(s)} s` : fmt.mmss(s));
const fileStem = (m: StoredLogMeta) => `acwing_flight_${String(m.logId).padStart(5, "0")}`;

async function exportBin(m: StoredLogMeta) {
  const x = await logStore.get(m.key);
  if (x) await shareFiles([{ name: `${fileStem(m)}.bin`, data: x.data, type: "application/octet-stream" }], fileStem(m));
}

async function exportCsv(m: StoredLogMeta, L?: DecodedLog) {
  const data = L ?? decodeLog((await logStore.get(m.key))!.data);
  const files = toCsv(data, fileStem(m));
  await shareFiles(Object.entries(files).map(([name, text]) => ({ name, data: text, type: "text/csv" })), fileStem(m));
}

/* ------------------------------------------------------------------ */
/* List                                                                 */
/* ------------------------------------------------------------------ */

export function LogsScreen({ sub, arg }: { sub?: string; arg?: string }) {
  if (sub === "view" && arg) return <LogViewer logKey={decodeURIComponent(arg)} />;
  return <LogsList autoLast={sub === "last"} />;
}

function JobPanel({ job }: { job: Job }) {
  const { t } = useT();
  const pct = job.total ? Math.round((job.done / job.total) * 100) : 0;
  const eta = job.rate > 0 ? (job.total - job.done) / job.rate : null;
  return (
    <section className={`card${job.state === "failed" ? "" : " accent"}`} role="status" aria-live="polite" style={job.state === "failed" ? { borderColor: "var(--crit)" } : undefined}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, flexWrap: "wrap" }}>
        <b style={{ flex: 1, minWidth: 160 }}>
          {job.state === "running" ? t("logs.downloading", { id: job.logId }) : job.state === "done" ? t("logs.downloaded", { id: job.logId }) : job.state === "cancelled" ? t("logs.err.cancelled") : t(job.error ?? "logs.err.failed")}
          {job.of > 1 && <span className="muted" style={{ fontWeight: 400 }}> · {job.n}/{job.of}</span>}
        </b>
        {job.state === "running" && <button className="btn small" onClick={logsSession.cancel}>{t("common.cancel")}</button>}
        {job.state === "done" && job.saved && <a className="btn primary" style={{ minHeight: 36 }} href={`#/logs/view/${encodeURIComponent(job.saved.key)}`}>{t("logs.open")}</a>}
        {job.state !== "running" && <button className="btn small" onClick={logsSession.dismissJob} aria-label={t("common.close")}><Icon name="close" size={16} /></button>}
      </div>
      {job.state === "running" && (
        <>
          <div className="progress" role="progressbar" aria-valuenow={pct} aria-valuemin={0} aria-valuemax={100}><span style={{ width: `${pct}%` }} /></div>
          <span className="small muted">{kb(job.done)} / {kb(job.total)} · {(job.rate / 1000).toFixed(1)} kB/s{eta != null ? ` · ${t("logs.left", { t: dur(eta) })}` : ""}</span>
        </>
      )}
    </section>
  );
}

function LogsList({ autoLast }: { autoLast: boolean }) {
  const { t } = useT();
  const s = useDrone();
  const L = useLogs();
  const stored = useStored();
  const vp = useViewport();
  const [askErase, setAskErase] = useState(false);
  const caps = s.caps;
  const connected = s.link === "connected" && !!caps;
  const armed = s.flight?.arm === ArmState.ARMED;
  const hasFlash = !!caps && !!(caps.sensors & SENSOR.FLASH_LOG);
  const running = L.job?.state === "running";
  const ios = Capacitor.getPlatform() === "ios";
  const nativeAndroid = Capacitor.getPlatform() === "android";

  // another drone, or none: forget the last-flight entry
  useEffect(() => {
    if (!connected) logsSession.resetBle();
  }, [connected]);
  // a USB port granted earlier: reopen it without the picker
  useEffect(() => {
    if (logsSession.usbSupported()) void logsSession.connectUsb({ silent: true });
  }, []);
  // from the cockpit: "Download this flight"
  useEffect(() => {
    if (!autoLast || !connected || armed || !hasFlash) return;
    void (async () => {
      await logsSession.listBle();
      if (logsSession.get().ble.entry && !storedAs(await logStore.list(), logsSession.get().ble.entry!)) await logsSession.downloadBle();
    })();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [autoLast, connected]);

  const back = s.link !== "idle" ? "#/settings/about" : "#/connect";
  const e = L.ble.entry;
  const eStored = e ? storedAs(stored, e) : null;

  const bleCard = (
    <section className="card">
      <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
        <span style={{ color: "var(--accent)", display: "flex" }}><Icon name="bt" size={22} /></span>
        <h2 style={{ flex: 1 }}>{t("logs.lastFlight")}</h2>
      </div>
      {!connected ? (
        <div className="banner">{t("logs.connectFirst")} <a href="#/connect" style={{ marginLeft: "auto" }}>{t("connect.connectBtn")}</a></div>
      ) : !hasFlash ? (
        <div className="banner">{t("logs.noFlash")}</div>
      ) : armed ? (
        <div className="banner warn">{t("logs.err.armed")}</div>
      ) : !L.ble.listed || L.ble.busy ? (
        <>
          <span className="small muted">{t("logs.bleHint")}</span>
          <div className="btn-row"><button className="btn primary" disabled={L.ble.busy || running} onClick={() => void logsSession.listBle()}>{L.ble.busy ? t("common.loading") : t("logs.check")}</button></div>
        </>
      ) : L.ble.error ? (
        <div className="banner crit" style={{ alignItems: "center" }}><span style={{ flex: 1 }}>{t(L.ble.error)}</span><button className="btn small" onClick={() => void logsSession.listBle()}>{t("common.restart")}</button></div>
      ) : !e ? (
        <div className="banner">{t("logs.noLogs")}</div>
      ) : (
        <>
          <dl className="facts">
            <div><dt>{t("logs.logN", { id: e.id })}</dt><dd>{t("logs.afterPowerOn", { t: dur(e.t0Ms / 1000) })}</dd></div>
            <div><dt>{t("logs.size")}</dt><dd>{kb(e.bytes)}</dd></div>
            <div><dt>{t("logs.flightTime")}</dt><dd>≈ {dur(logsSession.durationEstimateS(e.bytes))}</dd></div>
            <div><dt>{t("logs.downloadTime")}</dt><dd>≈ {dur(logsSession.bleEstimateS(e.bytes))}</dd></div>
          </dl>
          {drone.param("logMode") >= 1.5 && <span className="small muted">{t("logs.alwaysHint")}</span>}
          <div className="btn-row">
            {eStored ? <a className="btn" href={`#/logs/view/${encodeURIComponent(eStored.key)}`}><Icon name="log" size={18} />{t("logs.onDevice")}</a> : null}
            <button className={`btn${eStored ? "" : " primary"}`} disabled={running} onClick={() => void logsSession.downloadBle()}><Icon name="download" size={18} />{eStored ? t("logs.again") : t("logs.download")}</button>
            <button className="btn ghost" disabled={running || L.ble.busy} onClick={() => void logsSession.listBle()}>{t("logs.refresh")}</button>
          </div>
        </>
      )}
    </section>
  );

  const usbRow = (x: LogEntryMsg) => {
    const st = storedAs(stored, x);
    return (
      <div key={x.id} className="row" style={{ flexWrap: vp.phone ? "wrap" : undefined, padding: vp.phone ? "8px 0" : undefined }}>
        <span className="grow">
          <span className="label">{t("logs.logN", { id: x.id })}{x.truncated && <span className="badge plain" style={{ marginLeft: 8 }}>{t("logs.truncated")}</span>}{x.active && <span className="badge" style={{ marginLeft: 8 }}>{t("logs.recording")}</span>}</span>
          <span className="hint">{t("logs.afterPowerOn", { t: dur(x.t0Ms / 1000) })} · ≈ {dur(logsSession.durationEstimateS(x.bytes))} · {kb(x.bytes)}{st ? ` · ${t("logs.onDevice")}` : ""}</span>
        </span>
        {st && <a className="btn small" href={`#/logs/view/${encodeURIComponent(st.key)}`}>{t("logs.open")}</a>}
        <button className="btn small" disabled={running || L.usb.erasing} onClick={() => void logsSession.downloadUsb([x.id])} aria-label={`${t("logs.download")} ${t("logs.logN", { id: x.id })}`}><Icon name="download" size={16} /></button>
      </div>
    );
  };

  const usbCard = (
    <section className="card">
      <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
        <span style={{ color: "var(--accent)", display: "flex" }}><Icon name="usb" size={22} /></span>
        <h2 style={{ flex: 1 }}>{t("logs.allLogs")}</h2>
        {L.usb.open && <span className="small muted">{L.usb.name}</span>}
      </div>
      {ios ? (
        <span className="small muted">{t("logs.usbIos")}</span>
      ) : !L.usb.open ? (
        <>
          <span className="small muted">{nativeAndroid ? t("logs.usbNative") : logsSession.usbSupported() ? t("logs.usbHint") : t("logs.usbBrowser")}</span>
          {L.usb.error && <div className="banner warn">{t(L.usb.error)}</div>}
          <div className="btn-row">
            {logsSession.usbSupported() && <button className="btn primary" onClick={() => void logsSession.connectUsb()}><Icon name="usb" size={18} />{t("logs.connectUsb")}</button>}
            <button className="btn" onClick={() => void logsSession.connectUsb({ sim: true })}><Icon name="sim" size={18} />{t("connect.simulator")}</button>
          </div>
        </>
      ) : (
        <>
          {L.usb.error && <div className="banner warn">{t(L.usb.error)}</div>}
          {L.usb.busy ? <span className="muted">{t("common.loading")}</span> : L.usb.entries.length === 0 ? <div className="banner">{t("logs.noLogs")}</div> : (
            <div className="card tight" style={{ padding: "0 0", border: "none" }}>{L.usb.entries.map(usbRow)}</div>
          )}
          <div className="btn-row">
            <button className="btn primary" disabled={running || L.usb.erasing || !L.usb.entries.length} onClick={() => void logsSession.downloadUsb(L.usb.entries.map((x) => x.id))}><Icon name="download" size={18} />{t("logs.downloadAll")}</button>
            <button className="btn ghost" disabled={running || L.usb.busy} onClick={() => void logsSession.listUsb()}>{t("logs.refresh")}</button>
            <span style={{ flex: 1 }} />
            <button className="btn" style={{ borderColor: "var(--crit)", color: "var(--crit-text)" }} disabled={running || L.usb.erasing || armed} onClick={() => setAskErase(true)}>
              <Icon name="trash" size={18} />{L.usb.erasing ? t("logs.erasing") : t("logs.eraseAll")}
            </button>
            <button className="btn ghost" onClick={() => void logsSession.disconnectUsb()}>{t("link.disconnect")}</button>
          </div>
        </>
      )}
    </section>
  );

  const storedCard = (
    <section className="card">
      <h2>{t("logs.thisDevice")}</h2>
      {stored.length === 0 ? <span className="muted">{t("logs.noneStored")}</span> : (
        <div className="card tight" style={{ padding: 0, border: "none" }}>
          {stored.map((m) => (
            <div key={m.key} className="row" style={{ flexWrap: vp.phone ? "wrap" : undefined, padding: vp.phone ? "8px 0" : undefined }}>
              <a className="grow" href={`#/logs/view/${encodeURIComponent(m.key)}`} style={{ color: "inherit", textDecoration: "none" }}>
                <span className="label">{m.drone} · {t("logs.logN", { id: m.logId })}{m.summary.dropped > 0 && <span className="badge plain" style={{ marginLeft: 8, borderColor: "var(--warn)", color: "var(--warn)" }}>{t("logs.dropped")}</span>}</span>
                <span className="hint">{new Date(m.downloadedAt).toLocaleString()} · {dur(m.summary.durationS)} · {kb(m.size)} · {m.source === "ble" ? "Bluetooth" : "USB"}</span>
              </a>
              <button className="btn small" onClick={() => void exportBin(m)} aria-label={`${t("logs.exportBin")} ${m.logId}`}>.bin</button>
              <button className="btn small" onClick={() => void exportCsv(m).catch(() => toast(t("logs.err.failed"), "warn"))} aria-label={`${t("logs.exportCsv")} ${m.logId}`}>CSV</button>
              <button className="btn small" onClick={() => void logStore.remove(m.key)} aria-label={`${t("logs.delete")} ${m.logId}`}><Icon name="trash" size={16} /></button>
            </div>
          ))}
        </div>
      )}
    </section>
  );

  const recordCard = connected && hasFlash && (
    <section className="card tight">
      <ParamSelect k="logMode" hint={t("logs.modeHint")} />
      <Row label={t("about.logUsed")}><span className="value">{s.system ? `${s.system.logPct}%` : "–"}</span></Row>
    </section>
  );

  return (
    <div className="page">
      <header className="topbar">
        <a className={vp.wide ? "btn" : "btn icon"} href={back} aria-label={t("common.back")}><Icon name="back" />{vp.wide && t("common.back")}</a>
        {vp.wide && <Mark />}
        <h1 style={{ flex: 1 }}>{t("logs.title")}</h1>
        {vp.wide && <span className="small muted">{caps?.name ?? t("link.notConnected")}</span>}
      </header>
      <main className="content" style={{ maxWidth: 1100 }}>
        {L.job && <JobPanel job={L.job} />}
        {vp.wide ? (
          <div className="grid two" style={{ alignItems: "start" }}>
            <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>{bleCard}{recordCard}{usbCard}</div>
            {storedCard}
          </div>
        ) : (
          <>{bleCard}{storedCard}{usbCard}{recordCard}</>
        )}
      </main>
      {askErase && (
        <Confirm danger title={t("logs.eraseAll")} body={t("logs.eraseConfirm")} yes={t("logs.eraseAll")}
          onNo={() => setAskErase(false)} onYes={() => { setAskErase(false); void logsSession.eraseUsb(); }} />
      )}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Viewer                                                               */
/* ------------------------------------------------------------------ */

const R2D = 180 / Math.PI;

function LogViewer({ logKey }: { logKey: string }) {
  const { t } = useT();
  const vp = useViewport();
  const [st, setSt] = useState<{ meta: StoredLogMeta; L: DecodedLog } | null | "missing">(null);
  useEffect(() => {
    let on = true;
    logStore.get(logKey).then((x) => on && setSt(x ? { meta: x.meta, L: decodeLog(x.data) } : "missing")).catch(() => on && setSt("missing"));
    return () => {
      on = false;
    };
  }, [logKey]);

  const view = useMemo(() => {
    if (!st || st === "missing") return null;
    const L = st.L;
    const all = [L.att.t, L.vert.t, L.flight.t, L.power.t, L.event.t].filter((x) => x.length);
    const t0 = L.info ? L.info.t0Ms / 1000 : all.length ? Math.min(...all.map((x) => x[0])) : 0;
    const t1 = Math.max(t0 + 1, ...all.map((x) => x[x.length - 1]));
    const rel = (x: number[]) => x.map((v) => v - t0);
    // attitude from q (body → NWU): roll, pitch (+ nose up), heading (clockwise from north)
    const roll: number[] = [], pitch: number[] = [], yaw: number[] = [];
    for (let i = 0; i < L.att.n; i++) {
      const [w, x, y, z] = [L.att.q[0][i], L.att.q[1][i], L.att.q[2][i], L.att.q[3][i]];
      roll.push(Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * R2D);
      pitch.push(-Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))) * R2D);
      yaw.push((((-Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * R2D) % 360) + 360) % 360);
    }
    return { t0, t1, rel, roll, pitch, yaw };
  }, [st]);

  if (st === null) return <div className="page"><main className="content">{t("common.loading")}</main></div>;
  if (st === "missing" || !view) {
    return (
      <div className="page"><header className="topbar"><a className="btn icon" href="#/logs" aria-label={t("common.back")}><Icon name="back" /></a><h1>{t("logs.title")}</h1></header>
        <main className="content"><div className="banner">{t("logs.missing")}</div></main></div>
    );
  }
  const { meta: m, L } = st;
  const { t0, t1, rel, roll, pitch, yaw } = view;
  const sm = m.summary;
  const evColor = (lv: number) => (lv >= EventLevel.ERROR ? "var(--crit)" : lv === EventLevel.WARN ? "var(--warn)" : "var(--muted)");
  const marks = L.event.t.map((x, i) => ({ t: x - t0, color: evColor(L.event.level[i]), label: t(`events.${L.event.code[i]}`) }));
  const T = t1 - t0;
  const evText = (i: number) => {
    const k = `events.${L.event.code[i]}`;
    const s = t(k);
    return s === k ? L.event.name[i] : s;
  };

  return (
    <div className="page">
      <header className="topbar">
        <a className={vp.wide ? "btn" : "btn icon"} href="#/logs" aria-label={t("common.back")}><Icon name="back" />{vp.wide && t("logs.title")}</a>
        <div style={{ display: "flex", flexDirection: "column", gap: 2, flex: 1, minWidth: 0 }}>
          <h1 style={{ fontSize: vp.wide ? 22 : 19 }}>{m.drone} · {t("logs.logN", { id: m.logId })}</h1>
          <span className="small muted">{t("logs.downloadedOn", { d: new Date(m.downloadedAt).toLocaleString() })}{L.info ? ` · fw ${L.info.fw}` : ""}</span>
        </div>
        <button className="btn" onClick={() => void exportBin(m)}><Icon name="share" size={18} />{vp.phone ? ".bin" : t("logs.exportBin")}</button>
        <button className="btn" onClick={() => void exportCsv(m, L)}><Icon name="share" size={18} />{vp.phone ? "CSV" : t("logs.exportCsv")}</button>
      </header>
      <main className="content">
        {sm.dropped > 0 && <div className="banner warn">{t("logs.droppedN", { n: sm.dropped })}</div>}
        {(L.bad > 0 || m.truncated) && <div className="banner warn">{m.truncated ? `${t("logs.truncatedHint")} ` : ""}{L.bad > 0 ? t("logs.badN", { n: L.bad }) : ""}</div>}
        <div className="grid kpi">
          <div className="kpi"><span className="k">{t("logs.duration")}</span><span className="v">{dur(sm.durationS)}</span><span className="s">{t("logs.armedFor", { t: dur(sm.armedS) })}</span></div>
          <div className="kpi"><span className="k">{t("logs.maxHeight")}</span><span className="v">{sm.maxHeight != null ? fmt.alt(sm.maxHeight) : "–"}</span></div>
          <div className="kpi"><span className="k">{t("logs.minCell")}</span><span className="v">{sm.minCell != null ? `${sm.minCell.toFixed(2)} V` : "–"}</span></div>
          <div className="kpi"><span className="k">{t("logs.used")}</span><span className="v">{sm.usedMah != null ? `${Math.round(sm.usedMah)} mAh` : "–"}</span></div>
          <div className="kpi"><span className="k">{t("logs.maxCurrent")}</span><span className="v">{sm.maxCurrent != null ? `${sm.maxCurrent.toFixed(1)} A` : "–"}</span></div>
          <div className="kpi"><span className="k">{t("logs.events")}</span><span className="v">{sm.events}</span></div>
        </div>
        <div className={vp.wide ? "grid two" : "grid"}>
          <TimeChart compact={vp.phone} title={t("logs.chartHeight")} unit="m" t0={0} t1={T} marks={marks}
            series={[
              { t: rel(L.vert.t), v: L.vert.h, color: "var(--accent)", label: t("logs.height") },
              { t: rel(L.vert.t), v: L.vert.hSp.map((v, i) => (L.vert.flags[i] & 16 ? v : NaN)), color: "var(--text-2)", label: t("logs.setPoint"), dashed: true },
              { t: rel(L.vert.t), v: L.vert.tof, color: "var(--ok)", label: "ToF", dashed: true },
            ]} />
          <TimeChart compact={vp.phone} title={t("logs.chartBattery")} unit="V" rightUnit="A" t0={0} t1={T} marks={marks}
            series={[
              { t: rel(L.power.t), v: L.power.cell, color: "var(--ok)", label: t("logs.cell") },
              { t: rel(L.power.t), v: L.power.cellRest, color: "var(--text-2)", label: t("logs.cellRest"), dashed: true },
              { t: rel(L.power.t), v: L.power.current, color: "var(--warn)", label: t("telemetry.current"), right: true },
            ]} />
          <TimeChart compact={vp.phone} title={t("logs.chartAttitude")} unit="°" rightUnit="°" t0={0} t1={T} marks={marks} fixed={{ right: [0, 360] }}
            series={[
              { t: rel(L.att.t), v: roll, color: "var(--accent)", label: t("calib.level.roll") },
              { t: rel(L.att.t), v: pitch, color: "var(--warn)", label: t("calib.level.pitch") },
              { t: rel(L.att.t), v: yaw, color: "var(--text-2)", label: t("logs.heading"), dashed: true, right: true },
            ]} />
          <TimeChart compact={vp.phone} title={t("logs.chartHealth")} unit="mg" rightUnit="Hz" t0={0} t1={T} marks={marks}
            series={[
              { t: rel(L.est.t), v: L.est.vibMg, color: "var(--warn)", label: t("telemetry.vibration") },
              { t: rel(L.est.t), v: L.est.loopHz, color: "var(--accent)", label: t("telemetry.loop"), right: true },
            ]} />
        </div>
        <section className="card">
          <h2>{t("logs.timeline")}</h2>
          {L.event.n === 0 ? <span className="muted">–</span> : (
            <div className="tablewrap">
              <table className="msgs" style={vp.phone ? { minWidth: 0 } : undefined}>
                <thead><tr><th>{t("telemetry.time")}</th><th>{t("telemetry.level")}</th><th>{t("telemetry.message")}</th></tr></thead>
                <tbody>
                  {L.event.t.map((x, i) => (
                    <tr key={i}>
                      <td className="num muted">{fmt.mmss(Math.max(0, x - t0))}.{String(Math.floor(((x - t0) % 1) * 10)).replace("-", "")}</td>
                      <td style={{ color: evColor(L.event.level[i]) === "var(--muted)" ? "var(--ok)" : evColor(L.event.level[i]) }}>{t(`levels.${L.event.level[i]}`)}</td>
                      <td>{evText(i)}{L.event.arg[i] && [3, 22, 30, 31, 32].includes(L.event.code[i]) ? <span className="muted"> · {L.event.code[i] === 3 ? `${(L.event.arg[i] / 100).toFixed(1)} m` : L.event.code[i] === 22 ? t(`auto.${L.event.arg[i]}`) : `${(L.event.arg[i] / 1000).toFixed(2)} V`}</span> : null}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </section>
        <span className="small muted">{t("logs.decodeStats", { s: L.sectors, b: L.bad, u: L.unknown })} · {kb(m.size)} · {m.source === "ble" ? "Bluetooth" : "USB"}</span>
        <div className="btn-row">
          <button className="btn ghost" style={{ color: "var(--crit-text)" }} onClick={() => void logStore.remove(m.key).then(() => (location.hash = "#/logs"))}><Icon name="trash" size={18} />{t("logs.delete")}</button>
        </div>
      </main>
    </div>
  );
}

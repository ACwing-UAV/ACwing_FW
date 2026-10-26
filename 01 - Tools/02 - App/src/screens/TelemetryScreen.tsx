import { useState } from "react";
import { useT } from "../i18n";
import { drone } from "../link/drone";
import { flightLog } from "../link/flightLog";
import { ArmState, SENSOR, EventLevel } from "../protocol/ids";
import { useDrone, useSettings, useViewport } from "../ui/hooks";
import { Icon } from "../ui/icons";
import { Seg } from "../ui/kit";
import { fmt } from "../ui/format";

function Chart({ series, yMin, yMax, unit, label }: {
  series: { values: number[]; color: string; dashed?: boolean; scale?: [number, number] }[];
  yMin: number; yMax: number; unit: string; label: string;
}) {
  const W = 640, H = 200, x0 = 44, y0 = 12, y1 = 176;
  const n = Math.max(2, ...series.map((s) => s.values.length));
  const toPts = (vals: number[], lo: number, hi: number) =>
    vals.map((v, i) => `${(x0 + ((W - x0) * i) / (n - 1)).toFixed(1)},${(y1 - ((Math.min(hi, Math.max(lo, v)) - lo) / (hi - lo || 1)) * (y1 - y0)).toFixed(1)}`).join(" ");
  const grid = [0, 0.5, 1].map((f) => ({ y: y1 - f * (y1 - y0), v: yMin + f * (yMax - yMin) }));
  return (
    <svg viewBox={`0 0 ${W} ${H}`} role="img" aria-label={label} preserveAspectRatio="none">
      {grid.map((g) => (
        <g key={g.y}>
          <line x1={x0} x2={W} y1={g.y} y2={g.y} stroke="var(--line)" />
          <text x="0" y={g.y + 4} fontSize="11" fill="var(--muted)">{g.v.toFixed(Math.abs(yMax - yMin) < 5 ? 1 : 0)} {unit}</text>
        </g>
      ))}
      <text x={x0} y={H - 4} fontSize="11" fill="var(--muted)">−2:00</text>
      {series.map((s, i) => (
        <polyline key={i} fill="none" stroke={s.color} strokeWidth={s.dashed ? 1.5 : 2.5} strokeDasharray={s.dashed ? "5 4" : undefined}
          points={toPts(s.values, s.scale?.[0] ?? yMin, s.scale?.[1] ?? yMax)} />
      ))}
    </svg>
  );
}

export function TelemetryScreen() {
  const { t } = useT();
  const s = useDrone();
  useSettings();
  const vp = useViewport();
  const [tab, setTab] = useState<"alt" | "batt" | "motors">("alt");
  const f = s.flight, p = s.power, n = s.nav, y = s.system, caps = s.caps;
  const gps = !!(caps && caps.sensors & SENSOR.GNSS);
  const h = s.history;
  const armed = f?.arm === ArmState.ARMED;

  const altVals = h.map((x) => fmt.altNum(x.alt));
  const altMax = Math.max(3, ...altVals) * 1.15;
  const vbat = h.map((x) => x.vbat);
  const vLo = Math.min(...vbat, p ? p.vbatV : 0) - 0.1, vHi = Math.max(...vbat, p ? p.vbatV : 1) + 0.1;
  const altChart = (
    <figure className="card chart" style={{ margin: 0 }}>
      <figcaption style={{ display: "flex", justifyContent: "space-between", fontWeight: 600 }}><span>{t("telemetry.altChart")}</span><span className="small muted" style={{ fontWeight: 400 }}>{t("telemetry.last2min")}</span></figcaption>
      <Chart label={t("telemetry.altChart")} unit={fmt.altUnit()} yMin={0} yMax={altMax}
        series={[{ values: altVals, color: "var(--accent)" }, { values: h.map((x) => x.vz), color: "var(--warn)", dashed: true, scale: [-3, 3] }]} />
      <div className="legend"><span><i style={{ background: "var(--accent)" }} />{t("telemetry.altitude")}</span><span><i className="dash" />{t("telemetry.vertical")} (±3 m/s)</span></div>
    </figure>
  );
  const battChart = (
    <figure className="card chart" style={{ margin: 0 }}>
      <figcaption style={{ display: "flex", justifyContent: "space-between", fontWeight: 600 }}><span>{t("telemetry.battChart")}</span><span className="small muted" style={{ fontWeight: 400 }}>{t("telemetry.last2min")}</span></figcaption>
      <Chart label={t("telemetry.battChart")} unit="V" yMin={vLo} yMax={vHi}
        series={[{ values: vbat, color: "var(--ok)" }, { values: h.map((x) => x.current), color: "var(--warn)", dashed: true, scale: [0, Math.max(5, ...h.map((x) => x.current)) * 1.2] }]} />
      <div className="legend"><span><i style={{ background: "var(--ok)" }} />{t("telemetry.packV")}</span><span><i className="dash" />{t("telemetry.currentA")}</span></div>
    </figure>
  );
  const motorsCard = s.motors && (
    <section className="card">
      <h2 style={{ fontSize: 15, fontFamily: "var(--font)" }}>{t("telemetry.motors")}</h2>
      {s.motors.motors.map((m, i) => (
        <div className="motor-row" key={i}>
          <b>M{i + 1}</b>
          <span className="bar"><span style={{ width: `${m.outPct}%` }} /></span>
          <span className="num">{m.rpm !== null ? m.rpm.toLocaleString() : `${m.outPct}%`}</span>
          <span className="muted">{m.tempC !== null ? `${m.tempC} °C` : ""}</span>
        </div>
      ))}
      <span className="small muted">{s.motors.motors.some((m) => m.rpm !== null) ? t("telemetry.motorsNote") : t("telemetry.motorsNoRpm")}</span>
    </section>
  );

  const facts = (rows: [string, string][]) => (
    <dl className="facts">{rows.map(([k, v]) => <div key={k}><dt>{k}</dt><dd>{v}</dd></div>)}</dl>
  );
  const att = s.attitude;
  const attitudeCard = (
    <section className="card"><h2 style={{ fontSize: 15, fontFamily: "var(--font)" }}>{t("telemetry.attitude")}</h2>
      {facts([
        [t("telemetry.rollPitch"), att ? `${fmt.signed(att.rollRight)}° / ${fmt.signed(att.pitchUp)}°` : "–"],
        [t("telemetry.yawHeading"), fmt.deg(f?.headingDeg, 1)],
        [t("telemetry.uncertainty"), y ? `± ${y.attUncDeg.toFixed(1)}°` : "–"],
        [t("telemetry.vibration"), y ? `${y.vibrationG.toFixed(2)} g` : "–"],
      ])}
    </section>
  );
  const posCard = (
    <section className="card">
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", gap: 8 }}>
        <h2 style={{ fontSize: 15, fontFamily: "var(--font)" }}>{t("telemetry.position")}</h2>
        {gps && <span className="small muted">{t("telemetry.gpsOnlyNote")}</span>}
      </div>
      {facts([
        ...(gps ? [[t("telemetry.latLon"), n ? `${n.lat.toFixed(5)} / ${n.lon.toFixed(5)}` : "–"], ["GNSS", n ? `${n.fix === 3 ? "3D" : n.fix === 2 ? "2D" : "–"} · ${n.sats} · ${n.hdop.toFixed(1)}` : "–"]] as [string, string][] : []),
        [t("telemetry.velNed"), n ? n.velNed.map((v) => v.toFixed(1)).join(" / ") : "–"],
        [t("telemetry.tofAlt"), n?.tofM != null ? fmt.alt(n.tofM, 2) : t("common.outOfRange")],
        [t("telemetry.opticalFlow"), n?.flowQ != null ? t("telemetry.quality", { q: n.flowQ }) : t("common.na")],
        [t("telemetry.baro"), n ? `${n.baroHpa.toFixed(1)} hPa` : "–"],
      ])}
    </section>
  );
  const powerCard = (
    <section className="card"><h2 style={{ fontSize: 15, fontFamily: "var(--font)" }}>{t("telemetry.power")}</h2>
      {facts([
        [t("telemetry.packVoltage"), p ? `${p.vbatV.toFixed(2)} V` : "–"],
        [t("telemetry.current"), p ? `${p.currentA.toFixed(1)} A` : "–"],
        [t("telemetry.powerDraw"), p ? `${Math.round(p.vbatV * p.currentA)} W` : "–"],
        [t("telemetry.usedMah"), p ? `${p.usedMah} mAh` : "–"],
        [t("telemetry.cellAvg"), p ? `${p.cellV.toFixed(2)} V` : "–"],
        [t("telemetry.thresholds"), `${drone.param("battWarn").toFixed(2)} / ${drone.param("battCrit").toFixed(2)} V`],
      ])}
    </section>
  );
  const sysCard = (
    <section className="card"><h2 style={{ fontSize: 15, fontFamily: "var(--font)" }}>{t("telemetry.system")}</h2>
      {facts([
        [t("telemetry.elrs"), y?.lq != null ? `${y.rssi} dBm / ${y.snr} dB` : t("common.na")],
        [t("telemetry.rxRate"), `${(s.rxBytesPerS / 1000).toFixed(1)} kB/s`],
        [t("telemetry.bleDrop"), s.status ? String(s.status.bleDrop) : "–"],
        [t("telemetry.cpu"), y ? `${y.cpuPct}%` : "–"],
        [t("telemetry.loop"), y ? `${y.loopHz} Hz · jitter ${y.jitterUs} µs` : "–"],
        [t("telemetry.flashLog"), y ? `${y.logPct}%` : "–"],
        [t("telemetry.imuTemp"), y ? `${y.imuTempC.toFixed(1)} °C` : "–"],
        [t("telemetry.accClip"), y ? String(y.accClip) : "–"],
      ])}
    </section>
  );
  const messages = (
    <section className="card"><h2 style={{ fontSize: 15, fontFamily: "var(--font)" }}>{t("telemetry.messages")}</h2>
      <div className="tablewrap">
        <table className="msgs" style={vp.phone ? { minWidth: 0 } : undefined}>
          <thead><tr><th>{t("telemetry.time")}</th><th>{t("telemetry.level")}</th><th>{t("telemetry.message")}</th></tr></thead>
          <tbody>
            {s.events.slice(0, 30).map((e) => (
              <tr key={e.id}>
                <td className="muted">{new Date(e.at).toLocaleTimeString()}</td>
                <td style={{ color: e.level >= EventLevel.ERROR ? "var(--crit)" : e.level === EventLevel.WARN ? "var(--warn)" : "var(--ok)" }}>{t(`levels.${e.level}`)}</td>
                <td>{t(`events.${e.code}`) !== `events.${e.code}` ? t(`events.${e.code}`) : e.text}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </section>
  );

  return (
    <div className="page">
      <header className="topbar">
        <a className="btn icon" href="#/cockpit" aria-label={t("common.back")}><Icon name="back" /></a>
        <h1>{t("telemetry.title")}</h1>
        {armed && <span className="chip armed">{t("cockpit.armed")}</span>}
        <span className="small muted">{caps?.name ?? s.deviceName}{f ? ` · ${t(`modes.${f.mode}`)} · ${fmt.mmss(f.flightTimeS)}` : ""}</span>
        <span className="spacer" />
        <button className="btn" onClick={() => flightLog.export(caps?.name)}><Icon name="download" size={18} />{t("telemetry.exportCsv")}</button>
      </header>
      <main className="content">
        {!f && <div className="banner">{t("telemetry.noData")}</div>}
        <div className="grid kpi">
          <div className="kpi"><span className="k">{t("telemetry.battery")}</span><span className="v">{p ? `${p.pct}%` : "–"}</span><span className="s">{p ? `${p.vbatV.toFixed(2)} V · ${p.cellV.toFixed(2)} V/cell` : ""}</span></div>
          <div className="kpi"><span className="k">{t("cockpit.relAlt")}</span><span className="v">{fmt.alt(f?.altRelM)}</span><span className="s">{t("telemetry.vertShort", { v: fmt.vz(f?.vzMs) })}</span></div>
          <div className="kpi"><span className="k">{t("telemetry.groundSpeed")}</span><span className="v">{fmt.speed(f?.groundSpeedMs)}</span><span className="s">{t("telemetry.headingShort", { h: Math.round(f?.headingDeg ?? 0) })}</span></div>
          {gps && <div className="kpi"><span className="k">{t("telemetry.homeDist")}</span><span className="v">{fmt.dist(f?.homeDistM)}</span><span className="s">{t("common.gpsOnly")}</span></div>}
          {!gps && <div className="kpi"><span className="k">{t("telemetry.tofAlt")}</span><span className="v">{n?.tofM != null ? fmt.alt(n.tofM, 2) : "–"}</span><span className="s">{n?.flowQ != null ? `${t("telemetry.opticalFlow")} ${n.flowQ}%` : ""}</span></div>}
          <div className="kpi"><span className="k">{t("telemetry.radioLink")}</span><span className="v">{y?.lq != null ? `LQ ${y.lq}%` : s.rssi !== null ? `${s.rssi} dBm` : "–"}</span><span className="s">{y?.lq != null ? `${y.rssi} dBm · SNR ${y.snr} dB` : "Bluetooth"}</span></div>
          <div className="kpi"><span className="k">{t("telemetry.estRemaining")}</span><span className="v">{p?.remainS != null ? `≈ ${Math.round(p.remainS / 60)} min` : "–"}</span><span className="s">{gps && f?.rthEtaS != null ? t("telemetry.rthNeeds", { t: fmt.mmss(f.rthEtaS) }) : ""}</span></div>
        </div>
        {vp.phone ? (
          <>
            <Seg full label={t("telemetry.altChart")} value={tab} onChange={setTab}
              options={[{ v: "alt", label: t("telemetry.chartAlt") }, { v: "batt", label: t("telemetry.chartBatt") }, { v: "motors", label: t("telemetry.chartMotors") }]} />
            {tab === "alt" ? altChart : tab === "batt" ? battChart : motorsCard}
            {attitudeCard}
            {posCard}
            <div className="card tight">
              <details className="acc"><summary>{t("telemetry.power")}<span className="spacer" style={{ flex: 1 }} /><span className="small muted">{p ? `${p.currentA.toFixed(1)} A · ${p.usedMah} mAh` : ""}</span><Icon name="down" size={16} /></summary><div>{powerCard}</div></details>
              <details className="acc"><summary>{t("telemetry.system")}<span className="spacer" style={{ flex: 1 }} /><span className="small muted">{y ? `CPU ${y.cpuPct}%` : ""}</span><Icon name="down" size={16} /></summary><div>{sysCard}</div></details>
              {tab !== "motors" && motorsCard && <details className="acc"><summary>{t("telemetry.motors")}<span className="spacer" style={{ flex: 1 }} /><Icon name="down" size={16} /></summary><div>{motorsCard}</div></details>}
            </div>
            {messages}
          </>
        ) : (
          <>
            <div className="grid two">{altChart}{battChart}</div>
            <div className="grid auto">{attitudeCard}{posCard}{motorsCard}{powerCard}{sysCard}</div>
            {messages}
          </>
        )}
        <span className="small muted">{t("telemetry.recording")}: {flightLog.count} · {(s.rxBytesPerS / 1000).toFixed(1)} kB/s · CRC {s.crcErrors}</span>
      </main>
    </div>
  );
}

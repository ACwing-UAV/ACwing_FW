import { useEffect, useState } from "react";
import { useT } from "../i18n";
import { drone } from "../link/drone";
import { CalState, CalType, SENSOR, FrameType } from "../protocol/ids";
import type { CalibData } from "../protocol/messages";
import { go, useDrone } from "../ui/hooks";
import { Icon, Mark } from "../ui/icons";
import { toast, useCommand } from "../ui/kit";
import { fmt } from "../ui/format";
import { BACK, Bullets, DroneSide, Facts, Failed, Success, WizardShell, useAbortOnLeave, useCal } from "./Wizard";

type D<K extends CalibData["kind"]> = Extract<CalibData, { kind: K }>;
const done = (st?: number) => st === CalState.DONE || st === CalState.SAVED;

function SaveRow({ onDiscard, saveLabel, next }: { onDiscard: () => void; saveLabel?: string; next?: string }) {
  const { t } = useT();
  const run = useCommand();
  return (
    <div className="btn-row end">
      <button className="btn big" onClick={onDiscard}>{t("common.discard")}</button>
      <button className="btn primary big" onClick={async () => { if (await run(drone.calSave(), t("calib.successful"))) location.hash = next ?? BACK; }}>
        {saveLabel ?? t("common.save")}
      </button>
    </div>
  );
}

/* ================================================================ */
/* Compass                                                           */
/* ================================================================ */

function ring(coverage: boolean[]) {
  const segs: { d: string; on: boolean }[] = [];
  const cx = 135, cy = 135, rings = [[118, 132], [100, 114], [82, 96]];
  const p = (r: number, a: number) => `${(cx + r * Math.cos(a)).toFixed(1)} ${(cy + r * Math.sin(a)).toFixed(1)}`;
  for (let r = 0; r < 3; r++)
    for (let i = 0; i < 12; i++) {
      const a0 = ((i * 30 + 1.5 - 90) * Math.PI) / 180, a1 = (((i + 1) * 30 - 1.5 - 90) * Math.PI) / 180;
      const [ri, ro] = rings[r];
      segs.push({ d: `M${p(ro, a0)} A${ro} ${ro} 0 0 1 ${p(ro, a1)} L${p(ri, a1)} A${ri} ${ri} 0 0 0 ${p(ri, a0)} Z`, on: !!coverage[r * 12 + i] });
    }
  return segs;
}

export function CompassWizard() {
  const { t } = useT();
  const s = useDrone();
  const run = useCommand();
  const cal = useCal(CalType.COMPASS);
  const [which, setWhich] = useState(0);
  useAbortOnLeave(CalType.COMPASS);
  const hasGpsMag = !s.caps || !!(s.caps.sensors & SENSOR.MAG_GPS);
  const d = cal?.data as D<"compass"> | undefined;
  const step = !cal || cal.state === CalState.FAILED ? 0 : done(cal.state) ? 2 : 1;
  const steps = [t("calib.compass.prepare"), t("calib.compass.rotate"), t("calib.compass.result")];
  const restart = () => drone.calAbort();
  const pct = cal?.progress ?? 0;
  const segs = ring(d?.coverage ?? []);
  const missing = d ? [[0, t("calib.compass.spinFlat")], [1, t("calib.compass.noseUpDown")], [2, t("calib.compass.rollLR")]]
    .filter(([r]) => d.coverage.slice((r as number) * 12, (r as number) * 12 + 12).some((x) => !x)).map(([, n]) => n).join(", ") : "";

  const visual = step === 0 ? <Mark className="wiz-mark" alt="" /> : step === 1 ? (
    <svg width="100%" style={{ maxWidth: 380 }} viewBox="0 0 270 270" role="img" aria-label={`${t("calib.compass.coverage")} ${pct}%`}>
      {segs.map((g, i) => <path key={i} d={g.d} fill={g.on ? "var(--accent)" : "var(--line)"} />)}
      <text x="135" y="132" textAnchor="middle" fill="var(--text)" fontFamily="Chakra Petch, sans-serif" fontWeight="700" fontSize="40">{pct}%</text>
      <text x="135" y="158" textAnchor="middle" fill="var(--muted)" fontSize="13">{t("calib.compass.coverage")}</text>
    </svg>
  ) : (
    <>
      <Success text={t("calib.successful")} />
      <div className="card tight" style={{ width: "100%" }}>
        {d?.qualityGps != null && (
          <div className="row first"><div className="grow"><span className="label">{t("calib.compass.gpsCompass")}</span><span className="hint">{t("calib.compass.interference", { i: d.interfGps ?? 0 })}</span></div>
            <span className="num" style={{ fontSize: 20, fontWeight: 700 }}>{d.qualityGps}%</span><span style={{ color: "var(--ok)", width: 80 }}>{d.qualityGps >= 90 ? t("calib.excellent") : d.qualityGps >= 70 ? t("calib.good") : t("calib.poor")}</span></div>
        )}
        {d?.qualityFc != null && (
          <div className={`row${d.qualityGps == null ? " first" : ""}`}><div className="grow"><span className="label">{t("calib.compass.fcCompass")}</span><span className="hint">{t("calib.compass.interference", { i: d.interfFc })}</span></div>
            <span className="num" style={{ fontSize: 20, fontWeight: 700 }}>{d.qualityFc}%</span><span style={{ color: d.qualityFc >= 70 ? "var(--ok)" : "var(--warn)", width: 80 }}>{d.qualityFc >= 90 ? t("calib.excellent") : d.qualityFc >= 70 ? t("calib.good") : t("calib.poor")}</span></div>
        )}
      </div>
    </>
  );

  const panel = step === 0 ? (
    <>
      <h2>{steps[0]}</h2>
      {cal?.state === CalState.FAILED && <Failed reason={cal.reason} onRetry={restart} />}
      <Bullets items={[t("calib.compass.p1"), t("calib.compass.p2"), t("calib.compass.p3")]} />
      {hasGpsMag && (
        <fieldset className="card" style={{ margin: 0 }}>
          <legend className="small muted" style={{ padding: "0 6px" }}>{t("calib.compass.which")}</legend>
          {[t("calib.compass.both"), t("calib.compass.fcOnly"), t("calib.compass.gpsOnly")].map((l, i) => (
            <label key={l} style={{ display: "flex", gap: 10, alignItems: "center", minHeight: 36 }}>
              <input type="radio" name="which" checked={which === i} onChange={() => setWhich(i)} />{l}
            </label>
          ))}
        </fieldset>
      )}
      <span className="grow" />
      <div className="btn-row end"><button className="btn primary big" onClick={() => run(drone.calStart(CalType.COMPASS, hasGpsMag ? which : 1))}>{t("common.start")}</button></div>
    </>
  ) : step === 1 ? (
    <>
      <h2>{t("calib.compass.rotate")}</h2>
      <p>{t("calib.compass.rotateIntro")}</p>
      <div className="tiles">
        {[["noseUpDown", "M4 12a8 8 0 0 1 14-5.3M18 3v4h-4M8 10h8v4H8z"], ["rollLR", "M20 12a8 8 0 0 1-14 5.3M6 21v-4h4M10 7h4v10h-4z"], ["spinFlat", "M4 14a8 3 0 1 0 16 0 8 3 0 1 0-16 0M17 9l3 2-3 2"]].map(([k, path]) => (
          <div key={k} className="tile"><svg width="30" height="30" viewBox="0 0 24 24" fill="none" stroke="var(--accent)" strokeWidth="1.8" aria-hidden="true"><path d={path} /></svg>{t(`calib.compass.${k}`)}</div>
        ))}
      </div>
      {missing && <div className="banner" role="status">{t("calib.compass.missing", { what: missing })}</div>}
      <span className="grow" />
      <span className="small muted">{t("calib.compass.auto")}</span>
      <div className="btn-row end"><button className="btn big" onClick={restart}>{t("common.restart")}</button></div>
    </>
  ) : (
    <>
      <h2>{t("calib.compass.result")}</h2>
      {d?.qualityGps != null && d.qualityFc != null && (
        <div style={{ display: "flex", flexDirection: "column", gap: 6 }}>
          <label htmlFor="prim" className="small muted">{t("calib.compass.primary")}</label>
          <select id="prim" className="select" style={{ height: 48 }} value={Math.round(drone.param("compassPrimary"))} onChange={(e) => run(drone.setParam("compassPrimary", +e.target.value))}>
            <option value={1}>{t("actions.compassGps")}</option><option value={0}>{t("actions.compassFc")}</option>
          </select>
        </div>
      )}
      <button className="btn ghost" style={{ alignSelf: "flex-start", padding: 0, color: "var(--accent)" }} onClick={() => toast(t("calib.compass.advanced"))}>{t("calib.compass.advanced")}</button>
      <span className="grow" />
      <SaveRow onDiscard={restart} saveLabel={t("calib.saveToDrone")} />
    </>
  );
  return <WizardShell title={t("calib.names.4")} steps={steps} current={step} visual={visual} panel={panel} />;
}

/* ================================================================ */
/* Accelerometer                                                     */
/* ================================================================ */

const POS_TF = [
  "rotate(0 40 30)", "rotate(180 40 30)", "rotate(90 40 30)", "rotate(-90 40 30)",
  "translate(40 30) scale(0.45 1) rotate(-90) translate(-40 -30)", "translate(40 30) scale(0.45 1) rotate(90) translate(-40 -30)",
];

export function AccelWizard() {
  const { t } = useT();
  const run = useCommand();
  const cal = useCal(CalType.ACCEL);
  useAbortOnLeave(CalType.ACCEL);
  const d = cal?.data as D<"accel"> | undefined;
  const step = !cal || cal.state === CalState.FAILED ? 0 : done(cal.state) ? 2 : 1;
  const steps = [t("calib.accel.prepare"), t("calib.accel.positions"), t("calib.accel.result")];
  const cur = d?.current ?? 0;
  const visual = step === 0 ? <Mark className="wiz-mark" alt="" /> : step === 1 ? (
    <>
      <span className="small muted" style={{ letterSpacing: "0.08em", textTransform: "uppercase" }}>{t("calib.accel.position", { n: cur + 1 })}</span>
      <span className="display" style={{ fontSize: 28, fontWeight: 700 }}>{t(`calib.accel.pos.${cur}`)}</span>
      <DroneSide transform={POS_TF[cur]} size={240} label={t(`calib.accel.pos.${cur}`)} />
      <span className="muted">{t("calib.accel.arrowNose")}</span>
      <div className="progress" style={{ maxWidth: 360 }}><span style={{ width: `${d?.stillPct ?? 0}%` }} /></div>
    </>
  ) : (
    <>
      <Success text={t("calib.accel.allCaptured")} />
      <Facts rows={[
        [t("calib.accel.offset"), d ? `${d.offsetMg.join(" / ")} mg` : "–"],
        [t("calib.accel.scale"), d ? d.scale.map((v) => v.toFixed(3)).join(" / ") : "–"],
        [t("calib.accel.fitErr"), d ? `${d.fitErrPct.toFixed(1)}%` : "–"],
        [t("calib.accel.quality"), <span style={{ color: "var(--ok)" }}>{(cal?.quality ?? 0) >= 90 ? t("calib.excellent") : t("calib.good")}</span>],
      ]} />
    </>
  );
  const panel = step === 0 ? (
    <>
      <h2>{steps[0]}</h2>
      {cal?.state === CalState.FAILED && <Failed reason={cal.reason} onRetry={() => drone.calAbort()} />}
      <p>{t("calib.accel.intro")}</p>
      <Bullets items={[t("calib.accel.p1"), t("calib.accel.p2"), t("calib.accel.p3")]} />
      <span className="grow" />
      <div className="btn-row end"><button className="btn primary big" onClick={() => run(drone.calStart(CalType.ACCEL))}>{t("common.start")}</button></div>
    </>
  ) : step === 1 ? (
    <>
      <h2>{steps[1]}</h2>
      <div className="tiles">
        {[0, 1, 2, 3, 4, 5].map((i) => {
          const ok = d ? !!(d.doneMask & (1 << i)) : false, now = i === cur;
          return (
            <div key={i} className={`tile${now ? " cur" : ok ? " done" : ""}`}>
              <DroneSide transform={POS_TF[i]} size={64} />
              <span>{t(`calib.accel.pos.${i}`)}</span>
              <b style={{ color: ok ? "var(--ok)" : now ? "var(--accent)" : "var(--muted)" }}>{ok ? t("calib.accel.done") : now ? t("calib.accel.now") : t("calib.accel.todo")}</b>
            </div>
          );
        })}
      </div>
      <span className="grow" />
      <div className="btn-row end">
        <button className="btn big" onClick={() => drone.calAbort()}>{t("common.back")}</button>
        <button className="btn primary big" onClick={() => run(drone.calCapture())}>{t("calib.accel.capture")}</button>
      </div>
    </>
  ) : (
    <>
      <h2>{steps[2]}</h2>
      <p>{t("calib.accel.next")}</p>
      <span className="grow" />
      <SaveRow onDiscard={() => drone.calAbort()} next={`#/calib/${CalType.LEVEL}`} />
    </>
  );
  return <WizardShell title={t("calib.names.1")} steps={steps} current={step} visual={visual} panel={panel} />;
}

/* ================================================================ */
/* Level trim                                                        */
/* ================================================================ */

export function LevelWizard() {
  const { t } = useT();
  const run = useCommand();
  const cal = useCal(CalType.LEVEL);
  useAbortOnLeave(CalType.LEVEL);
  const d = cal?.data as D<"level"> | undefined;
  const step = !cal || cal.state === CalState.FAILED ? 0 : done(cal.state) ? 2 : 1;
  const steps = [t("calib.level.prepare"), t("calib.level.measure"), t("calib.level.result")];
  const bx = 120 + Math.max(-90, Math.min(90, (d?.rollDeg ?? 0) * 30)), by = 120 - Math.max(-90, Math.min(90, (d?.pitchDeg ?? 0) * 30));
  const trim = (idx: 0 | 1, delta: number) => {
    if (!d) return;
    const v = +((idx === 0 ? d.trimRollDeg : d.trimPitchDeg) + delta).toFixed(1);
    run(drone.calSetRef(CalType.LEVEL, idx, v));
  };
  const bubble = (
    <svg width="100%" style={{ maxWidth: 400 }} viewBox="0 0 240 240" role="img" aria-label={`${t("calib.level.roll")} ${d?.rollDeg.toFixed(1) ?? 0}°, ${t("calib.level.pitch")} ${d?.pitchDeg.toFixed(1) ?? 0}°`}>
      <circle cx="120" cy="120" r="110" fill="var(--surface-2)" stroke="var(--line-2)" strokeWidth="2" />
      <circle cx="120" cy="120" r="70" fill="none" stroke="var(--line)" strokeWidth="1.5" />
      <circle cx="120" cy="120" r="30" fill="none" stroke="var(--line-3)" strokeWidth="1.5" strokeDasharray="4 4" />
      <line x1="10" y1="120" x2="230" y2="120" stroke="var(--line)" /><line x1="120" y1="10" x2="120" y2="230" stroke="var(--line)" />
      <circle cx={bx} cy={by} r="16" fill="var(--accent)" fillOpacity="0.85" />
      <text x="120" y="226" textAnchor="middle" fill="var(--muted)" fontSize="11">{t("calib.level.ring")}</text>
    </svg>
  );
  const visual = step === 0 ? <Mark className="wiz-mark" alt="" /> : step === 1 ? bubble : (
    <>
      <Success text={t("calib.level.measured")} />
      <div className="card tight" style={{ width: "100%" }}>
        {([0, 1] as const).map((idx) => (
          <div key={idx} className={`row${idx === 0 ? " first" : ""}`} style={{ minHeight: 64 }}>
            <span className="grow label">{idx === 0 ? t("calib.level.rollTrim") : t("calib.level.pitchTrim")}</span>
            <button className="btn icon" aria-label={`${idx === 0 ? t("calib.level.rollTrim") : t("calib.level.pitchTrim")} −0.1`} onClick={() => trim(idx, -0.1)}>−</button>
            <span className="num" style={{ width: 70, textAlign: "center", fontSize: 18, fontWeight: 700 }}>{fmt.signed(idx === 0 ? d?.trimRollDeg ?? 0 : d?.trimPitchDeg ?? 0)}°</span>
            <button className="btn icon" aria-label={`${idx === 0 ? t("calib.level.rollTrim") : t("calib.level.pitchTrim")} +0.1`} onClick={() => trim(idx, 0.1)}>+</button>
          </div>
        ))}
      </div>
    </>
  );
  const panel = step === 0 ? (
    <>
      <h2>{steps[0]}</h2>
      <p>{t("calib.level.intro")}</p>
      <Bullets items={[t("calib.level.p1"), t("calib.level.p2"), t("calib.level.p3")]} />
      <span className="grow" />
      <div className="btn-row end"><button className="btn primary big" onClick={() => run(drone.calStart(CalType.LEVEL))}>{t("common.start")}</button></div>
    </>
  ) : step === 1 ? (
    <>
      <h2>{steps[1]}</h2>
      <p>{t("calib.level.dontTouch")}</p>
      <div className="grid two" style={{ gap: 10 }}>
        <div className="kpi"><span className="k">{t("calib.level.roll")}</span><span className="v">{fmt.signed(d?.rollDeg ?? 0)}°</span></div>
        <div className="kpi"><span className="k">{t("calib.level.pitch")}</span><span className="v">{fmt.signed(d?.pitchDeg ?? 0)}°</span></div>
      </div>
      <span className="muted">{t("calib.level.averaging", { s: Math.ceil((d?.remainingMs ?? 0) / 1000) })}</span>
      <div className="progress"><span style={{ width: `${cal?.progress ?? 0}%` }} /></div>
      <span className="grow" />
      <div className="btn-row end"><button className="btn big" onClick={() => drone.calAbort()}>{t("common.back")}</button></div>
    </>
  ) : (
    <>
      <h2>{steps[2]}</h2>
      <p>{t("calib.level.hint")}</p>
      <span className="grow" />
      <SaveRow onDiscard={() => drone.calAbort()} />
    </>
  );
  return <WizardShell title={t("calib.names.2")} steps={steps} current={step} visual={visual} panel={panel} />;
}

/* ================================================================ */
/* Gyroscope                                                         */
/* ================================================================ */

export function GyroWizard() {
  const { t } = useT();
  const run = useCommand();
  const cal = useCal(CalType.GYRO);
  useAbortOnLeave(CalType.GYRO);
  useEffect(() => {
    if (!drone.getSnapshot().calib || drone.getSnapshot().calib?.type !== CalType.GYRO) run(drone.calStart(CalType.GYRO));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const d = cal?.data as D<"gyro"> | undefined;
  const step = done(cal?.state) ? 1 : 0;
  const steps = [t("calib.gyro.keepStill"), t("calib.gyro.result")];
  const left = Math.ceil((d?.remainingMs ?? 5000) / 1000);
  const frac = (cal?.progress ?? 0) / 100;
  const visual = step === 0 ? (
    <>
      <svg width="240" height="240" viewBox="0 0 200 200" role="img" aria-label={`${5 - left} / 5 s`}>
        <circle cx="100" cy="100" r="84" fill="none" stroke="var(--line)" strokeWidth="14" />
        <circle cx="100" cy="100" r="84" fill="none" stroke="var(--accent)" strokeWidth="14" strokeLinecap="round" strokeDasharray={`${527.8 * frac} 527.8`} transform="rotate(-90 100 100)" />
        <text x="100" y="100" textAnchor="middle" fill="var(--text)" fontFamily="Chakra Petch, sans-serif" fontWeight="700" fontSize="40">{left} s</text>
        <text x="100" y="126" textAnchor="middle" fill="var(--muted)" fontSize="13">{t("calib.gyro.ofFive")}</text>
      </svg>
      <span className="small muted">{t("calib.gyro.band")}</span>
    </>
  ) : (
    <>
      <Success text={t("calib.gyro.updated")} />
      <Facts rows={[
        [t("calib.gyro.bias"), d ? `${d.biasDps.map((v) => v.toFixed(2)).join(" / ")} °/s` : "–"],
        [t("calib.gyro.noise"), d ? `${d.noiseDps.toFixed(2)} °/s` : "–"],
        [t("calib.gyro.temp"), d ? `${d.tempC.toFixed(1)} °C` : "–"],
        [t("calib.accel.quality"), <span style={{ color: "var(--ok)" }}>{t("calib.excellent")}</span>],
      ]} />
    </>
  );
  const panel = step === 0 ? (
    <>
      <h2>{steps[0]}</h2>
      {cal?.state === CalState.FAILED && <Failed reason={cal.reason} onRetry={() => run(drone.calStart(CalType.GYRO))} />}
      <p>{t("calib.gyro.intro")}</p>
      <span className="grow" />
    </>
  ) : (
    <>
      <h2>{steps[1]}</h2>
      <span className="grow" />
      <div className="btn-row end">
        <button className="btn big" onClick={async () => { await drone.calAbort(); run(drone.calStart(CalType.GYRO)); }}>{t("common.repeat")}</button>
        <button className="btn primary big" onClick={async () => { if (await run(drone.calSave(), t("calib.successful"))) go("settings/calibration"); }}>{t("common.save")}</button>
      </div>
    </>
  );
  return <WizardShell title={t("calib.names.3")} steps={steps} current={step} visual={visual} panel={panel} />;
}

/* ================================================================ */
/* Optical flow and ToF                                              */
/* ================================================================ */

export function FlowWizard() {
  const { t } = useT();
  const run = useCommand();
  const cal = useCal(CalType.FLOW_TOF);
  useAbortOnLeave(CalType.FLOW_TOF);
  useEffect(() => {
    run(drone.calStart(CalType.FLOW_TOF));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const d = cal?.data as D<"flow"> | undefined;
  const step = done(cal?.state) ? 2 : cal && cal.step >= 2 ? 1 : 0;
  const steps = [t("calib.flow.ground"), t("calib.flow.scale"), t("calib.flow.result")];
  const visual = step === 0 ? (
    <>
      <span className="muted">{t("calib.flow.readsNow")}</span>
      <span className="num" style={{ fontSize: 64, fontWeight: 700 }}>{d ? `${d.tofMm} mm` : "–"}</span>
      <span className="muted">{t("calib.flow.storedAsGround")}</span>
    </>
  ) : step === 1 ? (
    <figure style={{ margin: 0, width: "100%", display: "flex", flexDirection: "column", gap: 8 }}>
      <figcaption style={{ fontWeight: 600 }}>{t("calib.flow.chart")}</figcaption>
      <FlowChart match={d?.matchPct ?? 0} />
      <div className="legend"><span><i style={{ background: "var(--accent)" }} />{t("calib.flow.gyro")}</span><span><i className="dash" />{t("calib.flow.flow")}</span></div>
    </figure>
  ) : (
    <>
      <Success text={t("calib.flow.calibrated")} />
      <Facts rows={[
        [t("calib.flow.tofOffset"), d ? `${d.tofOffMm >= 0 ? "+" : ""}${d.tofOffMm} mm` : "–"],
        [t("calib.flow.scaleXY"), d ? `${d.scaleX.toFixed(2)} / ${d.scaleY.toFixed(2)}` : "–"],
        [t("calib.flow.gyroMatch"), d ? `${d.matchPct}%` : "–"],
        [t("calib.accel.quality"), <span style={{ color: "var(--ok)" }}>{t("calib.good")}</span>],
      ]} />
    </>
  );
  const panel = step === 0 ? (
    <>
      <h2>{steps[0]}</h2>
      {cal?.state === CalState.FAILED && <Failed reason={cal.reason} onRetry={() => run(drone.calStart(CalType.FLOW_TOF))} />}
      <p>{t("calib.flow.groundIntro")}</p>
      <div className="banner">{t("calib.flow.wipe")}</div>
      <span className="grow" />
      <div className="btn-row end"><button className="btn primary big" disabled={!cal} onClick={() => run(drone.calCapture())}>{t("calib.flow.setGround")}</button></div>
    </>
  ) : step === 1 ? (
    <>
      <h2>{steps[1]}</h2>
      <p>{t("calib.flow.scaleIntro")}</p>
      <div className="grid three" style={{ gap: 8, gridTemplateColumns: "repeat(3, minmax(0,1fr))" }}>
        <div className="kpi"><span className="k">{t("calib.flow.height")}</span><span className="v" style={{ fontSize: 20 }}>{d ? fmt.alt(d.heightMm / 1000, 2) : "–"}</span></div>
        <div className="kpi"><span className="k">{t("calib.flow.flowQuality")}</span><span className="v" style={{ fontSize: 20 }}>{d ? `${d.flowQ}%` : "–"}</span></div>
        <div className="kpi"><span className="k">{t("calib.flow.match")}</span><span className="v" style={{ fontSize: 20 }}>{d ? `${d.matchPct}%` : "–"}</span></div>
      </div>
      <div className="grid two" style={{ gap: 8, gridTemplateColumns: "repeat(2, minmax(0,1fr))" }}>
        {([1, 2] as const).map((b) => {
          const ok = d ? !!(d.axesDone & b) : false;
          const now = !ok && (b === 1 || (d ? !!(d.axesDone & 1) : false));
          return (
            <div key={b} className={`tile${now ? " cur" : ok ? " done" : ""}`} style={{ flexDirection: "row", justifyContent: "space-between", padding: "10px 12px", fontSize: 14 }}>
              <span>{b === 1 ? t("calib.flow.roll") : t("calib.flow.pitch")}</span>
              <b style={{ color: ok ? "var(--ok)" : now ? "var(--accent)" : "var(--muted)" }}>{ok ? t("calib.flow.done") : now ? t("calib.flow.now") : "–"}</b>
            </div>
          );
        })}
      </div>
      <span className="grow" />
      <div className="btn-row end"><button className="btn big" onClick={async () => { await drone.calAbort(); run(drone.calStart(CalType.FLOW_TOF)); }}>{t("common.back")}</button></div>
    </>
  ) : (
    <>
      <h2>{steps[2]}</h2>
      <p>{t("calib.flow.hint")}</p>
      <span className="grow" />
      <SaveRow onDiscard={async () => { await drone.calAbort(); run(drone.calStart(CalType.FLOW_TOF)); }} />
    </>
  );
  return <WizardShell title={t("calib.names.5")} steps={steps} current={step} visual={visual} panel={panel} />;
}

function FlowChart({ match }: { match: number }) {
  const [phase, setPhase] = useState(0);
  useEffect(() => {
    const id = setInterval(() => setPhase((p) => p + 0.15), 60);
    return () => clearInterval(id);
  }, []);
  const err = (100 - match) / 100;
  const pts = (k: number) => Array.from({ length: 31 }, (_, i) => {
    const x = i * 10, y = 80 - 55 * Math.sin(i / 3 + phase) * (1 - k * err * 0.6) + k * err * 10 * Math.sin(i + phase * 2);
    return `${x},${y.toFixed(1)}`;
  }).join(" ");
  return (
    <svg viewBox="0 0 300 160" style={{ width: "100%", height: 180 }} role="img" aria-label={`match ${match}%`}>
      <line x1="0" y1="80" x2="300" y2="80" stroke="var(--line-2)" />
      <polyline fill="none" stroke="var(--accent)" strokeWidth="2" points={pts(0)} />
      <polyline fill="none" stroke="var(--warn)" strokeWidth="2" strokeDasharray="5 4" points={pts(1)} />
    </svg>
  );
}

/* ================================================================ */
/* Battery                                                           */
/* ================================================================ */

export function BatteryWizard() {
  const { t } = useT();
  const s = useDrone();
  const run = useCommand();
  const cal = useCal(CalType.BATTERY);
  useAbortOnLeave(CalType.BATTERY);
  const [meter, setMeter] = useState("");
  const [mah, setMah] = useState("");
  useEffect(() => {
    run(drone.calStart(CalType.BATTERY));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const d = cal?.data as D<"battery"> | undefined;
  const step = done(cal?.state) ? 2 : cal && cal.step >= 2 ? 1 : 0;
  const steps = [t("calib.battery.voltage"), t("calib.battery.current"), t("calib.battery.result")];
  const cells = s.power?.cells ?? s.caps?.cells ?? 1;
  const num = (x: string) => parseFloat(x.replace(",", "."));
  const visual = step === 0 ? (
    <div className="grid" style={{ width: "100%", gap: 12 }}>
      <div className="kpi" style={{ alignItems: "center", padding: 20 }}><span className="k">{t("calib.battery.droneReads")}</span><span className="v" style={{ fontSize: 44 }}>{d ? `${d.vbatV.toFixed(2)} V` : "–"}</span></div>
      <label className="kpi" style={{ alignItems: "center", padding: 20, borderColor: "var(--accent)" }}>
        <span className="k">{t("calib.battery.meterReads")}</span>
        <span style={{ display: "flex", alignItems: "baseline", gap: 6 }}>
          <input inputMode="decimal" value={meter} onChange={(e) => setMeter(e.target.value)} placeholder={d ? d.vbatV.toFixed(2) : ""} aria-label={t("calib.battery.meterReads")}
            style={{ width: 170, border: "none", borderBottom: "3px solid var(--accent)", background: "transparent", fontFamily: "var(--display)", fontWeight: 700, fontSize: 44, textAlign: "center" }} />
          <span className="display" style={{ fontSize: 26 }}>V</span>
        </span>
      </label>
    </div>
  ) : step === 1 ? (
    <div className="card tight" style={{ width: "100%" }}>
      <div className="row first" style={{ minHeight: 70 }}><span className="grow label">{t("calib.battery.counted")}</span><span className="num" style={{ fontSize: 22 }}>{d ? `${d.usedMah} mAh` : "–"}</span></div>
      <div className="row" style={{ minHeight: 70 }}>
        <label htmlFor="mah" className="grow label">{t("calib.battery.charged")}</label>
        <input id="mah" className="input num" inputMode="numeric" style={{ width: 110, height: 46, fontSize: 20 }} value={mah} onChange={(e) => setMah(e.target.value)} />
        <span className="muted">mAh</span>
      </div>
      <div className="row" style={{ minHeight: 70 }}><span className="grow label muted">{t("calib.battery.scaleI")}</span><span className="num" style={{ fontSize: 22 }}>{d && num(mah) > 0 ? `× ${((d.scaleI * num(mah)) / d.usedMah).toFixed(2)}` : d ? `× ${d.scaleI.toFixed(2)}` : "–"}</span></div>
    </div>
  ) : (
    <>
      <Success text={t("calib.battery.corrected")} />
      <Facts rows={[
        [t("calib.battery.voltageNow"), s.power ? `${s.power.vbatV.toFixed(2)} V` : "–"],
        [t("calib.battery.perCell", { n: cells }), s.power ? `${s.power.cellV.toFixed(2)} V` : "–"],
        [t("calib.battery.scaleV"), d ? d.scaleV.toFixed(1) : "–"],
        [t("calib.battery.scaleI"), d ? `× ${d.scaleI.toFixed(2)}` : "–"],
      ]} />
    </>
  );
  const panel = step === 0 ? (
    <>
      <h2>{steps[0]}</h2>
      <p>{t("calib.battery.intro")}</p>
      <div className="card" style={{ flexDirection: "row", justifyContent: "space-between" }}><span className="muted">{t("calib.battery.scaleV")}</span><span className="num">{d ? `${d.scaleV.toFixed(1)}${num(meter) > 0 ? ` → ${(d.scaleV * num(meter) / d.vbatV).toFixed(1)}` : ""}` : "–"}</span></div>
      {cells === 1 && <span className="small muted">{t("calib.battery.indoorNote")}</span>}
      <span className="grow" />
      <div className="btn-row end"><button className="btn primary big" disabled={!(num(meter) > 0)} onClick={() => run(drone.calSetRef(CalType.BATTERY, 0, num(meter)))}>{t("calib.battery.applyContinue")}</button></div>
    </>
  ) : step === 1 ? (
    <>
      <h2>{steps[1]}</h2>
      <p>{t("calib.battery.currentIntro")}</p>
      <span className="grow" />
      <div className="btn-row end">
        <button className="btn big" onClick={() => run(drone.calCapture())}>{t("common.skip")}</button>
        <button className="btn primary big" disabled={!(num(mah) > 0)} onClick={() => run(drone.calSetRef(CalType.BATTERY, 1, num(mah)))}>{t("common.apply")}</button>
      </div>
    </>
  ) : (
    <>
      <h2>{steps[2]}</h2>
      <span className="grow" />
      <SaveRow onDiscard={async () => { await drone.calAbort(); run(drone.calStart(CalType.BATTERY)); }} />
    </>
  );
  return <WizardShell title={t("calib.names.6")} steps={steps} current={step} visual={visual} panel={panel} />;
}

/* ================================================================ */
/* Motors                                                            */
/* ================================================================ */

// Quad X, Betaflight order: M1 rear right CW, M2 front right CCW, M3 rear left CCW, M4 front left CW
const QUAD = [
  { pos: "rr", cw: true, x: 0.72, y: 0.74 },
  { pos: "fr", cw: false, x: 0.72, y: 0.26 },
  { pos: "rl", cw: false, x: 0.28, y: 0.74 },
  { pos: "fl", cw: true, x: 0.28, y: 0.26 },
];

export function MotorsWizard() {
  const { t } = useT();
  const s = useDrone();
  const run = useCommand();
  const cal = useCal(CalType.MOTORS);
  useAbortOnLeave(CalType.MOTORS);
  const [c1, setC1] = useState(false);
  const [c2, setC2] = useState(false);
  const [sel, setSel] = useState(1);
  const [ok, setOk] = useState<Record<number, boolean>>({});
  const [finished, setFinished] = useState(false);
  const d = cal?.data as D<"motors"> | undefined;
  const n = s.caps?.nMotors ?? 4;
  const quad = (s.caps?.frame ?? FrameType.QUAD_X) === FrameType.QUAD_X && n === 4;
  const step = !cal ? 0 : finished ? 2 : 1;
  const steps = [t("calib.motors.safety"), t("calib.motors.spin"), t("calib.motors.result")];
  const rev = (m: number) => !!(d && d.revMask & (1 << (m - 1)));
  const cw = (m: number) => (quad ? QUAD[m - 1].cw !== rev(m) : (m % 2 === 1) !== rev(m));
  const posName = (m: number) => (quad ? t(`calib.motors.pos.${QUAD[m - 1].pos}`) : `#${m}`);
  const stateOf = (m: number) => (d?.spinning && d.motor === m ? "spinning" : rev(m) ? "reversed" : ok[m] ? "ok" : "notTested");
  const color = (st: string) => (st === "spinning" ? "var(--accent)" : st === "ok" ? "var(--ok)" : st === "reversed" ? "var(--warn)" : "var(--muted)");
  const label = (st: string) => (st === "ok" ? t("common.ok") : t(`calib.motors.${st}`));
  const spin = (m: number) => {
    setSel(m);
    run(drone.motorTest(m));
  };

  const diagram = (big: boolean) => {
    const W = big ? 420 : 300, H = big ? 380 : 270, B = big ? 104 : 84;
    return (
      <div style={{ position: "relative", width: W, height: H, maxWidth: "100%" }}>
        <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`} aria-hidden="true" style={{ position: "absolute", inset: 0 }}>
          <line x1={W * 0.28} y1={H * 0.26} x2={W * 0.72} y2={H * 0.74} stroke="var(--line-3)" strokeWidth="10" strokeLinecap="round" />
          <line x1={W * 0.72} y1={H * 0.26} x2={W * 0.28} y2={H * 0.74} stroke="var(--line-3)" strokeWidth="10" strokeLinecap="round" />
          <rect x={W / 2 - 35} y={H / 2 - 42} width="70" height="84" rx="10" fill="var(--surface-2)" stroke="var(--accent)" strokeWidth="1.5" />
          <path d={`M${W / 2} ${H / 2 - 32} l10 12 h-20 z`} fill="var(--accent)" />
          <text x={W / 2} y={H / 2 + 26} textAnchor="middle" fill="var(--muted)" fontSize="12">{t("calib.motors.front")} ↑</text>
        </svg>
        {QUAD.map((q, i) => {
          const m = i + 1, st = stateOf(m);
          return (
            <button key={m} onClick={() => spin(m)} aria-label={`M${m}, ${posName(m)}`}
              style={{ position: "absolute", left: q.x * W - B / 2, top: q.y * H - B / 2, width: B, height: B, borderRadius: "50%", border: `2px solid ${st === "notTested" ? "var(--line-3)" : color(st)}`,
                background: st === "spinning" ? "color-mix(in srgb, var(--accent) 18%, var(--surface))" : "var(--surface)", display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", cursor: "pointer", fontFamily: "var(--display)" }}>
              <b style={{ fontSize: big ? 22 : 18 }}>M{m}</b>
              <span className="small" style={{ color: "var(--text-2)" }}>{cw(m) ? "CW ↻" : "CCW ↺"}</span>
              <span style={{ fontSize: 11, color: color(st) }}>{label(st)}</span>
            </button>
          );
        })}
      </div>
    );
  };

  const visual = step === 0 ? (
    <div className="banner crit" style={{ flexDirection: "column", gap: 8, padding: 24, fontSize: 16, width: "100%" }}>
      <b className="display" style={{ fontSize: 26 }}>{t("calib.motors.warnTitle")}</b>
      <span>{t("calib.motors.warnText")}</span>
    </div>
  ) : step === 1 ? (
    quad ? diagram(innerWidth >= 900) : (
      <div className="tiles" style={{ width: "100%" }}>
        {Array.from({ length: n }, (_, i) => i + 1).map((m) => (
          <button key={m} className={`tile${sel === m ? " cur" : ""}`} onClick={() => spin(m)}><b>M{m}</b><span>{cw(m) ? "CW ↻" : "CCW ↺"}</span><span style={{ color: color(stateOf(m)) }}>{label(stateOf(m))}</span></button>
        ))}
      </div>
    )
  ) : (
    <>
      <Success text={t("calib.motors.checked")} />
      <div className="card tight" style={{ width: "100%" }}>
        {Array.from({ length: n }, (_, i) => i + 1).map((m, i) => (
          <div key={m} className={`row${i === 0 ? " first" : ""}`}><b className="num" style={{ width: 40 }}>M{m}</b><span className="grow muted">{posName(m)} · {cw(m) ? t("calib.motors.cw") : t("calib.motors.ccw")}</span><span style={{ color: color(stateOf(m)) }}>{label(stateOf(m))}</span></div>
        ))}
      </div>
    </>
  );
  const panel = step === 0 ? (
    <>
      <h2>{steps[0]}</h2>
      <label className="choice"><input type="checkbox" checked={c1} onChange={(e) => setC1(e.target.checked)} />{t("calib.motors.c1")}</label>
      <label className="choice"><input type="checkbox" checked={c2} onChange={(e) => setC2(e.target.checked)} />{t("calib.motors.c2")}</label>
      <span className="grow" />
      <div className="btn-row end">
        <button className="btn primary big" disabled={!(c1 && c2)} onClick={() => run(drone.calStart(CalType.MOTORS))}>{c1 && c2 ? t("calib.motors.startTest") : t("calib.motors.tickBoth")}</button>
      </div>
    </>
  ) : step === 1 ? (
    <>
      <h2>{steps[1]}</h2>
      <p>{t("calib.motors.intro")}</p>
      <div className="card accent" role="status">
        <b>{t("calib.motors.question", { m: `M${sel}`, pos: posName(sel), dir: cw(sel) ? t("calib.motors.cw") : t("calib.motors.ccw") })}</b>
        <div className="btn-row">
          <button className="btn" style={{ flex: 1, background: "var(--ok)", color: "#04140b", borderColor: "var(--ok)" }} onClick={() => { setOk({ ...ok, [sel]: true }); setSel((sel % n) + 1); }}>{t("calib.motors.yes")}</button>
          <button className="btn warn" style={{ flex: 1 }} onClick={() => run(drone.motorReverse(sel))}>{t("calib.motors.reverse")}</button>
        </div>
        <button className="btn ghost" style={{ alignSelf: "flex-start", padding: 0, color: "var(--accent)", minHeight: 30 }} onClick={() => toast(t("calib.motors.remap"))}>{t("calib.motors.remap")}</button>
      </div>
      <span className="grow" />
      <div className="btn-row end"><button className="btn primary big" onClick={() => setFinished(true)}>{t("common.finish")}</button></div>
    </>
  ) : (
    <>
      <h2>{steps[2]}</h2>
      <p>{t("calib.motors.hint")}</p>
      <span className="grow" />
      <div className="btn-row end">
        <button className="btn big" onClick={() => setFinished(false)}>{t("calib.motors.testAgain")}</button>
        <button className="btn primary big" onClick={async () => { if (await run(drone.calSave(), t("calib.successful"))) go("settings/calibration"); }}>{t("common.save")}</button>
      </div>
    </>
  );
  return <WizardShell title={t("calib.names.7")} steps={steps} current={step} visual={visual} panel={panel} />;
}

/* ================================================================ */
/* ELRS sticks                                                       */
/* ================================================================ */

export function SticksWizard() {
  const { t } = useT();
  const s = useDrone();
  const run = useCommand();
  const cal = useCal(CalType.STICKS);
  useAbortOnLeave(CalType.STICKS);
  const hasRx = !s.caps || !!(s.caps.sensors & SENSOR.ELRS);
  useEffect(() => {
    if (hasRx) run(drone.calStart(CalType.STICKS));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [hasRx]);
  const d = cal?.data as D<"sticks"> | undefined;
  const step = done(cal?.state) ? 2 : cal && cal.step >= 2 ? 1 : 0;
  const steps = [t("calib.sticks.centre"), t("calib.sticks.range"), t("calib.sticks.result")];
  const linked = hasRx;
  const full = (i: number) => !!d && d.min[i] <= 1000 && d.max[i] >= 2000;
  const box = (xi: number, yi: number, caption: string) => {
    const nx = (v: number) => 10 + ((v - 988) / 1024) * 120, ny = (v: number) => 130 - ((v - 988) / 1024) * 120;
    const ok = full(xi) && full(yi);
    return (
      <figure className="card" style={{ margin: 0, alignItems: "center" }}>
        <svg width="100%" style={{ maxWidth: 220 }} viewBox="0 0 140 140" role="img" aria-label={caption}>
          <rect x="10" y="10" width="120" height="120" fill="none" stroke="var(--line-2)" strokeDasharray="4 4" />
          {d && <rect x={nx(d.min[xi])} y={ny(d.max[yi])} width={nx(d.max[xi]) - nx(d.min[xi])} height={ny(d.min[yi]) - ny(d.max[yi])}
            fill={ok ? "var(--ok)" : "var(--accent)"} fillOpacity="0.15" stroke={ok ? "var(--ok)" : "var(--accent)"} strokeWidth="1.5" />}
          <circle cx="70" cy="70" r="6" fill="var(--text)" />
        </svg>
        <figcaption className="small muted">{caption}</figcaption>
      </figure>
    );
  };
  if (!linked && s.caps) {
    return <WizardShell title={t("calib.names.8")} steps={steps} current={0} visual={<Mark className="wiz-mark" alt="" />} panel={<><h2>{steps[0]}</h2><div className="banner warn">{t("calib.sticks.noRadio")}</div></>} />;
  }
  const visual = step === 0 ? (
    <div className="card" style={{ width: "100%", gap: 18 }}>
      {[0, 1, 2, 3].map((i) => (
        <div key={i} style={{ display: "grid", gridTemplateColumns: "90px minmax(0,1fr) 60px", gap: 12, alignItems: "center" }}>
          <span>{t(`calib.sticks.channels.${i}`)}</span>
          <span className="bar" style={{ overflow: "visible" }}><span style={{ left: `calc(${d ? ((d.centre[i] - 988) / 1024) * 100 : 50}% - 2px)`, width: 5, top: -4, bottom: -4, background: "var(--accent)" }} /></span>
          <span className="num" style={{ textAlign: "right" }}>{d ? d.centre[i] : "–"}</span>
        </div>
      ))}
    </div>
  ) : step === 1 ? (
    <div className="grid two" style={{ width: "100%", gridTemplateColumns: "repeat(2, minmax(0,1fr))" }}>
      {box(1, 0, t("calib.sticks.throttleYaw"))}
      {box(3, 2, t("calib.sticks.pitchRoll"))}
    </div>
  ) : (
    <>
      <Success text={t("calib.sticks.mapped")} />
      <div className="card tight" style={{ width: "100%" }}>
        <div className="row first"><span className="grow label">{t("calib.sticks.endpoints")}</span><span style={{ color: "var(--ok)" }}>{t("calib.sticks.fullRange")}</span></div>
        {(d?.auxPos ?? []).map((p, i) => (
          <div key={i} className="row"><span className="grow label">{t("calib.sticks.aux", { n: i + 1, what: t(`calib.sticks.auxNames.${i + 1}`) })}</span>
            <span className="muted">{p === 2 ? t("calib.sticks.pos2") : p === 3 ? t("calib.sticks.pos3") : t("calib.sticks.none")}</span></div>
        ))}
      </div>
    </>
  );
  const panel = step === 0 ? (
    <>
      <h2>{steps[0]}</h2>
      <p>{t("calib.sticks.centreIntro")}</p>
      <span style={{ color: "var(--ok)" }}>{t("calib.sticks.centresOk")}</span>
      <span className="grow" />
      <div className="btn-row end"><button className="btn primary big" disabled={!cal} onClick={() => run(drone.calCapture())}>{t("common.continue")}</button></div>
    </>
  ) : step === 1 ? (
    <>
      <h2>{steps[1]}</h2>
      <p>{t("calib.sticks.rangeIntro")}</p>
      <div className="card tight">
        {[0, 1, 2, 3].map((i) => (
          <div key={i} className={`row${i === 0 ? " first" : ""}`} style={{ minHeight: 44 }}>
            <span className="grow">{t(`calib.sticks.channels.${i}`)}</span>
            <span className="num" style={{ color: full(i) ? undefined : "var(--warn)" }}>{d ? `${d.min[i]} – ${d.max[i]}` : "–"}{!full(i) ? ` · ${t("calib.sticks.keepGoing")}` : ""}</span>
          </div>
        ))}
      </div>
      <span className="grow" />
      <div className="btn-row end">
        <button className="btn big" onClick={async () => { await drone.calAbort(); run(drone.calStart(CalType.STICKS)); }}>{t("common.back")}</button>
        <button className="btn primary big" onClick={() => run(drone.calCapture())}>{t("common.finish")}</button>
      </div>
    </>
  ) : (
    <>
      <h2>{steps[2]}</h2>
      <button className="btn ghost" style={{ alignSelf: "flex-start", padding: 0, color: "var(--accent)" }} onClick={() => toast(t("calib.sticks.changeSwitches"))}>{t("calib.sticks.changeSwitches")}</button>
      <span className="grow" />
      <SaveRow onDiscard={async () => { await drone.calAbort(); run(drone.calStart(CalType.STICKS)); }} />
    </>
  );
  return <WizardShell title={t("calib.names.8")} steps={steps} current={step} visual={visual} panel={panel} />;
}

export function CalibRoute({ type }: { type: number }) {
  const { t } = useT();
  switch (type) {
    case CalType.ACCEL: return <AccelWizard />;
    case CalType.LEVEL: return <LevelWizard />;
    case CalType.GYRO: return <GyroWizard />;
    case CalType.COMPASS: return <CompassWizard />;
    case CalType.FLOW_TOF: return <FlowWizard />;
    case CalType.BATTERY: return <BatteryWizard />;
    case CalType.MOTORS: return <MotorsWizard />;
    case CalType.STICKS: return <SticksWizard />;
    default: return <div className="page"><main className="content"><a href={BACK}><Icon name="back" />{t("common.back")}</a></main></div>;
  }
}

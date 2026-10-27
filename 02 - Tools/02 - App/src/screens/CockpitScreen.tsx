import { useEffect, useRef, useState } from "react";
import { useT } from "../i18n";
import { drone } from "../link/drone";
import { ArmState, AutoAction, FlightMode, InputSource, SENSOR, BattWarn } from "../protocol/ids";
import { attachKeyboard, gamepad, keyboardAxes, sticksToAxes, toRc } from "../input/controls";
import { go, useDrone, useSettings, useViewport } from "../ui/hooks";
import { Icon, Mark } from "../ui/icons";
import { Confirm, HoldButton, SlideToConfirm, useCommand } from "../ui/kit";
import { BatteryIcon, HeadingTape, Hud, Stick } from "../ui/flightWidgets";
import { fmt } from "../ui/format";

type XY = { x: number; y: number };

export function CockpitScreen() {
  const { t } = useT();
  const s = useDrone();
  const st = useSettings();
  const vp = useViewport();
  const run = useCommand();
  const [left, setLeft] = useState<XY>({ x: 0, y: 0 });
  const [right, setRight] = useState<XY>({ x: 0, y: 0 });
  const [killAsk, setKillAsk] = useState(false);
  const [modePick, setModePick] = useState(false);
  const [portraitOk, setPortraitOk] = useState(false);

  const f = s.flight;
  const caps = s.caps;
  const gps = !!(caps && caps.sensors & SENSOR.GNSS);
  const flow = !!(caps && caps.sensors & SENSOR.FLOW);
  const armed = f?.arm === ArmState.ARMED;
  const inAir = !!f && !!(f.flags & 1);
  const holdMode = !f || f.mode === FlightMode.ALT_HOLD || f.mode === FlightMode.POS_HOLD;
  const radio = f?.input === InputSource.ELRS;
  const beginner = drone.param("beginner") > 0.5;
  const compact = vp.compactCockpit;
  const connected = s.link !== "idle";

  // throttle stick: springs to centre in the hold modes, stays put (and starts at the bottom) otherwise
  const throttleIsLeft = st.stickMode === 2;
  const prevHold = useRef(holdMode);
  useEffect(() => {
    if (prevHold.current !== holdMode) {
      prevHold.current = holdMode;
      const y = holdMode ? 0 : -1;
      if (throttleIsLeft) setLeft((v) => ({ ...v, y }));
      else setRight((v) => ({ ...v, y }));
    }
  }, [holdMode, throttleIsLeft]);
  useEffect(() => {
    if (!holdMode) (throttleIsLeft ? setLeft : setRight)((v) => ({ ...v, y: -1 }));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // stream RC at 25 Hz while the cockpit is open
  const latest = useRef({ left, right });
  latest.current = { left, right };
  useEffect(() => {
    let raf = 0;
    const loop = () => {
      const touch = sticksToAxes(latest.current.left, latest.current.right, appMode());
      const g = gamepad();
      const pad = g ? sticksToAxes(g.left, g.right, appMode()) : touch;
      drone.setRc(toRc([touch, pad, keyboardAxes()], currentSettings()));
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    const detach = attachKeyboard({
      KeyT: () => run(drone.takeoff(drone.param("takeoffAlt"))),
      KeyL: () => run(drone.land()),
      KeyH: () => drone.hasGps && run(drone.rth()),
      Space: () => run(drone.hold()),
    });
    return () => {
      cancelAnimationFrame(raf);
      detach();
      drone.setRc(null);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const settingsRef = useRef(st);
  settingsRef.current = st;
  const appMode = () => settingsRef.current.stickMode;
  const currentSettings = () => settingsRef.current;

  const kill = () => (inAir && st.killConfirm ? setKillAsk(true) : run(drone.kill()));
  const setMode = (m: number) => {
    setModePick(false);
    run(drone.setMode(m));
  };

  const modes = [
    { m: FlightMode.ACRO, dis: beginner },
    { m: FlightMode.ANGLE, dis: false },
    { m: FlightMode.ALT_HOLD, dis: false },
    { m: FlightMode.POS_HOLD, dis: !gps && !flow },
  ];
  const att = s.attitude ?? { rollRight: 0, pitchUp: 0, yaw: 0 };
  const heading = f?.headingDeg ?? 0;
  const p = s.power;
  const tof = s.nav?.tofM ?? null;
  const ceiling = drone.param("gfCeiling");

  const statusLine = () => {
    if (!caps) return t("telemetry.noData");
    const ekf = s.system ? (s.system.estFlags & 1 ? t("cockpit.ekfOk") : t("cockpit.ekfBad")) : t("cockpit.ekfOk");
    if (!gps) return `${t("cockpit.indoorStatus")} · ${ekf}`;
    const comp = s.system && s.system.estFlags & 8 ? t("cockpit.compassBad", { p: s.system.magInterfPct }) : t("cockpit.compassOk");
    return `${ekf} · ${comp} · ${t("cockpit.geofence", { d: Math.round(drone.param("gfMaxDist")) })}`;
  };
  const statusDot = s.link === "lost" || (f && f.failsafe) ? "var(--crit)" : s.system && s.system.estFlags & 8 ? "var(--warn)" : "var(--ok)";

  const stickSize = compact ? 140 : 180;
  const throttleTags = { top: t("cockpit.up"), bottom: t("cockpit.down") };
  const leftStick = (
    <Stick size={stickSize} value={left} onChange={setLeft} label={st.stickMode === 2 ? t("cockpit.leftStick") : t("cockpit.leftStickM1")}
      springY={!throttleIsLeft || holdMode} tags={throttleIsLeft ? throttleTags : { top: t("cockpit.fwd") }} />
  );
  const rightStick = (
    <Stick size={stickSize} value={right} onChange={setRight} label={st.stickMode === 2 ? t("cockpit.rightStick") : t("cockpit.rightStickM1")}
      springY={throttleIsLeft || holdMode} tags={throttleIsLeft ? { top: t("cockpit.fwd") } : throttleTags} />
  );

  const armControl = armed ? (
    <SlideToConfirm compact={compact} armed label={compact ? t("cockpit.armed") : t("cockpit.slideToDisarm")} aria={t("cockpit.slideToDisarmAria")} onConfirm={() => run(drone.disarm())} />
  ) : (
    <SlideToConfirm compact={compact} label={t("connect.slideToArm")} aria={t("connect.slideToArmAria")} onConfirm={() => run(drone.arm())} disabled={!connected} />
  );

  const actions = (
    <div className="ck-actions">
      <button className="btn" onClick={() => run(drone.takeoff(drone.param("takeoffAlt")))} disabled={!armed || inAir}><Icon name="up" size={18} />{t("cockpit.autoTakeoff")}</button>
      <button className="btn" onClick={() => run(drone.land())} disabled={!inAir}><Icon name="land" size={18} />{t("cockpit.autoLand")}</button>
      {gps && <button className="btn warn" onClick={() => run(drone.rth())} disabled={!inAir}><Icon name="home" size={18} />{t("cockpit.rth")}</button>}
      <button className="btn" onClick={() => run(drone.hold())} disabled={!inAir} style={gps ? undefined : { gridColumn: "span 2" }}><Icon name="pause" size={18} />{t("cockpit.pause")}</button>
    </div>
  );

  const autoChip = f && f.auto !== AutoAction.NONE ? <span className="chip auto">{t(`auto.${f.auto}`)}</span> : null;

  if (vp.phone && vp.h > vp.w && !portraitOk) {
    return (
      <div className="page" style={{ alignItems: "center", justifyContent: "center", padding: 24, textAlign: "center", gap: 16 }}>
        <Mark />
        <p style={{ maxWidth: 300 }}>{t("telemetry.rotateHint")}</p>
        <div className="btn-row" style={{ justifyContent: "center" }}>
          <a className="btn" href="#/connect">{t("common.back")}</a>
          <button className="btn primary" onClick={() => setPortraitOk(true)}>{t("common.continue")}</button>
        </div>
      </div>
    );
  }

  return (
    <div className={`cockpit${compact ? " compact" : ""}`}>
      <header className="ck-top">
        <a href="#/connect" aria-label={t("common.back")} style={{ display: "flex" }}><Mark /></a>
        <div className="name">
          <b>{caps?.name ?? s.deviceName ?? "ACwing"}</b>
          <span className="small muted">{caps ? `FC rev ${caps.hwRev} · ${p?.cells ?? caps.cells}S · ${t(`about.frames.${caps.frame}`)}` : ""}</span>
        </div>
        <span className={`chip ${armed ? "armed" : "disarmed"}`}>{armed ? t("cockpit.armed") : t("cockpit.disarmed")}</span>
        <button className="chip mode" onClick={() => setModePick((v) => !v)} aria-expanded={modePick} style={{ cursor: "pointer" }}>
          {f ? t(`modes.${f.mode}`) : "–"} <Icon name="down" size={12} stroke={2.4} />
        </button>
        {autoChip}
        <span className="spacer" style={{ flex: 1 }} />
        {s.system && s.system.lq !== null && <div className="ck-stat full"><span className="k">{t("cockpit.linkElrs")}</span><span className="v">LQ {s.system.lq}% · {s.system.rssi} dBm</span></div>}
        {s.system && s.system.lq !== null && compact && <span className="num small">LQ {s.system.lq}%</span>}
        <div className="ck-stat full"><span className="k">{t("cockpit.linkBle")}</span><span className="v">{s.rssi !== null ? `${s.rssi} dBm` : "–"} · {(s.rxBytesPerS / 1000).toFixed(1)} kB/s</span></div>
        {gps ? (
          <div className="ck-stat full"><span className="k">{t("cockpit.gnss")}</span><span className="v">{s.nav ? `${s.nav.fix === 3 ? "3D" : s.nav.fix === 2 ? "2D" : "–"} · ${s.nav.sats} sat · HDOP ${s.nav.hdop.toFixed(1)}` : "–"}</span></div>
        ) : (
          <div className="ck-stat full"><span className="k">{t("cockpit.flow")}</span><span className="v">{s.nav?.flowQ != null ? `${s.nav.flowQ}%` : "–"}</span></div>
        )}
        {compact && <span className="num small">{gps ? `GNSS ${s.nav?.sats ?? "–"}` : `FLOW ${s.nav?.flowQ ?? "–"}%`}</span>}
        <div className="ck-stat"><span className="k full">{t("cockpit.flightTime")}</span><span className="v">{fmt.mmss(f?.flightTimeS ?? 0)}</span></div>
        <div className="batt">
          <BatteryIcon pct={p?.pct ?? 0} warn={p?.warn ?? 0} />
          <div style={{ display: "flex", flexDirection: "column", gap: 1 }}>
            <b>{p ? `${p.pct}%` : "–"}</b>
            {!compact && <span className="small muted">{p ? `${p.vbatV.toFixed(2)} V · ${p.currentA.toFixed(1)} A` : ""}</span>}
          </div>
        </div>
        {!compact && <a className="btn icon" href="#/telemetry" aria-label={t("icons.telemetry")}><Icon name="pulse" /></a>}
        <a className="btn icon" href="#/settings" aria-label={t("icons.settings")}><Icon name="gear" /></a>
      </header>

      {modePick && (
        <div className="ck-modepick" role="group" aria-label={t("cockpit.flightMode")}>
          {modes.map(({ m, dis }) => (
            <button key={m} className="btn small" aria-pressed={f?.mode === m} disabled={dis} onClick={() => setMode(m)}
              style={f?.mode === m ? { background: "var(--accent)", color: "var(--accent-ink)", borderColor: "var(--accent)" } : undefined}>{t(`modes.${m}`)}</button>
          ))}
        </div>
      )}

      {radio && <div className="override banner warn" style={{ top: compact ? 100 : 130, zIndex: 3 }}>{t("cockpit.radioOverride")}</div>}
      {s.link === "lost" && <div className="override banner crit" style={{ top: compact ? 52 : 70, zIndex: 4 }}>{t("link.lost")}</div>}

      {!compact && <div className="tape"><HeadingTape heading={heading} width={Math.min(520, vp.w * 0.46)} /></div>}
      <div className="hud"><Hud rollRight={att.rollRight} pitchUp={att.pitchUp} width={compact ? 260 : Math.min(520, vp.w * 0.46)} /></div>

      {compact ? (
        <>
          <div className="ck-mini">
            <div><div className="k">{gps ? t("cockpit.short.alt") : `${t("cockpit.short.alt")} · TOF`}</div><div className="v">{fmt.alt(gps ? f?.altRelM : tof ?? f?.altRelM)}</div></div>
            <div><div className="k">{t("cockpit.short.speed")}</div><div className="v">{fmt.speed(f?.groundSpeedMs)}</div></div>
            {gps ? <div><div className="k">{t("cockpit.short.home")}</div><div className="v">{fmt.dist(f?.homeDistM)}</div></div>
              : <div><div className="k">{t("cockpit.ceiling").toUpperCase()}</div><div className="v">{fmt.alt(ceiling)}</div></div>}
          </div>
          <div className="ck-side l">
            <HoldButton ms={2000} onHold={kill} className="btn icon danger" label={`${t("cockpit.kill")}, ${t("cockpit.killHold")}`}><Icon name="power" size={22} stroke={2} /></HoldButton>
            <a className="btn icon" href="#/telemetry" aria-label={t("icons.telemetry")}><Icon name="pulse" size={22} /></a>
          </div>
          <div className="ck-side r">
            <button className="btn icon" aria-label={t("cockpit.autoTakeoff")} disabled={!armed || inAir} onClick={() => run(drone.takeoff(drone.param("takeoffAlt")))}><Icon name="up" size={22} /></button>
            {gps && <button className="btn icon warn" aria-label={t("cockpit.rthLong")} disabled={!inAir} onClick={() => run(drone.rth())}><Icon name="home" size={22} /></button>}
            <button className="btn icon" aria-label={t("cockpit.autoLand")} disabled={!inAir} onClick={() => run(drone.land())}><Icon name="land" size={22} /></button>
            <button className="btn icon" aria-label={t("cockpit.pause")} disabled={!inAir} onClick={() => run(drone.hold())}><Icon name="pause" size={22} /></button>
          </div>
          <div className="ck-bottom">
            {armControl}
            <div className="ck-status" role="status"><span className="dot" style={{ background: statusDot }} />{statusLine()}</div>
          </div>
        </>
      ) : (
        <>
          <section className="ck-left" aria-label={t("telemetry.title")}>
            <div className="ck-card">
              <span className="k">{t("cockpit.relAlt")}</span>
              <span className="big">{fmt.altNum(f?.altRelM ?? 0).toFixed(1)}<small>{fmt.altUnit()}</small></span>
              <span className="small muted">{tof != null ? `ToF ${fmt.alt(tof, 2)}` : `Baro${gps ? " + GNSS" : ""} · ToF ${t("common.outOfRange")}`}</span>
            </div>
            <div className="ck-card"><span className="k">{t("cockpit.vSpeed")}</span><span className="big">{fmt.signed(fmt.speedNum(f?.vzMs ?? 0))}<small>{fmt.speedUnit()}</small></span></div>
            <div className="ck-card"><span className="k">{t("cockpit.gSpeed")}</span><span className="big">{fmt.speedNum(f?.groundSpeedMs ?? 0).toFixed(1)}<small>{fmt.speedUnit()}</small></span></div>
            {gps ? (
              <div className="ck-card" style={{ flexDirection: "row", alignItems: "center", gap: 12 }}>
                <svg width="44" height="44" viewBox="0 0 44 44" aria-hidden="true">
                  <circle cx="22" cy="22" r="20" fill="none" stroke="var(--line-3)" strokeWidth="1.5" />
                  <g transform={`rotate(${(f?.homeBearingDeg ?? 0) - heading} 22 22)`}><path d="M22 6 L28 30 L22 25 L16 30 Z" fill="var(--accent)" /></g>
                </svg>
                <div style={{ display: "flex", flexDirection: "column", gap: 2 }}>
                  <span className="k">{t("cockpit.home")}</span>
                  <span className="num" style={{ fontSize: 22, fontWeight: 700 }}>{fmt.dist(f?.homeDistM)}</span>
                  <span className="small muted">{t("cockpit.homeDetail", { brg: Math.round(f?.homeBearingDeg ?? 0), eta: fmt.mmss(f?.rthEtaS) })}</span>
                </div>
              </div>
            ) : (
              <div className="ck-card"><span className="k">{t("cockpit.ceiling")}</span><span className="big">{fmt.altNum(ceiling).toFixed(1)}<small>{fmt.altUnit()}</small></span><span className="small muted">{t("cockpit.indoorStatus")}</span></div>
            )}
            {s.motors && (
              <div className="ck-card">
                <span className="k">{s.motors.motors.some((m) => m.rpm !== null) ? t("cockpit.motorsRpm") : t("cockpit.motorsOut")}</span>
                <div style={{ display: "grid", gridTemplateColumns: "repeat(2, minmax(0,1fr))", gap: 6 }} className="num">
                  {s.motors.motors.map((m, i) => <span key={i}>M{i + 1} · {m.rpm !== null ? `${(m.rpm / 1000).toFixed(1)}k` : `${m.outPct}%`}</span>)}
                </div>
              </div>
            )}
          </section>

          <section className="ck-right" aria-label={t("cockpit.flight")}>
            <div className="ck-card">
              <span className="k">{t("cockpit.flightMode")}</span>
              <div className="ck-modes">
                {modes.map(({ m, dis }) => (
                  <button key={m} aria-pressed={f?.mode === m} disabled={dis} onClick={() => setMode(m)} title={dis && m === FlightMode.POS_HOLD ? t("cockpit.noPosHold") : undefined}>{t(`modes.${m}`)}</button>
                ))}
              </div>
            </div>
            <div className="ck-card">
              <span className="k">{t("cockpit.flight")}</span>
              {armControl}
              {actions}
            </div>
            <HoldButton ms={2000} onHold={kill} className="btn danger big" label={`${t("cockpit.kill")}, ${t("cockpit.killHold")}`}>
              <span style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 2 }}>
                <span className="display" style={{ letterSpacing: "0.06em" }}>{t("cockpit.kill")}</span>
                <span className="small" style={{ fontWeight: 500 }}>{t("cockpit.killHold")}</span>
              </span>
            </HoldButton>
          </section>

          <div className="ck-bottom">
            <div className="grid" style={{ gridTemplateColumns: "repeat(4, minmax(0,1fr))", gap: 8 }}>
              <div className="kpi" style={{ padding: "8px 10px" }}><span className="k">{t("cockpit.avgCell")}</span><span className="num" style={{ fontSize: 16, fontWeight: 600 }}>{p ? `${p.cellV.toFixed(2)} V` : "–"}</span></div>
              <div className="kpi" style={{ padding: "8px 10px" }}><span className="k">{t("cockpit.used")}</span><span className="num" style={{ fontSize: 16, fontWeight: 600 }}>{p ? `${p.usedMah} mAh` : "–"}</span></div>
              <div className="kpi" style={{ padding: "8px 10px" }}><span className="k">{t("cockpit.remaining")}</span><span className="num" style={{ fontSize: 16, fontWeight: 600 }}>{p?.remainS != null ? `≈ ${Math.round(p.remainS / 60)} min` : "–"}</span></div>
              <div className="kpi" style={{ padding: "8px 10px" }}><span className="k">{gps ? t("cockpit.wind") : "ToF"}</span><span className="num" style={{ fontSize: 16, fontWeight: 600 }}>{gps ? fmt.speed(s.nav?.windMs) : fmt.alt(tof, 2)}</span></div>
            </div>
            <div className="ck-status" role="status">
              <span className="dot" style={{ background: statusDot }} />
              <span>{statusLine()}</span>
              <span style={{ flex: 1 }} />
              {p && p.warn !== BattWarn.OK && <span style={{ color: p.warn >= 2 ? "var(--crit)" : "var(--warn)" }}>{t(`events.${29 + p.warn}`)}</span>}
            </div>
          </div>
        </>
      )}

      <div className="stick-wrap l">{leftStick}<span className="stick-caption">{st.stickMode === 2 ? t("cockpit.throttleYaw") : t("cockpit.pitchYaw")}</span></div>
      <div className="stick-wrap r">{rightStick}<span className="stick-caption">{st.stickMode === 2 ? t("cockpit.pitchRoll") : t("cockpit.throttleRoll")}</span></div>

      {killAsk && (
        <Confirm danger title={t("cockpit.kill")} body={t("cockpit.killConfirm")} yes={t("cockpit.killYes")}
          onYes={() => { setKillAsk(false); run(drone.kill()); }} onNo={() => setKillAsk(false)} />
      )}
      {!connected && (
        <div className="modal-back" style={{ background: "rgba(0,0,0,.55)" }}>
          <div className="modal">
            <h2 className="card-title">{t("link.notConnected")}</h2>
            <button className="btn primary" onClick={() => go("connect")}>{t("connect.title")}</button>
          </div>
        </div>
      )}
    </div>
  );
}

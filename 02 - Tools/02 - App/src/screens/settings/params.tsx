import { useT } from "../../i18n";
import { drone } from "../../link/drone";
import { PARAM_BY_KEY, ACTION } from "../../protocol/ids";
import { useDrone } from "../../ui/hooks";
import { GpsBadge, Row, SliderRow, Switch, toast, useCommand } from "../../ui/kit";
import { fmt } from "../../ui/format";
import { SENSOR } from "../../protocol/ids";

/** True if the connected drone has the sensors a parameter needs (or nothing is connected yet). */
export function useVisible() {
  const s = useDrone();
  return (key: string) => {
    const d = PARAM_BY_KEY[key];
    if (!d?.needs || !s.caps) return true;
    return (s.caps.sensors & d.needs) === d.needs;
  };
}

function badgeFor(key: string) {
  const d = PARAM_BY_KEY[key];
  return d.needs && d.needs & (SENSOR.GNSS | SENSOR.MAG_GPS) ? <GpsBadge /> : undefined;
}

export function ParamSlider({ k, hint }: { k: string; hint?: string }) {
  const { t } = useT();
  useDrone();
  const run = useCommand();
  const d = PARAM_BY_KEY[k];
  const visible = useVisible();
  if (!visible(k)) return null;
  return (
    <SliderRow label={t(`params.${k}`)} hint={hint} badge={badgeFor(k)} value={drone.param(k)} min={d.min} max={d.max} step={d.step}
      format={(v) => fmt.param(v, d.unit, d.step)} onCommit={(v) => run(drone.setParam(k, +v.toFixed(3)))} />
  );
}

export function ParamSelect({ k, hint }: { k: string; hint?: string }) {
  const { t } = useT();
  useDrone();
  const run = useCommand();
  const d = PARAM_BY_KEY[k];
  const visible = useVisible();
  if (!visible(k)) return null;
  const id = `p-${k}`;
  const gps = drone.hasGps;
  return (
    <Row label={<label htmlFor={id}>{t(`params.${k}`)}</label>} hint={hint} badge={badgeFor(k)}>
      <select id={id} className="select" style={{ maxWidth: 200 }} value={Math.round(drone.param(k))} onChange={(e) => run(drone.setParam(k, +e.target.value))}>
        {(d.options ?? []).map((o) => {
          const name = ACTION[k]?.[o];
          // without GPS, "Return to home" is not available: the firmware lands instead
          if (name === "rth" && !gps && drone.getSnapshot().caps) return null;
          // options without a name show the value and the unit (e.g. "3S")
          return <option key={o} value={o}>{name ? t(`actions.${name}`) : `${o}${d.unit ?? ""}`}</option>;
        })}
      </select>
    </Row>
  );
}

export function ParamSwitch({ k, hint }: { k: string; hint?: string }) {
  const { t } = useT();
  useDrone();
  const run = useCommand();
  const visible = useVisible();
  if (!visible(k)) return null;
  return (
    <Row label={t(`params.${k}`)} hint={hint} badge={badgeFor(k)}>
      <Switch label={t(`params.${k}`)} checked={drone.param(k) > 0.5} onChange={(v) => run(drone.setParam(k, v ? 1 : 0))} />
    </Row>
  );
}

export function ParamNumber({ k, hint, suffix }: { k: string; hint?: string; suffix?: string }) {
  const { t } = useT();
  useDrone();
  const run = useCommand();
  const d = PARAM_BY_KEY[k];
  const visible = useVisible();
  const id = `p-${k}`;
  const v = drone.param(k);
  const dec = d.step >= 1 ? 0 : 2;
  if (!visible(k)) return null;
  return (
    <Row label={<label htmlFor={id}>{t(`params.${k}`)}</label>} hint={hint}>
      <input id={id} key={v} className="input num" type="text" inputMode="decimal" defaultValue={v.toFixed(dec)}
        onBlur={(e) => {
          const n = parseFloat(e.target.value.replace(",", "."));
          if (!Number.isFinite(n) || n < d.min || n > d.max) {
            toast(t("reasons.14"), "warn");
            e.target.value = v.toFixed(dec);
          } else if (Math.abs(n - v) > 1e-4) run(drone.setParam(k, n));
        }} />
      {suffix && <span className="small muted" style={{ width: 52 }}>{suffix}</span>}
    </Row>
  );
}

/** "Changes not saved on the drone yet" bar, shown while any parameter differs from flash. */
export function SaveBar() {
  const { t } = useT();
  const s = useDrone();
  const run = useCommand();
  const dirty = Object.values(s.params).some((p) => p.dirty);
  if (!dirty || s.link === "idle") return null;
  return (
    <div className="savebar" role="status">
      <span style={{ flex: 1 }}>{t("settings.unsaved")}</span>
      <button className="btn primary" onClick={() => run(drone.saveParams(), t("settings.saved"))}>{t("settings.saveToDrone")}</button>
    </div>
  );
}

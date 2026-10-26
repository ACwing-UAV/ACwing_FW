import { useEffect, useRef, useState, type ReactNode, useSyncExternalStore } from "react";
import { Icon } from "./icons";
import { useT } from "../i18n";
import { CommandError } from "../link/drone";
import { AckResult } from "../protocol/ids";

/* ---------- small pieces ---------- */

export function Switch({ checked, onChange, label, disabled }: { checked: boolean; onChange: (v: boolean) => void; label: string; disabled?: boolean }) {
  return <button type="button" role="switch" className="switch" aria-checked={checked} aria-label={label} disabled={disabled} onClick={() => onChange(!checked)} />;
}

export function Seg<T extends string | number>({ value, options, onChange, label, full }: {
  value: T; options: { v: T; label: string; disabled?: boolean }[]; onChange: (v: T) => void; label: string; full?: boolean;
}) {
  return (
    <div className={`seg${full ? " full" : ""}`} role="group" aria-label={label}>
      {options.map((o) => (
        <button key={String(o.v)} type="button" aria-pressed={o.v === value} disabled={o.disabled} onClick={() => onChange(o.v)}>{o.label}</button>
      ))}
    </div>
  );
}

export function Dot({ state }: { state: "ok" | "warn" | "fail" | "na" }) {
  return <span className={`dot ${state === "ok" ? "" : state}`} aria-hidden="true" />;
}

export function GpsBadge() {
  return <span className="badge">GPS</span>;
}

export function Row({ label, hint, children, badge, first }: { label: ReactNode; hint?: ReactNode; children?: ReactNode; badge?: ReactNode; first?: boolean }) {
  return (
    <div className={`row${first ? " first" : ""}`}>
      {badge}
      <div className="grow">
        <span className="label">{label}</span>
        {hint && <span className="hint">{hint}</span>}
      </div>
      {children}
    </div>
  );
}

let sliderId = 0;
export function SliderRow({ label, value, min, max, step, format, onChange, onCommit, hint, badge, disabled }: {
  label: string; value: number; min: number; max: number; step: number; format: (v: number) => string; onChange?: (v: number) => void;
  onCommit?: (v: number) => void; hint?: string; badge?: ReactNode; disabled?: boolean;
}) {
  const [id] = useState(() => `sl${++sliderId}`);
  const [local, setLocal] = useState(value);
  const dragging = useRef(false);
  useEffect(() => {
    if (!dragging.current) setLocal(value);
  }, [value]);
  const commit = () => {
    dragging.current = false;
    if (Math.abs(local - value) > step / 2) onCommit?.(local);
  };
  return (
    <div className="row col">
      <div className="head">
        {badge}
        <label htmlFor={id} className="label" style={{ flex: 1 }}>{label}</label>
        <span className="value">{format(local)}</span>
      </div>
      <input id={id} type="range" min={min} max={max} step={step} value={local} disabled={disabled}
        onChange={(e) => { dragging.current = true; const v = +e.target.value; setLocal(v); onChange?.(v); }}
        onPointerUp={commit} onKeyUp={commit} onBlur={() => dragging.current && commit()} />
      {hint && <span className="hint">{hint}</span>}
    </div>
  );
}

/* ---------- slide to confirm (arm / disarm) ---------- */

export function SlideToConfirm({ label, aria, onConfirm, armed, disabled, compact }: {
  label: string; aria: string; onConfirm: () => void; armed?: boolean; disabled?: boolean; compact?: boolean;
}) {
  const track = useRef<HTMLDivElement>(null);
  const [x, setX] = useState(0);
  const start = useRef<number | null>(null);
  const max = () => (track.current ? track.current.clientWidth - (compact ? 46 : 60) : 200);
  const end = () => {
    if (start.current === null) return;
    start.current = null;
    if (x > max() * 0.85) onConfirm();
    setX(0);
  };
  const dir = armed ? -1 : 1;
  return (
    <div ref={track} className={`slide${armed ? " armed" : ""}`} role="slider" aria-label={aria} aria-valuemin={0} aria-valuemax={100}
      aria-valuenow={Math.round((x / Math.max(1, max())) * 100)} aria-disabled={disabled} tabIndex={0}
      style={compact ? { height: 46, borderRadius: 23 } : undefined}
      onKeyDown={(e) => { if (!disabled && (e.key === "Enter" || e.key === " " || e.key === (armed ? "ArrowLeft" : "ArrowRight"))) { e.preventDefault(); onConfirm(); } }}
      onPointerDown={(e) => { if (disabled) return; start.current = e.clientX; (e.target as Element).setPointerCapture?.(e.pointerId); }}
      onPointerMove={(e) => { if (start.current === null) return; setX(Math.max(0, Math.min(max(), (e.clientX - start.current) * dir))); }}
      onPointerUp={end} onPointerCancel={() => { start.current = null; setX(0); }}>
      <span className="lbl" style={compact ? { fontSize: 12 } : undefined}>{label}</span>
      <span className="knob" style={{
        ...(armed ? { left: "auto", right: 5 + x } : { left: 5 + x }),
        ...(compact ? { width: 36, height: 36 } : {}),
      }}>
        <Icon name={armed ? "arrowL" : "arrowR"} size={20} stroke={2.4} />
      </span>
    </div>
  );
}

/* ---------- hold-to-activate (motor kill) ---------- */

export function HoldButton({ ms, onHold, children, className, label }: { ms: number; onHold: () => void; children: ReactNode; className?: string; label?: string }) {
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const [pct, setPct] = useState(0);
  const raf = useRef(0);
  const t0 = useRef(0);
  const stop = () => {
    if (timer.current) clearTimeout(timer.current);
    timer.current = null;
    cancelAnimationFrame(raf.current);
    setPct(0);
  };
  const begin = () => {
    t0.current = performance.now();
    const step = () => {
      setPct(Math.min(100, ((performance.now() - t0.current) / ms) * 100));
      raf.current = requestAnimationFrame(step);
    };
    raf.current = requestAnimationFrame(step);
    timer.current = setTimeout(() => { stop(); onHold(); }, ms);
  };
  return (
    <button type="button" className={className} aria-label={label} onPointerDown={begin} onPointerUp={stop} onPointerLeave={stop} onPointerCancel={stop}
      onKeyDown={(e) => { if ((e.key === "Enter" || e.key === " ") && !timer.current) begin(); }} onKeyUp={stop}
      style={{ backgroundImage: pct ? `linear-gradient(90deg, rgba(255,77,79,.35) ${pct}%, transparent ${pct}%)` : undefined }}>
      {children}
    </button>
  );
}

/* ---------- modal ---------- */

export function Confirm({ title, body, yes, onYes, onNo, danger }: { title: string; body?: string; yes: string; onYes: () => void; onNo: () => void; danger?: boolean }) {
  const { t } = useT();
  return (
    <div className="modal-back" role="dialog" aria-modal="true" aria-label={title} onClick={onNo}>
      <div className="modal" onClick={(e) => e.stopPropagation()}>
        <h2 className="card-title">{title}</h2>
        {body && <p style={{ margin: 0, color: "var(--text-2)" }}>{body}</p>}
        <div className="btn-row end">
          <button className="btn" onClick={onNo} autoFocus>{t("common.cancel")}</button>
          <button className={`btn ${danger ? "danger" : "primary"}`} onClick={onYes}>{yes}</button>
        </div>
      </div>
    </div>
  );
}

/* ---------- toasts ---------- */

type Toast = { id: number; text: string; level: "info" | "warn" | "error" | "critical" };
let toasts: Toast[] = [];
let tid = 0;
const tl = new Set<() => void>();
export const toast = (text: string, level: Toast["level"] = "info") => {
  const id = ++tid;
  toasts = [...toasts.slice(-3), { id, text, level }];
  tl.forEach((l) => l());
  setTimeout(() => {
    toasts = toasts.filter((x) => x.id !== id);
    tl.forEach((l) => l());
  }, level === "info" ? 3000 : 6000);
};

export function Toasts() {
  const list = useSyncExternalStore((f) => { tl.add(f); return () => tl.delete(f); }, () => toasts);
  return (
    <div className="toasts" role="status" aria-live="polite">
      {list.map((x) => (
        <div key={x.id} className={`toast ${x.level}`}>
          <Dot state={x.level === "info" ? "ok" : x.level === "warn" ? "warn" : "fail"} />
          <span>{x.text}</span>
        </div>
      ))}
    </div>
  );
}

/** Runs a drone command and turns a refusal into a readable toast. Returns true on success. */
export function useCommand() {
  const { t } = useT();
  return async (p: Promise<unknown>, okText?: string) => {
    try {
      await p;
      if (okText) toast(okText);
      return true;
    } catch (e) {
      if (e instanceof CommandError) {
        if (e.result === "timeout") toast(t("cmdError.timeout"), "error");
        else if (e.result === "offline") toast(t("cmdError.offline"), "warn");
        else if (e.result === AckResult.REJECTED || e.result === AckResult.INVALID || e.result === AckResult.BUSY)
          toast(t("cmdError.rejected", { why: t(`reasons.${e.reason}`) || t("cmdError.other") }), "warn");
        else toast(t("cmdError.other"), "warn");
      } else toast(String(e), "error");
      return false;
    }
  };
}

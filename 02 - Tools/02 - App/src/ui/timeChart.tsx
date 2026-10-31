/**
 * Time-series chart for the flight-log viewer: several series on a shared
 * time axis (s from the log start), optional right-hand scale, event marks.
 * Long series are reduced to min/max pairs per pixel column, so a long flight
 * stays light to draw.
 */
export interface TSeries {
  t: number[];
  v: number[];
  color: string;
  label: string;
  dashed?: boolean;
  /** drawn against the right-hand axis */
  right?: boolean;
}


function bounds(vals: number[][], pad = 0.08): [number, number] {
  let lo = Infinity, hi = -Infinity;
  for (const v of vals) for (const x of v) if (Number.isFinite(x)) { if (x < lo) lo = x; if (x > hi) hi = x; }
  if (!Number.isFinite(lo)) return [0, 1];
  // a flat line (e.g. a steady loop rate): open the scale around it
  const flat = Math.max(1e-6, Math.abs(hi) * 0.02);
  if (hi - lo < flat) { const c = (hi + lo) / 2, h = Math.max(0.5, Math.abs(c) * 0.05); lo = c - h; hi = c + h; }
  const m = (hi - lo) * pad;
  return [lo - m, hi + m];
}

function reduce(t: number[], v: number[], t0: number, t1: number, cols: number) {
  const pts: [number, number][] = [];
  if (t.length <= cols * 2) {
    for (let i = 0; i < t.length; i++) if (Number.isFinite(v[i])) pts.push([t[i], v[i]]);
    return pts;
  }
  const bin = (t1 - t0) / cols;
  let b = -1, lo = 0, hi = 0, tl = 0, th = 0;
  const flush = () => { if (b >= 0) { if (tl <= th) pts.push([tl, lo], [th, hi]); else pts.push([th, hi], [tl, lo]); } };
  for (let i = 0; i < t.length; i++) {
    const x = v[i];
    if (!Number.isFinite(x)) continue;
    const k = Math.floor((t[i] - t0) / bin);
    if (k !== b) { flush(); b = k; lo = hi = x; tl = th = t[i]; }
    else { if (x < lo) { lo = x; tl = t[i]; } if (x > hi) { hi = x; th = t[i]; } }
  }
  flush();
  return pts;
}

const fmtT = (s: number) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
const fmtV = (v: number, span: number) => (span < 2 ? v.toFixed(2) : span < 20 ? v.toFixed(1) : v.toFixed(0));

export function TimeChart({ title, series, unit, rightUnit, t0, t1, marks = [], fixed, compact }: {
  title: string; series: TSeries[]; unit: string; rightUnit?: string; t0: number; t1: number;
  marks?: { t: number; color: string; label: string }[];
  fixed?: { left?: [number, number]; right?: [number, number] };
  /** phone width: a narrower drawing so the labels keep a readable size */
  compact?: boolean;
}) {
  const W = compact ? 380 : 720, H = compact ? 200 : 220, X0 = compact ? 38 : 48, X1 = compact ? 346 : 680, Y0 = 12, Y1 = H - 30;
  const left = series.filter((s) => !s.right), right = series.filter((s) => s.right);
  const [lLo, lHi] = fixed?.left ?? bounds(left.map((s) => s.v));
  const [rLo, rHi] = fixed?.right ?? bounds(right.map((s) => s.v));
  const span = Math.max(1e-6, t1 - t0);
  const X = (t: number) => X0 + ((t - t0) / span) * (X1 - X0);
  const Y = (v: number, lo: number, hi: number) => Y1 - ((Math.min(hi, Math.max(lo, v)) - lo) / (hi - lo)) * (Y1 - Y0);
  const ticks = [0, 0.25, 0.5, 0.75, 1];
  const visible = series.filter((s) => s.v.some((x) => Number.isFinite(x)));
  return (
    <figure className="card chart" style={{ margin: 0 }}>
      <figcaption style={{ fontWeight: 600 }}>{title}</figcaption>
      <svg viewBox={`0 0 ${W} ${H}`} role="img" aria-label={title} preserveAspectRatio="none" style={{ height: H }}>
        {ticks.map((f) => (
          <g key={f}>
            <line x1={X0} x2={X1} y1={Y1 - f * (Y1 - Y0)} y2={Y1 - f * (Y1 - Y0)} stroke="var(--line)" />
            <text x={X0 - 6} y={Y1 - f * (Y1 - Y0) + 4} fontSize="11" fill="var(--muted)" textAnchor="end">{fmtV(lLo + f * (lHi - lLo), lHi - lLo)}</text>
            {right.length > 0 && <text x={X1 + 6} y={Y1 - f * (Y1 - Y0) + 4} fontSize="11" fill="var(--muted)">{fmtV(rLo + f * (rHi - rLo), rHi - rLo)}</text>}
            <text x={X(t0 + f * span)} y={H - 8} fontSize="11" fill="var(--muted)" textAnchor={f === 0 ? "start" : f === 1 ? "end" : "middle"}>{fmtT(f * span)}</text>
          </g>
        ))}
        <text x={2} y={10} fontSize="10" fill="var(--muted)">{unit}</text>
        {rightUnit && right.length > 0 && <text x={W - 2} y={10} fontSize="10" fill="var(--muted)" textAnchor="end">{rightUnit}</text>}
        {marks.map((m, i) => (
          <line key={i} x1={X(m.t)} x2={X(m.t)} y1={Y0} y2={Y1} stroke={m.color} strokeDasharray="2 3" strokeWidth="1.2"><title>{m.label}</title></line>
        ))}
        {visible.map((s, i) => {
          const [lo, hi] = s.right ? [rLo, rHi] : [lLo, lHi];
          const pts = reduce(s.t, s.v, t0, t1, X1 - X0);
          return <polyline key={i} fill="none" stroke={s.color} strokeWidth={s.dashed ? 1.4 : 2} strokeDasharray={s.dashed ? "5 4" : undefined}
            vectorEffect="non-scaling-stroke" points={pts.map(([t, v]) => `${X(t).toFixed(1)},${Y(v, lo, hi).toFixed(1)}`).join(" ")} />;
        })}
      </svg>
      <div className="legend">
        {visible.map((s) => <span key={s.label}><i className={s.dashed ? "dash" : undefined} style={s.dashed ? { borderTopColor: s.color } : { background: s.color }} />{s.label}{s.right && rightUnit ? ` (${rightUnit})` : ""}</span>)}
      </div>
    </figure>
  );
}

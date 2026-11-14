import { useRef, useState } from "react";
import { haptic } from "../input/haptics";

/** Touch / mouse stick. Returns x right, y up in -1..1. Each axis can spring back to centre or stay. */
export function Stick({ value, onChange, springX = true, springY = true, size, label, tags }: {
  value: { x: number; y: number };
  onChange: (v: { x: number; y: number }) => void;
  springX?: boolean;
  springY?: boolean;
  size: number;
  label: string;
  tags?: { top?: string; bottom?: string };
}) {
  const ref = useRef<HTMLDivElement>(null);
  const active = useRef<number | null>(null);
  const [, force] = useState(0);
  const lastEdge = useRef("");
  const r = size / 2 - 20;

  const move = (cx: number, cy: number) => {
    const box = ref.current!.getBoundingClientRect();
    let x = (cx - (box.left + box.width / 2)) / r;
    let y = -(cy - (box.top + box.height / 2)) / r;
    x = Math.max(-1, Math.min(1, x));
    y = Math.max(-1, Math.min(1, y));
    const edge = `${Math.abs(x) > 0.98 ? Math.sign(x) : 0}${Math.abs(y) > 0.98 ? Math.sign(y) : 0}${Math.abs(x) < 0.04 && Math.abs(y) < 0.04 ? "c" : ""}`;
    if (edge !== lastEdge.current && edge !== "00") haptic("tick");
    lastEdge.current = edge;
    onChange({ x, y });
  };
  const release = () => {
    active.current = null;
    onChange({ x: springX ? 0 : value.x, y: springY ? 0 : value.y });
    force((n) => n + 1);
  };

  return (
    <div ref={ref} className="stick" role="img" aria-label={label} style={{ width: size, height: size }}
      onPointerDown={(e) => { active.current = e.pointerId; ref.current!.setPointerCapture(e.pointerId); move(e.clientX, e.clientY); }}
      onPointerMove={(e) => { if (active.current === e.pointerId) move(e.clientX, e.clientY); }}
      onPointerUp={release} onPointerCancel={release}>
      {tags?.top && <span className="tag" style={{ top: 4 }}>{tags.top}</span>}
      {tags?.bottom && <span className="tag" style={{ bottom: 4 }}>{tags.bottom}</span>}
      <span className="knob" style={{ left: size / 2 + value.x * r, top: size / 2 - value.y * r }} />
    </div>
  );
}

/**
 * Attitude indicator: horizon and pitch ladder rotate with the roll; the aircraft symbol is fixed.
 * Everything is symmetric about one centre (CX, CY): the aircraft symbol, the ladder bands, the
 * horizon gap, the roll pivot and the roll scale.
 */
const HUD_CX = 260, HUD_CY = 170, HUD_ROLL_R = 150, HUD_ROLL_MAX = 60;
export function Hud({ rollRight, pitchUp, width = 520 }: { rollRight: number; pitchUp: number; width?: number }) {
  const ppd = 6; // pixels per degree of pitch
  const off = Math.max(-60, Math.min(60, pitchUp)) * ppd;
  const cx = HUD_CX, cy = HUD_CY, r = HUD_ROLL_R;
  // roll scale: arc of radius r about the roll pivot, ±HUD_ROLL_MAX (the pointer's travel)
  const sx = r * Math.sin(HUD_ROLL_MAX * Math.PI / 180), sy = cy - r * Math.cos(HUD_ROLL_MAX * Math.PI / 180);
  return (
    <svg viewBox="0 0 520 340" width={width} role="img" aria-label={`Roll ${rollRight.toFixed(0)}°, pitch ${pitchUp.toFixed(0)}°`} style={{ overflow: "visible" }}>
      {/* the ladder stays inside the HUD box (it ran over the heading tape at large pitch angles); the coloured
          horizon behind the cockpit (HorizonBackdrop) shows the attitude everywhere */}
      <defs><clipPath id="hudbox"><rect x="-400" y="0" width="1320" height="340" /></clipPath></defs>
      <g clipPath="url(#hudbox)">
      <g transform={`rotate(${-rollRight} ${cx} ${cy}) translate(0 ${off})`} stroke="var(--hud-line)">
        <line x1={cx - 240} y1={cy} x2={cx - 55} y2={cy} strokeWidth="2" />
        <line x1={cx + 55} y1={cy} x2={cx + 240} y2={cy} strokeWidth="2" />
        {[-20, -10, 10, 20].map((p) => (
          <g key={p} strokeWidth="1.5" fill="none" opacity="0.85" strokeDasharray={p < 0 ? "6 5" : undefined}>
            <path d={`M${cx - 65} ${cy - p * ppd} h-50 v${p > 0 ? 8 : -8}`} />
            <path d={`M${cx + 65} ${cy - p * ppd} h50 v${p > 0 ? 8 : -8}`} />
            <text x={cx - 121} y={cy + 4 - p * ppd} textAnchor="end" fontSize="12" fill="var(--hud-line)" stroke="none" fontFamily="Chakra Petch, sans-serif">{p}</text>
            <text x={cx + 121} y={cy + 4 - p * ppd} textAnchor="start" fontSize="12" fill="var(--hud-line)" stroke="none" fontFamily="Chakra Petch, sans-serif">{p}</text>
          </g>
        ))}
      </g>
      </g>
      {/* aircraft symbol: wings 36 px each side of a 24 px chevron whose apex is at the centre */}
      <path d={`M${cx - 48} ${cy} h36 l12 12 l12 -12 h36`} fill="none" stroke="var(--accent)" strokeWidth="3" strokeLinecap="round" strokeLinejoin="round" />
      <circle cx={cx} cy={cy} r="3" fill="var(--accent)" />
      <path d={`M${cx - sx} ${sy} A ${r} ${r} 0 0 1 ${cx + sx} ${sy}`} fill="none" stroke="var(--line-3)" strokeWidth="1.5" />
      <g transform={`rotate(${-Math.max(-HUD_ROLL_MAX, Math.min(HUD_ROLL_MAX, rollRight))} ${cx} ${cy})`}>
        <path d={`M${cx} ${cy - r} l-7 -12 h14 z`} fill="var(--accent)" />
      </g>
    </svg>
  );
}

/** Heading tape: 0 = north, clockwise. */
export function HeadingTape({ heading, width = 520 }: { heading: number; width?: number }) {
  const pxPerDeg = 4;
  const ticks = [];
  for (let d = Math.floor((heading - 70) / 10) * 10; d <= heading + 70; d += 10) {
    const x = 260 + (d - heading) * pxPerDeg;
    const dd = ((d % 360) + 360) % 360;
    // a label is drawn only when it fits whole: near the edges "210" was clipped to "10"
    const name = dd === 0 ? "N" : dd === 90 ? "E" : dd === 180 ? "S" : dd === 270 ? "W" : dd % 30 === 0 ? String(dd) : "";
    const lbl = x >= 14 + 4 * name.length && x <= 506 - 4 * name.length ? name : "";
    ticks.push(
      <g key={d}>
        <line x1={x} y1={lbl ? 22 : 26} x2={x} y2={34} stroke="var(--text-2)" strokeWidth="1.5" />
        {lbl && <text x={x} y={16} textAnchor="middle" fontSize="13" fill="var(--text-2)" fontFamily="Chakra Petch, sans-serif">{lbl}</text>}
      </g>,
    );
  }
  return (
    <svg viewBox="0 0 520 44" width={width} role="img" aria-label={`Heading ${Math.round(heading)}°`}>
      <defs><clipPath id="tapeclip"><rect x="0" y="0" width="520" height="44" /></clipPath></defs>
      <g clipPath="url(#tapeclip)">
        <line x1="0" y1="34" x2="520" y2="34" stroke="var(--line-3)" />
        {ticks}
      </g>
      <rect x="226" y="0" width="68" height="26" rx="4" fill="var(--bg)" stroke="var(--accent)" strokeWidth="1.5" />
      <text x="260" y="19" textAnchor="middle" fontSize="16" fontWeight="700" fill="var(--text)" fontFamily="Chakra Petch, sans-serif">{String(Math.round(heading) % 360).padStart(3, "0")}°</text>
      <path d="M254 34 L260 42 L266 34 Z" fill="var(--accent)" />
    </svg>
  );
}

export function BatteryIcon({ pct, warn }: { pct: number; warn: number }) {
  const color = warn >= 2 ? "var(--crit)" : warn === 1 ? "var(--warn)" : "var(--ok)";
  return (
    <svg width="34" height="18" viewBox="0 0 34 18" aria-hidden="true">
      <rect x="1" y="1" width="28" height="16" rx="3" fill="none" stroke="var(--text)" strokeWidth="1.5" />
      <rect x="30" y="6" width="3" height="6" rx="1" fill="var(--text)" />
      <rect x="4" y="4" width={Math.max(1, 22 * pct / 100)} height="10" rx="1.5" fill={color} />
    </svg>
  );
}

/** Sky and ground behind the whole cockpit. Same viewBox, centre and pitch scale as Hud, so that placed in a box of
 * the same size and position the horizon lines up with the HUD; the cockpit clips it to the screen. */
export function HorizonBackdrop({ rollRight, pitchUp, width }: { rollRight: number; pitchUp: number; width: number }) {
  const off = Math.max(-60, Math.min(60, pitchUp)) * 6;
  return (
    <svg viewBox="0 0 520 340" width={width} style={{ overflow: "visible" }} aria-hidden="true">
      <defs>
        <linearGradient id="hzsky" gradientUnits="userSpaceOnUse" x1="0" y1="170" x2="0" y2="-530">
          <stop offset="0" stopColor="var(--sky-horizon)" /><stop offset="1" stopColor="var(--sky-zenith)" />
        </linearGradient>
        <linearGradient id="hzgnd" gradientUnits="userSpaceOnUse" x1="0" y1="170" x2="0" y2="870">
          <stop offset="0" stopColor="var(--ground-horizon)" /><stop offset="1" stopColor="var(--ground-deep)" />
        </linearGradient>
      </defs>
      <g transform={`rotate(${-rollRight} 260 170) translate(0 ${off})`}>
        <rect x="-4000" y="-6000" width="8520" height="6170" fill="url(#hzsky)" />
        <rect x="-4000" y="170" width="8520" height="6000" fill="url(#hzgnd)" />
      </g>
    </svg>
  );
}

/** Heading-up compass: the drone points up, the card turns with the heading. North in red, home (when known) as an
 * arrow, amber with "MAG" when the firmware reports magnetic interference (SYSTEM.estFlags bit 3). */
export function MiniCompass({ heading, homeBearing, magBad, size = 116, labels }: {
  heading: number; homeBearing: number | null; magBad: boolean; size?: number;
  labels: { n: string; e: string; s: string; w: string; mag: string };
}) {
  const c = 60;
  const ring = magBad ? "var(--warn)" : "var(--line-3)";
  const ticks = [];
  for (let d = 0; d < 360; d += 30) {
    const lbl = d === 0 ? labels.n : d === 90 ? labels.e : d === 180 ? labels.s : d === 270 ? labels.w : "";
    ticks.push(
      <g key={d} transform={`rotate(${d} ${c} ${c})`}>
        <line x1={c} y1="7" x2={c} y2={lbl ? 15 : 12} stroke={magBad ? "var(--warn)" : "var(--text-2)"} strokeWidth="1.5" />
        {lbl && <text x={c} y="24" textAnchor="middle" dominantBaseline="middle" fontSize="11" fontWeight="700"
          transform={`rotate(${heading - d} ${c} 24)`} fill={d === 0 ? "var(--crit)" : "var(--text)"}>{lbl}</text>}
      </g>,
    );
  }
  const hdg = String(Math.round(((heading % 360) + 360) % 360) % 360).padStart(3, "0");
  return (
    <svg viewBox="0 0 120 134" width={size} height={(size * 134) / 120} role="img" aria-label={`Heading ${hdg}°${magBad ? `, ${labels.mag}` : ""}`}>
      <circle cx={c} cy={c} r="56" fill="color-mix(in srgb, var(--surface) 86%, transparent)" stroke={ring} strokeWidth="1.5" />
      <g transform={`rotate(${-heading} ${c} ${c})`}>
        {ticks}
        {homeBearing !== null && (
          <g transform={`rotate(${homeBearing} ${c} ${c})`}><path d={`M${c} 33 l-6 10 h12 z`} fill="var(--accent)" /></g>
        )}
      </g>
      <path d={`M${c} 46 l-9 20 l9 -5 l9 5 z`} fill="var(--text)" />
      <text x={c} y="85" textAnchor="middle" fontSize="15" fontWeight="700" fill={magBad ? "var(--warn)" : "var(--text)"} fontFamily="Chakra Petch, sans-serif">{hdg}°</text>
      {magBad && (
        <g>
          <rect x={c - 16} y="119" width="32" height="14" rx="3" fill="var(--warn)" />
          <text x={c} y="129.5" textAnchor="middle" fontSize="9.5" fontWeight="700" fill="var(--bg)">{labels.mag}</text>
        </g>
      )}
    </svg>
  );
}

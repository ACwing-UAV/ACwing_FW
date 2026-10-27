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

/** Attitude indicator: horizon and pitch ladder rotate with the roll; the aircraft symbol is fixed. */
export function Hud({ rollRight, pitchUp, width = 520 }: { rollRight: number; pitchUp: number; width?: number }) {
  const ppd = 6; // pixels per degree of pitch
  const off = Math.max(-60, Math.min(60, pitchUp)) * ppd;
  return (
    <svg viewBox="0 0 520 340" width={width} role="img" aria-label={`Roll ${rollRight.toFixed(0)}°, pitch ${pitchUp.toFixed(0)}°`} style={{ overflow: "visible" }}>
      <g transform={`rotate(${-rollRight} 260 170) translate(0 ${off})`} stroke="var(--hud-line)">
        <line x1="20" y1="170" x2="205" y2="170" strokeWidth="2" />
        <line x1="315" y1="170" x2="500" y2="170" strokeWidth="2" />
        {[-20, -10, 10, 20].map((p) => (
          <g key={p} strokeWidth="1.5" fill="none" opacity="0.85" strokeDasharray={p < 0 ? "6 5" : undefined}>
            <path d={`M195 ${170 - p * ppd} h-50 v${p > 0 ? 8 : -8}`} />
            <path d={`M325 ${170 - p * ppd} h50 v${p > 0 ? 8 : -8}`} />
            <text x="118" y={174 - p * ppd} fontSize="12" fill="var(--hud-line)" stroke="none" fontFamily="Chakra Petch, sans-serif">{p}</text>
            <text x="385" y={174 - p * ppd} fontSize="12" fill="var(--hud-line)" stroke="none" fontFamily="Chakra Petch, sans-serif">{p}</text>
          </g>
        ))}
      </g>
      <path d="M200 170 h36 l12 12 l12 -12 h36" fill="none" stroke="var(--accent)" strokeWidth="3" strokeLinecap="round" />
      <circle cx="260" cy="170" r="3" fill="var(--accent)" />
      <path d="M150 52 A 150 150 0 0 1 370 52" fill="none" stroke="var(--line-3)" strokeWidth="1.5" />
      <g transform={`rotate(${-Math.max(-60, Math.min(60, rollRight))} 260 176)`}><path d="M260 26 l-7 -12 h14 z" fill="var(--accent)" /></g>
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
    const lbl = dd === 0 ? "N" : dd === 90 ? "E" : dd === 180 ? "S" : dd === 270 ? "W" : dd % 30 === 0 ? String(dd) : "";
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

/** Stroke icons, 24×24, currentColor. Decorative unless a label is given. */
const P: Record<string, string> = {
  back: "M15 6l-6 6 6 6",
  chevron: "M9 6l6 6-6 6",
  down: "M6 9l6 6 6-6",
  close: "M6 6l12 12M18 6L6 18",
  up: "M12 19V5M5 12l7-7 7 7",
  land: "M12 5v14M5 12l7 7 7-7",
  home: "M3 11l9-7 9 7M5 10v10h14V10",
  pause: "M7 5h3v14H7zM14 5h3v14h-3z",
  power: "M12 3v9M6.3 7.3a8 8 0 1 0 11.4 0",
  pulse: "M3 12h4l3-8 4 16 3-8h4",
  gear: "M12 9a3 3 0 1 0 0 6 3 3 0 0 0 0-6zM12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.1 2.1M17 17l2.1 2.1M4.9 19.1L7 17M17 7l2.1-2.1",
  globe: "M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18zM3 12h18M12 3a14 14 0 0 1 0 18M12 3a14 14 0 0 0 0 18",
  sticks: "M7 10a4 4 0 1 0 0 8 4 4 0 0 0 0-8zM17 10a4 4 0 1 0 0 8 4 4 0 0 0 0-8z",
  shield: "M12 3l8 3v6c0 5-3.5 8-8 9-4.5-1-8-4-8-9V6z",
  target: "M12 4a8 8 0 1 0 0 16 8 8 0 0 0 0-16zM12 4v3M12 17v3M4 12h3M17 12h3M12 10a2 2 0 1 0 0 4 2 2 0 0 0 0-4z",
  bt: "M7 7l10 10-5 5V2l5 5L7 17",
  bell: "M6 16v-5a6 6 0 0 1 12 0v5l2 2H4zM10 20a2 2 0 0 0 4 0",
  chip: "M5 5h14v14H5zM9 2v3M15 2v3M9 19v3M15 19v3M2 9h3M2 15h3M19 9h3M19 15h3",
  check: "M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20zM7 12l3 3 7-7",
  arrowR: "M9 6l6 6-6 6",
  arrowL: "M15 6l-6 6 6 6",
  sim: "M4 5h16v11H4zM8 20h8M12 16v4",
  usb: "M12 2v14M8 6l4-4 4 4M7 12h10v4a5 5 0 0 1-10 0z",
  wifi: "M2 8.8a15 15 0 0 1 20 0M5 12.6a10 10 0 0 1 14 0M8.5 16.4a5 5 0 0 1 7 0M12 20h.01",
  download: "M12 3v12M6 11l6 6 6-6M4 21h16",
  rec: "M12 7a5 5 0 1 0 0 10 5 5 0 0 0 0-10z",
};

export function Icon({ name, size = 20, label, stroke = 1.8 }: { name: keyof typeof P | string; size?: number; label?: string; stroke?: number }) {
  return (
    <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={stroke} strokeLinecap="round" strokeLinejoin="round"
      role={label ? "img" : undefined} aria-label={label} aria-hidden={label ? undefined : true}>
      <path d={P[name] ?? ""} />
    </svg>
  );
}

export function Mark({ className = "brand-mark", alt = "ACwing" }: { className?: string; alt?: string }) {
  return (
    <>
      <img className={`${className} on-dark`} src="./icons/mark_white.png" alt={alt} />
      <img className={`${className} on-day`} src="./icons/mark_black.png" alt={alt} />
    </>
  );
}

export function Logo({ className = "brand-logo" }: { className?: string }) {
  return (
    <>
      <img className={`${className} on-dark`} src="./icons/logo_white.png" alt="ACwing" />
      <img className={`${className} on-day`} src="./icons/logo_black.png" alt="ACwing" />
    </>
  );
}

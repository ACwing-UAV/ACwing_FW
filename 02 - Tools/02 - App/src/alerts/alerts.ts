/**
 * Alerts: drone events become toasts, voice (in the app language) and a
 * vibration; while armed the app also speaks periodic callouts (altitude,
 * battery, flight timer) as configured in Settings > Alerts & voice.
 */
import { drone, type EventEntry } from "../link/drone";
import { appSettings, type AppSettings } from "../settings/appSettings";
import { LANGS, translate } from "../i18n";
import { EventCode, EventLevel } from "../protocol/ids";
import { toast } from "../ui/kit";
import { haptic } from "../input/haptics";

/** Which "Warnings" toggle silences an event; codes not listed are always announced. */
const WARN_OF: Record<number, keyof AppSettings["warn"]> = {
  30: "battLow", 24: "gps", 25: "flow", 23: "fence", 40: "compass", 41: "vibration", 42: "hot", 43: "link",
};
/** Events worth a spoken line even at INFO level. */
const SPOKEN_INFO = new Set([1, 2, 3, 4, 6, 50, 51]);

export function speak(text: string, urgent = false) {
  const st = appSettings.get();
  if (!st.voice || typeof speechSynthesis === "undefined" || !text) return;
  try {
    if (urgent) speechSynthesis.cancel();
    const u = new SpeechSynthesisUtterance(text);
    u.lang = LANGS.find((l) => l.code === st.lang)?.speech ?? "en-GB";
    u.volume = Math.max(0, Math.min(1, st.volume / 100));
    u.rate = 1.05;
    speechSynthesis.speak(u);
  } catch {
    /* speech not available in this webview */
  }
}

export function eventText(e: { code: number; text: string }, lang = appSettings.get().lang) {
  const k = `events.${e.code}`;
  const v = translate(lang, k);
  return v === k ? e.text : v;
}

function onEvent(e: EventEntry) {
  const st = appSettings.get();
  const w = WARN_OF[e.code];
  const critical = e.level >= EventLevel.ERROR;
  if (w && !st.warn[w] && !critical) return;
  const text = eventText(e);
  // the battery monitor stopping is a pre-flight blocker and, in flight, a critical alert (HANDOVER_APP B4)
  const crit = e.level >= EventLevel.CRITICAL || e.code === EventCode.BATTERY_MONITOR_LOST;
  const level = crit ? "critical" : e.level >= EventLevel.ERROR ? "error" : e.level >= EventLevel.WARN ? "warn" : "info";
  toast(text, level);
  if (e.level >= EventLevel.WARN || SPOKEN_INFO.has(e.code)) speak(text, e.level >= EventLevel.ERROR);
  if (e.level >= EventLevel.WARN && st.vibrate) haptic("warn");
}

let lastAltStep: number | null = null;
let lastBattStep: number | null = null;
let lastMinute = 0;

function callouts() {
  const s = drone.getSnapshot();
  const st = appSettings.get();
  const f = s.flight;
  if (!f || !f.arm) {
    lastAltStep = lastBattStep = null;
    lastMinute = 0;
    return;
  }
  const say = (k: string, v: number) => speak(translate(st.lang, `alerts.say.${k}`, { v }));
  if (st.callAlt) {
    const step = Math.floor(Math.max(0, f.altRelM) / st.callAlt);
    if (lastAltStep !== null && step !== lastAltStep && step > 0) say("alt", step * st.callAlt);
    lastAltStep = step;
  }
  if (st.callBatt && s.power) {
    const step = Math.floor(s.power.pct / st.callBatt);
    if (lastBattStep !== null && step < lastBattStep) say("batt", s.power.pct);
    lastBattStep = step;
  }
  const min = Math.floor(f.flightTimeS / 60);
  if (min > lastMinute) {
    if (st.callTimer === "min" || (st.callTimer === "5" && min === 5)) say("timer", min);
    lastMinute = min;
  }
}

let started = false;
export function startAlerts() {
  if (started) return;
  started = true;
  drone.onEvent(onEvent);
  setInterval(callouts, 500);
}

import { useSyncExternalStore } from "react";
import en, { type Dict } from "./en";
import it from "./it";
import fr from "./fr";
import de from "./de";
import es from "./es";
import { appSettings } from "../settings/appSettings";

export type Lang = "en" | "it" | "fr" | "de" | "es";

export const LANGS: { code: Lang; native: string; english: string; speech: string }[] = [
  { code: "en", native: "English", english: "English", speech: "en-GB" },
  { code: "it", native: "Italiano", english: "Italian", speech: "it-IT" },
  { code: "fr", native: "Français", english: "French", speech: "fr-FR" },
  { code: "de", native: "Deutsch", english: "German", speech: "de-DE" },
  { code: "es", native: "Español", english: "Spanish", speech: "es-ES" },
];

type DeepPartial<T> = { [K in keyof T]?: T[K] extends object ? DeepPartial<T[K]> : T[K] };
export type Translation = DeepPartial<Dict>;

export const DICTS: Record<Lang, Translation> = { en, it, fr, de, es };

function lookup(d: unknown, path: string[]): unknown {
  let cur = d;
  for (const k of path) {
    if (cur && typeof cur === "object" && k in (cur as Record<string, unknown>)) cur = (cur as Record<string, unknown>)[k];
    else return undefined;
  }
  return cur;
}

/** Translates a dotted key ("cockpit.autoTakeoff"); {name} placeholders are filled from vars. Falls back to English. */
export function translate(lang: Lang, key: string, vars?: Record<string, string | number>): string {
  const path = key.split(".");
  let v = lookup(DICTS[lang], path);
  if (typeof v !== "string") v = lookup(en, path);
  if (typeof v !== "string") return key;
  return vars ? v.replace(/\{(\w+)\}/g, (_, k) => (k in vars ? String(vars[k]) : `{${k}}`)) : v;
}

export function useT() {
  const s = useSyncExternalStore(appSettings.subscribe, appSettings.get);
  const lang = s.lang;
  const t = (key: string, vars?: Record<string, string | number>) => translate(lang, key, vars);
  return { t, lang };
}

/** Every key of the English dictionary, for the completeness test. */
export function allKeys(o: unknown = en, prefix = ""): string[] {
  if (typeof o !== "object" || o === null) return [prefix];
  return Object.entries(o).flatMap(([k, v]) => allKeys(v, prefix ? `${prefix}.${k}` : k));
}

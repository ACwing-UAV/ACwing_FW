import { describe, expect, it } from "vitest";
import { readFileSync, readdirSync, statSync } from "node:fs";
import { join } from "node:path";
import { DICTS, allKeys, translate } from ".";

function files(dir: string): string[] {
  return readdirSync(dir).flatMap((f) => {
    const p = join(dir, f);
    return statSync(p).isDirectory() ? files(p) : /\.tsx?$/.test(f) && !f.endsWith(".test.ts") ? [p] : [];
  });
}

describe("i18n", () => {
  const keys = new Set(allKeys());
  it("every literal key used in the code exists in English", () => {
    const missing: string[] = [];
    for (const f of files("src")) {
      const src = readFileSync(f, "utf8");
      for (const m of src.matchAll(/\bt\("([a-zA-Z0-9_.]+)"/g)) if (!keys.has(m[1])) missing.push(`${f}: ${m[1]}`);
    }
    expect(missing).toEqual([]);
  });
  for (const lang of ["it", "fr", "de", "es"] as const) {
    it(`${lang} translates every key and keeps placeholders`, () => {
      const missing: string[] = [];
      for (const k of keys) {
        const path = k.split(".");
        let v: unknown = DICTS[lang];
        for (const p of path) v = v && typeof v === "object" ? (v as Record<string, unknown>)[p] : undefined;
        if (typeof v !== "string") { missing.push(k); continue; }
        const ph = (s: string) => [...s.matchAll(/\{(\w+)\}/g)].map((m) => m[1]).sort().join(",");
        if (ph(v) !== ph(translate("en", k))) missing.push(`${k} (placeholders)`);
      }
      expect(missing).toEqual([]);
    });
  }

  // bench report (#95): "Non eseguito: è collegato un cavo USB a un computer: scollegalo" had two colons. The
  // texts that go into a "...: {why}" template must not carry one themselves.
  it("the texts inserted after a colon have no colon of their own", () => {
    const bad: string[] = [];
    for (const lang of Object.keys(DICTS) as (keyof typeof DICTS)[]) {
      for (const block of ["reasons", "alerts.launchAbort", "alerts.flipAbort", "alerts.radioOp"]) {
        for (const k of allKeys().filter((x) => x.startsWith(`${block}.`))) {
          const v = translate(lang, k);
          if (v.includes(":")) bad.push(`${lang} ${k}: ${v}`);
        }
      }
    }
    expect(bad).toEqual([]);
  });
  it("the event texts with an argument are filled in (Telemetry and the log viewer use eventText)", async () => {
    const { eventText } = await import("../alerts/alerts");
    for (const lang of Object.keys(DICTS) as (keyof typeof DICTS)[]) {
      for (const e of [{ code: 12, arg: 1 }, { code: 14, arg: 2 }, { code: 9, arg: (0x12 << 8) | 17 }]) {
        const v = eventText({ ...e, text: "" }, lang);
        expect(v, `${lang} event ${e.code}`).not.toMatch(/\{\w+\}/);
        expect((v.match(/:/g) ?? []).length, `${lang} event ${e.code}: ${v}`).toBeLessThanOrEqual(1);
      }
    }
  });
});

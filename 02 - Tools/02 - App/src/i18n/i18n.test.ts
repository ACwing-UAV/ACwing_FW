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
});

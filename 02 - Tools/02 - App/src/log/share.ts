/**
 * Hands files to the user: the share sheet where there is one (phones,
 * native apps), a download otherwise (PC browsers).
 */
import { Capacitor } from "@capacitor/core";

export interface OutFile {
  name: string;
  data: Uint8Array | string;
  type: string;
}

const toBase64 = (d: Uint8Array) => {
  let s = "";
  for (let i = 0; i < d.length; i += 0x8000) s += String.fromCharCode(...d.subarray(i, i + 0x8000));
  return btoa(s);
};

export async function shareFiles(files: OutFile[], title: string) {
  if (Capacitor.isNativePlatform()) {
    // native shells: write to the cache folder, then the system share sheet (Files, Drive, mail…)
    const { Filesystem, Directory, Encoding } = await import("@capacitor/filesystem");
    const { Share } = await import("@capacitor/share");
    const uris: string[] = [];
    for (const f of files) {
      const w = typeof f.data === "string"
        ? await Filesystem.writeFile({ path: f.name, data: f.data, directory: Directory.Cache, encoding: Encoding.UTF8 })
        : await Filesystem.writeFile({ path: f.name, data: toBase64(f.data), directory: Directory.Cache });
      uris.push(w.uri);
    }
    await Share.share({ title, files: uris });
    return;
  }
  const blobs = files.map((f) => new File([typeof f.data === "string" ? f.data : (f.data as Uint8Array<ArrayBuffer>)], f.name, { type: f.type }));
  const nav = navigator as Navigator & { canShare?: (d: unknown) => boolean };
  const touch = matchMedia?.("(pointer: coarse)").matches;
  if (touch && nav.canShare?.({ files: blobs })) {
    try {
      await navigator.share({ files: blobs, title });
      return;
    } catch (e) {
      if ((e as Error).name === "AbortError") return;
    }
  }
  for (const b of blobs) {
    const a = document.createElement("a");
    a.href = URL.createObjectURL(b);
    a.download = b.name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 4000);
    await new Promise((r) => setTimeout(r, 250)); // browsers drop back-to-back downloads
  }
}

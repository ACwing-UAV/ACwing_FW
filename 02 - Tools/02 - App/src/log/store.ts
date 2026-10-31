/**
 * Downloaded flight logs, kept on this device: the raw `.bin` (what the PC
 * tools read) plus a little metadata. IndexedDB in the browser/PWA and in the
 * native shells' WebView; an in-memory fallback where IndexedDB is missing
 * (tests, some private windows).
 *
 * A log downloaded twice (same drone, same log id, same start time) is kept once.
 */
import { decodeLog, summarize, type LogSummary } from "./format";

export interface StoredLogMeta {
  key: string;
  /** drone name, from the log's INFO record */
  drone: string;
  logId: number;
  /** ms after the drone's power-on (the drone has no clock) */
  t0Ms: number;
  size: number;
  /** ISO date of the download: the only wall-clock time a log has */
  downloadedAt: string;
  source: "ble" | "usb";
  fw: string | null;
  truncated: boolean;
  summary: LogSummary;
}

const DB = "acwing-pilot", META = "logMeta", DATA = "logData";

export const logKey = (drone: string, logId: number, t0Ms: number) => `${drone}|${logId}|${t0Ms}`;

type Listener = () => void;

class LogStore {
  private db: Promise<IDBDatabase | null> | null = null;
  private mem = new Map<string, { meta: StoredLogMeta; data: Uint8Array }>();
  private listeners = new Set<Listener>();
  private version = 0;

  subscribe = (fn: Listener) => {
    this.listeners.add(fn);
    return () => void this.listeners.delete(fn);
  };
  getVersion = () => this.version;
  private changed() {
    this.version++;
    this.listeners.forEach((l) => l());
  }

  private open(): Promise<IDBDatabase | null> {
    if (this.db) return this.db;
    this.db = new Promise((resolve) => {
      try {
        if (typeof indexedDB === "undefined") return resolve(null);
        const r = indexedDB.open(DB, 1);
        r.onupgradeneeded = () => {
          r.result.createObjectStore(META, { keyPath: "key" });
          r.result.createObjectStore(DATA);
        };
        r.onsuccess = () => resolve(r.result);
        r.onerror = () => resolve(null);
      } catch {
        resolve(null);
      }
    });
    return this.db;
  }

  private tx<T>(db: IDBDatabase, stores: string[], mode: IDBTransactionMode, fn: (t: IDBTransaction) => IDBRequest<T> | void): Promise<T | undefined> {
    return new Promise((resolve, reject) => {
      const t = db.transaction(stores, mode);
      const req = fn(t);
      t.oncomplete = () => resolve(req ? req.result : undefined);
      t.onerror = () => reject(t.error);
      t.onabort = () => reject(t.error);
    });
  }

  async list(): Promise<StoredLogMeta[]> {
    const db = await this.open();
    const all = db ? ((await this.tx(db, [META], "readonly", (t) => t.objectStore(META).getAll())) as StoredLogMeta[]) : [...this.mem.values()].map((x) => x.meta);
    return all.sort((a, b) => b.downloadedAt.localeCompare(a.downloadedAt) || b.logId - a.logId);
  }

  async get(key: string): Promise<{ meta: StoredLogMeta; data: Uint8Array } | null> {
    const db = await this.open();
    if (!db) return this.mem.get(key) ?? null;
    const meta = (await this.tx(db, [META], "readonly", (t) => t.objectStore(META).get(key))) as StoredLogMeta | undefined;
    const data = (await this.tx(db, [DATA], "readonly", (t) => t.objectStore(DATA).get(key))) as Uint8Array | undefined;
    return meta && data ? { meta, data: new Uint8Array(data) } : null;
  }

  /** Stores a downloaded log (replacing the same key) and returns its metadata. */
  async put(data: Uint8Array, info: { drone?: string; logId: number; t0Ms: number; source: "ble" | "usb"; truncated?: boolean }): Promise<StoredLogMeta> {
    const L = decodeLog(data);
    const drone = info.drone || L.info?.name || "ACwing";
    const meta: StoredLogMeta = {
      key: logKey(drone, info.logId, info.t0Ms), drone, logId: info.logId, t0Ms: info.t0Ms, size: data.length,
      downloadedAt: new Date().toISOString(), source: info.source, fw: L.info?.fw ?? null, truncated: !!info.truncated, summary: summarize(L),
    };
    const db = await this.open();
    if (!db) this.mem.set(meta.key, { meta, data });
    else
      await this.tx(db, [META, DATA], "readwrite", (t) => {
        t.objectStore(META).put(meta);
        t.objectStore(DATA).put(data, meta.key);
      });
    this.changed();
    return meta;
  }

  async has(key: string) {
    return (await this.list()).some((m) => m.key === key);
  }

  async remove(key: string) {
    const db = await this.open();
    if (!db) this.mem.delete(key);
    else
      await this.tx(db, [META, DATA], "readwrite", (t) => {
        t.objectStore(META).delete(key);
        t.objectStore(DATA).delete(key);
      });
    this.changed();
  }
}

export const logStore = new LogStore();

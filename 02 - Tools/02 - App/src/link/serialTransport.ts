/**
 * USB link to the FC: its USB-C port is a CDC virtual COM port (VID/PID
 * 0x0483/0x5740, baud rate ignored). Web Serial in Chrome and Edge on PC/Mac
 * (and Android Chrome where it supports USB serial). Not available on iOS,
 * nor inside the native Capacitor shells (no Web Serial in the WebView).
 *
 * Today the port carries only the flight-log commands (HANDOVER_APP.md §4.2).
 */
import { Capacitor } from "@capacitor/core";
import { USB_IDS } from "../protocol/ids";
import { LinkError, type Transport } from "./transport";

/* Minimal Web Serial typings (not in the TypeScript DOM library). */
interface SerialPortLike {
  open(o: { baudRate: number }): Promise<void>;
  close(): Promise<void>;
  readable: ReadableStream<Uint8Array> | null;
  writable: WritableStream<Uint8Array> | null;
  getInfo(): { usbVendorId?: number; usbProductId?: number };
}
interface SerialLike {
  requestPort(o?: { filters?: { usbVendorId?: number; usbProductId?: number }[] }): Promise<SerialPortLike>;
  getPorts(): Promise<SerialPortLike[]>;
  addEventListener(t: "disconnect", fn: (e: Event & { target: unknown }) => void): void;
  removeEventListener(t: "disconnect", fn: (e: Event & { target: unknown }) => void): void;
}
const serial = (): SerialLike | null => (typeof navigator !== "undefined" && "serial" in navigator ? ((navigator as unknown as { serial: SerialLike }).serial) : null);

/** Web Serial is usable here (desktop Chrome/Edge, Android Chrome; not iOS, not the native shells). */
export function usbAvailable() {
  return !!serial() && !Capacitor.isNativePlatform();
}

const isFc = (p: SerialPortLike) => {
  const i = p.getInfo();
  return i.usbVendorId === USB_IDS.vendorId && i.usbProductId === USB_IDS.productId;
};

/** A port granted earlier (no picker), if the FC is plugged in. */
export async function grantedFcPort(): Promise<SerialTransport | null> {
  const s = serial();
  if (!s) return null;
  const p = (await s.getPorts()).find(isFc);
  return p ? new SerialTransport(p) : null;
}

/** Shows the browser's port picker, filtered on the FC. */
export async function pickFcPort(): Promise<SerialTransport> {
  const s = serial();
  if (!s) throw new LinkError("unsupported", "Web Serial not available");
  try {
    return new SerialTransport(await s.requestPort({ filters: [{ usbVendorId: USB_IDS.vendorId, usbProductId: USB_IDS.productId }] }));
  } catch (e) {
    throw new LinkError((e as Error).name === "NotFoundError" ? "notFound" : "failed", (e as Error).message);
  }
}

export class SerialTransport implements Transport {
  readonly kind = "usb" as const;
  readonly deviceName = "ACwing FC (USB)";
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
  private running = false;
  private onDisconnect: ((e: Event & { target: unknown }) => void) | null = null;

  constructor(private port: SerialPortLike) {}

  async open(onData: (chunk: Uint8Array) => void, onClose: (reason: string) => void) {
    try {
      await this.port.open({ baudRate: 115200 }); // ignored by the CDC device
    } catch (e) {
      throw new LinkError("failed", (e as Error).message);
    }
    this.writer = this.port.writable!.getWriter();
    this.running = true;
    this.onDisconnect = (e) => {
      if ((e.target as unknown) === this.port) {
        this.running = false;
        onClose("unplugged");
      }
    };
    serial()?.addEventListener("disconnect", this.onDisconnect);
    void (async () => {
      while (this.running && this.port.readable) {
        this.reader = this.port.readable.getReader();
        try {
          for (;;) {
            const { value, done } = await this.reader.read();
            if (done) break;
            if (value?.length) onData(value);
          }
        } catch {
          // a framing/buffer error: reopen the reader, a cut cable ends the loop
        } finally {
          this.reader.releaseLock();
          this.reader = null;
        }
      }
      if (this.running) {
        this.running = false;
        onClose("closed");
      }
    })();
  }

  async write(bytes: Uint8Array) {
    if (!this.writer) throw new LinkError("failed", "not open");
    await this.writer.write(bytes);
  }

  async close() {
    this.running = false;
    if (this.onDisconnect) serial()?.removeEventListener("disconnect", this.onDisconnect);
    try {
      await this.reader?.cancel();
    } catch {
      /* already gone */
    }
    try {
      this.writer?.releaseLock();
      await this.port.close();
    } catch {
      /* unplugged */
    }
    this.writer = null;
  }

  async rssi() {
    return null;
  }

  async mtu() {
    return null;
  }
}

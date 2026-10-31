/**
 * The two ways to the drone's flight log (HANDOVER_APP.md §4):
 *  - BleLogPort: through the flight session (`drone`), commands written raw to
 *    the CMD characteristic, answers in the telemetry stream. Last flight only.
 *  - UsbLogPort: its own link on the FC's USB CDC port (Web Serial, or the
 *    simulator), commands wrapped in CMD frames. Every log, and erase.
 * The USB link is separate from the BLE session: a pilot connected over BLE
 * can plug USB in to download logs.
 */
import { drone, type Drone } from "../link/drone";
import type { Transport } from "../link/transport";
import { StreamParser } from "../protocol/frame";
import { encodeCmdFrame } from "../protocol/commands";
import { appSettings } from "../settings/appSettings";
import type { LogPort } from "./client";

const cmdBytes = (op: number, seq: number, args: Uint8Array) => {
  const b = new Uint8Array(2 + args.length);
  b[0] = op;
  b[1] = seq;
  b.set(args, 2);
  return b;
};

export class BleLogPort implements LogPort {
  readonly kind = "ble" as const;

  constructor(private d: Drone = drone) {}

  send(op: number, args: Uint8Array) {
    return this.d.sendRaw((seq) => cmdBytes(op, seq, args));
  }

  listen(fn: (type: number, p: Uint8Array) => void) {
    const off = this.d.onFrame(fn);
    return () => void off();
  }

  /** While a download runs the STATE stream would starve: 1 Hz meanwhile, then back to the user's rate. */
  begin() {
    this.d.setStateRate(1);
  }

  end() {
    this.d.setStateRate(appSettings.get().telemetryRate);
  }
}

export class UsbLogPort implements LogPort {
  readonly kind = "usb" as const;
  private parser = new StreamParser();
  private listeners = new Set<(type: number, p: Uint8Array) => void>();
  private seq = 0;
  private closed = false;

  constructor(readonly transport: Transport) {}

  get name() {
    return this.transport.deviceName;
  }

  async open(onClose?: (reason: string) => void) {
    await this.transport.open(
      (chunk) => {
        for (const f of this.parser.push(chunk)) this.listeners.forEach((l) => l(f.type, f.payload));
      },
      (reason) => {
        this.closed = true;
        onClose?.(reason);
      },
    );
  }

  get isClosed() {
    return this.closed;
  }

  send(op: number, args: Uint8Array) {
    this.seq = (this.seq % 255) + 1;
    this.transport.write(encodeCmdFrame(cmdBytes(op, this.seq, args)), true).catch(() => undefined);
    return this.seq;
  }

  listen(fn: (type: number, p: Uint8Array) => void) {
    this.listeners.add(fn);
    return () => void this.listeners.delete(fn);
  }

  async close() {
    this.closed = true;
    await this.transport.close();
  }
}

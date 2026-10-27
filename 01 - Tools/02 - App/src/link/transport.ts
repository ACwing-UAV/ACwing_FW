/** A byte pipe to one drone: BLE today, USB/Wi-Fi bridge later, simulator for development. */
export interface Transport {
  readonly kind: "ble" | "sim";
  readonly deviceName: string;
  /** Opens the link and starts delivering raw notification bytes. */
  open(onData: (chunk: Uint8Array) => void, onClose: (reason: string) => void): Promise<void>;
  /** Writes one command. withResponse = false for the 50 Hz RC stream and heartbeats. */
  write(bytes: Uint8Array, withResponse: boolean): Promise<void>;
  close(): Promise<void>;
  /** Signal strength seen by this device, dBm (null if the platform cannot read it). */
  rssi(): Promise<number | null>;
  /** ATT MTU in use, when the platform exposes it. */
  mtu(): Promise<number | null>;
}

export class LinkError extends Error {
  constructor(public code: "unsupported" | "denied" | "notFound" | "failed", message: string) {
    super(message);
  }
}

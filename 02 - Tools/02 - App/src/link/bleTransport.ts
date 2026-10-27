/**
 * BLE transport on @capacitor-community/bluetooth-le: native BLE on iOS and
 * Android, Web Bluetooth in Chrome/Edge on Windows, macOS, Linux, Android and
 * ChromeOS. Safari (iPhone/iPad web) has no Web Bluetooth: there the app must
 * run as the native iOS build.
 */
import { BleClient, type BleDevice } from "@capacitor-community/bluetooth-le";
import { Capacitor } from "@capacitor/core";
import { BLE } from "../protocol/ids";
import { LinkError, type Transport } from "./transport";

let initialised = false;

export function bleAvailable(): { ok: boolean; reason?: "noWebBluetooth" } {
  if (Capacitor.isNativePlatform()) return { ok: true };
  const nav = navigator as Navigator & { bluetooth?: unknown };
  return nav.bluetooth ? { ok: true } : { ok: false, reason: "noWebBluetooth" };
}

async function init() {
  if (initialised) return;
  try {
    await BleClient.initialize({ androidNeverForLocation: true });
    initialised = true;
  } catch (e) {
    throw new LinkError("unsupported", String(e));
  }
}

/** Opens the system picker filtered on ACwing names. */
export async function pickBleDrone(): Promise<BleTransport> {
  await init();
  let dev: BleDevice;
  try {
    dev = await BleClient.requestDevice({
      namePrefix: BLE.namePrefixes[0],
      optionalServices: [BLE.service],
    });
  } catch (e) {
    const msg = String(e);
    if (/cancel/i.test(msg)) throw new LinkError("denied", msg);
    throw new LinkError("notFound", msg);
  }
  return new BleTransport(dev);
}

export class BleTransport implements Transport {
  readonly kind = "ble" as const;
  constructor(private dev: BleDevice) {}

  get deviceName() {
    return this.dev.name ?? this.dev.deviceId;
  }

  async open(onData: (chunk: Uint8Array) => void, onClose: (reason: string) => void) {
    try {
      await BleClient.connect(this.dev.deviceId, () => onClose("disconnected"));
      await BleClient.startNotifications(this.dev.deviceId, BLE.service, BLE.tlm, (dv) =>
        onData(new Uint8Array(dv.buffer, dv.byteOffset, dv.byteLength).slice()),
      );
    } catch (e) {
      throw new LinkError("failed", String(e));
    }
  }

  async write(bytes: Uint8Array, withResponse: boolean) {
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    if (withResponse) await BleClient.write(this.dev.deviceId, BLE.service, BLE.cmd, dv);
    else await BleClient.writeWithoutResponse(this.dev.deviceId, BLE.service, BLE.cmd, dv);
  }

  async close() {
    try {
      await BleClient.stopNotifications(this.dev.deviceId, BLE.service, BLE.tlm);
    } catch {
      /* already gone */
    }
    try {
      await BleClient.disconnect(this.dev.deviceId);
    } catch {
      /* already gone */
    }
  }

  async rssi() {
    try {
      return await BleClient.readRssi(this.dev.deviceId);
    } catch {
      return null;
    }
  }

  async mtu() {
    try {
      return await BleClient.getMtu(this.dev.deviceId);
    } catch {
      return null;
    }
  }
}

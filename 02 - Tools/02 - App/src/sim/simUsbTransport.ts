/**
 * Simulated USB link to the FC (HANDOVER_APP.md §8): the drone's CDC port,
 * which today serves only the flight-log commands. Commands arrive wrapped in
 * CMD frames (0x20); answers leave as LOG_ENTRY / LOG_DATA / ACK frames, cut
 * in pieces like a USB read.
 *
 * It talks to the same SimDrone as the BLE simulator when one is connected
 * (same logs), otherwise to its own drone, which it then ticks itself.
 */
import type { Transport } from "../link/transport";
import { StreamParser } from "../protocol/frame";
import { FT } from "../protocol/ids";
import { SimDrone, type SimModel } from "./simDrone";
import { lastSimDrone } from "./simTransport";

export class SimUsbTransport implements Transport {
  readonly kind = "usb" as const;
  readonly deviceName = "ACwing FC (USB, simulator)";
  readonly drone: SimDrone;
  private timer: ReturnType<typeof setInterval> | null = null;
  private parser = new StreamParser();
  private last = 0;

  constructor(drone?: SimDrone, model: SimModel = "indoor") {
    this.drone = drone ?? lastSimDrone() ?? new SimDrone(model);
  }

  async open(onData: (chunk: Uint8Array) => void) {
    this.last = performance.now();
    this.timer = setInterval(() => {
      const now = performance.now();
      const dt = Math.min(0.1, (now - this.last) / 1000);
      this.last = now;
      // nobody else is simulating this drone: keep its clock running
      if (Date.now() - this.drone.lastTickAt > 60) {
        this.drone.tick(dt);
        this.drone.drain(); // its BLE frames go nowhere
      }
      const out = this.drone.drainUsb();
      if (!out.length) return;
      const all = new Uint8Array(out.reduce((n, f) => n + f.length, 0));
      let o = 0;
      for (const f of out) {
        all.set(f, o);
        o += f.length;
      }
      for (let i = 0; i < all.length; i += 1000) onData(all.subarray(i, i + 1000)); // USB reads come in pieces
    }, 20);
  }

  async write(bytes: Uint8Array) {
    setTimeout(() => {
      for (const f of this.parser.push(bytes)) if (f.type === FT.CMD) this.drone.usbCommand(f.payload);
    }, 2);
  }

  async close() {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
  }

  async rssi() {
    return null;
  }

  async mtu() {
    return null;
  }
}

/**
 * Simulator transport: the SimDrone frames are concatenated and cut into
 * 20-byte "notifications" (ATT MTU 23, the worst case), exactly like the BLE
 * TX thread of the firmware, so the app always goes through the real parser.
 */
import type { Transport } from "../link/transport";
import { SimDrone, type SimModel } from "./simDrone";

export class SimTransport implements Transport {
  readonly kind = "sim" as const;
  readonly drone: SimDrone;
  private timer: ReturnType<typeof setInterval> | null = null;
  private pending: number[] = [];
  private last = 0;

  constructor(model: SimModel, private chunk = 20) {
    this.drone = new SimDrone(model);
  }

  get deviceName() {
    return `${this.drone.name} (simulator)`;
  }

  async open(onData: (chunk: Uint8Array) => void) {
    this.drone.onSubscribe();
    this.last = performance.now();
    this.timer = setInterval(() => {
      const now = performance.now();
      const dt = Math.min(0.1, (now - this.last) / 1000);
      this.last = now;
      this.drone.tick(dt);
      for (const f of this.drone.drain()) for (const b of f) this.pending.push(b);
      // deliver in MTU-sized pieces that ignore frame boundaries
      while (this.pending.length) onData(Uint8Array.from(this.pending.splice(0, this.chunk)));
    }, 20);
  }

  async write(bytes: Uint8Array) {
    // a little radio latency, as on a real link
    setTimeout(() => this.drone.command(bytes), 15);
  }

  async close() {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
  }

  async rssi() {
    return -61;
  }

  async mtu() {
    return this.chunk + 3;
  }
}

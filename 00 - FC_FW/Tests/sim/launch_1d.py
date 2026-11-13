"""1-D estimate of a hand launch (ADR 0011): release detection, motor start once the drone is far enough from the
hand, vertical recovery. The hand is taken as staying where the drone left it.
Run: python3 launch_1d.py"""
import math

G = 9.81


def launch(tw, v0, detect_ms=60, sep_m=0.10, dead_ms=30, tau=0.035, dt=1e-4):
    """returns (lowest point, motor start ms, separation at the start) relative to the release point"""
    v = v0; h = 0.0; T = 0.0; t = 0.0; hmin = 0.0; cmd = 0.0; start = None
    while t < 2.5:
        # the separation is |h|: the hand stays at the release point
        if start is None and t * 1000 >= detect_ms and abs(h) >= sep_m:
            start = t
        if start is not None and t >= start + dead_ms * 1e-3:
            cmd = 1.0 if v < -0.05 else 1.0 / tw
        T += (cmd - T) * dt / tau
        v += (T * tw - 1) * G * dt
        h += v * dt
        hmin = min(hmin, h)
        t += dt
    return hmin, (start or 0) * 1000


if __name__ == "__main__":
    print("separation-gated start (10 cm), release detected after 60 ms of free fall, 30 ms to start, lag 35 ms")
    for name, tw in [("T400", 1.97), ("T800", 1.71), ("T1200", 1.55)]:
        for V in (4.2, 3.7):
            k = (V / 4.2) ** 2
            d = launch(tw * k, 0.0)
            u = launch(tw * k, 1.5)
            print(f"{name} {V} V T/W {tw * k:.2f}: drop lowest {d[0]:+.2f} m (start {d[1]:.0f} ms)   "
                  f"toss 1.5 m/s lowest {u[0]:+.2f} m (start {u[1]:.0f} ms)")

"""1-D estimate of a hand launch (ADR 0011): release detection, motor start, vertical recovery.
Run: python3 launch_1d.py"""
import math
def launch(tw, v0, detect_ms=60, dead_ms=30, tau=0.035, dt=1e-4):
    g=9.81; v=v0; h=0; T=0; t=0; hmin=0; thr_cmd=0
    while t<2.5:
        released = t*1000 >= detect_ms             # free fall seen
        if released and t*1000 >= detect_ms+dead_ms:
            # vertical recovery: full thrust while falling, then hover
            thr_cmd = 1.0 if v < -0.05 else 1/tw
        T += (thr_cmd - T)*dt/tau
        v += (T*tw - 1)*g*dt; h += v*dt; hmin=min(hmin,h); t+=dt
    return hmin, h
for name,tw in [("T400",1.97),("T800",1.71),("T1200",1.55)]:
    for V in (4.2,3.7):
        k=(V/4.2)**2
        drop=launch(tw*k,0.0); toss=launch(tw*k,1.5)
        print(f"{name} {V} V T/W {tw*k:.2f}: drop lowest {drop[0]:+.2f} m   toss 1.5 m/s up lowest {toss[0]:+.2f} m")

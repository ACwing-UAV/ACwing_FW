"""1-D estimate of an autonomous flip (ADR 0012): rotation with a torque limit and the motor lag, vertical with
the thrust available at the cell voltage. Not a validation: the closed-loop host simulation is (ADR 0012).
Run: python3 flip_1d.py"""
import math
def flip(tw, rate_dps=1400, t_climb=0.18, col_rot=0.25, alpha_max=520.0, tau=0.035, dt=1e-4):
    g=9.81; th=0; w=0; v=0; h=0; T=1/tw  # thrust as fraction of max, start at hover
    t=0; phase='climb'; hmin=0; hmax=0; tphase=0; wsp=math.radians(rate_dps)
    log=[]; done=None; trot=None
    while t<3:
        if phase=='climb':
            cmd=1.0; u=0
            if t>=t_climb: phase='rot'; tphase=t
        elif phase=='rot':
            cmd=col_rot
            # rate loop: bang toward wsp, braking lead
            lead = w*w/(2*alpha_max)
            if th >= 2*math.pi - lead - math.radians(10): phase='rec'; tphase=t
            u = max(-1,min(1,(wsp-w)/ (alpha_max*0.02)))
        elif phase=='rec':
            # attitude P to 0 (mod 2pi) + rate damping
            err = (2*math.pi - th)
            wcmd = 6*err if abs(err)>0 else 0
            u = max(-1,min(1,(wcmd - w)/(alpha_max*0.02)))
            cmd = min(1.0, (1/tw)/max(0.3,math.cos(th)) + 0.3*max(0,-v)) if math.cos(th)>0 else 0.3
        # torque authority shrinks if collective at extremes (airmode keeps it); assume alpha_max
        T += (cmd - T)*dt/tau
        a_rot = u*alpha_max
        w += a_rot*dt; th += w*dt
        v += (T*tw*math.cos(th)-1)*g*dt; h += v*dt
        hmin=min(hmin,h); hmax=max(hmax,h)
        t+=dt
        if phase=='rec' and abs(2*math.pi-th)<math.radians(5) and abs(w)<1 and done is None: done=t
        if phase=='rot' and trot is None: trot=t
    return dict(t=done,hmin=hmin,hmax=hmax,trot=tphase)
for name,tw in [("T400",1.97),("T800",1.71),("T1200",1.55)]:
    for V in (4.2,3.9,3.6):
        k=(V/4.2)**2
        r=flip(tw*k)
        print(f"{name} {V} V  T/W {tw*k:.2f}: done {r['t'] and round(r['t'],2)} s  lowest {r['hmin']:+.2f} m  highest {r['hmax']:+.2f} m")

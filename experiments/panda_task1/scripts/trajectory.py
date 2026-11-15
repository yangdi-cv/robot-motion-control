"""Rest-to-rest quintic reference; units radians and seconds."""
import numpy as np


def reference(t, duration, initial, target):
    u=np.clip(t/duration,0.,1.)
    blend=10*u**3-15*u**4+6*u**5
    rate=(30*u**2-60*u**3+30*u**4)/duration
    acceleration=(60*u-180*u**2+120*u**3)/duration**2
    start=np.asarray(initial,dtype=float);delta=np.asarray(target,dtype=float)-start
    return start+blend*delta,rate*delta,acceleration*delta

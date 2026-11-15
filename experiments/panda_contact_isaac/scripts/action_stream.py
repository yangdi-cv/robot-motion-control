"""Timed action chunks and bounded execution for synthetic policy-interface tests."""
from collections import deque
import numpy as np

CASES=('oracle','delayed_naive','delayed_timestamp','jitter_raw','jitter_limited','outage_stale','outage_hold')

def reference(t, initial, amplitude=3.):
    u=np.clip(t/5.,0.,1.)
    blend=10*u**3-15*u**4+6*u**5
    q=initial.copy()
    q[:3]+=amplitude*np.array([.06,.04,-.04])*blend
    return q

class ActionStream:
    def __init__(self, case, initial, period=.2, delay=.12, horizon_steps=40, timeout=.5):
        self.case=case
        self.period=period
        self.delay=delay
        self.horizon_steps=horizon_steps
        self.timeout=timeout
        self.exhausted=False
        self.initial=initial.copy()
        self.pending=deque()
        self.active=None
        self.next_generation=0.
        self.command=initial.copy()
        self.velocity=np.zeros_like(initial)
        self.hold=None
        self.timeout_events=0
        self.received=0
        self.generated=0

    def update(self,t,measured):
        case=self.case
        oracle=reference(t,self.initial)
        if case=='oracle':
            self.command=oracle.copy()
            return self.command.copy(),oracle,0.,False
        delay=self.delay
        while self.next_generation<=t+1e-6:
            generated=self.next_generation
            index=round(generated/self.period)
            outage=case.startswith('outage') and 2.<=generated<3.
            if not outage:
                chunk=np.stack([reference(generated+k*.01,self.initial) for k in range(self.horizon_steps)])
                if case.startswith('jitter'):
                    chunk[:,0]+=.04*(-1 if index%2 else 1)
                self.pending.append({'generated':generated,'arrival':generated+delay,'actions':chunk})
                self.generated+=1
            self.next_generation+=self.period
        while self.pending and self.pending[0]['arrival']<=t+1e-6:
            self.active=self.pending.popleft()
            self.received+=1
        age=t-self.active['generated'] if self.active else float(t)
        timed_out=age>self.timeout
        if self.active is None:
            target=self.initial.copy()
        else:
            elapsed=t-(self.active['arrival'] if case=='delayed_naive' else self.active['generated'])
            self.exhausted=(t-self.active["generated"])>=self.horizon_steps*.01-1e-6
            point=min(self.horizon_steps-1,max(0,int((elapsed+1e-6)/.01)))
            target=self.active['actions'][point].copy()
        if case=='outage_hold' and timed_out:
            if self.hold is None:
                self.hold=measured.copy()
                self.timeout_events+=1
            target=self.hold.copy()
        else:
            self.hold=None
        if case in ('jitter_limited','outage_hold'):
            # Velocity and acceleration bounds apply to the command, not a collision guarantee.
            velocity=np.clip((target-self.command)/.01,-.35,.35)
            self.velocity+=np.clip(velocity-self.velocity,-.01,.01)
            increment=self.velocity*.01
            self.command+=increment
            self.velocity=increment/.01
        else:
            self.command=target.copy()
        return self.command.copy(),oracle,age,timed_out

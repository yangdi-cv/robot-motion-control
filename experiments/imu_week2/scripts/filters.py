"""Causal six-axis filters; state never reads future samples."""
from collections import deque
import numpy as np
from scipy.signal import butter,sosfilt,sosfilt_zi
class FilterBank:
 def __init__(self,rate=25,cutoff=3,window=5):
  self.rate=rate;self.cutoff=cutoff;self.window=window;self.buffer=deque(maxlen=window);self.ema=None
  self.sos=butter(2,cutoff,fs=rate,output='sos');self.zi=None
 def update(self,value,dt=None):
  value=np.asarray(value,dtype=float)
  if value.shape!=(6,) or not np.isfinite(value).all():raise ValueError('Expected finite accel+gyro six-vector')
  self.buffer.append(value.copy())
  dt=1/self.rate if dt is None else dt
  alpha=1-np.exp(-2*np.pi*self.cutoff*dt)
  self.ema=value.copy() if self.ema is None else self.ema+alpha*(value-self.ema)
  if self.zi is None:self.zi=sosfilt_zi(self.sos)[:,:,None]*value[None,None,:]
  y,self.zi=sosfilt(self.sos,value[None,:],axis=0,zi=self.zi)
  return {'raw':value.copy(),'moving_average':np.mean(self.buffer,axis=0),'ema':self.ema.copy(),'butterworth':y[0]}

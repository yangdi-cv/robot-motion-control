"""Frozen command matrix; deterministic cases are not independent random trials."""
CASES=[
 dict(id='zero',duration=20.,segments=[[0.,[0.,0.,0.]]]),
 dict(id='forward_02',duration=20.,segments=[[0.,[.2,0.,0.]]]),
 dict(id='forward_05',duration=20.,segments=[[0.,[.5,0.,0.]]]),
 dict(id='forward_08',duration=20.,segments=[[0.,[.8,0.,0.]]]),
 dict(id='backward_02',duration=20.,segments=[[0.,[-.2,0.,0.]]]),
 dict(id='left_02',duration=20.,segments=[[0.,[0.,.2,0.]]]),
 dict(id='right_02',duration=20.,segments=[[0.,[0.,-.2,0.]]]),
 dict(id='yaw_left_05',duration=20.,segments=[[0.,[0.,0.,.5]]]),
 dict(id='yaw_right_05',duration=20.,segments=[[0.,[0.,0.,-.5]]]),
 dict(id='forward_turn',duration=20.,segments=[[0.,[.5,0.,.3]]]),
 dict(id='walk_stop',duration=20.,segments=[[0.,[0.,0.,0.]],[3.,[.5,0.,0.]],[12.,[0.,0.,0.]]]),
 dict(id='mixed_commands',duration=25.,segments=[[0.,[0.,0.,0.]],[3.,[.5,0.,0.]],[8.,[0.,.2,0.]],[13.,[0.,0.,.5]],[18.,[0.,0.,0.]]]),
]
THRESHOLDS=dict(settle_s=3.,linear_rmse_mps=.15,yaw_rate_rmse_radps=.20,stop_speed_mps=.10,stop_yaw_rate_radps=.15,stop_hold_s=1.,contact_force_N=10.,low_base_height_m=.4,tilt_rad=1.0471975512,tilt_hold_s=.1,nonfoot_force_N=20.,nonfoot_hold_s=.1)
def command_at(case,t):
 cmd=case['segments'][0][1]
 for start,value in case['segments']:
  if t+1e-9>=start:cmd=value
 return cmd

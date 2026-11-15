"""Independent audits of causality, continuous motion and real motor execution."""
from pathlib import Path
import json,hashlib
import numpy as np
import mujoco
from engine import transition,SAFE_HALF
R=Path(__file__).resolve().parents[1]
def load(kind):
    m=json.loads((R/f'{kind}_manifest.json').read_text());a=json.loads((R/f'{kind}_results.json').read_text())
    current=[];archived=[]
    for name in m['source_files']:
        p=R/name if '/' in name else R/'scripts'/name
        current.append(p.read_bytes());archived.append((R/m['root']/Path(name).name).read_bytes())
    assert hashlib.sha256(b''.join(current)).hexdigest()==m['source_sha256']
    assert current==archived
    return m,a

def check_network():
    m,data=load('network');samples=0;maxres=0.;guards=0
    for key,r in data.items():
        c=r['config'];raw=np.load(R/m['root']/key/'physics.npz');x=raw['state'];cop=raw['cop'];feet=raw['foot']
        obs=np.load(R/m['root']/key/'observer.npz')['data']
        assert np.all(obs[:,1]<=obs[:,0]+1e-10)
        assert np.all(np.diff(obs[:,1])>=-1e-10)
        np.testing.assert_allclose(obs[:,2],obs[:,0]-obs[:,1],atol=1e-12)
        assert np.all(np.abs(cop-feet[:-1])<=SAFE_HALF+1e-8)
        assert len(x)==len(cop)+1
        t=np.arange(len(cop))*c['dt'];mask=(t>=c['push_time']-1e-10)&(t<c['push_time']+c['push_duration']-1e-10)
        force=mask[:,None]*np.array([c['impulse_x'],c['impulse_y']])/c['push_duration']
        A,B=transition(c['height'],c['dt']);expected=x[:-1]@A.T+(cop-force/(c['mass']*(9.81/c['height'])))[:,:,None]*B
        error=float(np.max(np.abs(x[1:]-expected)));assert error<1e-10;maxres=max(maxres,error)
        assert abs((len(x)-1)*c['dt']-r['duration_s'])<1e-9
        if r['reason']=='stale_feedback':
            assert obs[-1,2]>c['max_age'];guards+=1
        samples+=len(x)
    return {'runs':len(data),'physics_states':samples,'max_zoh_residual':maxres,'causality_checked':True,'stale_terminations':guards}

def check_gait():
    m,data=load('gait');samples=0;maxres=0.;clearance=1.
    for key,r in data.items():
        c=r['config'];raw=np.load(R/m['root']/key/'physics.npz');x=raw['state'];cop=raw['cop'];feet=raw['feet'];loads=raw['load'];s=raw['swing']
        assert np.isfinite(x).all() and np.isfinite(s).all()
        np.testing.assert_allclose(cop,(1-loads[:,1,None])*loads[:,2:4]+loads[:,1,None]*loads[:,4:6],atol=1e-8)
        assert np.all((loads[:,1]>=-1e-8)&(loads[:,1]<=1+1e-8))
        assert np.all(np.abs(loads[:,4:6]-feet[:,2:4])<=SAFE_HALF+1e-8)
        ds=loads[:,0]>0
        assert np.all(np.abs(loads[ds,2:4]-feet[ds,:2])<=SAFE_HALF+1e-8)
        t=np.arange(len(x)-1)*c['dt'];mask=(t>=c['push_time'])&(t<c['push_time']+c['push_duration']-1e-10)
        force=mask[:,None]*np.array([c['impulse_x'],c['impulse_y']])/c['push_duration']
        A,B=transition(c['height'],c['dt'])
        expected=x[:-1]@A.T+(cop[:-1]-force/(c['mass']*(9.81/c['height'])))[:,:,None]*B
        error=float(np.max(np.abs(x[1:]-expected))) if len(x)>1 else 0.
        assert error<1e-10,(key,error);maxres=max(maxres,error)
        if r['success']:
            assert len(r['touchdowns'])==c['steps']-1
            assert s[:,2].min()>=-1e-8
            assert s[:,2].max()<=.0800001
            clearance=min(clearance,float(s[:,2].min()))
            for positions in [s[:,9:12],s[:,12:15]]:
                speeds=np.linalg.norm(np.diff(positions,axis=0)/c['dt'],axis=1)
                assert speeds.max()<=c['swing_speed']+1e-6,(key,'swing speed',speeds.max())
            assert r['max_swing_speed_bound_m_s']<=c['swing_speed']+1e-6
            assert np.max(np.abs(np.diff(cop,axis=0)))<=c['cop_speed']/c['hz']+1e-8
        samples+=len(x)
    return {'runs':len(data),'physics_states':samples,'max_zoh_residual':maxres,'nonnegative_load_split_checked':True,'individual_sole_cops_checked':True,'swing_ground_clearance_checked':True,'swing_speed_checked':True,'minimum_successful_swing_z_m':clearance}

def check_biped():
    m,data=load('biped');model=mujoco.MjModel.from_xml_path(str(R/'models/biped.xml'));d=mujoco.MjData(model)
    assert model.nu==6 and model.nv==9
    assert np.all(model.actuator_trnid[:,0]>=3),'Floating base must have no actuator'
    ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,s+'_sole') for s in ['left','right']]
    root=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'pelvis')
    samples=0;maxfk=0.;maxint=0.;maxdyn=0.
    for key,r in data.items():
        c=r['config'];x=np.load(R/m['root']/key/'trace.npz')['data'];assert x.shape[1]==45
        assert np.isfinite(x).all()
        limits=np.array([80,80,30,80,80,30])*c['torque_scale']
        assert np.all(np.abs(x[:,10:16])<=limits+1e-8)
        np.testing.assert_allclose(np.diff(x[:,0]),c['dt'],atol=1e-12)
        q=x[:,17:26];v=x[:,26:35]
        error=float(np.max(np.abs(np.diff(q,axis=0)-c['dt']*v[1:])))
        assert error<1e-10,(key,'state reset or inconsistent integration',error);maxint=max(maxint,error)
        for k in np.linspace(0,len(x)-1,min(25,len(x)),dtype=int):
            d.qpos[:]=q[k];d.qvel[:]=v[k];mujoco.mj_forward(model,d)
            actual=np.concatenate([d.site_xpos[site,[0,2]] for site in ids])
            err=float(np.max(np.abs(actual-x[k,4:8])));maxfk=max(maxfk,err)
            assert err<1e-10
            np.testing.assert_allclose(d.subtree_com[root,[0,2]],x[k,1:3],atol=1e-10)
        if r['success']:
            assert abs(r['duration_s']-c['duration'])<1e-8
            if c['walk']:
                assert all(r['verified_landings']) and len(r['verified_landings'])==8
                assert r['progress_m']>=8*c['step_length']-.10
                assert r['terminal_foot_error_m']<.04
                assert x[:,5].max()>.03 and x[:,7].max()>.03
            assert x[:,16].max()<1e-3
        maxdyn=max(maxdyn,float(x[:,16].max()));samples+=len(x)
    return {'runs':len(data),'physics_samples':samples,'unactuated_base_checked':True,'torque_limits_checked':True,'max_forward_kinematics_residual_m':maxfk,'max_joint_integration_residual':maxint,'max_qp_dynamics_residual':maxdyn,'eight_measured_landings_checked':True,'joint_state_resets_detected':False}
if __name__=='__main__':
    result={'network':check_network(),'gait':check_gait(),'biped':check_biped()}
    (R/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

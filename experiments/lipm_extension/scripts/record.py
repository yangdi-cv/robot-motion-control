"""Record matched physical cases; recording repetitions are not new trials."""
from pathlib import Path
from biped import BipedConfig,simulate
R=Path(__file__).resolve().parents[1]
if __name__=='__main__':
    for scale,name in [(1.,'full_torque'),(.5,'half_torque')]:
        out=R/'recordings'/name
        if (out/'frames.npy').exists():continue
        r=simulate(BipedConfig(push=40.,torque_scale=scale,record=True),out)
        print(name,r['success'],r['failure'],r['progress_m'],flush=True)

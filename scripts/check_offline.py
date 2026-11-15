"""Check packaged controller contracts; never launch suites or connect hardware."""
from pathlib import Path
import argparse,json,os,subprocess,sys
R=Path(__file__).resolve().parents[1]
GROUPS={'humanoid':[('g1_contracts','experiments/humanoid_control/scripts/tests.py')],
'planar':[('lipm_contracts','experiments/lipm_balance/scripts/tests.py'),('gait_contracts','experiments/lipm_extension/scripts/tests.py'),('biped_contracts','experiments/biped_feedback/scripts/tests.py')],
'metrics':[('contact_metrics','experiments/panda_contact_optimization/scripts/test_metrics.py')]}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--suite',choices=list(GROUPS),default='humanoid');ap.add_argument('--out',type=Path);args=ap.parse_args();out=[]
 for name,path in GROUPS[args.suite]:
  p=subprocess.run([sys.executable,str(R/path)],cwd=R,env={**os.environ,'CUDA_VISIBLE_DEVICES':'','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'},capture_output=True,text=True,timeout=180)
  out.append(dict(group=name,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr));print(name,'PASS' if p.returncode==0 else 'FAIL',flush=True)
 dst=args.out or R/f'repository/offline_checks_{args.suite}.json';dst.parent.mkdir(parents=True,exist_ok=True);dst.write_text(json.dumps(out,ensure_ascii=False,indent=2));return int(any(x['returncode'] for x in out))
if __name__=='__main__':raise SystemExit(main())

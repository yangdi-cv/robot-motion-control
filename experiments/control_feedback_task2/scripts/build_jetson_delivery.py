"""Build the Task2 update including the Jetson measurements."""
from pathlib import Path
import ast,sys,shutil,json,hashlib,re,subprocess,zipfile
sys.path.insert(0,'/tmp/task2-tools')
import markdown
T=Path(__file__).resolve().parents[1]; ROOT=T.parents[1]
OUT=ROOT/'deliverables/20261020_字节Task2阶段'
backup=OUT/'backup_更新Jetson前'
backup.mkdir(parents=True,exist_ok=True)
for name in ['字节Task2_阶段进展.pdf','字节Task2_源码.zip','字节Task2_实验材料.zip']:
 p=OUT/name
 if p.exists() and not (backup/name).exists():shutil.copy2(p,backup/name)
for name in ['REPORT.md','REPORT.pdf','PROVENANCE.json','DELIVERY.json']:
 p=T/name; q=T/(p.stem+'_before_jetson'+p.suffix)
 if p.exists() and not q.exists():shutil.copy2(p,q)
s=(T/'REPORT_before_jetson.md').read_text()
s=s.replace('本轮在Ubuntu工作站完成，没有连接自有IMU或Jetson。公开数据分析不替代要求中的自有设备采集，工作站进程间延迟也不作为Jetson硬件指标。','前半部分为工作站仿真与公开数据分析；随后补充 Jetson 实体 IMU 静止、运动采集及三档发布测试。实体设备的新测量约25Hz，发布频率达到50/100/200Hz，具体结果见第8节。')
s=s.replace('8. 后续硬件工作','8. Jetson实体IMU测试').replace('| Jetson连接、采集、硬件延迟 | 尚未进行，待设备到位后补充 |','| Jetson连接、采集、软件链路时延 | 已完成实体IMU两段采集及三档频率测试；未测设备内部采样至执行器时延 |')
start=s.index('## 8. Jetson实体IMU测试');end=s.find('\n<a id=',start)
if end<0:raise RuntimeError('Appendix boundary missing')
j=(T/'JETSON_REPORT.md').read_text().split('\n',1)[1]
j=re.sub(r'^## ', '### ',j,flags=re.M)
s=s[:start]+'## 8. Jetson实体IMU测试\n'+j+'\n'+s[end:]
(T/'REPORT.md').write_text(s)
style=next(ast.literal_eval(n.value) for n in ast.parse((T/'scripts/build_delivery.py').read_text()).body if isinstance(n,ast.Assign) and any(isinstance(x,ast.Name) and x.id=='style' for x in n.targets))
for stem in ['REPORT','JETSON_REPORT']:
 md=(T/(stem+'.md')).read_text()
 for image in re.findall(r'!\[[^\]]*\]\(([^)]+)\)',md):assert (T/image).exists(),image
 html=T/(stem+'.html');html.write_text('<!doctype html><meta charset="utf-8"><style>'+style+'</style>'+markdown.markdown(md,extensions=['tables','fenced_code']))
 subprocess.run(['/usr/bin/google-chrome','--headless','--no-sandbox','--disable-gpu','--no-pdf-header-footer','--print-to-pdf='+str(T/(stem+'.pdf')),html.as_uri()],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
shutil.copy2(T/'REPORT.pdf',OUT/'字节Task2_阶段进展.pdf')
prov=json.loads((T/'PROVENANCE_before_jetson.json').read_text());prov['hardware_results']={'jetson':True,'own_board_capture':True,'physical_motion_commands':False,'timestamp':'same-host parse/receipt; not device sampling time','conda_an_project_modified':False};prov['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (T/'scripts').glob('*.py')};(T/'PROVENANCE.json').write_text(json.dumps(prov,ensure_ascii=False,indent=2))
source=[p for folder in ['scripts','config','vendor'] for p in (T/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.pyc'] and p.name not in {'log_progress.py','after_control.py','after_feedback.py','finalize.py','finalize_norender.py','build_delivery.py','build_jetson_delivery.py'}]
source += [T/'README.md',T/'requirements-analysis.txt']
# Preserve all previous material entries, then add the new measurements and refreshed sources.
old=backup/'字节Task2_实验材料.zip'
material={}
if old.exists():
 with zipfile.ZipFile(old) as z:
  material={n:z.read(n) for n in z.namelist()}
for p in source+[T/n for n in ['REPORT.md','REPORT.pdf','JETSON_REPORT.md','JETSON_REPORT.pdf','PROVENANCE.json']]+[p for p in (T/'runs/jetson_20261020').rglob('*') if p.is_file()]:material[str(p.relative_to(ROOT))]=p.read_bytes()
manifest=[]
for name,files in [('字节Task2_源码.zip',{str(p.relative_to(ROOT)):p.read_bytes() for p in source}),('字节Task2_实验材料.zip',material)]:
 path=OUT/name
 with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as z:
  for n,b in sorted(files.items()):z.writestr(n,b)
 with zipfile.ZipFile(path) as z:assert z.testzip() is None
 manifest.append({'file':name,'bytes':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
p=OUT/'字节Task2_阶段进展.pdf';manifest.append({'file':p.name,'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(T/'DELIVERY.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2));print(json.dumps(manifest,ensure_ascii=False,indent=2))

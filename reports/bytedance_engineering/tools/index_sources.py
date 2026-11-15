"""Refresh report navigation and copied media inventory, without moving implementations."""
from pathlib import Path
import json,hashlib,shutil
D=Path(__file__).resolve().parents[1];R=D.parents[1]
st=json.loads((D/'outline.json').read_text());assets=json.loads((D/'asset_manifest.json').read_text())
extra=[('s14','experiments/panda_contact_optimization/comparison.mp4','videos/ch05/panda_gravity_contact_comparison.mp4')]
for sid,src,dst in extra:
 p=D/dst;p.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(R/src,p)
 if not any(a['local']==dst for a in assets):assets.append(dict(kind='video',stage=sid,source=src,local=dst,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),caption='带重力接触控制与同步曲线'))
 p=D/'chapters/05_optimization.tex';s=p.read_text();ref='\\stagevideo{'+dst+'}'
 if ref not in s:s=s.replace(r'\stagecode{s14}',r'\stagecode{s14}'+'\n'+ref);p.write_text(s)
(D/'asset_manifest.json').write_text(json.dumps(assets,ensure_ascii=False,indent=2))
out=[r'\chapter{章节与代码位置}',r'\label{app:code}','代码路径均相对项目根目录。阶段目录保存实现、配置与记录；第三方模型和厂商解析库保留许可证，不能将文件总量当成自主编写的代码量。']
for x in st:
 sid=x['id'];ch=x['chapter'];idx=f'code_index/ch{ch:02d}/{sid}.md';p=D/idx;p.parent.mkdir(exist_ok=True,parents=True)
 roots=list(dict.fromkeys(str(Path(s).parent) for s in x['sources']));files=[]
 for root in roots:
  for folder in ['scripts','models','config','configs']:
   for f in sorted((R/root/folder).rglob('*')):
    if f.is_file() and '__pycache__' not in str(f) and f.suffix not in ['.pyc','.png','.mp4']:files.append(str(f.relative_to(R)))
 x['code']=list(dict.fromkeys(x['code']+(['experiments/panda_contact_optimization/scripts'] if sid=='s16' else [])))
 p.write_text('# '+x['title']+'\n\n对应报告第'+str(ch)+'章，阶段'+sid.upper()+'。\n\n## 相关代码\n\n'+'\n'.join('- `'+f+'`' for f in x['code'])+'\n\n## 源报告\n\n'+'\n'.join('- `'+f+'`' for f in x['sources'])+'\n\n## 实现与配置文件\n\n'+'\n'.join('- `'+f+'`' for f in files)+'\n\n运行前阅读原目录README，新输出目录不得覆盖旧结果。\n')
 out.extend([r'\section*{'+sid.upper()+' '+x['title']+'}',r'\hypertarget{code-'+sid+'}{}',r'阅读索引：\path{'+idx+'}。',r'\paragraph{相关代码}'])
 out.extend(r'\noindent\path{'+f+r'}\par' for f in x['code']);out.append(r'\paragraph{原报告与记录}');out.extend(r'\noindent\path{'+f+r'}\par' for f in x['sources'])
(D/'outline.json').write_text(json.dumps(st,ensure_ascii=False,indent=2));(D/'appendices/code_map.tex').write_text('\n'.join(out)+'\n')
out=[r'\chapter{图片与视频清单}','素材来自保存的实验记录；视频为实际物理仿真录制，不增加实验样本数。文件名与目录均使用英文，未来中英文版可共享。原路径与SHA256见\\path{asset_manifest.json}。']
for kind,title in [('video','视频文件'),('figure','图片文件')]:
 out.extend([r'\section{'+title+'}',r'\begin{longtable}{p{.08\linewidth}p{.82\linewidth}}',r'\toprule 阶段 & 相对路径\\\midrule\endhead'])
 out.extend(x['stage'].upper()+r' & \path{'+x['local']+r'}\\' for x in assets if x['kind']==kind);out.append(r'\bottomrule\end{longtable}')
(D/'appendices/media_index.tex').write_text('\n'.join(out)+'\n')
print('Stages',len(st),'figures',sum(x['kind']=='figure' for x in assets),'videos',sum(x['kind']=='video' for x in assets))

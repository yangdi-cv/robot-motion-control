"""Reviewed Markdown-to-LaTeX helpers; executed in the migration namespace."""
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def tex(s):
 return s.replace('\\',r'\textbackslash{}').replace('&',r'\&').replace('%',r'\%').replace('#',r'\#').replace('_',r'\_').replace('$',r'\$').replace('π0.5',r'\pifive{}')
def plain(inlines):
 if isinstance(inlines,list):return ''.join(plain(x) for x in inlines)
 if isinstance(inlines,dict):
  if inlines.get('t')=='Str':return inlines['c']
  if inlines.get('t') in ['Space','SoftBreak','LineBreak']:return ' '
  if inlines.get('t')=='Code':return inlines['c'][1]
  return plain(inlines.get('c',[]))
 return ''
def section_blocks(path):
 text=(R/path).read_text();lines=text.splitlines(keepends=True);heads=[];fence=False
 for i,l in enumerate(lines):
  if re.match(r'^\s*(```|~~~)',l):fence=not fence
  if not fence and re.match(r'^## (?!#)',l):heads.append((i,l[3:].strip()))
 blocks=[]
 for j,(i,h) in enumerate(heads):blocks.append((h,''.join(lines[i:(heads[j+1][0] if j+1<len(heads) else len(lines))])))
 return text,blocks
parts={s['id']:[] for s in stages}
def manual(sid,text,title):
 parts[sid].append({'source':None,'title':title,'markdown':text})
def select(sid,path,keys=None,title=None):
 full,blocks=section_blocks(path)
 if keys is None:
  keys=[h for h,_ in blocks if h not in ['摘要','目录'] and not re.search(r'结论.*(后续|下一步)|下一阶段|代码目录|文件与运行|复现与源码目录|文件目录和启动方式|代码、权重与后续|代码、复现与后续',h)]
 selected=[]
 for key in keys:
  match=[(h,b) for h,b in blocks if h==str(key) or re.match(r'^'+re.escape(str(key))+r'[.．、 ]',h)]
  if not match:raise ValueError((sid,path,key,[h for h,_ in blocks]))
  for h,b in match:
   selected.append((h,b));coverage.append({'stage':sid,'source':path,'section':h,'source_sha256':digest(R/path)})
 text='\n\n'.join(b for _,b in selected)
 if title:
  text='## '+title+'\n\n'+re.sub(r'^(#{2,5}) ',lambda m:'#'+m[1]+' ',text,flags=re.M)
 parts[sid].append({'source':path,'title':title or path,'markdown':text})
def excerpt(sid,path,start,end,title):
 full=(R/path).read_text();a=full.index(start);b=full.index(end,a) if end else len(full);body=full[a:b]
 body=re.sub(r'^#{2,5} .*\n','',body,flags=re.M)
 coverage.append({'stage':sid,'source':path,'section':title,'source_sha256':digest(R/path),'selection_start':start,'selection_end':end})
 parts[sid].append({'source':path,'title':title,'markdown':'## '+title+'\n\n'+body})
def normalize(text):
 # Keep exact identifiers and command blocks; edit prose only.
 chunks=re.split(r'(```[\s\S]*?```|`[^`\n]+`|\$\$[\s\S]*?\$\$|(?<!\$)\$[^$\n]+\$)',text)
 replacements={
  '参考标签来自 agent 对图像的独立视觉审阅，不是被测模型生成的标签。':'参考标签来自单独一次自动图像审阅，未采用被测模型的答案作为标签，也未完成独立人工复核。',
  '本轮':'该阶段','本次':'该次','证据':'记录','失败痕迹':'失败记录','官方检查点':'官方模型权重','自己的训练成果':'自主训练成果','本机':'工作站','全设备显式同步':'设备级显式同步',
 }
 # Device reports use 本机 for Jetson too: only make neutral wording below.
 replacements.pop('本机')
 for i,chunk in enumerate(chunks):
  if i%2:continue
  for a,b in replacements.items():chunk=chunk.replace(a,b)
  chunk=chunk.replace('真实策略回合','实际策略仿真回合').replace('真实回合','实际仿真回合')
  chunk=chunk.replace("{'input_token_sha256': 1014, 'pixel_values_sha256': 1014, 'image_grid_thw': 1014, 'output_token_ids': 774}",'输入词元、图像张量和图像网格各1014条完全一致，输出词元774条一致')
  chunk=chunk.replace("{'pswpin': 0, 'pswpout': 0}",'换入和换出页数增量均为0')
  chunk=re.sub(r'10([⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)',lambda m:'$10^{'+m[1].translate(str.maketrans('⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺','0123456789-+'))+'}$',chunk)
  for a,b in [('π0.5',r'\(\pi\)0.5'),('σ',r'\(\sigma\)'),('μ',r'\(\mu\)'),('Δ',r'\(\Delta\)'),('≥',r'\(\geq\)'),('≤',r'\(\leq\)'),('≈',r'\(\approx\)'),('→',r'\(\rightarrow\)'),('×',r'\(\times\)')]:chunk=chunk.replace(a,b)
  # Original per-report headings carry numbers; LaTeX supplies the new numbering.
  chunk=re.sub(r'^(#{2,6})\s+\d+(?:\.\d+)*[.．、:]?\s*',r'\1 ',chunk,flags=re.M)
  cleaned=[]
  for line in chunk.splitlines():
   if line.startswith('|') and 'MoveIt execution returned' in line:
    cells=line.split('|');m=re.search(r'MoveIt execution returned (-?\d+)',cells[-2]);
    if m:cells[-2]=' 规划或执行返回码 '+m[1]+' ';line='|'.join(cells)
   cleaned.append(line)
  chunks[i]='\n'.join(cleaned)
 return ''.join(chunks)

def resolve(path,origin):
 path=path.split('#')[0];p=Path(path)
 if p.is_absolute():return p if p.exists() else None
 candidates=[]
 if origin:candidates.extend([(R/origin).parent/p,R/p])
 candidates.extend([D/p,R/p])
 return next((p.resolve() for p in candidates if p.is_file()),None)
def asset(s,src,kind,caption):
 sha=digest(src);key=(s['id'],sha)
 if key in known:rel=known[key]
 else:
  folder='videos' if kind=='video' else 'figures';stem=re.sub('[^A-Za-z0-9_.-]','_',src.stem)[:65] or kind
  if not re.search('[A-Za-z]',stem):stem=kind
  suffix=src.suffix.lower();rel=str(Path(folder)/f"ch{s['report_chapter']:02d}"/f"{s['id']}_{stem}_{sha[:8]}{suffix}")
  (D/rel).parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(src,D/rel)
  known[key]=rel
  assets.append({'kind':kind,'stage':s['id'],'source':str(src.relative_to(R)) if src.is_relative_to(R) else str(src),'local':rel,'sha256':sha,'caption':caption,'note':'从已有运行材料迁入，未重新运行模型。'})
 return rel

def codeinline(text):
 for delim in ['|','!','+',';']:
  if delim not in text:return r'\path'+delim+text+delim
 return r'\texttt{'+tex(text)+'}'

current={}
def walk(node,s,origin):
 if isinstance(node,list):return [walk(x,s,origin) for x in node]
 if not isinstance(node,dict):return node
 t=node.get('t');c=node.get('c')
 if t=='Header':
  current['heading']=plain(c[2]);current['header_count']+=1;c[1][0]=f"{s['id']}-detail-{current['header_count']}"
 if t=='Str':
  words={'True':'是','False':'否','timeout':'超时','oracle':'真值定位','base':'基座','lora':'低秩适配','pc':'工作站','jetson':'Jetson','2b':'2B','4b':'4B','unknown':'不确定','not_done':'未完成'}
  if c in words:return {'t':'Str','c':words[c]}
  for a,b in [('worker','独立推理进程'),('rollout','执行回合'),('checkpoint','模型检查点')]:
   if c==a:return {'t':'Str','c':b}
  if re.fullmatch(r'[A-Za-z0-9_./:\-]{24,}',c):return {'t':'RawInline','c':['latex',codeinline(c)]}
 if t=='Code':
  text=c[1]
  if text.endswith('.mp4') and not any(x in text for x in ['*','<','>']):
   src=resolve(text,origin)
   if src:
    rel=asset(s,src,'video',text);used_videos[s['id']].add(rel);return {'t':'RawInline','c':['latex',codeinline(rel)]}
  if len(text)>24 or any(x in text for x in ['/', '_']):return {'t':'RawInline','c':['latex',codeinline(text)]}
 if t=='Image':
  caption=re.sub(r'^图\s*\d+\s*[.：:、]?\s*','',plain(c[1]));src=resolve(c[2][0],origin)
  if not src:
   missing.append({'stage':s['id'],'kind':'image','path':c[2][0],'source':origin});return {'t':'Str','c':'（原图位置需核对，见素材清单。）'}
  rel=asset(s,src,'figure',caption)
  if rel in used_figures[s['id']]:return {'t':'Str','c':''}
  used_figures[s['id']].add(rel);current['images']+=1
  return {'t':'RawInline','c':['latex',r'\stagefigure{'+rel+'}{'+tex(caption or s['title'])+'}{fig:'+s['id']+'-detail-'+str(current['images'])+'}']}
 if t=='Link':
  url=c[2][0]
  if not url.startswith(('http://','https://','mailto:')):
   src=resolve(url,origin)
   if src and src.suffix.lower()=='.mp4':
    rel=asset(s,src,'video',plain(c[1]));used_videos[s['id']].add(rel);return {'t':'RawInline','c':['latex',codeinline(rel)]}
   return {'t':'RawInline','c':['latex',codeinline(url)]}
 if t=='CodeBlock':
  lang=c[0][1][0] if c[0][1] else '';body=c[1]
  if lang=='mermaid':
   labels=[]
   for label in re.findall(r'[A-Za-z][\w]*\[([^\]]+)\]',body):
    if label not in labels:labels.append(label)
   if not labels:raise ValueError(('Unsupported diagram',origin,body))
   return {'t':'OrderedList','c':[[1,{'t':'Decimal'},{'t':'Period'}],[[{'t':'Plain','c':[{'t':'Str','c':label}]}] for label in labels]]}
  if len(body.splitlines())>14 or (lang in ['text','bash','sh','shell'] and len(body)>600):
   key=f"{s['id']}-command-{len(commands)+1}";commands.append({'stage':s['id'],'id':key,'language':lang,'text':body,'source':origin})
   return {'t':'RawBlock','c':['latex',r'\noindent 运行与目录示例见\hyperlink{'+key+'}{附录D的对应条目}。\par']}
  return {'t':'RawBlock','c':['latex','\\begin{Verbatim}[breaklines=true,breakanywhere=true,fontsize=\\footnotesize]\n'+body+'\n\\end{Verbatim}']}
 if t=='Table':
  current['tables']+=1;n=len(c[2]);rows=list(c[3][1])
  for body in c[4]:rows.extend(body[2]);rows.extend(body[3])
  measures=[[] for _ in range(n)]
  for row in rows:
   for j,cell in enumerate(row[1][:n]):
    text=plain(cell[4]);measures[j].append(sum(.55 if ord(ch)<128 else 1. for ch in text))
  widths=[math.sqrt(max(4.,sum(min(v,80.) for v in vals)/max(1,len(vals)))) for vals in measures]
  weights=[v/sum(widths) for v in widths]
  for _ in range(8):
   weights=[max(.065,min(.70 if n==2 else .48,v)) for v in weights];weights=[v/sum(weights) for v in weights]
  c[2]=[[col[0],{'t':'ColWidth','c':weights[j]}] for j,col in enumerate(c[2])]
  heading=current.get('heading') or s['title'];caption=re.sub(r'\s*\(.*?\)\s*$','',heading)+'：设置与结果'
  c[1]=[None,[{'t':'Plain','c':[{'t':'Str','c':caption}]}]]
  c[0][0]=f"tab:{s['id']}-{current['tables']}"
 result={k:(walk(v,s,origin) if k=='c' else v) for k,v in node.items()}
 return result


"""Validate document structure and copied media, without running control code."""
from pathlib import Path
import re,json,hashlib
D=Path(__file__).resolve().parents[1];R=D.parents[1]
st=json.loads((D/'outline.json').read_text());a=json.loads((D/'asset_manifest.json').read_text())
assert len(st)==29 and len({s['id'] for s in st})==29
for ch in range(2,9):assert sum(s['chapter']==ch for s in st)==4
assert sum(p.read_text().count('\\section{') for p in (D/'chapters').glob('0[2-8]_*.tex'))==28
for s in st:
 assert (D/f"code_index/ch{s['chapter']:02d}/{s['id']}.md").exists()
 for p in s['sources']+s['code']:assert (R/p).exists(),p
for x in a:
 p=D/x['local'];assert p.is_file(),p
 assert hashlib.sha256(p.read_bytes()).hexdigest()==x['sha256'],p
 assert x['local'].isascii(),p
for p in D.rglob('*.tex'):
 for x in re.findall(r'\\(?:stagefigure|motionfigure|stagevideo)\{([^}]+)\}',p.read_text()):assert (D/x).exists(),(p,x)
assert json.loads((D/'missing_assets.json').read_text())==[]
assert (D/'build/main_zh.pdf').stat().st_size>10000
text='\n'.join(p.read_text() for p in (D/'chapters').glob('*.tex'))
assert len(set(re.findall(r'\\(?:stagefigure|motionfigure)\{([^}]+)\}',text)))==sum(x['kind']=='figure' for x in a)
assert text.count('\\begin{longtable}')>=75
print(f"29 stages; {sum(x['kind']=='figure' for x in a)} figures; {sum(x['kind']=='video' for x in a)} videos; paths and hashes: OK")

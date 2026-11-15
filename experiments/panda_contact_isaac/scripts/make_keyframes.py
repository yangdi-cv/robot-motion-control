"""Build a report contact sheet from the compensated-position simulation frames."""
from pathlib import Path
from PIL import Image, ImageDraw
R=Path(__file__).resolve().parents[1]
sheet=Image.new('RGB',(960,270),'#eef2f7');draw=ImageDraw.Draw(sheet)
for column,(index,label) in enumerate([(0,'Initial pose'),(85,'Approaching contact'),(180,'Contact phase')]):
 image=Image.open(R/f'runs/position_final/frames/{index:05d}.png').convert('RGB').resize((320,240))
 sheet.paste(image,(column*320,0));draw.text((column*320+10,247),f'{label} | t={index/10:.1f}s',fill='#202b38')
sheet.save(R/'figures/keyframes.png')

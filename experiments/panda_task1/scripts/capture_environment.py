import re,subprocess,time
from pathlib import Path
from Xlib import display,X
from PIL import Image
root=Path(__file__).resolve().parents[1]
for _ in range(25):
 tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
 match=re.search(r'(0x[0-9a-f]+) "Panda Task1 Environment Evidence"',tree)
 if match:break
 time.sleep(.2)
if not match:raise RuntimeError('Environment terminal not found')
connection=display.Display();window=connection.create_resource_object('window',int(match.group(1),16));geometry=window.get_geometry()
pixels=window.get_image(0,0,geometry.width,geometry.height,X.ZPixmap,0xffffffff)
image=Image.frombytes('RGB',(geometry.width,geometry.height),pixels.data,'raw','BGRX')
image.save(root/'setup_logs/ubuntu_environment.png');connection.close()
print(root/'setup_logs/ubuntu_environment.png')

"""Capture only the Isaac X11 window; record real rendered screen pixels."""
import argparse,json,time
from fractions import Fraction
from pathlib import Path
import av
from PIL import Image
from Xlib import display,X,error
parser=argparse.ArgumentParser();parser.add_argument('--window-id',required=True);parser.add_argument('--out',type=Path,required=True);parser.add_argument('--seconds',type=float,default=12)
args=parser.parse_args();connection=display.Display();window=connection.create_resource_object('window',int(args.window_id,16))
geometry=window.get_geometry();width,height=geometry.width,geometry.height
started=time.monotonic();count=0;args.out.mkdir(parents=True,exist_ok=True)
with av.open(str(args.out/'simulation_recording.mp4'),'w') as output:
 stream=output.add_stream('libx264',rate=20);stream.width=width;stream.height=height;stream.pix_fmt='yuv420p';stream.options={'crf':'20','preset':'fast'};stream.time_base=Fraction(1,90000);stream.codec_context.time_base=Fraction(1,90000)
 while time.monotonic()-started<args.seconds:
  tick=time.monotonic()
  try:pixels=window.get_image(0,0,width,height,X.ZPixmap,0xffffffff)
  except (error.XError,AttributeError):break
  image=Image.frombytes('RGB',(width,height),pixels.data,'raw','BGRX')
  if count==0:image.save(args.out/'isaac_environment.png');(args.out/'recording_ready').touch()
  if count==60:image.save(args.out/'trajectory_midpoint.png')
  frame=av.VideoFrame.from_image(image);frame.pts=round((time.monotonic()-started)*90000);frame.time_base=Fraction(1,90000)
  for packet in stream.encode(frame):output.mux(packet)
  count+=1;time.sleep(max(0,.05-(time.monotonic()-tick)))
 for packet in stream.encode():output.mux(packet)
connection.close()
(args.out/'recording_metadata.json').write_text(json.dumps({'frames':count,'wall_seconds':time.monotonic()-started,'window_id':args.window_id,'width':width,'height':height,'source':'actual Isaac Sim X11 window pixels; timestamped video'},indent=2))
if count<30:raise RuntimeError(f'Insufficient recording frames: {count}')
print(f'Recorded {count} actual screen frames',flush=True)

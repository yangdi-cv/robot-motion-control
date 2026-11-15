"""Render actual simulation frames; failed runs freeze with a termination label."""
from pathlib import Path
import json, subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import imageio_ffmpeg
R=Path(__file__).resolve().parents[1]
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
def font(n):return ImageFont.truetype(FONT,n)
def main():
    names=['full_torque','half_torque']; runs=[]
    for name in names:
        p=R/'recordings'/name
        runs.append((np.load(p/'frames.npy',mmap_mode='r'),np.load(p/'trace.npz')['data'],json.loads((p/'result.json').read_text())))
    width,height=1400,840; fps=20; count=121
    dest=R/'biped_comparison.mp4'
    proc=subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-f','rawvideo','-vcodec','rawvideo','-s',f'{width}x{height}','-pix_fmt','rgb24','-r',str(fps),'-i','-','-an','-vcodec','libx264','-crf','20','-pix_fmt','yuv420p',str(dest)],stdin=subprocess.PIPE,stderr=subprocess.DEVNULL)
    for i in range(count):
        t=i/fps; im=Image.new('RGB',(width,height),'#eef2f6');d=ImageDraw.Draw(im)
        d.text((25,12),'平面双足：相同 40 N 推扰，比较电机力矩上限',font=font(28),fill='#15283d')
        for j,(frames,x,result) in enumerate(runs):
            ox=15+j*700; ended=t>result['duration_s']+.001
            idx=min(int(t*fps),len(frames)-1)
            camera=Image.fromarray(frames[idx]).convert('RGB').resize((680,510))
            im.paste(camera,(ox,95));d=ImageDraw.Draw(im)
            title=['完整力矩上限：完成八步','力矩上限减半：4.191 秒终止'][j]
            d.text((ox+10,60),title,font=font(23),fill='#18334b')
            row=x[min(np.searchsorted(x[:,0],min(t,result['duration_s'])),len(x)-1)]
            d.text((ox+10,615),f'时间 {min(t,result["duration_s"]):.2f} s   质心 x {row[1]:.3f} m',font=font(21),fill='#20354a')
            d.text((ox+10,650),f'躯干倾角 {np.degrees(row[3]):.2f}°   左/右足载荷 {row[8]:.0f} / {row[9]:.0f} N',font=font(19),fill='#20354a')
            if ended:d.text((ox+10,690),'已终止；保留最后一帧，不再推进仿真',font=font(22),fill='#b83232')
            else:d.text((ox+10,690),'2.70–2.80 s：施加水平推力 40 N',font=font(21),fill='#20354a')
        d.text((25,770),'实际 MuJoCo 相机帧 · 六关节力矩驱动 · 基座无执行器 · 平面约束、理想状态反馈',font=font(21),fill='#45566a')
        if i==90:im.save(R/'figures/biped_keyframe.png')
        proc.stdin.write(im.tobytes())
    proc.stdin.close();assert proc.wait()==0
    # Physical frames sampled at three times, without compositing synthetic poses.
    sheet=Image.new('RGB',(1440,440),'#eef2f6');draw=ImageDraw.Draw(sheet)
    for k,t in enumerate([.5,2.7,5.4]):
        frames=runs[0][0];frame=Image.fromarray(frames[min(round(t*fps),len(frames)-1)]).resize((480,360))
        sheet.paste(frame,(k*480,50));draw.text((k*480+15,10),f't = {t:.1f} s',font=font(24),fill='#20354a')
    sheet.save(R/'figures/step_sequence.png')
    (R/'video_metadata.json').write_text(json.dumps({'fps':fps,'frames':count,'duration_s':count/fps,'size':[width,height],'failed_panel':'Freeze actual final frame after termination','recording_results':[v[2] for v in runs]},indent=2))
    print(dest)
if __name__=='__main__':main()

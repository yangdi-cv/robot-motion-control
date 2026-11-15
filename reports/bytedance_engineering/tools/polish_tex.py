"""One-time editorial pass after Markdown migration. Normal build does not run this."""
from pathlib import Path
import re
D=Path(__file__).resolve().parents[1]
for p in list((D/'chapters').glob('*.tex'))+list((D/'appendices').glob('*.tex')):
 s=p.read_text()
 # Chinese math prose belongs to normal CJK text, not a verbatim URL command.
 s=re.sub(r'\\path\|([^|\n]*[\u4e00-\u9fff][^|\n]*)\|',lambda m:m[1].replace('_',r'\_').replace('%',r'\%'),s)
 # Break actual unformatted identifiers; existing \path forms remain untouched.
 for token in ['scripts/run.py','scripts/suite.py','scripts/validation\\_suite.py','scripts/refine\\_suite.py','OPEN\\_SOURCE\\_REVIEW.md','stream\\_coverage.csv','scripts/analyze\\_stability.py','scripts/stability\\_*suite.sh']:
  s=re.sub(r'(?<![|{])'+re.escape(token)+r'(?![|}])',lambda m:r'\path|'+m[0].replace('\\_','_')+'|',s)
 s=s.replace('moving\\_average &','移动平均 &').replace('butterworth &','巴特沃斯 &').replace('ema &','指数平均 &')
 s=s.replace('/imu\\_week2/filtered &',r'\path|/imu_week2/filtered| &').replace('/imu\\_week2/raw &',r'\path|/imu_week2/raw| &')
 # Wide nine-column frequency tables need smaller type, while preserving readable CJK headers.
 s=re.sub(r'\\FloatBarrier\\begingroup\\small\\setlength\{\\tabcolsep\}\{2.5pt\}([\s\S]*?)\\end\{longtable\}\\endgroup',lambda m: ('\\FloatBarrier\\begingroup\\footnotesize\\setlength{\\tabcolsep}{2pt}' if m[1].count('\\real{')>=8 else '\\FloatBarrier\\begingroup\\small\\setlength{\\tabcolsep}{2.5pt}')+m[1]+'\\end{longtable}\\endgroup',s)
 p.write_text(s)
p=D/'chapters/02_platform.tex';s=p.read_text();s=s.replace(r'令\path|u=t/T|，\texttt{T=5\ s}，插值函数： \path|q(t)=q0+(qf-q0)(10u^3-15u^4+6u^5)|。',r'''令 $u=t/T$，$T=5\,\mathrm{s}$，采用五次插值：
\begin{equation}
q(t)=q_0+(q_f-q_0)(10u^3-15u^4+6u^5).
\end{equation}
其中 $q_0$ 和 $q_f$ 分别为起点与终点关节角。速度和加速度由该式解析求导，起止两端均为零。''');p.write_text(s)
p=D/'chapters/04_contact.tex';s=p.read_text();s=s.replace('采用 M·偏移加速度＋B·偏移速度＋K·偏移＝测量力−目标力，M=2kg、B=100Ns/m。',r'''法向偏移 $x$ 满足：
\begin{equation}
M\ddot{x}+B\dot{x}+Kx=F_{\mathrm{meas}}-F_{\mathrm{ref}}.
\end{equation}
其中质量参数 $M=2\,\mathrm{kg}$，阻尼参数 $B=100\,\mathrm{N\,s/m}$。正方向按该阶段的法向坐标约定，后文Isaac的向下压入坐标另行说明。''');p.write_text(s)
p=D/'chapters/06_lipm.tex';s=p.read_text();s=s.replace('控制律为τ=−Kθ·θ−Kω·θ\\_dot。',r'控制律为 $\tau=-K_\theta\theta-K_\omega\dot\theta$。');p.write_text(s)
# References are tied to actual use, rather than a disconnected bibliography list.
for f,t in [('02_platform',r'通信桥与机械臂接口参考Isaac Sim官方教程\cite{isaac}。'),('03_feedback',r'公开数据采用TUM-VI\cite{tumvi}；滤波延迟的定义参照SciPy文档\cite{scipy}。'),('04_contact',r'模型与物理接口分别参考MuJoCo Menagerie\cite{menagerie}、MuJoCo API\cite{mujoco}和Isaac接触接口\cite{contact}。'),('06_lipm',r'控制分层参考公开LIPM控制器\cite{lipm}，本地模型与约束保持独立定义。'),('07_biped',r'任务分层与接触逆动力学参考PlaCo\cite{placo}及TSID\cite{tsid}，不整体复制其控制栈。'),('08_humanoid',r'G1模型来自Menagerie\cite{menagerie}，Clarabel调用遵循官方QP接口\cite{clarabel}。')]:
 p=D/'chapters'/f'{f}.tex';s=p.read_text();n=s.index('\n');s=s[:n+1]+t+'\n\n'+s[n+1:];p.write_text(s)

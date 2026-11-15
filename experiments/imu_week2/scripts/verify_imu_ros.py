"""Check independent received ROS messages against publication ROS timestamps and values."""
from pathlib import Path
import csv,json,numpy as np
r=Path(__file__).resolve().parents[1]/'runs/run001'
pub=list(csv.DictReader((r/'imu_publisher_v2/published.csv').open()));received=list(csv.DictReader((r/'imu_sink_v2/received.csv').open()));index={int(v['published_ros_ns']):v for v in pub};report={}
for topic in ('raw','filtered'):
 rows=[x for x in received if x['topic']==topic];keys=[int(x['header_ros_ns']) for x in rows];times=np.array([int(x['host_receive_ns']) for x in rows])/1e9
 assert set(keys)<=set(index)
 latency=[(int(x['host_receive_ns'])-int(index[int(x['header_ros_ns'])]['published_monotonic_ns']))/1e6 for x in rows]
 assert min(latency)>=0
 values=[]
 for x in rows:
  expected=index[int(x['header_ros_ns'])];prefix=topic
  values.extend([abs(float(x[a])-float(expected[f'{prefix}_{a}'])) for a in ('ax','ay','az','gx','gy','gz')])
 report[topic]={'published':len(pub),'received':len(rows),'missing':len(set(index)-set(keys)),'duplicates':len(keys)-len(set(keys)),'wall_hz':(len(times)-1)/(times[-1]-times[0]),'hop_latency_p50_ms':float(np.median(latency)),'hop_latency_p95_ms':float(np.percentile(latency,95)),'max_message_value_difference':max(values)}
 assert report[topic]['missing']==report[topic]['duplicates']==0 and max(values)<1e-10
(r/'imu_ros_verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

#!/usr/bin/env python3
"""Offline per-trial analysis, world-linear-twist hypothesis checked against pose."""
import csv,json,sqlite3,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]);out=root/'analysis';out.mkdir(exist_ok=True)
results=[];fig,axes=plt.subplots(5,2,figsize=(14,17));names=['w','a','d','q','e']
for bag in sorted(root.glob('*_level*')):
 db=next(bag.glob('*.db3'));c=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
 topics={n:(i,t) for i,n,t in c.execute('select id,name,type from topics')}
 def messages(name):
  i,t=topics[name];typ=get_message(t)
  return [(ns,deserialize_message(b,typ)) for ns,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
 commands=[json.loads(m.data) for _,m in messages('/whill/controller/command')]
 starts=[d['time_ns']/1e9 for d in commands if d.get('action')=='keyboard_drag_begin'];ends=[d['time_ns']/1e9 for d in commands if d.get('action')=='keyboard_drag_end']
 if not starts or not ends:print('INCOMPLETE',bag.name,flush=True);continue
 start=starts[0];end=ends[-1];offset=None
 for _,msg in messages('/tf_static'):
  for tf in msg.transforms:
   if tf.header.frame_id=='base_link' and tf.child_frame_id=='imu':
    p=tf.transform.translation;q=tf.transform.rotation;offset=np.array([p.x,p.y,p.z]);rbi=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
 if offset is None:raise RuntimeError('missing base_link->imu TF')
 rows=[]
 for ns,m in messages('/odin1/odometry_highfreq'):
  q=m.pose.pose.orientation;p=m.pose.pose.position;v=m.twist.twist.linear;w=m.twist.twist.angular
  roi=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();rob=roi@rbi.T
  wb=rbi@np.array([w.x,w.y,w.z]);vb=rob.T@np.array([v.x,v.y,v.z])-np.cross(wb,offset)
  pb=np.array([p.x,p.y,p.z])-rob@offset
  stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  rows.append([stamp-start,*pb,np.arctan2(rob[1,0],rob[0,0]),*vb,wb[2]])
 x=np.array(rows);x=x[np.argsort(x[:,0])];t=x[:,0];duration=end-start
 grid=np.arange(max(0,t[0]),min(t[-1],duration+3),.1)
 y=[];gt=[]
 for b in grid:
  chunk=x[(t>=b)&(t<b+.1)]
  if len(chunk):y.append(np.median(chunk,axis=0));gt.append(b+.05)
 y=np.array(y);gt=np.array(gt);yaw=np.unwrap(y[:,4]);pv=np.gradient(y[:,1],gt)*np.cos(yaw)+np.gradient(y[:,2],gt)*np.sin(yaw)
 core=(gt>=max(.7,duration-1.5))&(gt<duration-.15)
 if sum(core)<3:core=(gt>.1)&(gt<duration)
 hold=(gt>=0)&(gt<duration)
 moving=(abs(y[:,5])>.03)|(abs(y[:,8])>.06)
 onset=np.flatnonzero(moving&hold)
 stops=[i for i in range(len(gt)-3) if gt[i]>=duration and not moving[i:i+4].any()]
 key=bag.name[-1];level=int(bag.name.split('level')[1][0]);amp=[.375,.625,1][level-1] if key in 'wad' else level/3
 row=dict(trial=bag.name,level=level,key=key,amplitude=amp,hold_s=duration,samples=len(x),max_gap_s=float(np.max(np.diff(t))),
          late_v=float(np.median(y[core,5])),late_vy=float(np.median(y[core,6])),late_w=float(np.median(y[core,8])),
          peak_v=float(np.max(y[hold,5])),peak_abs_w=float(np.max(abs(y[hold,8]))),
          v_spread=float(np.ptp(y[core,5])),w_spread=float(np.ptp(y[core,8])),
          onset_s=float(gt[onset[0]]) if len(onset) else None,stop_s=float(gt[stops[0]]-duration) if stops else None,
          pose_v_rmse=float(np.sqrt(np.mean((pv-y[:,5])**2))))
 results.append(row);print(json.dumps(row),flush=True)
 np.savetxt(out/(bag.name+'.csv'),y,delimiter=',',header='t,x,y,z,yaw,vx,vy,vz,wz',comments='')
 for j,col in enumerate([5,8]):axes[names.index(key),j].plot(gt,y[:,col],label=f'L{level} #{bag.name[:3]}');axes[names.index(key),j].axvline(duration,color='gray',alpha=.15)
csvfile=out/'trials.csv'
with csvfile.open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(results[0]));w.writeheader();w.writerows(results)
(out/'trials.json').write_text(json.dumps(results,indent=2))
for i,key in enumerate(names):
 for j,unit in enumerate(['vx m/s','wz rad/s']):axes[i,j].set_title(key+' '+unit);axes[i,j].legend(fontsize=7);axes[i,j].grid();axes[i,j].set_xlabel('seconds from drag command')
fig.tight_layout();fig.savefig(out/'responses.png',dpi=140)

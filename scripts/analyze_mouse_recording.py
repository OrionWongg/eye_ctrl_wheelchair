#!/usr/bin/env python3
"""Analyze recorded mouse commands and measured velocities; no hardware access."""
import json,sqlite3,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]);out=root/'analysis';out.mkdir(exist_ok=True);summary=[]
for bag in sorted(root.glob('*_mouse')):
 c=sqlite3.connect('file:'+str(next(bag.glob('*.db3')))+'?mode=ro',uri=True)
 def read(topic):
  row=c.execute('select id,type from topics where name=?',(topic,)).fetchone()
  if not row:return []
  typ=get_message(row[1]);return [(t/1e9,deserialize_message(b,typ)) for t,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(row[0],))]
 stamp=lambda m:m.header.stamp.sec+m.header.stamp.nanosec/1e9
 cmd=np.array([[stamp(m),m.twist.linear.x,m.twist.angular.z] for _,m in read('/experiment/cmd_vel')])
 measured=np.array([[stamp(m),m.twist.linear.x,m.twist.angular.z] for _,m in read('/experiment/measured_velocity')])
 markers=[(t,json.loads(m.data)['event']) for t,m in read('/experiment/marker')]
 if len(cmd)<3 or len(measured)<3:continue
 measured=measured[np.argsort(measured[:,0])];cmd=cmd[np.argsort(cmd[:,0])]
 origin=max(cmd[0,0],measured[0,0]);end=min(cmd[-1,0],measured[-1,0]);grid=np.arange(origin,end,.05)
 target=cmd[np.maximum(0,np.searchsorted(cmd[:,0],grid,side='right')-1),1:]
 actual=np.column_stack([np.interp(grid,measured[:,0],measured[:,j]) for j in (1,2)])
 # Report interpolation coverage, never hide sparse measurement intervals.
 row=dict(bag=bag.name,duration_s=end-origin,command_count=len(cmd),measured_count=len(measured),
          measured_gap_p95=float(np.percentile(np.diff(measured[:,0]),95)),measured_gap_max=float(np.max(np.diff(measured[:,0]))),markers=markers)
 row['raw_rmse']=np.sqrt(np.mean((target-actual)**2,axis=0)).tolist()
 fits=[]
 for j in range(2):
  candidates=[]
  for delay in np.arange(0,1.51,.05):
   u=cmd[np.maximum(0,np.searchsorted(cmd[:,0],grid-delay,side='right')-1),j+1]
   mask=grid>=origin+1.5
   gain=float(u[mask]@actual[mask,j]/max(1e-12,u[mask]@u[mask]))
   mse=float(np.mean((gain*u[mask]-actual[mask,j])**2))
   candidates.append((mse,float(delay),gain))
  mse,delay,gain=min(candidates);fits.append(dict(delay_s=delay,gain=gain,rmse=mse**.5))
 row['delay_gain_fit']=fits
 # Stable-input windows: no changes during preceding 1s, nonzero target.
 stable=np.array([i>=20 and np.max(np.ptp(target[i-20:i+1],axis=0))<.015 for i in range(len(grid))])
 row['stable_samples']=int(sum(stable));row['stable_rmse']=np.sqrt(np.mean((target[stable]-actual[stable])**2,axis=0)).tolist() if any(stable) else None
 # Pose at empirically fitted control center, same reference as velocity.
 off=None
 for _,m in read('/tf_static'):
  for t in m.transforms:
   if (t.header.frame_id,t.child_frame_id)==('base_link','imu'):
    p=t.transform.translation;q=t.transform.rotation;off=np.array([p.x,p.y,p.z]);rbi=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
 poses=[];ages=[]
 if off is not None:
  for t,m in read('/odin1/odometry_highfreq'):
   q=m.pose.pose.orientation;p=m.pose.pose.position;rob=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()@rbi.T
   xyz=np.array([p.x,p.y,p.z])+rob@(np.array([-.266192,.078385,0])-off)
   poses.append([stamp(m),xyz[0],xyz[1],np.arctan2(rob[1,0],rob[0,0])]);ages.append(t-stamp(m))
 poses=np.array(poses);poses=poses[np.argsort(poses[:,0])]
 xy=np.column_stack([np.interp(grid,poses[:,0],poses[:,j]) for j in (1,2)]);yaw=np.interp(grid,poses[:,0],np.unwrap(poses[:,3]));h=yaw[0]
 xy=(xy-xy[0])@np.array([[np.cos(h),-np.sin(h)],[np.sin(h),np.cos(h)]])
 pred=np.zeros((len(grid),3))
 for i in range(1,len(grid)):
  v,w=target[i-1];theta=pred[i-1,2];pred[i]=pred[i-1]+np.array([v*np.cos(theta)*.05,v*np.sin(theta)*.05,w*.05])
 row['final_position_error_m']=float(np.linalg.norm(pred[-1,:2]-xy[-1]));row['final_yaw_error_rad']=float(np.arctan2(np.sin(pred[-1,2]-(yaw[-1]-yaw[0])),np.cos(pred[-1,2]-(yaw[-1]-yaw[0]))))
 row['odom_header_age_p50_p95']=np.percentile(ages,[50,95]).tolist()
 row['target_range']=[target.min(axis=0).tolist(),target.max(axis=0).tolist()]
 np.savetxt(out/(bag.name+'_aligned.csv'),np.column_stack([grid-origin,target,actual]),delimiter=',',header='t,pred_v,pred_w,actual_v,actual_w',comments='')
 fig,axes=plt.subplots(3,1,figsize=(12,10))
 for j in (0,1):
  axes[j].plot(grid-origin,target[:,j],label='predicted');axes[j].plot(grid-origin,actual[:,j],label='measured');axes[j].set_ylabel(['v m/s','w rad/s'][j]);axes[j].legend();axes[j].grid()
 axes[2].plot(pred[:,0],pred[:,1],label='integrated prediction');axes[2].plot(xy[:,0],xy[:,1],label='measured pose');axes[2].axis('equal');axes[2].legend();axes[2].grid();axes[2].set_xlabel('x m');axes[2].set_ylabel('y m')
 fig.tight_layout();fig.savefig(out/(bag.name+'.png'),dpi=130);plt.close(fig)
 summary.append(row);print(json.dumps(row),flush=True)
(out/'summary.json').write_text(json.dumps(summary,indent=2))

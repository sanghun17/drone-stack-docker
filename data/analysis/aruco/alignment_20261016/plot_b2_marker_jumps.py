from pathlib import Path
import json,sys
import numpy as np,cv2,rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[4];out=root/'data/results/b2_marker_jumps_20261016';base=Path('/home/ml/drone-data/shared/results/webcam-recordings')
fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True)
fig2,ims=plt.subplots(4,2,figsize=(10,17))
for k,s in enumerate(['20-29-49','20-30-35','20-31-12','20-31-49']):
 name='flight_2026-10-16_'+s;x=json.loads((out/(name+'.json')).read_text());rows=x['rows'];rep=x['report'];t=np.array([r['t_s'] for r in rows])-rep['approach_s'][0];err=np.array([r['error_m'] for r in rows])*100
 axes[k].plot(t,err,color='steelblue',lw=.7)
 no19=np.array([r['inliers'] is not None and 19 not in r['inliers'] for r in rows])
 axes[k].scatter(t[no19],err[no19],s=5,color='crimson',label='ID19 absent from reported inlier set')
 axes[k].axhline(20,color='grey',ls='--',lw=.6);axes[k].set(ylabel='GT error (cm)',title=s.replace('-',':'),ylim=(0,70));axes[k].grid(alpha=.2)
 if k==0:axes[k].legend(loc='upper right',fontsize=8)
 selected={}
 with rosbag.Bag(str(base/name/(name+'.bag'))) as b:
  for _,m,_ in b.read_messages(topics=['/landing/debug/image/compressed']):
   stamp=m.header.stamp.to_sec()-.041589316816677754;j=int(np.argmin([abs(r['stamp']-stamp) for r in rows]));r=rows[j]
   if abs(r['stamp']-stamp)>.0001:continue
   tag='bad' if r['error_m']>.2 else 'good'
   if tag in selected:continue
   selected[tag]=(m,r)
   if len(selected)==2:break
 for c,tag in enumerate(['good','bad']):
  m,r=selected[tag];im=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR)
  cv2.imwrite(str(out/(s+'_'+tag+'.jpg')),im)
  ims[k,c].imshow(cv2.cvtColor(im,cv2.COLOR_BGR2RGB));ims[k,c].axis('off')
  ims[k,c].set_title(s.replace('-',':')+' '+tag+' | '+str(round(r['error_m']*100,1))+' cm\nreported inliers '+str(r['inliers']),fontsize=9)
axes[-1].set_xlabel('Seconds after APPROACH entry');fig.suptitle('Recorded estimator output: brief, frequent pose jumps');fig.tight_layout();fig.savefig(out/'live_position_jumps.png',dpi=160);plt.close(fig)
fig2.suptitle('Recorded debug images matched by corrected capture stamp\nGreen: reported accepted marker; orange: detected but excluded marker',fontsize=11);fig2.tight_layout(rect=[0,0,1,.96]);fig2.savefig(out/'live_good_bad_images.jpg',dpi=120);plt.close(fig2)
# Raw pair: red crosses show production detector ID19 corner locations.
x=json.loads((out/'first_b2_raw_replay.json').read_text());rr=x['rows'];j=next(i for i,r in enumerate(rr) if r['poses'].get('all',{}).get('position_m',0)>.2)
fig,axes=plt.subplots(1,2,figsize=(10,5))
for ax,tag,r in zip(axes,['good','bad'],[rr[j-1],rr[j]]):
 im=cv2.imread(str(out/('raw_'+tag+'.png')));ax.imshow(cv2.cvtColor(im,cv2.COLOR_BGR2RGB));c=np.array(r['corners']['19']);ax.plot(np.r_[c[:,0],c[0,0]],np.r_[c[:,1],c[0,1]],'r-',lw=1);ax.plot(c[:,0],c[:,1],'rx',ms=7);ax.set(xlim=(280,490),ylim=(485,280),title=tag+': ID19 corners | error %.1f cm'%(100*r['poses']['all']['position_m']));ax.axis('off')
fig.suptitle('Original raw frames 16.7 ms apart: ID19 quad moves inward');fig.tight_layout();fig.savefig(out/'raw_id19_corner_jump.png',dpi=170)

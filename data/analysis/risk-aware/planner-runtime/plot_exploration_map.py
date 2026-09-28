#!/usr/bin/env python3
"""Show recorded collision-map slices and GT motion, with no map modification."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.ndimage import minimum_filter,label
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();m=json.loads((a.directory/'map_metadata.json').read_text());d=np.load(a.directory/'map.npz');r=m['resolution'];b=m['bbox']
lo=np.array([b[k+'_min'] for k in 'xyz']);hi=np.array([b[k+'_max'] for k in 'xyz']);shape=np.ceil((hi-lo)/r).astype(int)
grid=np.zeros(shape,dtype=np.uint8)
for name,value in [('free',1),('occupied',2)]:
 idx=np.floor((d[name]-lo)/r).astype(int);valid=((idx>=0)&(idx<shape)).all(1);idx=idx[valid];grid[tuple(idx.T)]=value
# A conservative raster diagnostic; the planner uses exact OctoMap box checks.
clear=minimum_filter((grid==1).astype(np.uint8),size=(5,5,3),mode='constant')>0
z=int((1.375-lo[2])/r);cm=clear[:,:,z];regions,n=label(cm);path=d['gt'];start=np.floor((path[0,1:3]-lo[:2])/r).astype(int);region=int(regions[tuple(start)])
fig,axes=plt.subplots(2,1,figsize=(14,9),layout='constrained')
from matplotlib.colors import ListedColormap
for ax in axes:
 ax.imshow(grid[:,:,z].T,origin='lower',extent=[lo[0],hi[0],lo[1],hi[1]],cmap=ListedColormap(['#eeeeee','#b8d8eb','#333333']),vmin=0,vmax=2,interpolation='nearest')
 ax.contour(lo[0]+(np.arange(shape[0])+.5)*r,lo[1]+(np.arange(shape[1])+.5)*r,cm.T,levels=[.5],colors=['#249b52'],linewidths=.7)
 ax.plot(path[::5,1],path[::5,2],color='#d55e00',lw=1,label='GT trajectory');ax.plot(path[0,1],path[0,2],'o',color='#d55e00');ax.plot(path[-1,1],path[-1,2],'x',color='#d55e00',ms=8)
 ax.set(xlabel='GT x [m]',ylabel='GT y [m]',aspect='equal');ax.grid(alpha=.2);ax.legend()
axes[0].set_title('Recorded map at z=1.375m: unknown gray, free blue, occupied dark; green = body clearance')
known=np.concatenate([d['free'],d['occupied']]);axes[1].set(xlim=(known[:,0].min()-1,known[:,0].max()+1),ylim=(known[:,1].min()-1,known[:,1].max()+1),title='Observed-region detail')
fig.savefig(a.directory/'map_clearance.png',dpi=170)
(a.directory/'clearance_summary.json').write_text(json.dumps({'raster_body_dimensions_m':[1.25,1.25,.75],'connected_components':n,'start_component':region,'start_component_cells':int((regions==region).sum()) if region else 0},indent=2)+'\n')

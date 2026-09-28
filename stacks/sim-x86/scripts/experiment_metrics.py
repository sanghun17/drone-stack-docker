"""Backend map adapters for the unchanged historical eval_core metrics."""
from pathlib import Path
import sys
import threading
import time
import numpy as np
import rospy
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2

ROOT = Path(__file__).resolve().parents[3]
LEGACY = ROOT / 'ws/risk-aware-comparison/src/risk_aware_planning/mav_active_3d_planning/active_3d_planning_app_reconstruction/scripts'
sys.path.insert(0, str(LEGACY))
from eval_core import compute, load_gt_voxel_set, read_bbox_from_rosparams, pointcloud2_to_xyz, pointcloud2_to_xyzi, split_tsdf_by_threshold


def expand_markers(message, voxel_size, bbox):
    """Expand variable-size leaves to global evaluation voxel centers.

    The RHEM publisher sends a complete MarkerArray snapshot, with identity
    poses, world-frame CUBE_LISTs, and DELETE for empty depth levels.
    """
    chunks = []
    lower = np.array([bbox.x_min, bbox.y_min, bbox.z_min])
    upper = np.array([bbox.x_max, bbox.y_max, bbox.z_max])
    for marker in message.markers:
        if marker.action in (marker.DELETE, marker.DELETEALL):
            continue
        if marker.header.frame_id != 'odom' or marker.type != marker.CUBE_LIST:
            raise ValueError('Expected RHEM world-frame cube-list snapshot')
        p, q = marker.pose.position, marker.pose.orientation
        if any(abs(v)>1e-9 for v in [p.x,p.y,p.z,q.x,q.y,q.z]) or q.w not in (0.,1.):
            raise ValueError('Non-identity OctoMap marker pose')
        size=np.array([marker.scale.x,marker.scale.y,marker.scale.z])
        if not np.isfinite(size).all() or np.any(size < voxel_size-1e-8):
            raise ValueError('OctoMap leaf smaller than evaluation voxel or invalid')
        centers=np.array([(p.x,p.y,p.z) for p in marker.points],dtype=float).reshape(-1,3)
        if not np.isfinite(centers).all():
            raise ValueError('Non-finite OctoMap leaf')
        lo=np.maximum(centers-size/2,lower);hi=np.minimum(centers+size/2,upper)
        first=np.ceil(lo/voxel_size-.5-1e-8).astype(int)
        last=np.ceil(hi/voxel_size-.5-1e-8).astype(int)
        valid=np.all(last>first,axis=1)
        extent=(last-first)[valid];first=first[valid]
        # Leaves at one depth share their voxel offsets. Expand each distinct
        # clipped shape in a batch, instead of one Python meshgrid per leaf.
        shapes,groups=np.unique(extent,axis=0,return_inverse=True)
        for group,shape in enumerate(shapes):
            offsets=np.stack(np.meshgrid(*(np.arange(n) for n in shape),indexing='ij'),axis=-1).reshape(-1,3)
            starts=first[groups==group]
            batch=max(1,1000000//len(offsets))
            for begin in range(0,len(starts),batch):
                chunks.append(((starts[begin:begin+batch,None,:]+offsets[None,:,:]+.5)*voxel_size).reshape(-1,3))
    return np.concatenate(chunks) if chunks else np.empty((0,3))


class MapMetrics:
    def __init__(self, planner, get_param=rospy.get_param, gt_path=None, subscribe=True):
        self.voxel=float(get_param('/system/voxel_size'))
        self.bbox=read_bbox_from_rosparams(get_param)
        import os
        self.gt_path=Path(gt_path) if gt_path else Path(os.environ['RISK_AWARE_GT_DIR'])/'ModernLivingroom_long_ros.ply'
        if self.bbox is None:raise ValueError('Missing evaluation bounding box')
        self.gt=load_gt_voxel_set(str(self.gt_path),self.bbox,self.voxel)
        if not self.gt:raise RuntimeError('Empty GT evaluation voxel set')
        self.lock=threading.Lock();self.maps={};self.subscribers=[]
        if not subscribe:return
        if planner=='rhem':
            for kind in ('occupied','free'):
                self.subscribers.append(rospy.Subscriber('/rhem/bsp_planner/octomap_'+kind,MarkerArray,
                    self.receive,callback_args=(kind,'markers'),queue_size=1))
        elif planner=='pure':
            self.subscribers.append(rospy.Subscriber('/planner/voxblox_node/tsdf_pointcloud',PointCloud2,
                self.receive,callback_args=('both','tsdf'),queue_size=1))
        else:
            for kind,topic in [('occupied','/sdf_map/occupancy_all'),('free','/sdf_map/free')]:
                self.subscribers.append(rospy.Subscriber(topic,PointCloud2,self.receive,callback_args=(kind,'points'),queue_size=1))

    def receive(self,message,args):
        kind,fmt=args
        # Retain snapshots; expansion/metric work happens at evaluation cadence.
        with self.lock:
            self.maps[kind]=(message,fmt,time.monotonic())

    def sample(self):
        with self.lock:maps=dict(self.maps)
        if 'both' in maps:
            msg,_,stamp=maps['both']
            occ,free=split_tsdf_by_threshold(*pointcloud2_to_xyzi(msg),self.voxel,.75)
            age=time.monotonic()-stamp
        else:
            if not all(k in maps for k in ('occupied','free')):return None
            arrays=[]
            for kind in ('occupied','free'):
                msg,fmt,_=maps[kind]
                arrays.append(expand_markers(msg,self.voxel,self.bbox) if fmt=='markers' else pointcloud2_to_xyz(msg))
            occ,free=arrays
            age=time.monotonic()-min(maps[k][2] for k in ('occupied','free'))
        m=compute(occ,free,self.gt,self.voxel,self.bbox)
        return dict(surface_rate_vio=m.surface_rate_vio,volume_rate_vio=m.volume_rate_vio,
                    volume_m3=m.volume_m3,gt_total=m.gt_total,known_voxels=m.gt_in_observed,
                    map_age_s=age)

    def close(self):
        for subscriber in self.subscribers:subscriber.unregister()

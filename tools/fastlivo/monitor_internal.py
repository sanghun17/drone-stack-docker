#!/usr/bin/env python3
# Monitor FAST-LIVO2 INTERNAL health (no rebuild): per-frame effective-LiDAR-point count
# (/cloud_effected = effct_feat_num_, the LIO point-to-plane constraint count) and the visual
# sub-map point count (/cloud_visual_sub_map_before = VIO tracked patches), alongside the pose.
# A collapse in either under a given motion (e.g. pitch -> narrow vertical FOV) is the degeneracy
# that lets a bad update poison the map. Read-only ROS subscriber: safe on a live OR replay roscore.
#
#   docker exec -it drone-stack-d435i-voxblox bash -lc \
#     'source /opt/ros/noetic/setup.bash; source /work/ws/fast-livo/devel/setup.bash; \
#      python3 /work/tools/fastlivo/monitor_internal.py [--csv /work/tools/fastlivo/internal.csv]'
#
# Prints one line per LIO frame: t  lidar_eff_pts  visual_pts  |pos|  roll pitch yaw
import sys, argparse
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

ap = argparse.ArgumentParser()
ap.add_argument("--csv", default="")
a = ap.parse_args()

state = {"vis": 0, "pos": (0, 0, 0), "rpy": (0, 0, 0), "t0": None}
csv = open(a.csv, "w") if a.csv else None
if csv: csv.write("t,lidar_eff,visual,px,py,pz,roll,pitch,yaw\n")

def npts(m):  # PointCloud2 point count
    return m.width * m.height

def on_vis(m): state["vis"] = npts(m)

def on_odom(m):
    p = m.pose.pose.position; o = m.pose.pose.orientation
    state["pos"] = (p.x, p.y, p.z)
    state["rpy"] = tuple(math.degrees(x) for x in euler_from_quaternion([o.x, o.y, o.z, o.w]))

def on_eff(m):                              # fires once per LIO frame -> our sample tick
    t = m.header.stamp.to_sec()
    if state["t0"] is None: state["t0"] = t
    tr = t - state["t0"]
    le = npts(m); vp = state["vis"]
    px, py, pz = state["pos"]; r, p_, y = state["rpy"]
    pos = math.sqrt(px*px + py*py + pz*pz)
    flag = "  <-- LIO STARVED" if le < 200 else ("  <-- vis low" if vp < 50 else "")
    print("t=%6.2f  lidar_eff=%5d  visual=%4d  |pos|=%7.2f  rpy=%6.1f %6.1f %6.1f%s"
          % (tr, le, vp, pos, r, p_, y, flag))
    sys.stdout.flush()
    if csv: csv.write("%.3f,%d,%d,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n" % (tr, le, vp, px, py, pz, r, p_, y)); csv.flush()

rospy.init_node("livo_internal_monitor", anonymous=True)
rospy.Subscriber("/cloud_effected", PointCloud2, on_eff, queue_size=50)
rospy.Subscriber("/cloud_visual_sub_map_before", PointCloud2, on_vis, queue_size=50)
rospy.Subscriber("/aft_mapped_to_odom", Odometry, on_odom, queue_size=50)
print("[monitor] subscribed: /cloud_effected (LIO eff pts), /cloud_visual_sub_map_before (VIO pts), /aft_mapped_to_odom")
print("[monitor] waiting for FAST-LIVO2 frames (live or replay)…  Ctrl-C to stop")
rospy.spin()

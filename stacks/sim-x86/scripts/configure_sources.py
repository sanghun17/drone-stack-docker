#!/usr/bin/env python3
"""Select simulation planning/control inputs before starting the runtime."""
import argparse
import json
import rosnode
import rospy

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--check-only', action='store_true')
parser.add_argument('--planning-source', choices=['gt', 'fast-livo'], default='fast-livo')
parser.add_argument('--control-source', choices=['gt', 'fast-livo'], default=None)
args = parser.parse_args()
args.control_source = args.control_source or args.planning_source
rospy.init_node('configure_comparison_sources', anonymous=True)
active = [n for n in rosnode.get_node_names() if n in (
    '/airsim_gt_odom_publisher', '/initialize_simulator', '/laserMapping', '/so3_control_bridge', '/traj_server',
    '/exploration_node', '/jax_mppi_controller', '/planner/planner_node') or n.startswith('/rhem/')]
if active:
    raise SystemExit('Stop sensor/estimator/planner/control before switching sources: ' + ', '.join(active))
if args.check_only:
    raise SystemExit(0)
if rospy.get_param('/system/platform') != 'sim':
    raise SystemExit('Source selection is only supported for simulation')
gt = args.planning_source == 'gt'
rospy.set_param('/system/localization', 'gt' if gt else 'vio')
rospy.set_param('/system/body_frame', 'base_link' if gt else 'aft_mapped')
rospy.set_param('/system/context_vel_topic', '/comparison/gt/world_odom' if gt else '/LIVO2/imu_propagate')
control_topic = ('/robot/odom' if args.control_source == args.planning_source else
                 '/gt_odom' if args.control_source == 'gt' else '/comparison/fast_livo/odom')
sources = dict(planning_source=args.planning_source, control_source=args.control_source,
               estimation_source='fast-livo', planning_odom_topic='/robot/odom',
               control_odom_topic=control_topic, rhem_belief_source='rovio')
rospy.set_param('/comparison/sources', sources)
rospy.set_param('/comparison/rhem_belief_mode', 'rovio')
rospy.set_param('/comparison/rhem_filter_profile', 'historical')
rospy.set_param('/comparison/rhem_progress_profile', 'historical')
rospy.set_param('/comparison/rhem_map_rays', 'clipped')
rospy.set_param('/comparison/rhem_diagnostics', False)
rospy.set_param('/so3_control_bridge/max_thrust', 15.60)
rospy.set_param('/comparison/rhem_gt_conservative', False)
rospy.set_param('/comparison/sensor_calibration', 'historical')
print(json.dumps(sources, indent=2))

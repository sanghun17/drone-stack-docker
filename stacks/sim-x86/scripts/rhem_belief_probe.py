#!/usr/bin/env python3
"""Read-only GT diagnostic snapshots of raw and externally aligned ROVIO belief."""
import time
import rospy
from std_msgs.msg import Header
from bsp_msgs.msg import ROVIO_FilterStateMsg
from bsp_msgs.srv import BSP_SrvSendFilterState

rospy.init_node('rhem_belief_probe')
proxies = [(rospy.ServiceProxy(service, BSP_SrvSendFilterState),
            rospy.Publisher('/rhem/diagnostics/'+name, ROVIO_FilterStateMsg, queue_size=1))
           for name, service in [('raw_belief','/rhem/rovio/send_filter_state'),
                                 ('aligned_belief','/rhem/belief/send_filter_state')]]
while not rospy.is_shutdown():
    for proxy, pub in proxies:
        try:
            pub.publish(proxy(Header(stamp=rospy.Time.now(),frame_id='odom')).filterState)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(5, 'Belief diagnostic: %s', error)
    time.sleep(2)

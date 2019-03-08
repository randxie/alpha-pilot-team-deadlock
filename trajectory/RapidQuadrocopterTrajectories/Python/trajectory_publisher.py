#!/usr/bin/env python
import rospy

def publisher():
    pub = rospy.Publisher('waypoints', array, queue_size = 10)
    rospy.init_node('trajectory_publisher', anonymous = True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        

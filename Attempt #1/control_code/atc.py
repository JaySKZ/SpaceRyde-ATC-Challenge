#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Bool, Empty

# Find plane position at 10 Hz
def ATC_query():
    pub_query = rospy.Publisher('/plane/query', Bool, queue_size=1)

    rospy.init_node('ATC_query', anonymous=False)

    rate = rospy.Rate(10) #10 Hz to receive odom 

    while not rospy.is_shutdown():
        pub_query.publish(True) 
        rate.sleep()



#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int32

def int_cd(data):
    print(data)

rospy.init_node('int_sub')
rospy.Subscriber('/int_count', Int32, int_cd)
rospy.spin()

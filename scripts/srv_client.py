#!/usr/bin/env python3

import rospy

from std_srvs.srv import SetBool

rospy.init_node('srv_client')

try:
    srv_call = rospy.ServiceProxy('srv_call', SetBool)
    srv_call(False)
except rospy.service.ServiceException as e:
    print('SrvErr\n%s'%(e))

    #import traceback as tb
    #tb.print_exc()

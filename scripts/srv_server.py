#!/usr/bin/env python3

import rospy

# サービスライブラリの追加
from std_srvs.srv import SetBool, SetBoolResponse

def srv_cb(data):
    resp = SetBoolResponse()

    if data.data is True:
        resp.message = 'calles'
        resp.success = True
    else:
        resp.message = 'ready'
        resp.success = False

    print(resp.message)
    
    return resp

rospy.init_node('srvs_server')

srv = rospy.Service('srv_call', SetBool, srv_cb)
rospy.spin()

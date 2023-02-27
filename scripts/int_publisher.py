#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int32

# ノードの名前を定義する
rospy.init_node('int_pub')

# パブリッシュするトピックの名前を指定する
pub = rospy.Publisher('int_count', Int32, queue_size=1)

# 実行周期を設定する。今回は1秒おきに配信するようにする。
rate = rospy.Rate(1)

count = 0

while not rospy.is_shutdown():
    # count 変数をパブリッシュ
    pub.publish(count)

    print(count)
    count += 1

    rate.sleep()

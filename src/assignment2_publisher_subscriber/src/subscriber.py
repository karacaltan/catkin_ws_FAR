#/usr/bin/env python

import rospy
from autominy_msgs.msg import Speed
from std_msgs.msg import String

def callback_listener(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/sensors/speed", String, callback_listener)
    rospy.spin()


def callback(raw_msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", raw_msg.value)

def sub_speed():
    rospy.init_node('sub_speed', anonymous=True)
    rospy.Subscriber("/sensors/speed", Speed, callback)
    rospy.spin()


if __name__ == '__main__':
    #listener()
    sub_speed()
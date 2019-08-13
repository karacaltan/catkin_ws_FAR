# /usr/bin/env python

import rospy
from autominy_msgs.msg import Speed
from std_msgs.msg import String
import std_msgs.msg


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        msg = "Message: %s" % rospy.get_time()
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

def pub_speed():
    pub = rospy.Publisher('/sensors/speed', Speed, queue_size=10)
    rospy.init_node('pub_speed')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        pub.publish(header, 12.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        #talker()
        pub_speed()
    except rospy.ROSInterruptException:
        pass

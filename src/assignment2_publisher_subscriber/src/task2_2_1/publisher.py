import rospy
from autominy_msgs.msg import Speed
import std_msgs.msg


def publisher():
    pub = rospy.Publisher('/sensors/speed', Speed, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        pub.publish(header, 0.3)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        raise

import rospy
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand
import std_msgs.msg


def publisher():
    pub = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    pub_2 = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    rospy.init_node('publisher_2', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header_2 = std_msgs.msg.Header()
        pub.publish(header, 1.0)
        pub_2.publish(header_2, 0.3)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        raise

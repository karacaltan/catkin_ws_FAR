import rospy
from autominy_msgs.msg import SpeedCommand
from std_msgs.msg import String

def callback(raw_msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", raw_msg.value)

def subscriber():
    rospy.init_node('subscriber_2', anonymous=True)
    rospy.Subscriber("/actuators/speed", SpeedCommand, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()

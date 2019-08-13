import rospy
from autominy_msgs.msg import Speed
from std_msgs.msg import String

def callback(raw_msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", raw_msg.value)

def subscriber():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("/sensors/speed", Speed, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()

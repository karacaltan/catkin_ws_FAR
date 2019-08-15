import rospy
import std_msgs.msg
from sensor_msgs.msg import CameraInfo

class BasicSubscriber:

    def __init__(self):
        rospy.init_node("basic_subscriber")
        self.cameraInfo_subscriber = rospy.Subscriber("/sensors/camera/infra1/camera_info", CameraInfo, self.msg_cameraInfo, queue_size=10)
        rospy.spin()

    def msg_cameraInfo(self, msg):
        array_extracted_msg_K = [
            [
                "K.fx",
                msg.K[0]
            ],
            [
                "K.fy",
                msg.K[4]
            ],
            [
                "K.cx",
                msg.K[2]
            ],
            [
                "K.cy",
                msg.K[5]
            ],
        ]

        array_extracted_msg_D = [
            [
                "D.k1",
                msg.D[0]
            ],
            [
                "D.k2",
                msg.D[1]
            ],
            [
                "D.t1",
                msg.D[2]
            ],
            [
                "D.t2",
                msg.D[3]
            ],
            [
                "D.k3",
                msg.D[4]
            ],
        ]
        print (array_extracted_msg_K)
        print (array_extracted_msg_D)

if __name__ == "__main__":
    BasicSubscriber()
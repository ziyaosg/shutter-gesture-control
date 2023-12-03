#!/usr/bin/env python3

import cv2
import time
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError


NEUTRAL = [0.0, 0.0, 0.0, 0.0]
FIVE_SECONDS = 5


class CameraNode():
    def __init__(self):
        rospy.init_node('camera')

        # joints
        self.joints_pub = rospy.Publisher("/joint_group_controller/command", Float64MultiArray, queue_size=5)
        self.joints_sub = rospy.Subscriber("/joint_states", JointState, self.joints_callback, queue_size=5)

        # camera
        self.camera_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.init_time = time.time()
        self.photo_taken = False
        print("Photo will be taken in 5 seconds...")

        rospy.spin()
        

    def joints_callback(self, msg):
        try:
            if msg.position != NEUTRAL:
                msg = Float64MultiArray()
                msg.data = NEUTRAL
                self.joints_pub.publish(msg)
        except:
            print("Joints position update failed.")
            

    def image_callback(self,msg):
        '''
        takes one photo after 5 seconds this program is called
        '''
        if self.photo_taken:
            return

        current_time = time.time()
        if current_time - self.init_time > FIVE_SECONDS:
            bridge = CvBridge()
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                filename = 'shutter_photo_{}.jpeg'.format(time.strftime("%Y%m%d-%H%M%S"))
                cv2.imwrite(filename, cv_image)
                self.photo_taken = True
                print("Photo taken!")
            except CvBridgeError as e:
                print(e)


if __name__ == '__main__':
    try:
        node = CameraNode()
    except rospy.ROSInterruptException:
        pass

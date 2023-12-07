#!/usr/bin/env python3

import cv2
import time
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge, CvBridgeError
from Sign_Language_Detection.wrapper import gesture_recognition
import Sign_Language_Detection.HandTrackingModule as htm


NUM_JOINTS = 32
HAND_LEFT = 8
ELBOW_LEFT = 6
WRIST_LEFT = 7
HAND_RIGHT = 15
ELBOW_RIGHT = 13
WRIST_RIGHT = 14
SMALL_DELTA = 0.02
MID_DELTA = 0.04
BIG_DELTA = 0.1
CLAPS_HANDS_THRESHOLD = 0.05
RELATIVITY_VERTICAL_THRESHOLD = 0.1
RELATIVITY_HORIZONTAL_THRESHOLD = 0.15
NEUTRAL = [0.0, 0.0, 0.0, 0.0]
JOINT1_CONSTRAINT = [-1.5, 1.5]
JOINT3_CONSTRAINT = [-0.7, 1.2]
JOINT4_CONSTRAINT = [-0.5, 0.8]
FIVE_SECONDS = 5


class ControlNode():
    def __init__(self):
        rospy.init_node('control')

        # data
        self.current_pose = NEUTRAL
        self.img_msg = None

        # params
        self.body_tracking_topic = rospy.get_param('~body_tracking_topic', '/body_tracking_data')

        # publishers
        self.joints_pub = rospy.Publisher("/joint_group_controller/command", Float64MultiArray, queue_size=5)

        # subscribers
        self.joints_sub = rospy.Subscriber("/joint_states", JointState, self.joints_callback, queue_size=5)
        self.body_sub = rospy.Subscriber(self.body_tracking_topic, MarkerArray, self.body_callback, queue_size=5)

        # camera
        self.camera_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.photo_taken = False
        print("Photo will be taken when you show the gesture of ok...")
        self.init_time = None
        self.start_photo = False

        # gesture module
        self.detector = htm.handDetector(detectionCon = 0)

        rospy.Rate(10)
        rospy.spin()
        

    def joints_callback(self, msg):
        try:
            self.current_pose = [msg.position[0], 
                                 msg.position[1], 
                                 msg.position[2], 
                                 msg.position[3]]
        except:
            print("Joints position update failed.")


    def body_callback(self, msg):
        '''
        when facing shutter, for both body joint relativity and shutter joint control
        left: -; right: +; 
        up: +; down: -
        '''
        if not msg:
            print("Body tracking message emtpy.")
            return

        # shutter control with body tracking
        delta_x = 0
        delta_y = 0
        delta_y_right = 0
        if msg.markers:
            
            # find closest person
            num_people = len(msg.markers) // NUM_JOINTS
            offset = 0
            closest_person_dis = float("inf")

            for i in range(num_people):
                curr_offset = i * NUM_JOINTS
                curr_dis = msg.markers[ELBOW_LEFT + curr_offset].pose.position.z + msg.markers[WRIST_LEFT + curr_offset].pose.position.z
                if curr_dis < closest_person_dis:
                    offset = curr_offset
                    closest_person_dis = curr_dis
            
            # get closest person's joints relativity
            elbow_left = ELBOW_LEFT + offset
            wrist_left = WRIST_LEFT + offset
            relativity_horizontal = msg.markers[elbow_left].pose.position.x - msg.markers[wrist_left].pose.position.x
            relativity_vertical = msg.markers[elbow_left].pose.position.y - msg.markers[wrist_left].pose.position.y
            elbow_right = ELBOW_RIGHT + offset
            wrist_right = WRIST_RIGHT + offset
            relativity_vertical_right = msg.markers[elbow_right].pose.position.y - msg.markers[wrist_right].pose.position.y
            
            # shutter only starts moving if joints relativity exceeds a certain threshold
            if abs(relativity_horizontal) >= RELATIVITY_HORIZONTAL_THRESHOLD:
                delta_x = MID_DELTA if relativity_horizontal > 0 else -MID_DELTA
            if abs(relativity_vertical) >= RELATIVITY_VERTICAL_THRESHOLD:
                delta_y = BIG_DELTA if relativity_vertical > 0 else -SMALL_DELTA
            else:
                delta_y = SMALL_DELTA
            if abs(relativity_vertical_right) >= RELATIVITY_VERTICAL_THRESHOLD:
                delta_y_right = BIG_DELTA if relativity_vertical_right > 0 else -SMALL_DELTA

        # if self.current_pose != NEUTRAL:
        joint_horizontal = self.current_pose[0]
        joint_vertical = self.current_pose[2]
        joint_vertical_right = self.current_pose[3]

        new_joint_vertical = joint_vertical + delta_y
        new_joint_horizontal = joint_horizontal + delta_x
        new_joint_vertical_right = joint_vertical_right + delta_y_right

        new_joint_vertical = JOINT3_CONSTRAINT[0] if new_joint_vertical < JOINT3_CONSTRAINT[0] else new_joint_vertical
        new_joint_vertical = JOINT3_CONSTRAINT[1] if new_joint_vertical > JOINT1_CONSTRAINT[1] else new_joint_vertical
        new_joint_horizontal = JOINT1_CONSTRAINT[0] if new_joint_horizontal < JOINT1_CONSTRAINT[0] else new_joint_horizontal
        new_joint_horizontal = JOINT1_CONSTRAINT[1] if new_joint_horizontal > JOINT1_CONSTRAINT[1] else new_joint_horizontal
        new_joint_vertical_right = JOINT4_CONSTRAINT[0] if new_joint_vertical_right < JOINT4_CONSTRAINT[0] else new_joint_vertical_right
        new_joint_vertical_right = JOINT4_CONSTRAINT[1] if new_joint_vertical_right > JOINT4_CONSTRAINT[1] else new_joint_vertical_right

        new_msg = Float64MultiArray()
        new_msg.data = [new_joint_horizontal, 0.0, new_joint_vertical, new_joint_vertical_right]
        # new_msg.data = [0.0, 0.0, 0.0, new_joint_vertical_right]

        if not self.start_photo:
            self.joints_pub.publish(new_msg)
        
        if not msg.markers:
            return

        # photo taking with gesture recognition
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(self.img_msg, "bgr8")
        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)
        if self.start_photo == False:
            cv_image = self.detector.findHands(cv_image)
            posList = self.detector.findPosition(cv_image, draw=False)
            if len(posList) != 0:
                result = gesture_recognition(posList=posList)
            else:
                result = None
            if (result == 'F') and self.start_photo == False:
                self.init_time = time.time()
                self.start_photo = True
                print('Five Second Count Down Begin!!!')
        
        current_time = time.time()
        
        if self.start_photo == True:
            if (current_time - self.init_time > FIVE_SECONDS):
                self.start_photo = False
                bridge = CvBridge()
                try:
                    cv_image = bridge.imgmsg_to_cv2(self.img_msg, "bgr8")

                    filename = 'shutter_photo_{}.jpeg'.format(time.strftime("%Y%m%d-%H%M%S"))
                    cv2.imwrite(filename, cv_image)
                    self.photo_taken = True
                    self.start_photo = False
                    print("Photo taken!")
                except CvBridgeError as e:
                    print(e)

    def image_callback(self, msg):
        if not msg:
            print("Image message empty.")
        self.img_msg = msg


if __name__ == '__main__':
    try:
        node = ControlNode()
    except rospy.ROSInterruptException:
        pass

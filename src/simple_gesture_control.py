#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray


ELBOW_LEFT = 6
WRIST_LEFT = 7
ELBOW_RIGHT = 13
WRIST_RIGHT = 14
SMALL_DELTA = 0.02
BIG_DELTA = 0.1
RELATIVITY_VERTICAL_THRESHOLD = 0.1
RELATIVITY_HORIZONTAL_THRESHOLD = 0.15
NEUTRAL = [0.0, 0.0, 0.0, 0.0]
JOINT1_CONSTRAINT = [-1.5, 1.5]
JOINT3_CONSTRAINT = [-0.7, 1.2]


class ControlNode():
    def __init__(self):
        rospy.init_node('control')

        # data
        self.current_pose = NEUTRAL

        # params
        self.body_tracking_topic = rospy.get_param('~body_tracking_topic', '/body_tracking_data')

        # publishers
        self.joints_pub = rospy.Publisher("/joint_group_controller/command", Float64MultiArray, queue_size=5)

        # subscribers
        self.joints_sub = rospy.Subscriber("/joint_states", JointState, self.joints_callback, queue_size=5)
        self.body_sub = rospy.Subscriber(self.body_tracking_topic, MarkerArray, self.body_callback, queue_size=5)

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

        delta_x = 0
        delta_y = 0
        if msg.markers:
            relativity_horizontal = msg.markers[ELBOW_LEFT].pose.position.x - msg.markers[WRIST_LEFT].pose.position.x
            relativity_vertical = msg.markers[ELBOW_LEFT].pose.position.y - msg.markers[WRIST_LEFT].pose.position.y
            
            if abs(relativity_horizontal) >= RELATIVITY_HORIZONTAL_THRESHOLD:
                delta_x = SMALL_DELTA if relativity_horizontal > 0 else -SMALL_DELTA
            if abs(relativity_vertical) >= RELATIVITY_VERTICAL_THRESHOLD:
                delta_y = BIG_DELTA if relativity_vertical > 0 else -SMALL_DELTA
            else:
                delta_y = SMALL_DELTA

        if self.current_pose != NEUTRAL:
            joint_horizontal = self.current_pose[0]
            joint_vertical = self.current_pose[2]
            new_joint_vertical = joint_vertical + delta_y
            new_joint_horizontal = joint_horizontal + delta_x

            new_joint_vertical = JOINT3_CONSTRAINT[0] if new_joint_vertical < JOINT3_CONSTRAINT[0] else new_joint_vertical
            new_joint_vertical = JOINT3_CONSTRAINT[1] if new_joint_vertical > JOINT1_CONSTRAINT[1] else new_joint_vertical
            new_joint_horizontal = JOINT1_CONSTRAINT[0] if new_joint_horizontal < JOINT1_CONSTRAINT[0] else new_joint_horizontal
            new_joint_horizontal = JOINT1_CONSTRAINT[1] if new_joint_horizontal > JOINT1_CONSTRAINT[1] else new_joint_horizontal

            # TODO: control joint 4 to fine tune camera angle with right arm

            new_msg = Float64MultiArray()
            new_msg.data = [new_joint_horizontal, 0.0, new_joint_vertical, 0.0]
            self.joints_pub.publish(new_msg)


if __name__ == '__main__':
    try:
        node = ControlNode()
    except rospy.ROSInterruptException:
        pass

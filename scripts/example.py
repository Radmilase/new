#!/usr/bin/env python3

import rospy
import cv2
import os
from cv_bridge import CvBridge


from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Time
from std_msgs.msg import Header
from std_msgs.msg import Duration
import numpy as np


from controller_manager_msgs.srv import SwitchController

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class Example(object):

    def __init__(self):
        rospy.loginfo("[Example] loaging")
        rospy.on_shutdown(self.shutdown)
        self.gui = os.getenv('GUI')=='true' or os.getenv('GUI')=='True'

        sub_image_topic_name = "/head/camera1/image_raw"
        point_cloud_topic = "/head/camera1/image_raw"

        self.camera_subscriber = rospy.Subscriber(sub_image_topic_name, Image, self.camera_callback)
        self.point_cloud_subscriber = rospy.Subscriber(point_cloud_topic, PointCloud2, self.point_cloud_callback)
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        self.joint_position_publisher = rospy.Publisher("/cmd_vel", JointState, queue_size=1)
        self.joint_trajectory_publisher = rospy.Publisher('/delta_robot/delta_robot_controller/command', JointTrajectory, queue_size=10)
        
        rospy.wait_for_service("/controller_manager/switch_controller")
        self.switch_controller_service = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

        self.curent_image = None
        self.joints_pose = None
        self.joints_velocity = None

        self.bridge = CvBridge()


        rospy.loginfo("[Example] loaded")

    def __del__(self):
        pass

    def switch_controller(self, start_controllers: list, stop_controllers: list):
        response = self.switch_controller_service(
            start_controllers, stop_controllers,
            strictness=1, start_asap=False, timeout=0.0
        )
        print("Switch controller: ", response.ok)
        if not response.ok:
            print(response)

    def go_to_using_trajectory(self, goal: np.ndarray, moving_time: float = 5):
        self.switch_controller(
            start_controllers=["eff_joint_traj_controller"],
            stop_controllers=["joint_group_eff_controller"]
        )
        joints_str = JointTrajectory()
        joints_str.header = Header()
        joints_str.header.stamp = rospy.Time.now()
        joints_str.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = goal.tolist()
        point.time_from_start = rospy.Duration(moving_time)
        joints_str.points.append(point)

        self.joint_trajectory_publisher.publish(joints_str)
        rospy.loginfo("position updated")
        
    def go_to_using_servo(self):
        # Do not use trajectory planning before start moving, go to goal immediately
        self.switch_controller(
            start_controllers=["joint_group_eff_controller"],
            stop_controllers=["eff_joint_traj_controller"]
        )
        


    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        # some processing here
        if self.gui:
            cv2.imshow("output", frame)
            cv2.waitKey(1)

    def joint_states_callback(self, msg: JointState):
        self.joints_pose = msg.position
        self.joints_velocity = msg.velocity
    
    def shutdown(self):
        # stop robots here
        self.switch_controller(
            start_controllers=["joint_group_eff_controller"],
            stop_controllers=["eff_joint_traj_controller"]
        )
        self.joint_position_publisher()

    def spin(self):

        rate = rospy.Rate(30)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time() - t0
            if if t%60<10:
                # Find garbage
                self.go_to_using_servo(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            if t%60==10:
                # To home position
                self.go_to_using_trajectory(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 5)
            if t%60==15:
                # To garbage
                self.go_to_using_trajectory(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 5)
            rate.sleep()
            


def main(args=None):
    rospy.init_node("example_node")

    exp = Example()
    exp.spin()


if __name__ == "__main__":
    main()

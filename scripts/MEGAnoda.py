#!/usr/bin/env python3

import rospy
import json
import numpy as np
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, JointState
from std_msgs.msg import String, Header, Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from controller_manager_msgs.srv import SwitchController
import sensor_msgs.point_cloud2 as pc2

# Общие константы
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class ObjectDetector:
    def __init__(self):
        rospy.loginfo("[ObjectDetector] Initializing...")
        
        # Подписка на детекции (например, YOLOv5)
        self.detected_objects_sub = rospy.Subscriber(
            "/detected_objects", String, self.detection_callback
        )
        
        # Подписка на PointCloud2 для получения глубины
        self.point_cloud_sub = rospy.Subscriber(
            "/hand_eye/camera/depth/points", PointCloud2, self.point_cloud_callback
        )
        
        # Публикация позы объекта
        self.object_pose_pub = rospy.Publisher(
            "/object_pose", PoseStamped, queue_size=1
        )
        
        self.latest_pointcloud = None
        rospy.loginfo("[ObjectDetector] Loaded")
    
    def point_cloud_callback(self, msg):
        self.latest_pointcloud = msg
    
    def detection_callback(self, msg):
        # Пример: msg.data содержит JSON-строку с bbox
        try:
            detections = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"Failed to parse detection message: {e}")
            return
        
        if not self.latest_pointcloud:
            rospy.logwarn("No pointcloud received yet!")
            return
        
        for det in detections.get("objects", []):
            bbox = det.get("bbox", None)
            if not bbox or len(bbox) != 4:
                continue
            
            # Центр bbox в пикселях
            xmin, ymin, xmax, ymax = bbox
            u = int((xmin + xmax) / 2)
            v = int((ymin + ymax) / 2)
            
            # Получаем 3D точку из pointcloud
            gen = pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), 
                                  skip_nans=True, uvs=[(u, v)])
            try:
                point = next(gen)
                x, y, z = point
            except StopIteration:
                rospy.logwarn("No valid 3D point for detection!")
                continue
            
            # Заполняем PoseStamped
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.latest_pointcloud.header.frame_id
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            
            self.object_pose_pub.publish(pose)
            rospy.loginfo(f"Published object pose at ({x:.3f}, {y:.3f}, {z:.3f})")


class RobotController:
    def __init__(self):
        # Паблишеры для управления манипулятором и хватателем
        self.joint_position_publisher = rospy.Publisher(
            "/joint_group_eff_controller/command", Float64MultiArray, queue_size=1)
        self.joint_trajectory_publisher = rospy.Publisher(
            '/eff_joint_traj_controller/command', JointTrajectory, queue_size=1)
        self.gripper_trajectory_publisher = rospy.Publisher(
            '/gripper/command', JointTrajectory, queue_size=3)
        
        rospy.wait_for_service("/controller_manager/switch_controller")
        self.switch_controller_service = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController)
        
        self.current_controller = "joint_group_eff_controller"
        
        # Для хранения текущих положений сочленений
        self.joints_pose = None
        self.joint_state_subscriber = rospy.Subscriber(
            "/joint_states", JointState, self.joint_states_callback)
        
        # Предопределённые позы для корзин
        self.bin_poses = {
            'aluminum_can': self.create_pose(x=0.5, y=0.3, z=0.2),
            'tetra_pak': self.create_pose(x=0.5, y=-0.3, z=0.2)
        }
    
    def create_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        return pose
    
    def joint_states_callback(self, msg: JointState):
        self.joints_pose = msg.position
    
    def switch_controller(self, start_controllers: list, stop_controllers: list):
        if start_controllers[0] != self.current_controller:
            response = self.switch_controller_service(
                start_controllers=start_controllers, stop_controllers=stop_controllers,
                strictness=1, start_asap=False, timeout=0.2
            )
            
            if not response.ok:
                rospy.logwarn(f"Switch controller failed: {response}")
            self.current_controller = start_controllers[0]
    
    def go_to_using_trajectory(self, goal: np.ndarray, moving_time: float = 5):
        self.switch_controller(
            start_controllers=["eff_joint_traj_controller"],
            stop_controllers=["joint_group_eff_controller"]
        )
        
        joints_trj = JointTrajectory()
        joints_trj.header = Header()
        joints_trj.header.stamp = rospy.get_rostime()
        joints_trj.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = goal.tolist()
        point.time_from_start = rospy.Duration.from_sec(moving_time)
        joints_trj.points.append(point)
        
        self.joint_trajectory_publisher.publish(joints_trj)
        rospy.loginfo("Trajectory sent")
        rospy.sleep(moving_time)
    
    def open_gripper(self):
        gripper_trj = JointTrajectory()
        gripper_trj.header = Header()
        gripper_trj.header.stamp = rospy.get_rostime()
        gripper_trj.joint_names = ['gripper_finger1_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0]
        point.time_from_start = rospy.Duration.from_sec(1.0)
        gripper_trj.points.append(point)
        
        self.gripper_trajectory_publisher.publish(gripper_trj)
        rospy.loginfo("Open gripper")
        rospy.sleep(1)
    
    def close_gripper(self):
        gripper_trj = JointTrajectory()
        gripper_trj.header = Header()
        gripper_trj.header.stamp = rospy.get_rostime()
        gripper_trj.joint_names = ['gripper_finger1_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.2]
        point.time_from_start = rospy.Duration.from_sec(1.0)
        gripper_trj.points.append(point)
        
        self.gripper_trajectory_publisher.publish(gripper_trj)
        rospy.loginfo("Close gripper")
        rospy.sleep(1)
    
    def pick_object(self, pose_stamped):
        rospy.loginfo("Picking object")
        # Здесь должна быть логика планирования: получите joint_angles для pose_stamped.pose
        # Пример: goal = inverse_kinematics(pose_stamped.pose)
        # Здесь для примера - просто "домашняя" поза:
        goal = np.array([1.57, -1.57, 1.57, -1.57, -1.57, 0.0])
        
        self.go_to_using_trajectory(goal, moving_time=5)
        self.open_gripper()
        rospy.sleep(1)  # время на захват
        self.close_gripper()
        rospy.loginfo("Object grasped")
    
    def place_object(self, garbage_type='aluminum_can'):
        place_pose = self.bin_poses.get(garbage_type)
        if place_pose:
            rospy.loginfo("Placing object")
            # Аналогично pick: получите joint_angles для place_pose.pose
            # Пример: goal = inverse_kinematics(place_pose.pose)
            goal = np.array([1.0, -1.0, 1.0, -1.0, -1.0, 0.0])  # пример
            
            self.go_to_using_trajectory(goal, moving_time=5)
            self.open_gripper()
            rospy.sleep(1)
            rospy.loginfo("Object placed")
        else:
            rospy.logwarn("Unknown garbage type")


class Detect(smach.State):
    def __init__(self, object_detector):
        smach.State.__init__(self, outcomes=['object_found', 'no_object'])
        self.object_detector = object_detector
        self.object_pose = None
        self.pose_subscriber = rospy.Subscriber('/object_pose', PoseStamped, self.pose_callback)
    
    def pose_callback(self, msg):
        self.object_pose = msg
    
    def execute(self, userdata):
        rospy.loginfo('Detecting object...')
        # Ожидаем получения позы объекта
        start_time = rospy.Time.now()
        timeout = rospy.Duration(10.0)  # 10 секунд на поиск объекта
        
        while not rospy.is_shutdown():
            if self.object_pose:
                rospy.loginfo('Object detected!')
                return 'object_found'
            
            if (rospy.Time.now() - start_time) > timeout:
                rospy.loginfo('No object detected within timeout')
                return 'no_object'
            
            rospy.sleep(0.1)


class PickPlace(smach.State):
    def __init__(self, robot_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot_controller
        self.object_pose = None
        self.pose_subscriber = rospy.Subscriber('/object_pose', PoseStamped, self.pose_callback)
    
    def pose_callback(self, msg):
        self.object_pose = msg
    
    def execute(self, userdata):
        rospy.loginfo('Picking and placing object...')
        
        if self.object_pose:
            # Захват объекта
            self.robot.pick_object(self.object_pose)
            
            # Определяем корзину по типу объекта (замените на реальный тип)
            garbage_type = 'aluminum_can'
            self.robot.place_object(garbage_type)
        else:
            # Если нет позы объекта, выполняем демонстрационную последовательность
            home_pose = np.array([1.57, -1.57, 1.57, -1.57, -1.57, 0.0])
            pick_pose = np.array([1.0, -1.2, 1.2, -1.5, -1.2, 0.0])
            place_pose = np.array([0.7, -1.0, 1.0, -1.2, -1.0, 0.0])
            
            # Перейти в исходное положение
            self.robot.go_to_using_trajectory(home_pose, moving_time=3)
            self.robot.open_gripper()
            
            # Перейти к объекту
            self.robot.go_to_using_trajectory(pick_pose, moving_time=3)
            rospy.sleep(1)
            self.robot.close_gripper()
            
            # Перейти к месту сброса
            self.robot.go_to_using_trajectory(place_pose, moving_time=3)
            self.robot.open_gripper()
            
            # Вернуться домой
            self.robot.go_to_using_trajectory(home_pose, moving_time=3)
        
        return 'done'


def main():
    rospy.init_node('waste_sorting_system')
    
    # Инициализация компонентов
    object_detector = ObjectDetector()
    robot_controller = RobotController()
    
    # Создание и настройка конечного автомата
    sm = smach.StateMachine(outcomes=['finished', 'failed'])
    
    with sm:
        smach.StateMachine.add('DETECT', Detect(object_detector), 
                               transitions={'object_found': 'PICKPLACE',
                                           'no_object': 'failed'})
        smach.StateMachine.add('PICKPLACE', PickPlace(robot_controller), 
                               transitions={'done': 'finished'})
    
    # Запуск визуализации состояний
    sis = smach_ros.IntrospectionServer('sorting_system_server', sm, '/SM_ROOT')
    sis.start()
    
    # Запуск конечного автомата
    outcome = sm.execute()
    
    sis.stop()
    
    rospy.loginfo(f"State machine completed with outcome: {outcome}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Bool, Float32, Int32
from nav2_simple_commander.robot_navigator import BasicNavigator
import csv
from command_msgs.msg import Command
from rclpy.executors import MultiThreadedExecutor
import time
import os

from autodock_interfaces.action import AutoDocking
'''
a code that handle all topic from the ui and chatbot
'''


class UIHandleNode(Node):
    def __init__(self):
        super().__init__('ui_handle_node')
        self.position_ = []
        # self.prefix_ = "/home/tc4801/asd2_ws/src/cpp_pubsub/"  # CHANGE
        self.prefix_ = "/home/asd/gg_ws/src/cpp_pubsub/"  # CHANGE
        self.location_ = ''
        self.battery_ = 0
        self.best_error_ = float('inf')
        self.best_pose_ = None
        self.candidate_poses_ = []
        self.current_index_ = 0
        self.timer_ = None
        self.last_command_ = ''
        self.orientations_ = [
            (0.0, 1.0),       # [z, w] for (0, 0, 0, 1)
            (0.707, 0.707),   # +90 degrees
            (1.0, 0.0),       # +180 degrees
            (-0.707, 0.707)   # -90 degrees
        ]
        self.location_code_ = {'A': 1, 'B': 2, 'C': 3, 'D': 4, 'E': 5, 'F': 6}
        self.autodock_client_ = ActionClient(self, AutoDocking, 'autodocking')

        self.map_request_pub_ = self.create_publisher(
            String, '/map_request/pcd', 10)
        self.initialpose_pub_ = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.current_position_pub_ = self.create_publisher(
            String, '/current_position', 10)
        self.loading_pub_ = self.create_publisher(Bool, '/loading', 10)
        self.loading_pub_.publish(Bool(data=True))  # initialisation
        self.autodock_arrived_pub_ = self.create_publisher(
            Bool, '/autodock_arrived', 10)
        self.Chatbot_command_response_pub_ = self.create_publisher(
            Command, '/Chatbot_command_response', 10)
        self.chosen_path_pub_ = self.create_publisher(
            String, '/chosen_path', 10)
        self.ui_speech_control_pub_ = self.create_publisher(
            String, '/speech_control', 10)  # only use speechcontrol in main screen
        self.target_pub_ = self.create_publisher(String, '/target', 10)

        self.change_map_sub_ = self.create_subscription(
            String, "/change_map", self.changeMapCallback, 10)
        self.change_map_sub_
        self.odom_sub_ = self.create_subscription(
            Odometry, 'odom', self.odomCallback, 10)
        self.position_sub_ = self.create_subscription(
            String, 'position', self.positionCallback, 10)
        self.autodock_sub_ = self.create_subscription(
            Bool, '/autodocking', self.autodockCallback, 10)
        self.Chatbot_command_sub_ = self.create_subscription(
            Command, '/Chatbot_command', self.commandCallback, 10)
        self.pose_error_sub_ = self.create_subscription(
            Float32, '/match_err_msg', self.poseErrorCallback, 10)
        self.mic_control_sub_ = self.create_subscription(
            Bool, '/mic_control', self.micControlCallback, 10)
        self.task_result_sub_ = self.create_subscription(
            String, '/task_result', self.taskResultCallback, 10)
        self.battery_sub_ = self.create_subscription(
            Int32, '/mini_etransporter/battery_mode', self.batteryCallback, 10)

    def findBestPose(self, x: float, y: float):
        """Iterate through candidate poses to find the best initial pose."""
        self.candidate_poses_ = list()
        self.current_index_ = 0
        self.best_error_ = float('inf')
        self.best_pose_ = None
        for z, w in self.orientations_:
            poseMsg = PoseWithCovarianceStamped()
            poseMsg.header.frame_id = "map"  # Reference frame
            poseMsg.pose.pose.position.x = x  # Example position
            poseMsg.pose.pose.position.y = y
            poseMsg.pose.pose.orientation.z = z
            poseMsg.pose.pose.orientation.w = w
            self.candidate_poses_.append(poseMsg)

        self.get_logger().info("Starting pose estimation...")

        self.timer_ = self.create_timer(1.0, self.evaluatePose)

    def evaluatePose(self):
        """Publish a pose and wait for error update asynchronously."""
        if self.current_index_ >= len(self.candidate_poses_):
            self.timer_.cancel()  # Stop the timer when all poses are tested

            if self.best_pose_:
                self.initialpose_pub_.publish(self.best_pose_)
                self.get_logger().info(
                    f"Final best pose published with error: {self.best_error_}")
                self.current_index_ = 0
            else:
                self.get_logger().warn("No valid pose found!")

            return

        # Publish next candidate pose
        pose = self.candidate_poses_[self.current_index_]
        self.initialpose_pub_.publish(pose)
        self.get_logger().info(
            f"Published initial pose {self.current_index_}: (z: {pose.pose.pose.orientation.z}, w: {pose.pose.pose.orientation.w})")

        # Allow some time for the localization system to update before checking error
        self.current_index_ += 1

    def batteryCallback(self, msg: Int32):
        self.battery_ = msg.data

    def taskResultCallback(self, msg: String):
        if 'SUCCEEDED' in msg.data and 'navigation' in self.last_command_:
            response = Command()
            response.command = 'arrive'
            response.value = 0
            self.Chatbot_command_response_pub_.publish(response)

    def micControlCallback(self, msg: Bool):
        if msg.data:
            os.system("amixer -c 1 set Mic cap")
            print('qeqwafwe')
        elif not msg.data:
            os.system("amixer -c 1 set Mic nocap")
            print('qddddddddddddd')

    def poseErrorCallback(self, msg: Float32):
        # Check if the current error is better than the previous best error
        if self.current_index_ > 0 and msg.data < self.best_error_:
            self.best_error_ = msg.data
            self.best_pose_ = self.candidate_poses_[self.current_index_ - 1]
            self.get_logger().info(
                f"New best pose found with error: {self.best_error_}")

    def commandCallback(self, msg: Command):
        response = Command()
        response.command = msg.command
        speechControl = String()
        self.last_command_ = msg.command
        print(f"Received command '{msg.command}'")
        if 'navigation' in msg.command:
            response.value = 1
            path = ''
            for location in self.location_code_:
                if self.location_code_[location] == msg.value:
                    path = self.location_+location
                    self.target_pub_.publish(String(data=location))
                    # to make sure data is published due to lack of processing power of pc
                    # time.sleep(1.0)
                    # self.target_pub_.publish(String(data=location))
            time.sleep(1.0)
            if self.location_:
                response.value = 0
            self.chosen_path_pub_.publish(String(data=path))
            speechControl.data = 'navigation'

        elif 'location' in msg.command:
            response.value = -1
            for location in self.location_code_:
                if location == self.location_:
                    response.value = self.location_code_[location]
        elif 'followme_disable' in msg.command:
            # TODO
            response.value = 0
            speechControl.data = 'followme_disable'
        elif 'followme' in msg.command:
            # TODO
            response.value = 0
            speechControl.data = 'followme'
        elif 'remainingpower' in msg.command:
            response.value = self.battery_
            speechControl.data = 'remainingpower'
        elif 'powercharging' in msg.command:
            response.value = 0
            speechControl.data = 'powercharging'
            path = self.location_+'X'
            self.chosen_path_pub_.publish(String(data=path))
        else:
            print('Invalid command')
            return

        self.Chatbot_command_response_pub_.publish(response)
        self.ui_speech_control_pub_.publish(speechControl)
        print(
            f"Responded to chatbot command '{response.command}' with value {response.value}")

    def autodockCallback(self, msg: Bool):
        if msg.data:
            print('Received autodocking singal')
            time.sleep(5.0)
            self.startAutodock()

    def startAutodock(self):
        goalMsg = AutoDocking.Goal()
        self.autodock_client_.wait_for_server()
        self.sendGoalFuture = self.autodock_client_.send_goal_async(
            goal=goalMsg, feedback_callback=self.feedbackCallback)

    def feedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(feedback.state)
        if feedback.state == 1:
            print('Autodock arrived!!!')
            self.autodock_arrived_pub_.publish(Bool(data=True))
        elif "aborted" in feedback.status:
            print("Autodock aborted, cannot trace the marker")
            self.autodock_arrived_pub_.publish(Bool(data=False))

    def positionCallback(self, msg: String):
        if msg.data == "":
            print("position initiliased")
            return
        print(f"position: {msg.data}")

        for posed in self.position_:
            if msg.data == posed[0]:
                self.findBestPose(x=posed[1], y=posed[2])
                break

        # pose = PoseWithCovarianceStamped()
        # pose.header.stamp = BasicNavigator().get_clock().now().to_msg()
        # pose.header.frame_id = "map"
        # for posed in self.position_:
        #     if msg.data == posed[0]:
        #         pose.pose.pose.position.x = posed[1]
        #         pose.pose.pose.position.y = posed[2]
        #         pose.pose.pose.orientation.z = posed[3]
        #         pose.pose.pose.orientation.w = posed[4]
        #         self.initialpose_pub_.publish(pose)
        #         break

    def odomCallback(self, msg: Odometry):
        self.location_ = ""
        for pose in self.position_:
            if ((msg.pose.pose.position.x-1 <= pose[1] <= msg.pose.pose.position.x+1) and (msg.pose.pose.position.y-1 <= pose[2] <= msg.pose.pose.position.y+1)):
                self.location_ = pose[0]

        self.current_position_pub_.publish(String(data=self.location_))

    def changeMapCallback(self, msg: String):
        if msg.data == "":
            print("position initiliased")
            return
        print(f"Received msg: {msg.data}")
        filepath = self.prefix_+"maps/"+msg.data+".yaml"
        print(filepath)

        self.readcsv(msg.data)
        BasicNavigator().changeMap(map_filepath=filepath)

        # pcdprefix = "/home/tc4801/catkin_ws/src/hdl_localization/data/"  # CHANGE
        pcdprefix = "/home/asd/gg_ws/src/cpp_pubsub/maps/map_pcd/"  # CHANGE
        prefixfilepath = pcdprefix+msg.data+".pcd"
        self.map_request_pub_.publish(String(data=prefixfilepath))

    def readcsv(self, mapname: str):
        # filepath = "/home/tc4801/asd2_ws/src/cpp_pubsub/position/"+mapname+".csv"  # CHANGE
        filepath = "/home/asd/gg_ws/src/cpp_pubsub/position/"+mapname+".csv"  # CHANGE
        self.position_ = []  # reset
        try:
            with open(filepath, mode="r")as f:
                reader = csv.reader(f, delimiter=",")
                for row in reader:
                    pose = row[0]
                    x = float(row[1])
                    y = float(row[2])
                    z = float(row[3])
                    w = float(row[4])
                    self.position_.append((pose, x, y, z, w))
            print(self.position_)
        except:
            print('asd')


def ouob(args=None):
    rclpy.init(args=args)
    node = UIHandleNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    ouob()

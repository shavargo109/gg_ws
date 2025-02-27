#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import math
from enum import Enum
import numpy as np
import tf_transformations
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool, Int16
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from scipy.spatial.distance import euclidean
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor

'''
    read path from csv, compute path using nav2 service
    and send as NavThroughPoses or FollowPath method
'''


class FixedPathState(Enum):
    IDLE = 0
    PROCESSING = 1
    ERROR = 2
    SKIP = 3


class PathCsvNode(Node):
    def __init__(self):
        super().__init__("path_from_csv_node")
        self.declare_parameter(
            "prefix", "/home/asd/gg_ws/src/cpp_pubsub/")  # CHANGE
        # change to path prefix name only
        self.declare_parameter("save_path_filename", "path/map_")
        self.declare_parameter("isNavThroughPoses", False)
        self.declare_parameter("loggingPrefix", "distance_log/")
        self.prefix_ = self.get_parameter("prefix")
        self.save_path_filename = self.get_parameter("save_path_filename")
        self.isNavThroughPoses = self.get_parameter("isNavThroughPoses")
        self.loggingPrefix = self.get_parameter("loggingPrefix")

        self.behavior_tree = "/home/asd/gg_ws/src/cpp_pubsub/bt_config/custom_nav_through_poses.xml"
        self.coordinates = []
        self.afterCoordinates = []
        self.angle_ = []
        self.odom = Odometry()
        self.objflag_ = bool()
        self.mode_ = int()
        self.range_ = 2.
        self.currentAngle_ = float()
        self.isRecievedOdom = bool(False)
        self.startTime_ = datetime.now()
        self.state_ = FixedPathState.IDLE
        self.mapName_ = ""

        self.path_feedback_pub_ = self.create_publisher(
            String, "task_result", 10)
        self.goal_pub_ = self.create_publisher(PoseStamped, "fixed_pose", 10)
        self.buzzer_pub_ = self.create_publisher(Bool, "/buzzer_control", 10)
        self.initialpose_pub_ = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.odom_sub_ = self.create_subscription(
            Odometry, "odom", self.odomCallback, 10)
        self.go_sub_ = self.create_subscription(
            Bool, "fixed_path_signal", self.fixedPathSignalCallback, 10)
        self.objflag_sub_ = self.create_subscription(
            Bool, "/object_detected", self.objflagCallback, 10)
        self.mode_sub_ = self.create_subscription(
            Int16, "/mode", self.modeCallback, 10)
        self.angle_sub_ = self.create_subscription(
            Twist, "cmd_vel_tc", self.angleCallback, 10)
        self.choose_path_sub_ = self.create_subscription(
            String, "/chosen_path", self.chosenPathCallback, 10)
        self.change_map_sub_ = self.create_subscription(
            String, "/change_map", self.changeMapCallback, 10)

    def changeMapCallback(self, msg: String):
        self.mapName_ = msg.data

    def angleCallback(self, msg: Twist):
        self.currentAngle_ = msg.angular.z

    def objflagCallback(self, msg: Bool):
        self.objflag_ = msg.data
        print(f'Object detected: {self.objflag_}')

    def modeCallback(self, msg: Int16):
        self.mode_ = msg.data

    def odomCallback(self, msg: Odometry):
        self.odom = msg
        self.isRecievedOdom = True

    def fixedPathSignalCallback(self, data: Bool):
        if self.state_ == FixedPathState.IDLE:
            self.sendPath("123")
        else:
            self.get_logger().info(
                "The robot is still in task, please wait for the navigation finished")

    def chosenPathCallback(self, msg: String):
        print("QQQQQQQQQQQQQQQQQQQQQq")
        if msg.data == "":  # initialisation
            print("initialised")
            return
        if self.state_ == FixedPathState.IDLE:
            self.sendPath(msg.data)
        else:
            self.get_logger().info(
                "The robot is still in task, please wait for the navigation finished")

    def calDistancePath(self, msg: Path):
        pathLength = 0.
        for i in range(len(msg.poses) - 1):
            position_a_x = msg.poses[i].pose.position.x
            position_b_x = msg.poses[i+1].pose.position.x
            position_a_y = msg.poses[i].pose.position.y
            position_b_y = msg.poses[i+1].pose.position.y
            pathLength += np.sqrt(np.power((position_b_x - position_a_x),
                                  2) + np.power((position_b_y - position_a_y), 2))
        print("path length: ", pathLength)
        self.recordDistance(pathLength)

    def recordDistance(self, distance: float):
        filepath = self.prefix_.value + self.loggingPrefix.value + \
            datetime.now().strftime("%d-%m-%Y")+".csv"
        with open(filepath, mode='a') as f:
            writer = csv.writer(
                f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow([self.startTime_.strftime(
                "%d-%m-%Y, %H:%M:%S"), datetime.now().strftime(
                "%d-%m-%Y, %H:%M:%S"), distance])

    def _shortest_list(self):
        check = []
        index = int()
        for coordinate in self.coordinates:
            if (self.odom.pose.pose.position.x-self.range_ <= coordinate[0] <= self.odom.pose.pose.position.x+self.range_ and
                    self.odom.pose.pose.position.y-self.range_ <= coordinate[1] <= self.odom.pose.pose.position.y+self.range_):
                check.append(coordinate)

        asd = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
        if len(check) == 0:
            print("no overlap point")
            # when there is no overlap point (within 4*4), then issue toofar and stop navigation
            if not self.isNavThroughPoses.value:
                return False
            distances = [euclidean(asd, point) for point in self.coordinates]
            min_index = np.argmin(distances)
            index = min_index
        else:
            print("points in area, len: " + str(len(check)))
            distances = [euclidean(asd, point) for point in check]
            min_index = np.argmin(distances)  # min_index of check[]
            index = self.coordinates.index(check[min_index])
        self.afterCoordinates = self.coordinates[index:]
        print("first point of the path: ", self.afterCoordinates[0])

    def shortestPoint(self):
        if self._shortest_list() == False:
            anglepath = []
            anglepath.append(self.afterCoordinates[0])
            anglepath.append(self.afterCoordinates[3])
            return anglepath
        return False

    def shortestPath(self):
        if self._shortest_list() == False:
            self.get_logger().info(
                f"Robot's current position is too far from the path! (>{self.range_}m), path will not be published!")
            return False
        if not self.isNavThroughPoses.value:
            # need to cal. current position to path if it is too far for Followpath()
            print("==========================")
            print("Returning path to FollowPath")
            na = BasicNavigator()
            start = PoseStamped()
            start.header.frame_id = "map"
            start.header.stamp = na.get_clock().now().to_msg()
            start.pose.position.x = self.odom.pose.pose.position.x
            start.pose.position.y = self.odom.pose.pose.position.y
            start.pose.orientation.w = 1.
            start.pose.orientation.z = 0.

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = na.get_clock().now().to_msg()
            goal.pose.position.x = self.afterCoordinates[0][0]
            goal.pose.position.y = self.afterCoordinates[0][1]
            goal.pose.orientation.w = 1.
            goal.pose.orientation.z = 0.
            path = na.getPath(start=start, goal=goal)
            path: Path
            return path

    def angleAdjust(self):
        distance = self.shortestPoint()
        if distance != False:
            yaw = math.atan2(distance[1][1]-distance[0][1],
                             distance[1][0]-distance[0][0])
            delta = self.currentAngle_-yaw
            if abs(delta) < abs(0.25*math.pi):
                print("Angle is too small, no need to self-rotate")
                return
            q = tf_transformations.quaternion_from_euler(0., 0., yaw)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = BasicNavigator().get_clock().now().to_msg()
            pose.pose.position.x = self.odom.pose.pose.position.x
            pose.pose.position.y = self.odom.pose.pose.position.y
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            self.get_logger().info("FollowPath started, adjusting initial orientation")
            self.goal_pub_.publish(pose)

    def readPath(self, chosenPath: str):
        if chosenPath == "123":
            filepath = self.prefix_.value + self.save_path_filename.value
        else:
            filepath = self.prefix_.value + "path/" + \
                self.mapName_ + "_" + chosenPath + ".csv"
        originCoordinates = []
        print(filepath)
        try:
            with open(filepath, mode="r") as f:
                reader = csv.reader(f, delimiter=",")
                for row in reader:
                    # Skip the header row
                    if row == ["x", "y"]:
                        continue
                    x, y = map(float, row)
                    originCoordinates.append((x, y))
        except:
            print("No such file exist!!!!")
            self.state_ = FixedPathState.ERROR
            self.coordinates = []  # reset to not save previous path
            return
        if len(originCoordinates) == 0:
            self.state_ = FixedPathState.SKIP
            self.coordinates = []  # reset to not save previous path
            return

        self.angle_ = originCoordinates[-1]
        print(self.angle_)
        originCoordinates.pop()  # remove angle data
        self.coordinates = originCoordinates[::2]
        # # must include last point as goal
        # self.coordinates.append(coordinates[-1])

    def sendPath(self, chosenPath: str):
        if not self.isRecievedOdom:
            print("Odom is not initialised, please try again...")
            return

        self.readPath(chosenPath)
        if self.state_ == FixedPathState.ERROR:
            print("Path does not exist")
            self.path_feedback_pub_.publish(String(data="INVALID"))
            self.state_ = FixedPathState.IDLE
            return
        elif self.state_ == FixedPathState.SKIP:
            print("Recharge action")
            self.path_feedback_pub_.publish(String(data="SUCCEEDED"))
            self.state_ = FixedPathState.IDLE
            return

        print("Received current odom, start following path...")
        navigator = BasicNavigator()
        path_to_path = self.shortestPath()
        if not path_to_path:
            return

        self.state_ = FixedPathState.PROCESSING

        goal_poses = []
        for i in range(len(self.afterCoordinates)-1):
            goal_pose = PoseStamped()
            if i == (len(self.afterCoordinates)-2):
                goal_pose.header.frame_id = "map"
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = self.afterCoordinates[i][0]
                goal_pose.pose.position.y = self.afterCoordinates[i][1]
                goal_pose.pose.orientation.z = self.angle_[0]
                goal_pose.pose.orientation.w = self.angle_[1]
                self.goal_pub_.publish(goal_pose)
            else:
                goal_pose.header.frame_id = "map"
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = self.afterCoordinates[i][0]
                goal_pose.pose.position.y = self.afterCoordinates[i][1]
                goal_pose.pose.orientation.z = 0.
                goal_pose.pose.orientation.w = 1.
            goal_poses.append(goal_pose)

        navigator.clearGlobalCostmap()

        # self.angleAdjust()  # maybe need some feedback to know it is ended or not
        self.startTime_ = datetime.now()
        if self.isNavThroughPoses.value:
            self.get_logger().info("Using NavThroughPoses")
            navigator.goThroughPoses(
                goal_poses, behavior_tree=self.behavior_tree)
        else:
            self.get_logger().info("Using FollowPath")
            # self.angleAdjust()
            path_to_path.poses.extend(goal_poses)
            navigator.followPath(path_to_path, controller_id="FollowPath")
            goal_poses = path_to_path.poses  # update poses

        self.buzzer_pub_.publish(Bool(data=True))

        # publish end goal for adjusting orientation

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()

        result = navigator.getResult()
        task_result = String()

        if not self.isNavThroughPoses.value:
            # only handle task failed due to obstacle detected or controller in idle mode
            # while result == TaskResult.FAILED and (self.objflag_ or self.mode_ == 0):
            while result == TaskResult.FAILED:
                print(self.objflag_)
                self.get_logger().info("FollowPath failed, sending path again...")
                path = Path()
                path.header.frame_id = "map"
                path.header.stamp = navigator.get_clock().now().to_msg()
                path.poses = goal_poses
                navigator.followPath(path, controller_id="FollowPath")
                while not navigator.isTaskComplete():
                    # Do something with the feedback
                    feedback = navigator.getFeedback()
                result = navigator.getResult()
                print(result)
                self.get_logger().info("Clearing local costmap due to failed in FollowPath")
                navigator.clearLocalCostmap()

        if result == TaskResult.SUCCEEDED:
            task_result.data = "SUCCEEDED"
            self.calDistancePath(path_to_path)
        elif result == TaskResult.CANCELED:
            task_result.data = "CANCELED"
        elif result == TaskResult.FAILED:
            task_result.data = "FAILED"
        else:
            task_result.data = "INVALID"

        self.path_feedback_pub_.publish(task_result)
        self.buzzer_pub_.publish(Bool(data=False))
        self.get_logger().info(f"Task result: {task_result.data}")
        self.state_ = FixedPathState.IDLE


def ouob(args=None):
    rclpy.init(args=args)
    node = PathCsvNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

# def ouob(args=None):
#     rclpy.init(args=args)
#     node = PathCsvNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


if __name__ == "__main__":
    ouob()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Int8
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from scipy.spatial.distance import euclidean

'''
    read path from csv, compute path using nav2 service
    and send as NavThroughPoses or FollowPath method
'''


class PathCsvNode(Node):
    def __init__(self):
        super().__init__("path_from_csv_node")
        self.declare_parameter("save_path_filename",
                               "/home/asd/gg_ws/src/cpp_pubsub/path/map3.csv")
        self.declare_parameter("isNavThroughPoses", False)
        self.save_path_filename = self.get_parameter("save_path_filename")
        self.isNavThroughPoses = self.get_parameter("isNavThroughPoses")
        self.behavior_tree = "/home/asd/gg_ws/src/cpp_pubsub/bt_config/custom_nav_through_poses.xml"
        self.coordinates = []
        self.afterCoordinates = []
        self.odom = Odometry()
        self.objflag_ = bool()
        self.mode_ = int()
        self.isRecievedOdom = bool(False)
        self.path_feedback_pub_ = self.create_publisher(
            String, "task_result", 10)

        self.odom_sub_ = self.create_subscription(
            Odometry, "odom", self.odomCallback, 10)
        self.odom_sub_
        self.go_sub_ = self.create_subscription(
            Bool, "fixed_path_signal", self.fixedPathSignalCallback, 10)
        self.go_sub_
        self.objflag_sub_ = self.create_subscription(
            Bool, "/object_detected", self.objflagCallback, 10)
        self.mode_sub_ = self.create_subscription(
            Int8, "/mode", self.modeCallback, 10)

    def objflagCallback(self, msg: Bool):
        self.objflag_ = msg.data

    def modeCallback(self, msg: Int8):
        self.mode_ = msg.data

    def odomCallback(self, data: Odometry):
        self.odom = data
        if not self.isRecievedOdom:
            self.isRecievedOdom = True

    def fixedPathSignalCallback(self, data: Bool):
        self.sendPath(123)

    def shortestPath(self):
        check = []
        index = int()
        range = 2.
        for coordinate in self.coordinates:
            if (self.odom.pose.pose.position.x-range <= coordinate[0] <= self.odom.pose.pose.position.x+range and
                    self.odom.pose.pose.position.y-range <= coordinate[1] <= self.odom.pose.pose.position.y+range):
                check.append(coordinate)

        asd = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
        asd2 = asd
        if len(check) == 0:
            print("no overlap point")
            # when there is no overlap point (within 4*4), then issue toofar and stop navigation
            if not self.isNavThroughPoses.value:
                self.get_logger().info(
                    f"Robot's current position is {range}m away from path, path will not be published!")
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
        print("first point of the path: ", self.coordinates[index])
        # print(asd)
        if not self.isNavThroughPoses.value:
            # need to cal. current position to path if it is too far for Followpath()
            print("==========================")
            print("Returning path to FollowPath")
            na = BasicNavigator()
            start = PoseStamped()
            start.header.frame_id = "map"
            start.header.stamp = na.get_clock().now().to_msg()
            start.pose.position.x = asd2[0]
            start.pose.position.y = asd2[1]
            start.pose.orientation.w = 1.
            start.pose.orientation.z = 0.

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = na.get_clock().now().to_msg()
            goal.pose.position.x = self.coordinates[index][0]
            goal.pose.position.y = self.coordinates[index][1]
            goal.pose.orientation.w = 1.
            goal.pose.orientation.z = 0.
            path = na.getPath(start=start, goal=goal)
            path: Path
            return path

    def readPath(self):
        global coordinates
        coordinates = []
        filepath = self.save_path_filename.value
        with open(filepath, mode="r") as f:
            reader = csv.reader(f, delimiter=",")
            for row in reader:
                # Skip the header row
                if row == ["x", "y"]:
                    continue
                x, y = map(float, row)
                coordinates.append((x, y))
        self.coordinates = coordinates[::2]
        # must include last point as goal
        self.coordinates.append(coordinates[-1])

    def sendPath(self, event):
        if not self.isRecievedOdom:
            print("Odom is not initialised, please try again...")
            return

        print("Received current odom, start following path...")

        self.readPath()
        navigator = BasicNavigator()
        path_to_path = self.shortestPath()
        if not path_to_path:
            return

        goal_poses = []
        for coordinate in self.afterCoordinates:
            coordinate: list
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = coordinate[0]
            goal_pose.pose.position.y = coordinate[1]
            goal_pose.pose.orientation.z = 0.
            goal_pose.pose.orientation.w = 1.
            goal_poses.append(goal_pose)

        if self.isNavThroughPoses.value:
            self.get_logger().info("Using NavThroughPoses")
            navigator.goThroughPoses(
                goal_poses, behavior_tree=self.behavior_tree)
        else:
            self.get_logger().info("Using FollowPath")
            path_to_path.poses.extend(goal_poses)
            navigator.followPath(path_to_path, controller_id="FollowPath")
            goal_poses = path_to_path.poses  # update poses

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()

        result = navigator.getResult()
        task_result = String()

        if not self.isNavThroughPoses.value:
            # only handle task failed due to obstacle detected or controller in idle mode
            while result == TaskResult.FAILED and (self.objflag_ or self.mode_ == 0):
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
                # navigator.wait(duration=1)
                # while not navigator.isTaskComplete():
                #     pass
                navigator.clearLocalCostmap()
                # navigator.clearGlobalCostmap()

        if result == TaskResult.SUCCEEDED:
            task_result.data = "SUCCEEDED"
        elif result == TaskResult.CANCELED:
            task_result.data = "CANCELED"
        elif result == TaskResult.FAILED:
            task_result.data = "FAILED"
        else:
            task_result.data = "INVALID"
        self.path_feedback_pub_.publish(task_result)
        self.get_logger().info(f"Task result: {task_result.data}")


def main(args=None):
    rclpy.init(args=args)
    node = PathCsvNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

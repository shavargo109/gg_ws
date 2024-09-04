#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from scipy.spatial.distance import euclidean
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

'''
    read path from csv, compute path using nav2 service
    and send as NavThroughPoses method
'''


class PathCsvNode(Node):
    def __init__(self):
        super().__init__('path_from_csv_node')
        self.declare_parameter('save_path_filename',
                               '/home/asd/gg_ws/src/cpp_pubsub/path/map3.csv')
        self.declare_parameter('isNavThroughPoses', False)
        self.save_path_filename = self.get_parameter('save_path_filename')
        self.isNavThroughPoses = self.get_parameter('isNavThroughPoses')
        self.behavior_tree = '/home/asd/gg_ws/src/cpp_pubsub/bt_config/custom_nav_through_poses.xml'
        self.coordinates = []
        self.odom = Odometry()

        self.isRecievedOdom = False
        self.path_feedback_pub_ = self.create_publisher(
            String, 'task_result', 10)

        self.odom_sub_ = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.go_sub_ = self.create_subscription(
            Bool, 'fixed_path_signal', self.fixed_path_signal_callback, 10)

    def odom_callback(self, data: Odometry):
        self.odom = data
        self.isRecievedOdom = True

    def fixed_path_signal_callback(self, data: Bool):
        self.send_path(123)

    def shortest_path(self):
        check = []
        index = int()
        isTooFar = bool(False)
        for coordinate in self.coordinates:
            if (self.odom.pose.pose.position.x-1. <= coordinate[0] <= self.odom.pose.pose.position.x+1. and
                    self.odom.pose.pose.position.y-1. <= coordinate[1] <= self.odom.pose.pose.position.y+1.):
                check.append(coordinate)

        asd = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
        asd2 = asd
        if len(check) == 0:
            print('no overlap point')
            distances = [euclidean(asd, point) for point in self.coordinates]
            min_index = np.argmin(distances)
            index = min_index
            isTooFar = True
        else:
            print('points in area, len: ' + str(len(check)))
            distances = [euclidean(asd, point) for point in check]
            min_index = np.argmin(distances)  # min_index of check[]
            index = self.coordinates.index(check[min_index])
        print(self.coordinates[index])
        print(asd)
        if not self.isNavThroughPoses.value and isTooFar:
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
            goal.pose.position.x = self.coordinates[0][0]
            goal.pose.position.y = self.coordinates[0][1]
            goal.pose.orientation.w = 1.
            goal.pose.orientation.z = 0.
            path = na.getPath(start=start, goal=goal)
            return path
        asd = self.coordinates[index:]
        # return asd[::2]
        return asd

    def read_path(self):
        global coordinates
        coordinates = []
        filepath = self.save_path_filename.value
        with open(filepath, mode='r') as f:
            reader = csv.reader(f, delimiter=',')
            for row in reader:
                # Skip the header row
                if row == ['x', 'y']:
                    continue
                x, y = map(float, row)
                coordinates.append((x, y))
        self.coordinates = coordinates[::2]
        # must include last point as goal
        self.coordinates.append(coordinates[-1])

    def send_path(self, event):
        if self.isRecievedOdom:
            self.get_logger().info("Received current odom, start following path...")

            self.read_path()
            navigator = BasicNavigator()
            # navigator.waitUntilNav2Active()
            renew_coordinates = self.shortest_path()
            if self.isNavThroughPoses.value:
                goal_poses = []
                for coordinate in renew_coordinates:
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                    goal_pose.pose.position.x = coordinate[0]
                    goal_pose.pose.position.y = coordinate[1]
                    goal_pose.pose.orientation.w = 1.
                    goal_pose.pose.orientation.z = 0.
                    goal_poses.append(goal_pose)
                self.get_logger().info('Using NavThroughPoses')
                navigator.goThroughPoses(
                    goal_poses, behavior_tree=self.behavior_tree)
            else:
                path = Path()
                path.header.stamp = self.get_clock().now().to_msg()
                path.header.frame_id = "map"
                for coordinate in self.coordinates:
                    pose = PoseStamped()
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = coordinate[0]
                    pose.pose.position.y = coordinate[1]
                    pose.pose.position.z = 0.
                    pose.pose.orientation.x = 0.
                    pose.pose.orientation.y = 0.
                    pose.pose.orientation.z = 0.
                    pose.pose.orientation.w = 1.
                    if (isinstance(renew_coordinates, Path)):
                        renew_coordinates.poses.append(pose)
                    else:
                        path.poses.append(pose)
                self.get_logger().info("Using FollowPath")
                if (isinstance(renew_coordinates, Path)):
                    navigator.followPath(
                        renew_coordinates, controller_id="FollowPath")
                else:
                    navigator.followPath(path, controller_id="FollowPath")

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()

            result = navigator.getResult()
            task_result = String()

            if not self.isNavThroughPoses.value:
                while result == TaskResult.FAILED:
                    self.get_logger().info("FollowPath aborted, sending path again...")
                    navigator.followPath(path, controller_id="FollowPath")
                    while not navigator.isTaskComplete():
                        # Do something with the feedback
                        feedback = navigator.getFeedback()
                    self.get_logger().info("Entering recovery 'wait', and clearing local costmap")
                    result = navigator.getResult()
                    navigator.wait(duration=1)
                    while not navigator.isTaskComplete():
                        pass
                    navigator.clearLocalCostmap()

            if result == TaskResult.SUCCEEDED:
                task_result.data = 'SUCCEEDED'
            elif result == TaskResult.CANCELED:
                task_result.data = 'CANCELED'
            elif result == TaskResult.FAILED:
                task_result.data = 'FAILED'
            else:
                task_result.data = 'INVALID'
            self.path_feedback_pub_.publish(task_result)
            self.get_logger().info(f'Task result: {task_result.data}')
        else:
            print("Odom is not initialised, please try again...")


def main(args=None):
    rclpy.init(args=args)
    node = PathCsvNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

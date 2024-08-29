#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

'''
    sub to 'map_path' topic and send path to nav2 for follow path, 
    and pub 'task_result' as task feedback
'''


class SubPathNode(Node):
    def __init__(self):
        super().__init__('sub_path_node')

        self.path_feedback_pub_ = self.create_publisher(
            String, 'task_result', 10)

        self.map_path_sub_ = self.create_subscription(
            Path, 'map_path', self.map_path_callback, 10)
        self.map_path_sub_

    def map_path_callback(self, data: Path):
        navigator = BasicNavigator()

        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = -0.5
        # initial_pose.pose.position.y = -2.
        # initial_pose.pose.orientation.z = 1.0
        # initial_pose.pose.orientation.w = 0.0

        navigator.waitUntilNav2Active()

        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = 'map'
        # goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        # goal_pose.pose.position.x = 2.
        # goal_pose.pose.position.y = 0.5
        # goal_pose.pose.orientation.w = 1.0
        # path = navigator.getPath(initial_pose, goal_pose)

        data.header.stamp = navigator.get_clock().now().to_msg()
        for pose in data.poses:
            pose.header.stamp = navigator.get_clock().now().to_msg()

        # navigator.followPath(path)
        navigator.followPath(data)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated distance remaining to goal position: ' +
                      '{0:.3f}'.format(feedback.distance_to_goal) +
                      '\nCurrent speed of the robot: ' +
                      '{0:.3f}'.format(feedback.speed))

        result = navigator.getResult()
        task_result = String()
        if result == TaskResult.SUCCEEDED:
            task_result.data = 'SUCCEEDED'
        elif result == TaskResult.CANCELED:
            task_result.data = 'CANCELED'
        elif result == TaskResult.FAILED:
            task_result.data = 'FAILED'
        else:
            task_result.data = 'INVALID'
        self.path_feedback_pub_.publish(task_result)
        # navigator.lifecycleShutdown() #shutdown whole nav2, fking hell


def ouob(args=None):
    rclpy.init(args=args)
    node = SubPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    ouob()

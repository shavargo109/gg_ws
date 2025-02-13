#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String, Bool


class VelocitySmootherNode(Node):
    def __init__(self):
        super().__init__('vel_smoother_node')

        self.target_velocity_ = Twist()
        self.backward_velocity_ = Twist()
        self.mode_ = int()
        self.obj_flag_ = bool()
        self.bkwd_flag_ = bool()
        self.smooth_factor_ = 0.05  # Adjust smoothing factor as needed

        self.current_velocity_ = Twist()
        self.current_velocity_.linear.x = 0.
        self.current_velocity_.linear.y = 0.
        self.current_velocity_.linear.z = 0.
        self.current_velocity_.angular.x = 0.
        self.current_velocity_.angular.y = 0.
        self.current_velocity_.angular.z = 0.

        self.stop_cmd_vel_ = Twist()
        self.stop_cmd_vel_.linear.x = 0.
        self.stop_cmd_vel_.linear.y = 0.
        self.stop_cmd_vel_.linear.z = 0.
        self.stop_cmd_vel_.angular.x = 0.
        self.stop_cmd_vel_.angular.y = 0.
        self.stop_cmd_vel_.angular.z = 0.

        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.smooth_velocity)

        self.result_sub_ = self.create_subscription(
            String, 'task_result', self.result_callback, 10)
        self.result_sub_
        self.raw_cmd_vel_sub_ = self.create_subscription(
            Twist, 'cmd_vel_raw', self.raw_cmd_vel_callback, 10)
        self.raw_cmd_vel_sub_

        self.cmd_vel_object_sub_ = self.create_subscription(
            Bool, 'object_detected', self.cmd_vel_object_callback, 10)
        self.cmd_vel_object_sub_

        self.bkwd_sub_ = self.create_subscription(
            Bool, 'need_backward', self.bkwd_callback, 10)
        self.bkwd_sub_

        self.mode_sub_ = self.create_subscription(
            Int16, 'mode', self.mode_callback, 10)
        self.mode_sub_

        self.cmd_vel_backward_sub_ = self.create_subscription(
            Twist, 'cmd_vel_backward', self.cmd_vel_backward_callback, 10)
        self.cmd_vel_backward_sub_

        self.cmd_vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def result_callback(self, data: String):
        if ('SUCCEEDED' in data.data):
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.
            cmd_vel.linear.y = 0.
            cmd_vel.linear.z = 0.
            cmd_vel.angular.x = 0.
            cmd_vel.angular.y = 0.
            cmd_vel.angular.z = 0.
            print("Goal reached, sending stop command!")
            self.cmd_vel_pub_.publish(cmd_vel)

    def cmd_vel_backward_callback(self, data: Twist):
        self.backward_velocity_ = data

    def raw_cmd_vel_callback(self, data: Twist):
        self.target_velocity_ = data
        # self.cmd_vel_pub_.publish(data)

    def cmd_vel_object_callback(self, data: Bool):
        self.obj_flag_ = data.data

    def bkwd_callback(self, data: Bool):
        self.bkwd_flag_ = data.data

    def mode_callback(self, data: Int16):
        self.mode_ = data.data

    def smooth_velocity(self):
        # if (self.mode_ == 2 and not self.obj_flag_):
        if self.bkwd_flag_:
            self.cmd_vel_pub_.publish(self.backward_velocity_)
            print("Received bkwd vel from costmap detect, First priority!")
            return
        elif (not self.obj_flag_):
            # print('xd')
            if (self.target_velocity_.linear.x > 0.5):
                # cap the vel at 0.5
                self.target_velocity_.linear.x = 0.5

            delta_vel = self.target_velocity_.linear.x - self.current_velocity_.linear.x
            if (0 < delta_vel <= 0.05):
                # difference is small enough that no need smoothing
                self.current_velocity_ = self.target_velocity_
            elif (delta_vel > 0):
                # need smoothing
                self.current_velocity_.linear.x += self.smooth_factor_ * delta_vel
                self.current_velocity_.angular.z = self.target_velocity_.angular.z
            # elif (self.target_velocity_.linear.x < 0):
            #     # reached goal
            #     self.current_velocity_.linear.x = 0.
            #     self.current_velocity_.angular.z = 0.
            else:
                # decel or constant case
                self.current_velocity_ = self.target_velocity_
        else:
            if (self.obj_flag_):
                print("Obstacled detected, sending stop command!!")
            else:
                print("not in automode!")

            self.current_velocity_.linear.x = 0.
            self.current_velocity_.angular.z = 0.

        self.cmd_vel_pub_.publish(self.current_velocity_)


def ouob(args=None):
    rclpy.init(args=args)
    node = VelocitySmootherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    ouob()

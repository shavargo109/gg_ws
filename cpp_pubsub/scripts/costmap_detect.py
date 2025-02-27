#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
import numpy as np
import math
from nav2_simple_commander.robot_navigator import BasicNavigator

'''
    received local costmap and global path, detect obstacles on path
    and send stop signal to navigator
'''

class ObstacleExtractor(Node):
    def __init__(self):
        super().__init__("obstacle_extractor")

        self.costmap_ = OccupancyGrid()
        self.path_ = Path()
        self.odom_ = Odometry()
        self.occupiedCells = []
        self.flag_ = Bool()
        self.startTime_ = time.time()  # the time to hold the flag signal
        self.endTime_ = 0.  # the time where obstacles are still there
        self.firstTime_ = 0.  # the first time when detected obstacles
        self.costmapCoordinates_ = [0, 0]
        self.last_cmd_ = Twist()
        self.navigator = BasicNavigator()

        self.objflag_pub_ = self.create_publisher(Bool, "/object_detected", 10)

        self.bkwd_pub_ = self.create_publisher(Bool, "/need_backward", 10)
        self.initialpose_pub_ = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_backward_pub_ = self.create_publisher(
            Twist, '/cmd_vel_backward', 10)

        self.costmap_sub_ = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.costmapCallback, 10)
        self.costmap_sub_
        self.globalpath_sub_ = self.create_subscription(
            Path, "/received_global_plan", self.pathCallback, 10)
        self.globalpath_sub_
        self.odom_sub_ = self.create_subscription(
            Odometry, "/odom", self.odomCallback, 10)
        self.odom_sub_
        self.cmd_vel_sub_ = self.create_subscription(
            Twist, '/cmd_vel', self.cmdCallback, 10)
        self.cmd_vel_sub_
        self.path_feedback_sub_ = self.create_subscription(
            String, '/task_result', self.feedbackCallback, 10)
        self.path_feedback_sub_

    def feedbackCallback(self, msg: String):
        # to reset the path, not tracking when reached goal due to last data
        if "SUCCEEDED" in msg.data:
            print("SUCCEEDED, reset path")
            self.path_ = Path()

    def cmdCallback(self, msg: Twist):
        # use to save the cmd and send -ve cmd when obstacles hold too long
        stop_cmd_vel_ = Twist()
        stop_cmd_vel_.linear.x = 0.
        stop_cmd_vel_.linear.y = 0.
        stop_cmd_vel_.linear.z = 0.
        stop_cmd_vel_.angular.x = 0.
        stop_cmd_vel_.angular.y = 0.
        stop_cmd_vel_.angular.z = 0.
        if msg != stop_cmd_vel_:
            self.last_cmd_ = msg

    def pathCallback(self, msg: Path):
        self.path_ = msg

    def odomCallback(self, msg: Odometry):
        self.odom_ = msg

    def costmapCallback(self, msg: OccupancyGrid):
        origin_x_odom = msg.info.origin.position.x
        origin_y_odom = msg.info.origin.position.y

        dist = math.dist([origin_x_odom, origin_y_odom], [
            self.costmapCoordinates_[0], self.costmapCoordinates_[1]])
        if dist > 0.5:
            self.occupiedCells = []
            self.get_logger().info(
                f"Unstable localization detected, jumped {round(dist,1)}m")
            self.costmapCoordinates_ = [origin_x_odom, origin_y_odom]
            return
        self.costmapCoordinates_ = [origin_x_odom, origin_y_odom]
        self.detectObstacle(msg)

    def detectObstacle(self, msg: OccupancyGrid):
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        stillHere = 0
        origin_x_odom = msg.info.origin.position.x
        origin_y_odom = msg.info.origin.position.y

        # previous loops have obstacles near path
        if (len(self.occupiedCells) != 0 or self.flag_.data):
            for cell in self.occupiedCells:
                if (data[cell[0], cell[1]] > 90):
                    stillHere += 1
                self.endTime_ = time.time()  # obstacles still here
            if (stillHere == 0):
                # no more occupied cells
                print("No more obstacle!")
                self.occupiedCells = []  # reset
            self.flag_.data = True

            obstacleTime = self.endTime_ - self.firstTime_
            if obstacleTime >= 3.0:
                pose = PoseWithCovarianceStamped()
                pose.header.stamp = BasicNavigator().get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose = self.odom_.pose
                pose.pose.pose.position.x = self.odom_.pose.pose.position.x-0.2
                # self.initialpose_pub_.publish(pose)

                # print('move backward')
                # cmd_vel = self.last_cmd_
                # cmd_vel.linear.x = -0.1
                # self.cmd_vel_ _pub_.publish(cmd_vel)
                # self.bkwd_pub_.publish(Bool(data=True))
                # time.sleep(3.0)
                # cmd_vel.linear.x = 0.
                # self.cmd_vel_backward_pub_.publish(cmd_vel)
                # self.bkwd_pub_.publish(Bool(data=False))

                # self.get_logger().info("Clear local costmap by re-positioning")
                # self.navigator.clearGlobalCostmap()
                # self.navigator.clearLocalCostmap()
                self.firstTime_ = time.time()

            holdTime = self.startTime_ - self.endTime_
            print(f"hold time: {holdTime}")
            if (len(self.occupiedCells) == 0 and holdTime <= 2.):
                self.startTime_ = time.time()
                self.get_logger().info(
                    f"All obstacles are removed, holding stop for {round(holdTime,1)}/2.0")

            # only if holdtime signal is ended and no more occupied cells
            elif (len(self.occupiedCells) == 0):
                self.flag_.data = False
            self.objflag_pub_.publish(self.flag_)
            print(f'published flag {self.flag_.data}')
            return

        bufferRadius = 0.2  # 0.2 for actual robot
        bufferCells = math.ceil(bufferRadius / resolution)
        obstacleCells = 0

        ocCells = []

        # The path is updated when the robot pass through.
        # Therefore, in order to cal. the path starting from the head of robot,
        # need to chop some point at first [6:30]
        i = 0
        for pose in self.path_.poses[6:30]:
            pose: PoseStamped
            posei = math.ceil(
                (pose.pose.position.y-origin_y_odom)/resolution)
            posej = math.ceil(
                (pose.pose.position.x-origin_x_odom)/resolution)
            if (0 <= posei < height and 0 <= posej < width):  # posecell within costmap
                # posecell surrounding within costmap
                for celli in range(posei-bufferCells, posei+bufferCells):
                    if (0 <= celli < height):
                        for cellj in range(posej-bufferCells, posej+bufferCells):
                            if (0 <= cellj < width):
                                # print("celli, cellj:", celli, cellj)
                                if (data[celli, cellj] >= 90):
                                    ocCells.append([celli, cellj])
                                    obstacleCells += 1
                                    x = origin_x_odom + cellj * resolution
                                    y = origin_y_odom + celli * resolution
                                    print(
                                        f"Occupied cell at ({round(x,2)}, {round(y,2)}) on path point {i,round(pose.pose.position.x,2)},{round(pose.pose.position.y,2)}")
            i += 1

        self.flag_.data = obstacleCells > 3
        if (self.flag_.data):
            self.firstTime_ = time.time()
            self.endTime_ = time.time()  # detected obstacle
            self.occupiedCells = ocCells
            self.get_logger().info("Obstacle detected, sending stop cmd")
        self.objflag_pub_.publish(self.flag_)


def ouob(args=None):
    rclpy.init(args=args)
    node = ObstacleExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    ouob()

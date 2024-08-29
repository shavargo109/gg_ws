#!/usr/bin/env python3
import time
import rclpy
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import math


class ObstacleExtractor(Node):
    def __init__(self):
        super().__init__('obstacle_extractor')

        self.costmap_ = OccupancyGrid()
        self.path_ = Path()
        self.odom_ = Odometry()
        self.occupiedCells = []
        self.flag_ = Bool()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.startTime_ = time.time()
        self.endTime_ = 0.

        self.objflag_pub_ = self.create_publisher(Bool, '/object_detected', 10)

        self.costmap_sub_ = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmapCallback,
            10)
        self.costmap_sub_
        # self.globalpath_sub_ = self.create_subscription(
        #     Path, '/plan', self.pathCallback, 10)
        self.globalpath_sub_ = self.create_subscription(
            Path, '/received_global_plan', self.pathCallback, 10)

        self.globalpath_sub_
        self.odom_sub_ = self.create_subscription(
            Odometry, '/odom', self.odomCallback, 10)
        self.odom_sub_

    def pathCallback(self, msg: Path):
        self.path_ = msg

    def odomCallback(self, msg: Odometry):
        self.odom_ = msg

    def costmapCallback(self, msg: OccupancyGrid):
        # Extract the costmap info
        # if not self.path_.poses or not self.odom_:
        #     self.get_logger().info('No global path or odom received!')
        #     return

        # try:
        # transform = self.tf_buffer.lookup_transform(
        #     'map',  # Target frame
        #     'map',   # Source frame
        #     rclpy.time.Time(),  # Time at which to get the transform
        #     timeout=rclpy.duration.Duration(seconds=1.0)
        # )
        # Transform the origin of the costmap
        # origin_pose = PoseStamped()
        # origin_pose.header.frame_id = 'map'
        # origin_pose.pose.position.x = msg.info.origin.position.x
        # origin_pose.pose.position.y = msg.info.origin.position.y
        # origin_pose.pose.orientation = msg.info.origin.orientation

        # Transform the origin to the odom frame
        # transformed_origin = tf2_geometry_msgs.do_transform_pose(
        #     origin_pose.pose, transform)

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
                # if (stillHere > 1):
                #     x = origin_x_odom + cell[1] * resolution
                #     y = origin_y_odom + cell[0] * resolution
                #     self.get_logger().info(
                #         f'Obstacles are still here in {round(x,2)}, {round(y,2)}! ')
                self.endTime_ = time.time()  # obstacles still here
            if (stillHere == 0):
              # no more occupied cells
                self.get_logger().info('No more obstacle!')
                self.occupiedCells = []  # reset
            self.flag_.data = True

            holdTime = self.startTime_ - self.endTime_
            print(f"start time: {self.startTime_}")
            print(f"end time: {self.endTime_}")
            print(f"hold time: {holdTime}")
            if (len(self.occupiedCells) == 0 and 0 < holdTime <= 3.):
                self.startTime_ = time.time()
                self.get_logger().info(
                    f'All obstacles are removed, holding stop for {round(holdTime,1)}')

            # only if holdtime signal is ended and no more occupied cells
            elif (len(self.occupiedCells) == 0):
                self.flag_.data = False
            self.objflag_pub_.publish(self.flag_)
            return
        # print('======================================')
        # print(origin_x_odom, origin_y_odom)
        maxDistance = 2  # lookahead distance from robot along the path
        bufferRadius = 0.1
        bufferCells = math.ceil(bufferRadius / resolution)
        obstacleCells = 0

        robot_x = self.odom_.pose.pose.position.x
        robot_y = self.odom_.pose.pose.position.y
        # robot_grid_x = int((robot_x - origin_x_odom) / resolution)
        # robot_grid_y = int((robot_y - origin_y_odom) / resolution)

        ocCells = []

        # only cal. forward 20 points within maxDistance or not
        for pose in self.path_.poses[:20]:
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
                                print('celli, cellj:', celli, cellj)
                                if (data[celli, cellj] >= 90):
                                    ocCells.append([celli, cellj])
                                    obstacleCells += 1
                                    x = origin_x_odom + cellj * resolution
                                    y = origin_y_odom + celli * resolution
                                    print(
                                        f"Occupied cell at ({round(x,2)}, {round(y,2)}) on path point {round(pose.pose.position.x,2)},{round(pose.pose.position.y,2)}")

        self.flag_.data = obstacleCells > 3
        if (obstacleCells > 3):
            self.endTime_ = time.time()  # detected obstacle
            self.occupiedCells = ocCells
            self.get_logger().info("Obstacle detected, sending stop cmd")
        self.objflag_pub_.publish(self.flag_)

        ############### costmap checking ####################
        # data = np.array(msg.data).reshape((height, width))

        # # Print the coordinates of occupied cells
        # for i in range(height):
        #     for j in range(width):
        #         if data[i, j] > 0:  # Assuming >0 indicates an obstacle or relevant feature
        #             # Calculate the world coordinates of this cell
        #             x = origin_x_odom + j * resolution
        #             y = origin_y_odom + i * resolution
        #             print(
        #                 f"Occupied cell at ({x}, {y}) with value {data[i, j]}")
        # except tf2_ros.TransformException as ex:
        #     self.get_logger().warn(f"Could not transform costmap origin: {ex}")


def ouob(args=None):
    rclpy.init(args=args)
    node = ObstacleExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    ouob()

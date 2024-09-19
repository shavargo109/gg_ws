#! /usr/bin/env python3

import time
import math
from enum import Enum
import rclpy
import numpy as np
from rclpy.node import Node
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Bool, Int8
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from scipy.spatial.distance import euclidean
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


'''
    perform action under recovery mode
    self-rotation then parallel moving

    Flow: if |45|<angle<=|90|, perform rotation then parallel moving
    if 90<angle<|180|, perform rotation to opposite direction, parallel moving,
    then rotate back to desired orientation
    if 0<=angle<=|45|, parallel moving

'''


class RecoveryState(Enum):
    IDLE = 0
    ROTATING = 1
    PARALLEL_MOVING = 2
    ROTATE_BACK = 3


class obstacleDetectNode(Node):
    def __init__(self):
        super().__init__("obstacle_detect_node")
        self.declare_parameter("width", 0.25)
        self.declare_parameter("length", 0.5)
        self.declare_parameter("angle_tolerance", 0.0556*math.pi)
        self.width_ = self.get_parameter("width")
        self.length_ = self.get_parameter("length")
        self.tolerance_ = self.get_parameter("angle_tolerance")
        self.recoveryState_ = RecoveryState.IDLE
        self.globalPath_ = Path()
        self.odom_ = Odometry()
        # to check if the initial orientation need to rotate
        self.isfirstTime_ = bool(True)
        self.isOpposite_ = bool(False)  # to check needed rotate_back
        self.yaw_ = float()
        self.angle_ = float()

        ########### for left right part lidar detection #########
        self.thetaRight_ = math.floor(
            math.atan(0.3/1.0)*180/math.pi)
        self.rectDetectRight_ = []
        for i in range(90):
            if (i <= self.thetaRight_):
                dist = 1.0/math.cos(math.radians(i))
                self.rectDetectRight_.append(dist)
            elif (i > self.thetaRight_):
                dist = 0.3/math.sin(math.radians(i))
                self.rectDetectRight_.append(dist)
        self.rectDetectLeft_ = self.rectDetectRight_.copy()
        self.rectDetectLeft_.reverse()
        #########################################################

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.obsRight_pub_ = self.create_publisher(
            Int8, "/obstacle_R_msgs", 10)
        self.obsLeft_pub_ = self.create_publisher(Int8, "/obstacle_L_msgs", 10)
        self.turnRight_pub_ = self.create_publisher(
            Int8, "/obstacle_SR_msgs", 10)
        self.turnLeft_pub_ = self.create_publisher(
            Int8, "/obstacle_SL_msgs", 10)

        # actual robot topic
        self.laser_sub_ = self.create_subscription(
            LaserScan, "/scanner/scansd", self.laserCallback, qos)
        self.laser_sub_
        self.globalpath_sub_ = self.create_subscription(
            Path, "/received_global_plan", self.pathCallback, 10)
        self.globalpath_sub_
        self.isrecover_sub_ = self.create_subscription(
            Bool, "recovery_mode", self.recoverCallback, 10)
        self.isrecover_sub_
        self.odom_sub_ = self.create_subscription(
            Odometry, "odom", self.odomCallback, 10)
        self.odom_sub_
        self.angle_sub_ = self.create_subscription(
            Twist, "cmd_vel_tc", self.angleCallback, 10)
        self.angle_sub_

    def pathCallback(self, msg: Path):
        self.globalPath_ = msg

    def recoverCallback(self, msg: Bool):
        if msg.data and self.recoveryState_ == RecoveryState.IDLE:
            self.angleAdjust()
        elif not msg.data:
            print("Received wrong data to enter recovery mode")
        else:
            print("Previous rotation is processing...")

    def odomCallback(self, msg: Odometry):
        self.odom_ = msg

    def angleCallback(self, msg: Twist):
        # all cal. in radian
        if self.recoveryState_ == RecoveryState.ROTATING:
            self.handleRotation(msg)
        elif self.recoveryState_ == RecoveryState.ROTATE_BACK:
            self.handleRotationBack(msg)

    def laserCallback(self, msg: LaserScan):
        if self.recoveryState_ == RecoveryState.PARALLEL_MOVING:
            self.handleParallelMoving(msg)

    def handleRotation(self, msg: Twist):
        delta = msg.angular.z-self.yaw_
        self.angle_ = msg.angular.z
        print("yaw: ", self.yaw_)
        print("angle: ", msg.angular.z)

        if self.isfirstTime_:
            if abs(delta) < abs(0.25*math.pi):
                print("Angle is too small, no need to self-rotate")
                self.recoveryState_ = RecoveryState.PARALLEL_MOVING
                return
            elif abs(delta) > abs(0.5*math.pi):
                print("Angle > 90, performing opposite rotation")
                self.yaw_ = self.yaw_+math.pi
                delta = msg.angular.z-self.yaw_
                # delta = yaw-self.yaw_
                self.isOpposite_ = True
            self.isfirstTime_ = False

        # should rotate in reverse direction if delta is larger than 180 deg
        if delta > math.pi:
            delta = delta-2*math.pi
        elif delta < -math.pi:
            delta = delta+2*math.pi

        if not self.keepRotating(delta):
            self.recoveryState_ = RecoveryState.PARALLEL_MOVING

    def handleRotationBack(self, msg: Twist):
        self.yaw_ = self.yaw_-math.pi
        delta = msg.angular.z-self.yaw_
        self.angle_ = msg.angular.z
        print("yaw: ", self.yaw_)
        print("angle: ", msg.angular.z)

        # should rotate in reverse direction if delta is larger than 180 deg
        if delta > math.pi:
            delta = delta-2*math.pi
        elif delta < -math.pi:
            delta = delta+2*math.pi

        if not self.keepRotating(delta):
            self.get_logger().info("All actions are completed 2")
            self.recoveryState_ = RecoveryState.IDLE
            self.isfirstTime_ = True
            self.isOpposite_ = False
            self.turnRight_pub_.publish(Int8(data=0))
            self.turnLeft_pub_.publish(Int8(data=0))

    def handleParallelMoving(self, msg: LaserScan):
        # anticlockwise
        rangeDetectRaw = msg.ranges[90:270]  # actual robot
        # print(rangeDetectRaw)

        for i in range(90):
            leftMsg = Int8()
            rightMsg = Int8()
            leftMsg.data = 0
            rightMsg.data = 0
            if (rangeDetectRaw[i] <= self.rectDetectRight_[i]):
                rightMsg.data = 2
                # print("right ", i, rangeDetectRaw[i])

            if (rangeDetectRaw[90+i] <= self.rectDetectLeft_[i]):
                leftMsg.data = 1
                # print("left ", 90+i, rangeDetectRaw[90+i])
        self.obsRight_pub_.publish(rightMsg)
        self.obsLeft_pub_.publish(leftMsg)
        if leftMsg.data == rightMsg.data == 0 or (leftMsg.data == 1 and rightMsg.data == 2):
            print("Parallel moving completed")
            if not self.isOpposite_:
                # All actions are done, reset all flags
                self.get_logger().info("All actions are completed 1")
                self.recoveryState_ = RecoveryState.IDLE
                self.isfirstTime_ = True
                self.isOpposite_ = False
                self.turnRight_pub_.publish(Int8(data=0))
                self.turnLeft_pub_.publish(Int8(data=0))
            elif self.isOpposite_:
                self.recoveryState_ = RecoveryState.ROTATE_BACK

    def angleAdjust(self):
        if self.globalPath_ == Path():
            print("Not yet recieved path...")
            return

        distance = self.shortestPoint()
        self.yaw_ = math.atan2(distance[1][1]-distance[0][1],
                               distance[1][0]-distance[0][0])
        self.recoveryState_ = RecoveryState.ROTATING

    def keepRotating(self, delta: float):
        ########################################################
        # delta is -ve, rotate clockwise
        # delta is +ve, rotate anticlockwise

        # p.s.the +ve direction in simulation is anticlockwise,
        ########################################################

        # publish rotation signal topic here
        dataLeft = 0
        dataRight = 0
        if abs(delta) < abs(self.tolerance_.value):
            print("Current orientation is within tolerance, self-rotation is completed")
            # reset to non-zero data to prevent false resume
            self.turnRight_pub_.publish(Int8(data=7))
            self.turnLeft_pub_.publish(Int8(data=7))
            return False
        elif delta > 0:
            print("Rotating clockwise...")
            dataRight = 6
        elif delta < 0:
            print("Rotating anticlockwise...")
            dataLeft = 5

        self.turnRight_pub_.publish(Int8(data=dataRight))
        self.turnLeft_pub_.publish(Int8(data=dataLeft))
        return True

    def shortestPoint(self):
        check = []
        index = int()
        coordinates = []
        for pose in self.globalPath_.poses:
            pose: PoseStamped
            coordinates.append([pose.pose.position.x, pose.pose.position.y])

        for coordinate in coordinates:
            if (self.odom_.pose.pose.position.x-2. <= coordinate[0] <= self.odom_.pose.pose.position.x+2. and
                    self.odom_.pose.pose.position.y-2. <= coordinate[1] <= self.odom_.pose.pose.position.y+2.):
                check.append(coordinate)

        asd = [self.odom_.pose.pose.position.x,
               self.odom_.pose.pose.position.y]
        if len(check) == 0:
            print(asd)
            print("no overlap point")
            distances = [euclidean(asd, point) for point in coordinates]
            min_index = np.argmin(distances)
            index = min_index
        else:
            print("points in area, len: " + str(len(check)))
            distances = [euclidean(asd, point) for point in check]
            min_index = np.argmin(distances)  # min_index of check[]
            index = coordinates.index(check[min_index])
        # shortest point and point after 3 to calculate yaw
        print("index: ", index)
        anglepath = []
        anglepath.append(coordinates[index])
        anglepath.append(coordinates[index+3])
        return anglepath


def ouob(args=None):
    rclpy.init(args=args)
    minimal_subscriber = obstacleDetectNode()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    ouob()

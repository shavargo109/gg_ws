#! /usr/bin/env python3

import time
import math
import rclpy
import numpy as np
from rclpy.node import Node
# from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Bool, Int8
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from scipy.spatial.distance import euclidean
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


'''
    perform action under recovery mode
    self-rotation then parallel moving

    TODO: first rotate, then parallel
'''


class obstacleDetectNode(Node):
    def __init__(self):
        super().__init__("obstacle_detect_node")
        self.declare_parameter("width", 0.25)
        self.declare_parameter("length", 0.5)
        self.width_ = self.get_parameter("width")
        self.length_ = self.get_parameter("length")
        self.speedAll_ = 0.
        self.globalPath_ = Path()
        self.odom_ = Odometry()
        self.rotateFlag_ = bool(False)
        self.isfirstTime_ = bool(True)
        self.yaw_ = float()

        ########### for left right part lidar detection #########
        self.thetaRight_ = math.floor(
            math.atan(0.3/1.0)*180/3.141592654)
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
        print(self.thetaRight_)
        print(len(self.rectDetectLeft_), len(self.rectDetectRight_))
        print(self.rectDetectLeft_)
        #########################################################

        theta = math.ceil(
            math.atan(self.length_.value/self.width_.value)*180/3.141592654)
        self.rectDetect_ = []
        self.startTime_ = time.time()
        self.endTime_ = 0.
        self.previousDetect_ = [False]*180
        self.isObstacle_ = bool(False)
        self.previousDist_ = [0]*180
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.asddw)

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        # compute a list of detected range within the rectangle from 0 to 90 and 270 to 360
        for i in range(90):
            if (i <= theta):
                dist = self.length_.value/math.cos(math.radians(i))
                self.rectDetect_.append(dist)
            elif (i > theta):
                dist = self.width_.value/math.sin(math.radians(i))
                self.rectDetect_.append(dist)
        copy = self.rectDetect_.copy()
        self.rectDetect_.reverse()
        self.rectDetect_.extend(copy)
        # print(len(self.rectDetect_))
        # print(self.rectDetect_)

        # pub cmd to vel_smooth node for overall cmd pub
        # self.obj_pub_ = self.create_publisher(Bool, "/object_detected", 10)
        self.obsFront_pub_ = self.create_publisher(Int8, "/obstacle_msgs", 10)
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
        # self.laser_sub_ = self.create_subscription(
        #     LaserScan, "/scan", self.laserCallback, qos)
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

    def angleCallback(self, msg: Twist):
        # all cal. in radian
        if self.rotateFlag_:
            pi = 3.141592654
            # orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            #                     msg.pose.pose.orientation.z,
            #                     msg.pose.pose.orientation.w]
            # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            delta = msg.angular.z-self.yaw_
            # delta = yaw-self.yaw_
            print("yaw: ", self.yaw_)
            print("angle: ", msg.angular.z)

            if self.isfirstTime_:
                if (abs(delta) < abs(0.25*pi)):
                    print("Angle is too small, no need to self-rotate")
                    self.rotateFlag_ = False
                    return
                self.isfirstTime_ = False

            # should rotate in reverse direction if delta is larger than 180 deg
            if delta > pi:
                delta = delta-2*pi
            elif delta < -pi:
                delta = delta+2*pi

            if not self.keepRotating(delta):
                self.rotateFlag_ = False

    def pathCallback(self, msg: Path):
        self.globalPath_ = msg

    def recoverCallback(self, msg: Bool):
        self.angleAdjust()

    def odomCallback(self, data: Odometry):
        self.odom_ = data

    def laserCallback(self, data: LaserScan):
        # when >=3 laserscan data has obstacle, set flag
        ######### simulation ##########
        # rangeDetectRaw = data.ra/nges[0:90]
        # rangeDetectRaw.extend(data.ranges[270:360])
        ###############################
        # anticlockwise
        rangeDetectRaw = data.ranges[90:270]  # actual robot
        # print(rangeDetectRaw)
        no = 0
        asd = 0
        asDetect = [False]*180
        self.speedAll_ = 0.
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

        for i in range(180):
            if (rangeDetectRaw[i] <= self.rectDetect_[i]):
                # print(str(i)+" " +str(rangeDetectRaw[i])+" "+str(self.rectDetect_[i]))
                asDetect[i] = True
                # 0.2 = rate of data
                speed = (self.previousDist_[i]-rangeDetectRaw[i])/0.2
                self.speedAll_ += speed
                asd += 1
                # print("speed:", speed)
                if (asDetect[i] == self.previousDetect_[i] == True):
                    no += 1
        if (asd != 0):
            # print("speed ", self.speedAll_/asd)
            pass
        if (no >= 3):
            self.isObstacle_ = True
        else:
            self.isObstacle_ = False
        self.previousDetect_ = asDetect.copy()
        self.previousDist_ = rangeDetectRaw[:]
        # print(str(no)+" "+str(self.isObstacle_))

    def angleAdjust(self):
        if self.rotateFlag_:
            print("Previous rotation is processing...")
            return
        distance = self.shortestPoint()
        if distance == None:
            print("")
        self.yaw_ = math.atan2(distance[1][1]-distance[0][1],
                               distance[1][0]-distance[0][0])
        self.rotateFlag_ = True
        # q = quaternion_from_euler(0., 0., yaw)
        # imu = Imu()
        # imu.orientation.x = q[0]
        # imu.orientation.y = q[1]
        # imu.orientation.z = q[2]
        # imu.orientation.w = q[3]

    def keepRotating(self, delta: float, pi: float = 3.141592654):
        ######################################
        # delta is -ve, rotate clockwise
        # delta is +ve, rotate anticlockwise

        # p.s. somehow the +ve direction in simulation is anticlockwise,
        # so the above direction need to be reversed
        ######################################

        # publish signal topic here
        if abs(delta) < abs(0.05556*pi):
            print("Current orientation is within tolerance, self-rotation is completed")
            self.isfirstTime_ = True
            # reset
            self.turnRight_pub_.publish(Int8(data=0))
            self.turnLeft_pub_.publish(Int8(data=0))
            return False
        elif delta > 0:
            print("Rotating clockwise...")
            self.turnRight_pub_.publish(Int8(data=6))
            self.turnLeft_pub_.publish(Int8(data=0))
        elif delta < 0:
            print("Rotating anticlockwise...")
            self.turnLeft_pub_.publish(Int8(data=5))
            self.turnRight_pub_.publish(Int8(data=0))
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

    def asddw(self):  # if flag holds true for 1s, publish stop cmd
        asd = Int8()
        if (not self.isObstacle_):
            self.startTime_ = time.time()
            holdTime = self.startTime_ - self.endTime_

            if (holdTime <= 3.):
                # print("obstacles removed, holding stop signal for", holdTime)
                asd.data = 3
                self.obsFront_pub_.publish(asd)
            else:
                # print("nothing")
                asd.data = 0
                self.obsFront_pub_.publish(asd)
        else:
            self.endTime_ = time.time()
            obstacleTime = self.endTime_ - self.startTime_
            # print("Obstacle time:", obstacleTime)
            if (obstacleTime >= 1.):
                # print("obstacles detected, sending stop cmd to vel_smooth_node")
                asd.data = 3
                self.obsFront_pub_.publish(asd)


def ouob(args=None):
    rclpy.init(args=args)
    minimal_subscriber = obstacleDetectNode()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    ouob()

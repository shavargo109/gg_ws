#! /usr/bin/env python3

import time
import math
import rclpy
import numpy as np
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from std_msgs.msg import String, Bool, Int8
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry

from scipy.spatial.distance import euclidean
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy


class obstacleDetectNode(Node):
    def __init__(self):
        super().__init__('obstacle_detect_node')
        self.declare_parameter('width', 0.25)
        self.declare_parameter('length', 0.5)
        self.width_ = self.get_parameter('width')
        self.length_ = self.get_parameter('length')
        self.speedAll_ = 0.
        self.globalPath_ = Path()
        self.odom_ = Odometry()

        ########### for left right part lidar detection #########
        # self.thetaRight_ = math.floor(
        #     math.atan(0.3/0.6)*180/3.141592654)
        self.thetaRight_ = math.floor(
            math.atan(0.6/0.3)*180/3.141592654)
        self.rectDetectRight_ = []
        for i in range(90):
            if (i <= self.thetaRight_):
                dist = 1.0/math.cos(math.radians(i))
                # dist = 0.3/math.cos(math.radians(i))
                self.rectDetectRight_.append(dist)
            elif (i > self.thetaRight_):
                dist = 0.3/math.sin(math.radians(i))
                # dist = 0.6/math.sin(math.radians(i))
                self.rectDetectRight_.append(dist)
        self.rectDetectLeft_ = self.rectDetectRight_.copy()
        self.rectDetectLeft_.reverse()
        print(self.thetaRight_)
        print(len(self.rectDetectLeft_), len(self.rectDetectRight_))
        print(self.rectDetectLeft_)
        #########################################################

        self.theta_ = math.ceil(
            math.atan(self.length_.value/self.width_.value)*180/3.141592654)
        # print(self.theta_)
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
            if (i <= self.theta_):
                dist = self.length_.value/math.cos(math.radians(i))
                self.rectDetect_.append(dist)
            elif (i > self.theta_):
                dist = self.width_.value/math.sin(math.radians(i))
                self.rectDetect_.append(dist)
        # for i in range(90):
        #     if (i <= self.theta_):
        #         dist = 0.25/math.cos(math.radians(i))
        #         self.rectDetect_.append(dist)
        #     elif (i > self.theta_):
        #         dist = 1/math.sin(math.radians(i))
        #         self.rectDetect_.append(dist)
        copy = self.rectDetect_.copy()
        self.rectDetect_.reverse()
        self.rectDetect_.extend(copy)
        # print(len(self.rectDetect_))
        # print(self.rectDetect_)

        # pub cmd to vel_smooth node for overall cmd pub
        # self.obj_pub_ = self.create_publisher(Bool, '/object_detected', 10)
        self.obsFront_pub_ = self.create_publisher(Int8, '/obstacle_msgs', 10)
        self.obsRight_pub_ = self.create_publisher(
            Int8, '/obstacle_R_msgs', 10)
        self.obsLeft_pub_ = self.create_publisher(Int8, '/obstacle_L_msgs', 10)
        self.orientation_pub_ = self.create_publisher(Imu, '/imu_adjust', 10)

        # self.laser_sub_ = self.create_subscription(
        #     LaserScan, '/scanner/scansd', self.laserCallback, qos)
        self.laser_sub_ = self.create_subscription(
            LaserScan, '/scan', self.laserCallback, qos)
        self.laser_sub_  # prevent unused variable warning
        self.globalpath_sub_ = self.create_subscription(
            Path, '/received_global_plan', self.pathCallback, 10)
        self.globalpath_sub_
        self.isrecover_sub_ = self.create_subscription(
            Bool, 'is_recover', self.recoverCallback, 10)
        self.isrecover_sub_
        self.odom_sub_ = self.create_subscription(
            Odometry, 'odom', self.odomCallback, 10)
        self.odom_sub_

    def pathCallback(self, msg: Path):
        self.globalPath_ = msg

    def recoverCallback(self, msg: Bool):
        self.angleadjust()

    def odomCallback(self, data: Odometry):
        self.odom_ = data

    def laserCallback(self, data: LaserScan):
        # when >=3 laserscan data has obstacle, set flag
        start_time = time.time()
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
                print('right ', i, rangeDetectRaw[i])

            if (rangeDetectRaw[90+i] <= self.rectDetectLeft_[i]):
                leftMsg.data = 1
                print('left ', 90+i, rangeDetectRaw[90+i])
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
                # print('speed:', speed)
                if (asDetect[i] == self.previousDetect_[i] == True):
                    no += 1
        if (asd != 0):
            # print('speed ', self.speedAll_/asd)
            pass
        if (no >= 3):
            self.isObstacle_ = True
        else:
            self.isObstacle_ = False
        self.previousDetect_ = asDetect.copy()
        self.previousDist_ = rangeDetectRaw[:]
        # print(str(no)+" "+str(self.isObstacle_))
        end_time = time.time()
        execution_time = start_time - end_time
        # print("Execution time:", execution_time)

    def angleadjust(self):
        distance = self.shortestpoint()
        yaw = math.atan2(distance[1][1]-distance[0][1],
                         distance[1][0]-distance[0][0])
        q = quaternion_from_euler(0., 0., yaw)
        imu = Imu()
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        self.orientation_pub_.publish(imu)

    def shortestpoint(self):
        check = []
        index = int()
        coordinates = []
        for pose in self.globalPath_.poses:
            coordinates.append([pose.pose.position.x, pose.pose.position.y])

        for coordinate in coordinates:
            if (self.odom_.pose.pose.position.x-2. <= coordinate[0] <= self.odom_.pose.pose.position.x+2. and
                    self.odom_.pose.pose.position.y-2. <= coordinate[1] <= self.odom_.pose.pose.position.y+2.):
                check.append(coordinate)

        asd = [self.odom_.pose.pose.position.x,
               self.odom_.pose.pose.position.y]
        if len(check) == 0:
            print('no overlap point')
            distances = [euclidean(asd, point) for point in coordinates]
            min_index = np.argmin(distances)
            index = min_index
        else:
            print('points in area, len: ' + str(len(check)))
            distances = [euclidean(asd, point) for point in check]
            min_index = np.argmin(distances)  # min_index of check[]
            index = coordinates.index(check[min_index])
        # shortest point and point after 5 to calculate yaw
        anglepath = []
        anglepath.append(coordinates[index])
        anglepath.append(coordinates[index+5])
        return anglepath

    def asddw(self):  # if flag holds true for 1s, publish stop cmd
        asd = Int8()
        if (not self.isObstacle_):
            self.startTime_ = time.time()
            holdTime = self.startTime_ - self.endTime_

            if (holdTime <= 3.):
                # print('obstacles removed, holding stop signal for', holdTime)
                asd.data = 3
                self.obsFront_pub_.publish(asd)
            else:
                # print('nothing')
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


if __name__ == '__main__':
    ouob()

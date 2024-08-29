#!/usr/bin/env python3
import rclpy
import rclpy.clock
from rclpy.node import Node
import yaml
import numpy as np
import csv
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from PIL import Image
from scipy.spatial.distance import euclidean
from nav2_simple_commander.robot_navigator import BasicNavigator


class MapPathNode(Node):
    def __init__(self):
        super().__init__('map_path_node')
        self.declare_parameter(
            'image_path', '/home/asd/Robotics_ws/src/map_find_path/maps/map_LG2_indoor_0616_2_2023-06-16-12-23-40_2.jpg')
        self.declare_parameter(
            'yaml_path', '/home/asd/Robotics_ws/src/map_find_path/maps/map_LG2_indoor_0616_2_2023-06-16-12-23-40.yaml')
        self.declare_parameter(
            'savepath_filename', '/home/asd/Robotics_ws/src/map_find_path/path/0402_LG2_slope_path1')
        self.declare_parameter('first_pixels', [160, 194])
        self.image_path_ = self.get_parameter('image_path')
        self.yaml_path_ = self.get_parameter('yaml_path')
        self.savepath_filename_ = self.get_parameter('savepath_filename')
        self.origin_x_ = 0.
        self.origin_y_ = 0.
        self.resolution_ = 0.
        self.is_published_ = bool(False)
        self.first_pixels_ = self.get_parameter(
            'first_pixels')  # in ros format
        # self.first_pixels_ = [503, 468]  # in ros format
        print('map to path node started')

        self.path_pub_ = self.create_publisher(Path, 'map_path', 10)

    def read_yaml_file(self, file_path):
        '''
        initialise the origin and resolution of map 
        '''
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            self.origin_x_ = data['origin'][0]
            self.origin_y_ = data['origin'][1]
            self.resolution_ = data['resolution']

    def origin_pixels(self) -> list:
        pixels_x = (0-self.origin_x_)/self.resolution_
        pixels_y = (0-self.origin_y_)/self.resolution_
        return [pixels_x, pixels_y]

    def extract_red_pixels(self, image_path: str):
        img = Image.open(image_path)
        pixels = img.load()
        print(pixels[0, 0])
        width, height = img.size
        first_y = self.first_pixels_.value[1]
        self.first_pixels_.value[1] = height-first_y-1

        pixels_x = (0-self.origin_x_)/self.resolution_
        pixels_y = (0-self.origin_y_)/self.resolution_
        print("origin pixels(0,0): " + str([pixels_x, height-pixels_y]))

        red_pixels = []

        for y in range(height):
            for x in range(width):
                if ('.png' in image_path):
                    r, g, b, _ = pixels[x, y]  # png read RGBA instead of RGB
                elif ('.jpg' in image_path):
                    r, g, b = pixels[x, y]  # for jpg format

                if (r > 100 and g < 100 and b < 100):
                    # originally the lib read top left as (0,0), where ros read yaml bottom left as (0,0)
                    dy = height - y - 1
                    red_pixels.append((x, dy))

        # return list of (x,y)
        print("red pixels: " + str(len(red_pixels)))
        return red_pixels

    def find_shortest_path(self, start_point, points: list):
        path = [start_point]
        remaining_points = points.copy()

        while remaining_points:
            distances = [euclidean(path[-1], point)
                         for point in remaining_points]
            min_index = np.argmin(distances)
            closest_point = remaining_points[min_index]

            path.append(closest_point)
            remaining_points.remove(closest_point)

        return path

    def pixels_to_coorindate(self, pixels):
        map_coordinates = []
        for coordinates in pixels:
            x = coordinates[0]*self.resolution_+self.origin_x_
            y = coordinates[1]*self.resolution_+self.origin_y_
            map_coordinates.append((x, y))

        return map_coordinates

    def map_path(self):
        pixels = self.extract_red_pixels(self.image_path_.value)
        print('total pixels: '+str(len(pixels)))
        sorted_pixels = self.find_shortest_path(
            self.first_pixels_.value, pixels)
        # coordinates = self.pixels_to_coorindate(sorted_pixels) # convert all points
        # filtered_pixels = sorted_pixels[::10]
        filtered_pixels = sorted_pixels.copy()
        filtered_pixels.append(sorted_pixels[-1])
        print("last pixels: "+str(filtered_pixels[-1]))
        coordinates = self.pixels_to_coorindate(filtered_pixels)
        print('filtered pixels: '+str(len(coordinates)))

        self.record_data(coordinates)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for coordinate in coordinates:
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
            path.poses.append(pose)
        return path

    def record_data(self, cooridnates):
        filename = self.savepath_filename_.value
        filepath = filename
        header = ['x', 'y']
        is_header = bool(False)
        if (not is_header):
            with open(filepath, mode='w') as f:
                writer = csv.writer(
                    f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(header)
            is_header = True
            self.get_logger().info('written')

        for coordinate in cooridnates:
            with open(filepath, mode='a') as f:
                writer = csv.writer(
                    f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow([coordinate[0], coordinate[1]])

    def path_publish(self, event):
        self.read_yaml_file(self.yaml_path_.value)
        self.path_pub_.publish(self.map_path())
        self.get_logger().info('path published')


def ouob(args=None):
    rclpy.init(args=args)
    node = MapPathNode()
    node.path_publish(123)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    ouob()

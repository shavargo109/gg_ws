#! /usr/bin/env python3


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('test_sub')
        self.subscription = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Costmap):
        # costmap data starts from bottom right corner, upward as +ve value
        i = 0
        j = 0
        for cell in msg.data:
            if (cell != 0):
                print(str(i)+" "+str(cell))
                j += 1
            i += 1
        print(j)
        print('===============================')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

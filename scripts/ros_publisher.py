#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

#
# Add code here to import the new ROS message...
#

class Lab1CustomPublisher(Node):

    def __init__(self):
        super().__init__('lab1_publisher')
        #
        # Add code here to create a publisher that publishes to the topic mentioned
        # in the description
        #
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        #
        # Add code here to create a new object of the new ROS message, to assign the random integers,
        # and publish through the ROS topic...
        #
        pass


def main(args=None):
    rclpy.init(args=args)

    lab1_custom_publisher = Lab1CustomPublisher()

    rclpy.spin(lab1_custom_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lab1_custom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
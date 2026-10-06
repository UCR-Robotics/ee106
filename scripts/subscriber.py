#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

#
# Add code here to import the new ROS message...
#

class Lab1CustomSubscriber(Node):

    def __init__(self):
        super().__init__('lab1_listener')
        self.subscription = None    # Replace this with your susbscriber
        #
        # Initialize the ROS subscriber to capture the new  message-type ROS topic.
        # The function "listener_callback" will be the callback of the ROS subscriber.
        #
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #
        # Add code here to perform the addition of the two integer field of the variable data
        #
        pass


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Lab1CustomSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
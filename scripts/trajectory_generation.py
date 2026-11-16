#!/usr/bin/env python3

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from tf_transformations import euler_from_quaternion
import time
import threading

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # TODO: calculate P_term and D_term
        # error = 
        # P_term = 
        # D_term = 
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class Turtlebot(Node):
    def __init__(self):
        super().__init__("turtlebot_move")
        self.get_logger().info("Press Ctrl + C to terminate")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel_unstamped", 10)


        # subscribe to odometry
        self.pose = Pose2D()
        self.linear_vel = 0.0
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = self.creater_subscription( Odometry, "odom", self.odom_callback, 10)



    def run(self):
        waypoints = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],\
                      [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
                      [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])


    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        # first, get the start velocity self.linear_vel, then break that up into x and y components based off self.pose.theta
        # Next, if the set the magnitude of the end velocity to be (next_waypoint-current_waipont)/T and then use the angle of (next_waypoint-current_waipont) to set the x and y components of the end velocity
        # Now that you have all the information, use the polynomial_time_scaling_3rd_order() funtion to calculate the coefficients for the polynomial for bot the x and y components
        # at each time step, use the coefficients to calculate v_x and v_y, then ser the magnitute of this velocity (sqrt(v_x^2+v_y^2)) as the linear velocity and set the setpoint of the pd controller to be atan2(v_y, v_x) and get the angular velocity from the pd controller
        # now compile both of these into 1 Twist and publish it with self.vel_pub.
        pass


    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        pass


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.linear_vel = msg.twist.twist.linear.x

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            self.get_logger().info("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


def main(args = None):
    rclpy.init(args=args)
    
    # Create the publisher node
    turtlebot_node = Turtlebot()
    
    # Spin so that the node is busy processing
    # while no exceptions are encountered and ROS
    # is executing
    # For this lab - spin rclpy on separate thread
    thread = threading.Thread(target=rclpy.spin, args=(turtlebot_node,), daemon=True)
    thread.start()
    
    # Allow time for other nodes to start
    time.sleep(5)
    
    try:
        turtlebot_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # save trajectory into csv file
        np.savetxt('trajectory.csv', np.array(turtlebot_node.trajectory), fmt='%f', delimiter=',')
    
        # Destroy the node explicitly
        turtlebot_node.destroy_node()
    
        # Shut down this context
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

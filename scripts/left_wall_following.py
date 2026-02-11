#!/usr/bin/env python3

import rospy
import roslib
roslib.load_manifest('ee106w26')
import sys
import numpy as np

#
# TODO : include the imports of the used libraries...
#

class turtlebot_behavior:
    
  def __init__(self):
      
    #
    # TODO : initialize the Publishers and Subscribers, as well as,
    # the FSM state and transition parameters.
    #

    self.rate = rospy.Rate(10.0)

    #
    # TODO : add a While loop until the ROS node is shutdown, to update the FSM states and send the command velocities on the robot, given the computed next state. You can use this while loop to listen on the TF information between the left_limit and the base_scan to use it in the LiDAR callback function.
    #

    # while ...
    
        # listen to TF and update self.trans and self.rot

        # if self.state == "forward": 
        #     if self.extreme_left:
        #         ...
        
        # send velocity command ...
        # self.rate.sleep()

  # this is the LiDAR callback that will update our FSM transition parameters
  
  def callback(self,data):

    # min_dist = np.inf
    # for r in data.ranges:

        # check if the measurement "r" is between the left_side rad range. If yes, convert it to x,y, compute it's spatial transform on the "left_limit" frame, and its distance from the origin. Update the min_dist parameter given the computed measurement distance.
    
    # As the for iteration is completed, check the mininum distance of the left side and update the FSM transition parameters. These updated transition parameters will update the FSM state, on the next iteration step in the class __init__ function.
  
  def calculate_position_of_range(self, range, idx, angle_increment, angle_min):

    if str(range)=="inf":
          rospy.loginfo("The provided range is infinite!")
          return -1

    theta = idx * angle_increment + angle_min
    x = range * np.cos(theta)
    y = range * np.sin(theta)

    return x,y

def main(args):
    rospy.init_node('left_wall_following', anonymous=True)
    ic = turtlebot_behavior()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
if __name__ == '__main__':
    main(sys.argv)

Lab 2: ROS Tools and Testing on a Real Robot
========================

Overview
--------

In this lab, we are going to test ROS tools that are developed for illustrating the ROS Nodes and Topic graph relations and visualizing the ROS Topics in real-time. ROS, as a tool-based framework, features a plethora of applications, which can also be found `here <http://wiki.ros.org/Tools>`_.

Additionally, the `Clearpath Jackal <https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/>`_ mobile robot will be demonstrated in the class, along with the use of some of the ROS tools. Furthermore, the simulated version of the Jackal robot will be deployed in the Gazebo simulator and it will be compared with the real robot. A 2D LiDAR ranging scenario will be demonstrated and will be the base of this lab's assignment.

Installing Gazebo Simulator
-----------

The Gazebo simulator is a broadly used open-source robotic simulator, that has been used for ROS application development before testing on robots in the real world. To install the Gazebo simulator we need to perform,

.. code-block:: bash

  sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

To install the official ROS package of the Clearpath Jackal robot for its Gazebo simulation

.. code-block:: bash

  sudo apt-get install ros-noetic-jackal-gazebo

to install the official Gazebo package of the simulated Clearpath Jackal robot


ROS Launch files
----------

Before we proceed with the experiment on the Jackal robot and the ROS tools, we need to describe the ROS launch file functionality. Particularly, a ROS launch file is a ``XML``  type file with a ``.launch`` file extension, which can be used to launch multiple ROS nodes at the same time while it can set values on parameters in the Parameters Server before executing the ``rosrun``. The ROS launch files can are kept in the ``launch`` folder of each ROS package.

An example of a launch file could be,

.. code-block:: html

  <launch>
          <arg name="x_var" default="0" />
          <arg name="enable_var" default="true" />

          <!-- This is a comment -->
          <node pkg="package_name" name="node_name" type="node_filename" />

          <node pkg="package_name" name="node_name_2" type="node_filename_2" args="-x $(arg x_var) -enable_var $(arg enable_var)" />

  </launch>

where,

.. code-block:: html

  <arg name="x_var" default="0" />
  <arg name="enable_var" default="true" />

is the initialization of the variable ``x_var`` with the value 0 and the variable ``enable_var`` with True. The code part of,

.. code-block:: html

  <node pkg="package_name" name="node_name" type="node_filename" />

executes the node ``node_filename`` from the package ``package_name`` and names it ``node_name``. The second node call part, namely,

.. code-block:: html

  <node pkg="package_name" name="node_name_2" type="node_filename_2" args="-x $(arg x_var) -enable_var $(arg enable_var)" 

executes the ``node_filename_2`` node, but also provides argument information through the Parameter Server.

To execute the ROS launch file you can perform in a new terminal,

.. code-block:: bash

  roslaunch package_name file.launch

Now, let's try to create a launch file for our created ROS package, namely the ``ee106s25``. Specifically, create a ROS launch that you can execute at the same time both the `publisher` and `subscriber` nodes of the Lab 1. Show the results to the Teaching Assistant.

rqt and rqt_graph Tools
----------

The rqt tool as a QT-based framework developed for ROS to enable the creation of user interface-enabled applications. The ``rqt_graph`` is visualizing tool that can illustrate the relations of the running ROS nodes and topics in a graph illustration.
To test the result of the rqt-graph, first enable the ROS nodes of your application and then execute the below command in a new terminal.

.. code-block:: bash

  rqt_graph

RViz : ROS Visualisation Tool
--------------

In addition, the main visualization tool that is used in ROS software development, is the RViz. This tool is used to illustrate the raw information that is published by the ROS topics, in real-time, with respect to a predefined coordinate system. To enable RViz you can perform in a separate terminal,

.. code-block:: bash

  rviz

Gazebo Simulation and the Clearpath Jackal Robot
--------------

In order to start the Gazebo simulator with an empty world, you can execute, 

.. code-block:: bash

  roslaunch gazebo_ros empty_world.launch

To properly exit or terminate Gazebo you should use the window terminating button. In many cases, such as closing abruptly the terminal or if the Gazebo is not responding, you can terminate it by executing in a new terminal, 

.. code-block:: bash

  sudo killall gzserver
  sudo killall gzclient

As the Gazebo is up and running, we can spawn a Jackal robot inside the simulated environment. To achieve that, we will create a dedicated ROS launch file in ``ee106s25/launch`` folder and attach the following,   

.. code-block:: html
  
  <launch>
    <arg name="x" default="0" />
    <arg name="y" default="0" />
    <arg name="z" default="1" />
    <arg name="yaw" default="0" />
    <arg name="joystick" default="true" />

    <!-- Configuration of Jackal which you would like to simulate.
        See jackal_description for details. -->
    <arg name="config" default="front_laser" />

    <!-- Load Jackal's description, controllers, and teleop nodes. -->
    <include file="$(find jackal_description)/launch/description.launch">
      <arg name="config" value="$(arg config)" />
    </include>
    <include file="$(find jackal_control)/launch/control.launch" />
    <include file="$(find jackal_control)/launch/teleop.launch">
      <arg name="joystick" value="$(arg joystick)" />
    </include>

    <!-- Spawn Jackal -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
          args="-urdf -model jackal -param robot_description -x $(arg x) -y $(arg y) -z $(arg z) -R 0 -P 0 -Y $(arg yaw)" />

  </launch>

.. then create a ROS subscriber and try to collect the data from the raw pointcloud and check distances

This file will be the ROS launch file that we will use to start Gazebo and spawn a simulated Jackal robot inside the world.  Specifically, the variables `x,y,z` and `yaw` define the initial position and orientation values of the robot in the world. The 'joystick' variable enables the teleoperation of the Jackal robot through a joystick controller. This file can be saved with the name of ``jackal.launch`` file inside the ``ee106s25`` ROS package. To execute the ROS launch file, you can perform in a new terminal,

.. code-block:: bash

  roslaunch ee106s25 jackal.launch

After the execution of the above ROS launch file, you can use ``rviz`` to visualize the captured sensory information from the simulated Jackal robot. Place objects closely around the simulated robot and check the `front/scan` ROS topic of type `sensor_msgs/LaserScan` on how it updates the visualized information in RViz. 


Robot Teleoperation
-----------------

.. rosrun, rostopic, rosmsg, rosnode, rosbag

Nowadays, most of the robots support teleoperation through a connected keyboard or a gamepad/joystick. In ROS we can use the `teleop_twist_keyboard <https://github.com/ros-teleop/teleop_twist_keyboard>`_ to use our keyboard for robot teleoperation, which publishes `geometry_twist/Twist` message on the ``cmd_vel`` ROS topic. 

To install this package, 

.. code-block:: bash

  cd ~/catkin_ws/src/
  git clone https://github.com/ros-teleop/teleop_twist_keyboard.git

and build the catkin workspace.

Jackal Control inside the Gazebo World
----------
As we have completed the above steps, to spawn the simualted Jackal inside the Gazebo world we execute in separate terminals the below commands in the following order,

#. roslaunch gazebo_ros empty_world.launch
#. roslaunch ee106s25 jackal.launch
#. rosrun teleop_twist_keyboard teleop_twist_keyboard.py

RViz program can be executed also in a separate terminal, in case you want to visualize the sensory information that is captured by the simulated Jackal robot.

Submission
----------

.. roscore
.. roslaunch gazebo_ros empty_world.launch
.. roslaunch ee106s25 jackal.launch
.. rviz
.. rosrun teleop_keyboard_. .. 


#. Submission: individual submission via Gradescope

In this lab's submission, we will develop a ROS node that will receive the simulated Jackal LiDAR information and will notify the user if the robot is getting closer to an obstacle through a ROS topic publication. 

.. #. Demo: required (Demonstrate the ROS node functionality in the Gazebo world by using the Jackal.)

#. Due time: 11:59pm, May 1, Monday

#. Files to submit: 

   - lab2_report.pdf (A template .pdf is provided for the report.) **Please include screenshots were possible and describe in detail all followed steps by showing the reasoning and any important remarks.** The developed Python code can be included in the end of your report.

#. Grading rubric:

   - \+ 10% Create a new Gazebo world with various obstacles.
   - \+ 10% Create a new `ROS node <https://github.com/UCR-Robotics/ee106/tree/main/scripts/rangescheck_jackal.py>`_ that will subscribe to the robot's LiDAR ROS topic. The ROS message type of the LiDAR ROS Topic is the `sensor_msgs/LaserScan <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html>`_.
   - \+ 10% Include a ROS publisher inside the newly created ROS node to publish a `std_msgs/String  <http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html>`_ ROS message over a new ROS topic named `jackal_robot_status`.
   - \+ 20% Create a new Python function inside the ROS node to iterate over the LiDAR's captured distances, namely the variable `float32[] ranges` of the message `sensor_msgs/LaserScan`.


   - \+ 20% Use the ROS publisher to publish a `std_msgs/String` through `jackal_robot_status` ROS topic, including the message,
      - ``critical`` if any of the `ranges` is smaller than `0.2m`
      - ``major`` if any of the `ranges` is smaller than `0.5m` 
      - ``minor`` if all `ranges > 0.5m` 
   - \+ 20% Demonstrate the ROS node functionality, by teleoperating the Jackal inside the Gazebo world and showcasing the transmitted ROS topic messages for each of the three cases. Specifically, include a screenshot of the published messages of `jackal_robot_status` by using ``rostopic`` in a new terminal and take a photo of the robot inside the Gazebo at the corresponding moment by showing the surrounding obstacles, for each of the three cases.
   - \+ 10% Include a screenshot of the RViz while using the Jackal in the Gazebo world, having the RobotModel, TF, and the LiDAR visualization enabled. 
   - \- 15% Penalty applies for each late day (up to two days). 


Reading Materials
-----------------

ROS Nodes
~~~~~~~~~

- `Understanding ROS Nodes <http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes>`_

- `Initialization and Shutdown <http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown>`_

ROS Topics and Messages
~~~~~~~~~~~~~~~~~~~~~~~

- `Messages <http://wiki.ros.org/Messages>`_

- `Understanding ROS Topics <http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics>`_

- `Publishers and Subscribers <http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers>`_

ROS Conventions
~~~~~~~~~~~~~~~

- `REP 103 Standard Units of Measure and Coordinate Conventions 
  <https://www.ros.org/reps/rep-0103.html>`_

- `REP 105 Coordinate Frames for Mobile Platforms <https://www.ros.org/reps/rep-0105.html>`_



.. Solution Approach for Lab 2 Assignment
.. -----------------


.. .. code-block:: python

..   #!/usr/bin/env python3

..   import rospy
..   import sys
..   import numpy as np
..   from sensor_msgs.msg import LaserScan
..   from std_msgs.msg import String

..   class ranges_check:
      
..     def __init__(self):
..       #
..       # Initialize the ROS publisher and subscriber. Use "self." to initialize the publisher and subscriber variables, to be able to access them through all class methods. The function "callback" will be the callback of the ROS subscriber. 
..       #
..       rospy.Subscriber("front/scan", LaserScan, self.callback)
..       self.pub = rospy.Publisher("jackal_robot_status", String, queue_size=10)

..     def callback(self,data):

..       # Add code here to iterate over all values in LaserScan ranges[] field and check the criticality of the robot position. Additionally, initialize a String variable that will contain the criticality message.
..       #
      
..       # initialize the counter variables for each criticality level
..       counter_minor = 0
..       counter_major = 0
..       counter_critical = 0
      
..       for r in data.ranges:
..         if str(r)=="inf":
..           continue

..         # else check criticality

..         if r < 0.2:
..           counter_critical = counter_critical + 1
..         elif r < 0.5:
..           counter_major = counter_major + 1
..         else:
..           counter_minor = counter_minor + 1
          
..         str_msg = String()
..         if counter_critical > 0:
..           str_msg.data = "critical"
..         elif counter_major > 0:
..           str_msg.data = "major"
..         elif counter_minor > 0:
..           str_msg.data = "minor"
..         else: 
..           str_msg.data = "no obstacle"
        
..         # Publish the String through the created ROS publisher variable...
..         #
..         self.pub.publish(str_msg.data)
      
..   def main(args):
..       ## initialization of the class object
..       rospy.init_node('ranges_check', anonymous=True)
..       ic = ranges_check()
..       try:
..           rospy.spin()
..       except KeyboardInterrupt:
..           print("Shutting down")
          
..   if __name__ == '__main__':
..       main(sys.argv)




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



RViz and TF Visualisation
-----------

Initially, we start the Gazebo simulator with the simulated Jackal robot. Thus, in separate terminals execute,

.. code-block:: bash
 
 roslaunch gazebo_ros empty_world.launch

and 

.. code-block:: bash
 
 roslaunch ee106s25 jackal.launch

As the robot has been successfully spawned inside the Gazebo world, we can enable the ROS visualization tool, by executing in a separate terminal,

.. code-block:: bash
 
 rviz

In order to visualize the robot in the RViz tool, we have to set firstly the visualization `Global Options/Fixed Frame` to any of the listed coordinate systems. For our setup, we set `Global Options/Fixed Frame` to ``base_link``, as it represents the base coordinate system of the Jackal robot. Also, we by using the `Add` button of the left panel of RViz, we add the visualization of the `TF`, the `RobotModel`, and the `LaserScan` by selecting, the latter, to visualize the `front/scan` ROS topic. Finally, to include an obstacle inside the Gazebo world, we add a `Stop Sign` model at the position (2,0,0).

 .. image:: ./pics/jackal_stop_sign.png
 :align: center


 .. image:: ./pics/jackal_stop_sign_rviz.png
 :align: center


ROS Frames and TF Listener
-----------

`ROS frames <http://wiki.ros.org/tf2>`_ are fundamental entities in ROS, as they represent the existing coordinate systems of the robotic setup. Particularly, ROS frames can be assigned on any part of the robot, which can be considered rigid, as well as, on any onboard sensor. Thus, each captured measurement can be spatially described in the corresponding ROS frame of the capturing sensor, while multiple frames can be connected to each other spatially and form the ROS frame tree of the ROS setup.

 .. image:: ./pics/jackal_frames.jpg
 :align: center

In order to publish a transformation between two ROS frames that remains static over time, you can use the tool `static_transform_publisher` from the `tf` ROS package. For example in our case, it would be ideal to create a frame for the front bumper of the Jackal, so we can spatially describe all captured ranging measurements in respect to it to avoid any potential collisions as it moves forward.

To do that, we can describe the new `front_bumper` frame, with respect to the `base_link` frame of the robot, by executing in a new terminal, 

.. code-block:: bash

 rosrun tf static_transform_publisher 0.26 0 0.11 0 0 0 1 base_link front_bumper 100

where the arguments of this command are, 

.. code-block:: bash

 static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period(milliseconds)

One of the terminal commands to obtain the `transformation matrix` between two ROS frames is,

.. code-block:: bash

 rosrun tf tf_echo base_link front_laser

This information can be captured also inside a ROS node by using the ``tf.TransformListener()`` module. To test it, please initialize a new ROS node inside the ``ee106s25`` ROS package, under the name of ``tf_listener.py``, which will contain,

.. code-block:: python

 #!/usr/bin/env python
 import roslib
 roslib.load_manifest('ee106s25')
 import rospy
 import math
 import tf
 import geometry_msgs.msg
 import numpy as np

 # initialization of the ROS tf listener
 listener = tf.TransformListener()

 rate = rospy.Rate(10.0)
 # the goal of this node is to continously listen to the transformation relation between the base_link and front_laser ROS frames and print the Translation and Rotation of the captured transformation matrix.
 while not rospy.is_shutdown():
    try:
        # capture the tf of the two frames the exact moment of the command execution (rospy.Time(0))
        (trans,rot) = listener.lookupTransform('/base_link', '/front_laser', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    # print of the Translation and Rotation information, by demonstrating the Quaternion, Euler, and Rotation Matrix representation of the latter.
    print("The translation is (x,y,z) = " + str(trans))
    print("The rotation (quaternion) is (x,y,z,w) = " + str(rot))
    print("The rotation (euler) is (r,p,y) = " + str(tf.transformations.euler_from_quaternion(rot)))
    rot_mat = tf.transformations.quaternion_matrix(rot)
    print("The rotation (rotation matrix) is = " + str(tf.transformations.quaternion_matrix(rot)))
    
    # we assume that a Lidar point is detected, w.r.t the Lidar's frame
    laser_point_detected = [1, 0, 0, 1]
    
    # initialization of the tf matrix to describe it in the /base_link frame
    rot_mat[0,3] = trans[0]
    rot_mat[1,3] = trans[1]
    rot_mat[2,3] = trans[2]
    print(np.dot(rot_mat , laser_point_detected))
    
    rate.sleep()


.. roscore
.. roslaunch gazebo_ros empty_world.launch
.. roslaunch ee106s25 jackal.launch
.. rviz
.. rosrun teleop_keyboard_. .. 


Submission
----------

#. **How**: individual, via Gradescope  
#. **Demo**: required—teleoperate the Jackal in Gazebo and show your node responding to obstacles  
#. **When**: 11:59 pm, Sunday, May 7  
#. **What to submit**:  
   - ``lab23_report.pdf`` (use the provided template)  
   - Include all screenshots, detailed step descriptions, and your fully commented Python code at the end  

Demo Checklist
--------------

- Show the Jackal driving toward obstacles  
- In a separate terminal, run ``rostopic echo /jackal_robot_status`` and demonstrate “critical”, “major”, and “minor” messages  
- Open RViz with the RobotModel, TF, and LaserScan displays enabled  

Grading Rubric
--------------

**10 % – Gazebo world & frame setup**  
   - Create a world that includes your Jackal, a Stop Sign (from Lab 3), and at least three distinct obstacles.  
   - Define and broadcast a new frame called ``front_bumper`` (e.g. via a static_transform_publisher or in your launch file).  

**15 % – ROS node initialization**  
   - Initialize a ROS node (e.g. ``ranges_check``).  
   - Subscribe to the ``/front/scan`` topic (``sensor_msgs/LaserScan``).  
   - Create a TF listener and successfully retrieve the transform between ``/front_laser`` and ``/front_bumper``.  

**20 % – LiDAR data processing & coordinate transformation**  
   - Iterate over ``data.ranges``, skip “inf” values.  
   - Convert valid range + angle into a point in the ``front_laser`` frame.  
   - Map each point into the ``front_bumper`` frame using your 4×4 transform matrix.  
   - Classify each mapped range as:  
     - **critical** if < 0.2 m  
     - **major** if < 0.5 m  
     - **minor** otherwise  

**20 % – ROS publisher & status messages**  
   - Form a ``std_msgs/String`` whose ``data`` is “critical”, “major”, or “minor” based on the worst‐case reading.  
   - Publish to ``/jackal_robot_status`` once per scan.  

**25 % – Final demonstration & RViz visualization**  
   - Teleoperate through all three cases (minor, major, critical).  
   - For each:  
     - Gazebo screenshot (robot + obstacles)  
     - Terminal screenshot of ``rostopic echo /jackal_robot_status``  
     - RViz screenshot with RobotModel, TF frames, and LaserScan display  

**10 % – Report clarity & code quality**  
   - Clear, concise write-up with figure captions.  
   - Comments explaining each major code block.  
   - Discussion of any challenges or design decisions.  

**–15 % per late day** (up to two days)

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

Submission
----------

#. **How**: individual, via Gradescope  
#. **Demo**: required—teleoperate the Jackal in Gazebo and show your node responding to obstacles  
#. **When**: 11:59 pm, Thursday, May 1  
#. **What to submit**:  
   - ``lab2_report.pdf`` (use the provided template)  
   - Include all screenshots, detailed step descriptions, and your fully commented Python code at the end  

Demo Checklist
--------------

- Show the Jackal driving toward obstacles  
- In a separate terminal, run ``rostopic echo /jackal_robot_status`` and demonstrate “critical”, “major”, and “minor” messages  
- Open RViz with the RobotModel, TF, and LaserScan displays enabled  

Grading Rubric
--------------

**10 % – Gazebo world & frame setup**  
   - Create a world that includes your Jackal, a Stop Sign (from Lab 3), and at least three distinct obstacles.  
   - Define and broadcast a new frame called ``front_bumper`` (e.g. via a static_transform_publisher or in your launch file).  

**15 % – ROS node initialization**  
   - Initialize a ROS node (e.g. ``ranges_check``).  
   - Subscribe to the ``/front/scan`` topic (``sensor_msgs/LaserScan``).  
   - Create a TF listener and successfully retrieve the transform between ``/front_laser`` and ``/front_bumper``.  

**20 % – LiDAR data processing & coordinate transformation**  
   - Iterate over ``data.ranges``, skip “inf” values.  
   - Convert valid range + angle into a point in the ``front_laser`` frame.  
   - Map each point into the ``front_bumper`` frame using your 4×4 transform matrix.  
   - Classify each mapped range as:  
     - **critical** if < 0.2 m  
     - **major** if < 0.5 m  
     - **minor** otherwise  

**20 % – ROS publisher & status messages**  
   - Form a ``std_msgs/String`` whose ``data`` is “critical”, “major”, or “minor” based on the worst‐case reading.  
   - Publish to ``/jackal_robot_status`` once per scan.  

**25 % – Final demonstration & RViz visualization**  
   - Teleoperate through all three cases (minor, major, critical).  
   - For each:  
     - Gazebo screenshot (robot + obstacles)  
     - Terminal screenshot of ``rostopic echo /jackal_robot_status``  
     - RViz screenshot with RobotModel, TF frames, and LaserScan display  

**10 % – Report clarity & code quality**  
   - Clear, concise write-up with figure captions.  
   - Comments explaining each major code block.  
   - Discussion of any challenges or design decisions.  

**–15 % per late day** (up to two days)


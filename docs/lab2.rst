Lab 2: ROS Tools and Testing on a Simulated Robot
========================

Overview
--------

In this lab, we are going to test ROS tools that are developed for illustrating the ROS Nodes and Topic graph relations and visualizing the ROS Topics in real-time. ROS, as a tool-based framework, features a plethora of applications, which can also be found `here <https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html>`_.

Additionally, the `Turtlebot 4 <https://clearpathrobotics.com/turtlebot-4/>`_ mobile robot will be demonstrated in the class, along with the use of some of the ROS tools. Furthermore, the simulated version of the Turtlebot robot will be deployed in the Gazebo simulator and it will be compared with the real robot. A 2D LiDAR ranging scenario will be demonstrated and will be the base of this lab's assignment.

Installing Gazebo Simulator (Virtual Machine)
-----------

The Gazebo simulator is a broadly used open-source robotic simulator, that has been used for ROS application development before testing on robots in the real world.
To install the simulator platform and the Turtlebot4 packages on the **Virtual Machines**, we need to perform,

.. code-block:: bash

  sudo apt-get install ros-jazzy-turtlebot4-simulator ros-jazzy-irobot-create-nodes


ROS Launch files
----------

Before we proceed with the experiment on the Turtlebot robot and the ROS tools, we need to describe the ROS launch file functionality.
Particularly, a ROS launch file can be a ``Python`` or ``XML`` type file with a ``.launch.py`` (Python) or ``.launch.xml`` file (XML) extension,
which can be used to launch multiple ROS nodes at the same time while it can set values for the parameters.
The ROS launch files can are kept in the ``launch`` folder of each ROS package.

An example of a launch file could be,

.. code-block:: python

  from launch import LaunchDescription
  from launch.actions import DeclareLaunchArgument
  from launch.substitutions import LaunchConfiguration
  from launch_ros.actions import Node


  def generate_launch_description():
      return LaunchDescription([
          DeclareLaunchArgument('x_var', default_value='0'),
          DeclareLaunchArgument('enable_var', default_value='true'),

          # This is a comment
          Node(package='package_name', executable='node_filename', name='node_name'),

          Node(package='package_name', executable='node_filename_2', name='node_name_2',
               arguments=['-x', LaunchConfiguration('x_var'),
                          '-enable_var', LaunchConfiguration('enable_var')]),
      ])

where,

.. code-block:: python

  DeclareLaunchArgument('x_var', default_value='0'),
  DeclareLaunchArgument('enable_var', default_value='true'),

is the initialization of the variable ``x_var`` with the value 0 and the variable ``enable_var`` with True. The code part of,

.. code-block:: python

  Node(package='package_name', executable='node_filename', name='node_name')

executes the node ``node_filename`` from the package ``package_name`` and names it ``node_name``. The second node call part, namely,

.. code-block:: python

  Node(package='package_name', executable='node_filename_2', name='node_name_2',
               arguments=['-x', LaunchConfiguration('x_var'),
                          '-enable_var', LaunchConfiguration('enable_var')])

executes the ``node_filename_2`` node, but also provides arguments directly to the program.

To install the launch files, add the following lines to `CMakeLists.txt` before `ament_package()`:

.. code-block:: cmake

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

and build the package using `colcon build`.

To execute the ROS launch file you can perform in a new terminal,

.. code-block:: bash

  ros2 launch package_name file.launch.py

Now, let's try to create a launch file for our created ROS package, namely the ``ee106f26``.
Specifically, create a ROS launch that you can execute at the same time both the `publisher` and `subscriber` nodes of the Lab 1.
Show the results to the Teaching Assistant.

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

  rviz2

Gazebo Simulation and the Clearpath Turtlebot
--------------

In order to start the Gazebo simulator with an empty world, you can execute, 

.. code-block:: bash

  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf

To properly exit or terminate Gazebo you should use the window terminating button. In many cases, such as closing abruptly the terminal or if the Gazebo is not responding, you can terminate it by executing in a new terminal, 

.. code-block:: bash

  pkill -f "gz sim"

As the Gazebo is up and running, we can spawn a Turtlebot inside the simulated environment. To achieve that, we will create a dedicated ROS launch file in ``ee106f26/launch`` folder and attach the following,   

.. code-block:: python
  
  from ament_index_python.packages import get_package_share_directory

  from launch import LaunchDescription
  from launch.actions import DeclareLaunchArgument
  from launch.actions import IncludeLaunchDescription
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


  ARGUMENTS = [
      DeclareLaunchArgument('namespace', default_value='',
                            description='Robot namespace'),
      DeclareLaunchArgument('rviz', default_value='false',
                            choices=['true', 'false'], description='Start rviz.'),
      DeclareLaunchArgument('world', default_value='warehouse',
                            description='Simulation World'),
      DeclareLaunchArgument('model', default_value='standard',
                            choices=['standard', 'lite'],
                            description='Turtlebot4 Model'),
  ]

  for pose_element in ['x', 'y', 'z', 'yaw']:
      ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                      description=f'{pose_element} component of the robot pose.'))


  def generate_launch_description():
      # Directories
      pkg_turtlebot4_gz_bringup = get_package_share_directory(
          'turtlebot4_gz_bringup')
      pkg_ee106f26 = get_package_share_directory(
        'ee106f26')

      # Paths
      gazebo_launch = PathJoinSubstitution(
          [pkg_turtlebot4_gz_bringup, 'launch', 'sim.launch.py'])
      robot_spawn_launch = PathJoinSubstitution(
          [pkg_ee106f26, 'launch', 'turtlebot_spawn.launch.py'])

      gazebo = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([gazebo_launch]),
          launch_arguments=[
              ('world', LaunchConfiguration('world'))
          ]
      )

      # Includes the launch description of the robot
      # This loads the models of the components, and any other packages
      # required for spawning the robot
      robot_spawn = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([robot_spawn_launch]),
          launch_arguments=[
              ('namespace', LaunchConfiguration('namespace')),
              ('rviz', LaunchConfiguration('rviz')),
              ('x', LaunchConfiguration('x')),
              ('y', LaunchConfiguration('y')),
              ('z', LaunchConfiguration('z')),
              ('yaw', LaunchConfiguration('yaw'))]
      )

      # Create launch description and add actions
      ld = LaunchDescription(ARGUMENTS)
      ld.add_action(gazebo)
      ld.add_action(robot_spawn)
      return ld

.. then create a ROS subscriber and try to collect the data from the raw pointcloud and check distances

This file will be the ROS launch file that we will use to start Gazebo and spawn a simulated Turtlebot robot inside the world.
Specifically, the variables `x,y,z` and `yaw` define the initial position and orientation values of the robot in the world.
This file can be saved with the name of ``turtlebot.launch.py`` file inside the ``ee106f26`` ROS package.
**NOTE**: An accompanying launch file is required for launching the Turtlebot in simulation. This file will spawn the individual components
of the Turtlebot. It is provided `here <https://raw.githubusercontent.com/UCR-Robotics/ee106/refs/heads/Fall2026/launch/turtlebot_spawn.launch.py>`_.
Download this file and place it in the ``launch`` directory as ``turtlebot_spawn.launch.py``.
To execute the ROS launch file, you can perform in a new terminal,

.. code-block:: bash

  ros2 launch ee106f26 turtlebot.launch.py model:=lite world:=empty

After the execution of the above ROS launch file, you can use ``rviz`` to visualize the captured sensory information from the simulated Turtlebot robot.
Place objects closely around the simulated robot and check the `/scan` ROS topic of type `sensor_msgs/LaserScan` on how it updates the visualized information in RViz. 


Robot Teleoperation
-----------------

.. rosrun, rostopic, rosmsg, rosnode, rosbag

Nowadays, most of the robots support teleoperation through a connected keyboard or a gamepad/joystick.
In ROS we can use the `teleop_twist_keyboard <https://docs.ros.org/en/ros2_packages/humble/api/teleop_twist_keyboard/>`_ to use our keyboard for robot teleoperation, which publishes `geometry_msgs/msg/Twist` message on the ``cmd_vel`` ROS topic. 

To install this package, 

.. code-block:: bash

  cd ~/ros2_ws/src/
  git clone https://github.com/ros-teleop/teleop_twist_keyboard.git

and build the catkin workspace.

Turtlebot Control inside the Gazebo World
----------
As we have completed the above steps, to spawn the simualted Turtlebot inside the Gazebo world we execute in separate terminals the below commands in the following order,

.. code-block:: bash

  ros2 launch ee106f26 turtlebot.launch.py model:=lite world:=maze # In terminal 1
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_unstamped # In terminal 2

RViz program can be executed also in a separate terminal, in case you want to visualize the sensory information that is captured by the simulated Turtlebot robot.


RViz and TF Visualisation
-----------

Initially, we start the Gazebo simulator with the simulated Turtlebot robot. Thus, in a terminal execute,

.. code-block:: bash
 
 ros2 launch ee106f26 turtlebot.launch.py model:=lite world:=maze

As the robot has been successfully spawned inside the Gazebo world with a maze, we can enable the ROS visualization tool, by executing in a separate terminal,

.. code-block:: bash
 
 rviz2

In order to visualize the robot in the RViz tool, we have to set firstly the visualization `Global Options/Fixed Frame` to any of the listed coordinate systems.
For our setup, we set `Global Options/Fixed Frame` to ``base_link``, as it represents the base coordinate system of the Turtlebot.
Also, by using the `Add` button of the left panel of RViz, we add the visualization of the `TF` and the `LaserScan` by selecting, the latter, to visualize the `/scan` ROS topic.

 .. image:: ./pics/turtlebot_maze.png
 :align: center


 .. image:: ./pics/turtlebot_tf.png
 :align: center


ROS Frames and TF Listener
-----------

`ROS frames <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html>`_ are fundamental entities in ROS, as they represent the existing coordinate systems of the robotic setup.
Particularly, ROS frames can be assigned on any part of the robot, which can be considered rigid, as well as, on any onboard sensor.
Thus, each captured measurement can be spatially described in the corresponding ROS frame of the capturing sensor, while multiple frames can be connected to each other spatially and form the ROS frame tree of the ROS setup.

 .. image:: ./pics/turtlebot_frames.png
..  :align: center

In order to publish a transformation between two ROS frames that remains static over time, you can use the tool `static_transform_publisher` from the `tf2_ros` ROS package.
For example in our case, it would be ideal to create a frame for the front bumper of the Turtlebot, so we can spatially describe all captured ranging measurements in respect to it to avoid any potential collisions as it moves forward.

To do that, we can describe the new `front_bumper` frame, with respect to the `base_link` frame of the robot, by executing in a new terminal, 

.. code-block:: bash

 ros2 run tf2_ros static_transform_publisher --x 0.26 --y 0 --z 0.11 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_link --child-frame-id front_bumper

where the arguments of this command are, 

.. code-block:: bash

 ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id

One of the terminal commands to obtain the `transformation matrix` between two ROS frames is,

.. code-block:: bash

 ros2 run tf2_ros tf2_echo base_link front_bumper

This information can be captured also inside a ROS node by using the ``tf2_ros.TransformListener()`` module.
Some helper functions are also available to convert the transformation quaternion to a rotation matrix using the ``tf_transformations`` library.
The library can be installed using the following commands. In a new terminal window, execute the following commands:

.. code-block:: bash

 cd ~/ee106_ws/src
 git clone https://github.com/DLu/tf_transformations.git
 cd ~/ee106_ws/
 rosdep update
 rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
 colcon build

To test it, please initialize a new ROS node inside the ``ee106f26`` ROS package, under the name of ``tf_listener.py``, which will contain,

.. code-block:: python

  #!/usr/bin/env python3

  import math

  import numpy as np
  import rclpy
  from rclpy.node import Node
  from rclpy.time import Time
  from tf2_ros import TransformException
  from tf2_ros.buffer import Buffer
  from tf2_ros.transform_listener import TransformListener

  from tf_transformations import euler_from_quaternion, quaternion_matrix


  class TfListener(Node):

      def __init__(self):
          super().__init__('tf_listener')
          self.tf_buffer = Buffer()
          self.tf_listener = TransformListener(self.tf_buffer, self)
          self.timer = self.create_timer(1.0, self.timer_callback)

      def timer_callback(self):
          try:
              # capture the latest available tf of the two frames (Time() means "latest")
              t = self.tf_buffer.lookup_transform('base_link', 'turtlebot4/rplidar_link/rplidar', Time())
          except TransformException as ex:
              self.get_logger().info('Could not get transform: %s' % ex)
              return

          trans = t.transform.translation
          rot = t.transform.rotation

          # print of the Translation and Rotation information, by demonstrating the Quaternion, Euler, and Rotation Matrix representation of the latter.
          print("The translation is (x,y,z) = " + str((trans.x, trans.y, trans.z)))
          print("The rotation (quaternion) is (x,y,z,w) = " + str((rot.x, rot.y, rot.z, rot.w)))
          print("The rotation (euler) is (r,p,y) = " + str(euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])))
          rot_mat = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
          print("The rotation (rotation matrix) is = " + str(rot_mat))

          # we assume that a Lidar point is detected, w.r.t the Lidar's frame
          laser_point_detected = [1, 0, 0, 1]

          # initialization of the tf matrix to describe it in the base_link frame
          rot_mat[0, 3] = trans.x
          rot_mat[1, 3] = trans.y
          rot_mat[2, 3] = trans.z
          print(np.dot(rot_mat, laser_point_detected))


  def main(args=None):
      rclpy.init(args=args)

      tf_listener = TfListener()

      rclpy.spin(tf_listener)

      tf_listener.destroy_node()
      rclpy.shutdown()


  if __name__ == '__main__':
      main()


Submission
----------

#. **How**: individual, via Gradescope  
#. **Demo**: required—teleoperate the Turtlebot in Gazebo and show your node responding to obstacles  
#. **When**: 11:59 pm, Thursday, Oct 29  
#. **What to submit**:  
   - ``lab2_report.pdf`` (use the provided template)  
   - Include all screenshots, detailed step descriptions, and your fully commented Python code at the end  

Demo Checklist
--------------

- Show the Turtlebot driving toward obstacles   
- Open RViz with the TF and LaserScan displays enabled  

Grading Rubric
--------------

- \+ 10 % Gazebo world & frame setup  
  - Spawn a world containing Turtlebot with the maze.  
  - Define and broadcast a new frame called ``front_bumper`` (e.g. via ``static_transform_publisher``).  

- \+ 15 % ROS node initialization  
  - Initialize a ROS node (e.g. ``ranges_check``).  
  - Subscribe to ``/scan`` (``sensor_msgs/LaserScan``).  
  - Create a TF listener and retrieve the transform between ``/turtlebot4/rplidar_link/rplidar`` and ``/front_bumper``.  

- \+ 20 % LiDAR data processing & coordinate transformation  
  - Iterate over ``data.ranges``, skipping “inf” values.  
  - Convert each valid range+angle into a point in the ``turtlebot4/rplidar_link/rplidar`` frame.  
  - Use your 4×4 transform to map points into the ``front_bumper`` frame.  
  - Classify each mapped range as **critical** (< 0.2 m), **major** (< 0.5 m), or **minor** (≥ 0.5 m).  

- \+ 20 % ROS publisher & status messages  
  - Form a ``std_msgs/String`` whose ``data`` is the worst-case classification.  
  - Publish once per scan to ``/turtlebot_status``.  
  - In a separate terminal, run ``ros2 topic echo /turtlebot_status`` and demonstrate “critical”, “major”, and “minor” messages 

- \+ 25 % Final demonstration & RViz visualization  
  - Teleoperate through all three cases (minor, major, critical).  
  - For each case include:  
    - Gazebo screenshot (robot + obstacles)  
    - Terminal screenshot of ``ros2 topic echo /turtlebot_status``  
    - RViz screenshot with RobotModel, TF frames, and LaserScan  

- \+ 10 % Report clarity & code quality  
  - Clear, concise write-up with captions.  
  - Comments explaining each major code block.  
  - Discussion of challenges or design decisions.  

- \- 15 % per late day (up to two days)


Pseudo-code
--------------
.. code-block:: python

    #!/usr/bin/env python3

    import math

    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

    from tf_transformations import quaternion_matrix


    class Lab2TemplateNode(Node):

        def __init__(self):
            super().__init__('ranges_check')
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.pub = None # TODO: Publisher for worst case status
            self.sub = None # TODO: Subscriber for LiDAR

        def callback(self, scan: LaserScan):
            # Get the transformations and rotations
            try:
                t = self.tf_buffer.lookup_transform(
                    'base_link', 'turtlebot4/rplidar_link/rplidar', Time()
                )
            except TransformException:
                self.get_logger().warn('TF not available yet')
                return

            trans = t.transform.translation
            rot = t.transform.rotation

            # Build a 4x4 homogeneous transform matrix T
            T = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
            T[0:3, 3] = [trans.x, trans.y, trans.z]

            # Loop over all ranges, skip the inf ranges
            worst = 'minor'
            for idx, r in enumerate(scan.ranges):
                if not np.isfinite(r):
                    continue

                # transform each point
                theta = scan.angle_min + idx * scan.angle_increment
                x_l = r * math.cos(theta)
                y_l = 0.0 # TODO: Complete
                p_l = 0.0 # TODO: Complete

                # transform into base_link frame
                p_b = 0.0 # TODO: Complete

                # compute distance (in meters)
                dist = math.hypot(p_b[0], p_b[1])

                # COMPLETE CODE….
                # classify critical, major, minor

            # PUBLISH worst


    def main(args=None):
        rclpy.init(args=args)

        node = Lab2TemplateNode()

        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

Reading Materials
-----------------

ROS Nodes
~~~~~~~~~

- `Understanding ROS Nodes <https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html>`_

- `Initialization and Shutdown <https://docs.ros.org/en/jazzy/p/rclpy/api/init_shutdown.html>`_

ROS Topics and Messages
~~~~~~~~~~~~~~~~~~~~~~~

- `Interfaces <https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html>`_

- `Understanding ROS Topics <https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html>`_

- `Publishers and Subscribers <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_

ROS Conventions
~~~~~~~~~~~~~~~

- `REP 103 Standard Units of Measure and Coordinate Conventions 
  <https://www.ros.org/reps/rep-0103.html>`_

- `REP 105 Coordinate Frames for Mobile Platforms <https://www.ros.org/reps/rep-0105.html>`_





Lab 1: ROS Nodes, Topics, and Messages
====================

Overview
--------

In this lab, we are going to initialize ROS workspace, create our first ROS package, and create two communicating ROS Nodes.

While following the step-by-step tutorials, please take your time to think about 
what you are doing and what happens in each step.

Creation of ROS Package
----------

From now on, we assume that you have already installed Ubuntu 20.04 and ROS Noetic.

- Please open a new terminal, and create a new ROS workspace by the following commands (run them line by line).

.. code-block:: bash

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src 
    catkin_init_workspace
    cd ../
    catkin_make
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/catkin_ws/devel/setup.bash

- Take a look at ``catkin_ws`` directory and see what happens. 
  You can use ``ls`` command to see the files in this directory, or use ``ls -a`` to see all files including hidden files.
  Alternatively, you can open File Explorer and navigate to this folder, and use ``Ctrl + H`` to see hidden files.
  The file ``.catkin_workspace`` was created to tell the system that this current directory is a ROS workspace.

.. note::

  If you are fresh new to Linux, the instructions might be a bit hard to understand at this moment.
  No worries. Please just try it for the time being and you will have a better understanding as we move on.
  You can always ask me any question you want during lab sessions to help you better understand lab materials. 
  (Remember that there is no stupid question; even for those "too simple to ask" questions, We are always happy to answer.)

- Next let's create a new ROS package.

.. code-block:: bash
      
    cd ~/catkin_ws/src
    catkin_create_pkg ee106s24 std_msgs rospy

- Take a look at your new package ``ee106s25`` and see what happens. You should be able to see a ``package.xml`` file
  and a ``CMakeLists.txt`` file. Open them and take a quick look. 
  You may use Google to help you build up a high-level understanding.

- After creating a new package, we can go back to our workspace and build this package.
  This is to tell ROS that "Hey, we have a new package here. Please register it into the system."

.. code-block:: bash
      
    cd ~/catkin_ws
    catkin_make

- Now the system knows this ROS package, so that you can have access to it anywhere. 
  Try navigating to different directories first, and then go back to this ROS package by ``roscd`` command.
  See what happens when running the following commands.

.. code-block:: bash
      
    cd
    roscd ee106s25

    cd ~/catkin_ws
    roscd ee106s25
      
    cd ~/Documents
    roscd ee106s25

- Congratulations. You have initialized the ROS workspace and created the ee106s24 ROS package!
  Take some time to think about how the above steps work. 

  
ROS Publisher and Subcriber Python Nodes
----------


  
The next step is to head to our  `ROS tutorial`_ and create the ROS publisher and subscriber nodes. The Python scripts can be saved under the ``ee106s25/src/`` folder. To be able to use the developed ROS python nodes, you need to provide execution permissions by,

.. code-block:: bash

    roscd ee106s25/src/
    
    touch publisher.py 
    # copy and paste the publisher code in this file
    chmod +x publisher.py

    touch subscriber.py
    # copy and paste the subscriber code in this file
    chmod +x subscriber.py

To execute the created ROS nodes, firstly you need to launch the ROS core in a new terminal by doing,

.. code-block:: bash

    roscore

Then create two separate terminals, and execute,

.. code-block:: bash

    rosrun ee106s25 publisher.py

and

.. code-block:: bash

    rosrun ee106s25 subscriber.py

By performing these commands you have successfully created and executed your first ROS application, on which you transfer string data through a ROS topic from the ``talker`` to the ``listener`` ROS node. To preview the transmitted information through the ``chatter`` ROS topic, you can use,

.. code-block:: bash

    rostopic echo /chatter

The `ROS wiki <http://wiki.ros.org/ROS/Tutorials>`_ and `rospy <http://wiki.ros.org/rospy_tutorials>`_ contain the  analytic documentation of the followed steps.

.. _ROS tutorial: https://ucr-robotics.readthedocs.io/en/latest/intro_ros.html

Creation of Custom ROS Message
----------

As mentioned in the class, ROS features a simplified message description language for describing the data values that ROS nodes publish. In our example, we will create a new ROS message, named "EE106lab_custom", which will be described by the variables,

.. code-block:: bash

    Header header
    int32 int_data
    float32 float_data
    string string_data

To create this new message type, initially create a folder ``msg`` inside the ``ee106s25`` ROS package. Additionally, create a file ``EE106lab_custom.msg`` inside the created ``msg`` folder, by containing the information depicted above. 

To be able to use the new ROS message type, we need to indicate its creation to the ROS workspace and compile it. To achieve this, fistly you need to update the package.xml of ``ee106s25`` and make sure these two lines are in it,

.. code-block:: python

  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>

Additionally, to indicate this modification to the cmake compiler, you need to update the line of CMakeLists.txt of ``ee106s25`` package to contain the message_generation,

.. code-block:: python

  # Update the existing line
  find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

and uncomment this block,

.. code-block:: python

  # add_message_files(
  #   FILES
  #   Message1.msg
  #   Message2.msg
  # )

to modify it like,

.. code-block:: python

  add_message_files(
    FILES
    EE106lab_custom.msg
  )

to include the newly created ``msg`` type. Similarly, add the ``message_generation`` at ``catkin_package`` field, as appeared bellow,

.. code-block:: python

  catkin_package(
    ...
    CATKIN_DEPENDS rospy std_msgs message_runtime
    ...)

and uncomment the ``generate_messages`` cmake field. By performing ``catkin_make`` under the ``~\catkin_ws\`` directory the ROS package is compiled and  the ``EE106lab_custom.msg`` can be used by any node of any package, as soon as the depedencies are satisfied. This ``msg`` structure will be utilized and tested in the submission part of Lab 1. More information about the previous steps can be found in the official `ROS msg page  <http://wiki.ros.org/msg>`_.


Submission
----------

#. Submission: individual submission via Gradescope

#. Demo: required (Present the subscriber's additions results in real-time.)

#. Due time: 11:59pm, Apr 24, Thursday

#. Files to submit: 

   - lab1_report.pdf (A template .pdf is provided for the report. Please include the developed Python code in your report.)

#. Grading rubric:

   - \+ 20%  Create a new ROS publisher and subscriber node (python scripts). You can use the Python scripts provided at the ee106 class `repository <https://github.com/UCR-Robotics/ee106/tree/main/scripts>`_.
   - \+ 20%  Create a new ROS message type, named ``EE106lab_custom_new.msg``, that contains a Header and two int32 variables and save it in the ``msg`` folder. Build the ROS workspace following the above steps.
   - \+ 10% Import the ``EE106lab_custom_new.msg`` in both publisher and subscriber scripts.
   - \+ 10% Update the publisher ROS node to send a ROS topic named ``EE106lab_topic``, of ``EE106lab_custom_new`` msg type. Send random integers over the ROS topic and update the header with the corresponding timestamp. For the random integer generator you can use ``random.randint(a,b)`` function from the `random <https://www.w3schools.com/python/ref_random_randint.asp>`_ python library.
   - \+ 10% Update the subscriber ROS node to receive the ``EE106lab_topic`` and print the addition of the two int32 variables and the Header timestamp information during the callback. 
   - \+ 30%  Write down your lab report, by including comments and screenshots of the following steps, along with terminal results and important findings.


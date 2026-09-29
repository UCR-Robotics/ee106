Lab 0: Preparation
==================

General Information
-------------------

Here are some general ideas that can help you get prepared for this lab section.

When working with robots, we need skills in Linux, ROS, Python, Git/GitHub,
and Virtual Machine (VM).

1. The best way to learn **Linux** is to spend time playing with it.
   It's just like the first time you had your Windows/Mac computer.
   Additionally, you may follow some tutorials online
   and try those commonly used commands in terminals. 
   For example, it is recommended to go over chapter 1-3 of 
   `this tutorial <http://swcarpentry.github.io/shell-novice/>`_.

2. `Robot Operating System (ROS) <https://www.ros.org/>`_
   is a Linux software library specifically designed for programming on robots.
   It has some features of OS, but is not actually an independent OS.
   With ROS, people do not need to worry about low-level communications and
   keep reinventing the wheel.
   On top of ROS, developers all over the world can work on their own software
   packages, and contribute to ROS community.
   These packages are similar to those libraries that we "import" in Python.

3. A good way to learn **ROS** is to learn from `ROS wiki <https://docs.ros.org/en/jazzy/Tutorials.html>`_,
   which provides official tutorials. For this course, ROS 2 is preferred.
   Note that some tutorials on the ROS wiki are written in C++.
   Please focus on high-level design ideas and `rclpy <https://docs.ros.org/en/jazzy/p/rclpy/>`_
   library only (not rclcpp), since we will use Python (instead of C++) in this class.

4. For **Python**, basically you need to have a rough idea about data structures,
   operators, flow control, etc. There are also many good tutorials online.
   For example, the tutorial on `W3Schools <https://www.w3schools.com/python/>`_.
   Going through the first 20 sections (until Python Functions) would be sufficient for this class.

5. **Git** is a version control tool and `GitHub <https://github.com/>`_
   is a website (or company) that offers Git-based version control service.
   It's good to learn Git in the sense that you can better manage your code.
   With Git, you can see all your change history, and have backups of each version.
   Many ROS packages that we are going to use in this course are hosted on GitHub.
   However, it's not strictly required in this class. 
   Going through chapter 1-5 of `this tutorial <http://swcarpentry.github.io/git-novice/>`_ 
   might be helpful.

6. For **Virtual Machine**, there are mainly two kinds of software available online.
   One is `VMware <https://www.vmware.com/>`_ and the other is
   `VirtualBox <https://www.virtualbox.org/>`_.
   The former has better utilization of GPU and hence supports better graphics, 
   but it is not free of charge.
   The good news is that in recent years VMware has released a free "Player" version for
   individual users, which we will discuss later.
   On the other hand, VirtualBox is totally open source (free) for all platforms (Windows, Mac, Linux).
   However, it does not perform well in heavy simulation tasks in Gazebo.
   (Gazebo is a simulator that we are going to use throughout the course, together with ROS.)


Please familiarize yourself with the above concepts/tools, if they are new to you.

In the following, we will go through some basic steps to get our development environment ready.

Virtual Machine
---------------

- If you have a Linux laptop, or you can dual boot with Linux operating system,
  that's great! This is the best way to work on robots.
  (Please be careful about dual boot, since you have to take potential risks.
  We recommend using VMware instead.)

- If you have a Windows laptop, please go for
  `VMware Workstation Player <https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html>`_.
  Note that this version 16 or 17 should be free.

- If you have a Mac laptop, please go to `VMware Fusion <https://www.vmware.com/products/fusion.html>`_
  webpage, register under "Get a Free Personal Use License" tab and download **VMware Fusion Player**
  using a free personal license.


Install Linux
-------------

Once you have your VMware installed, let's create a new VM and install Ubuntu 24.04.

- Download Ubuntu 24.04 disc image from
  `official website <https://releases.ubuntu.com/noble/>`_ (64-bit PC Desktop).

- For mac users: If you are using a Mac with M architecture, download `Ubuntu 24.04 server image <https://cdimage.ubuntu.com/releases/20.04/release/ubuntu-20.04.5-live-server-arm64.iso>`_.

- In VMware, create a new VM.

  + Typical configuration
  + Choose the disc image you download
  + In case you're using **VirtualBox**, please uncheck the "unattended install" option.
  + Enter some information about this VM
  + Again, enter name
  + Please allocate at least 30GB (preferred 50GB or more)
  + Store virtual disk as a single file
  + Customize Hardware: Please allocate more memory and CPU processors for better performance
  + Finish

- Great. Now you have a (virtual) Linux computer. Take your time and play with it!

Note that the disk size 30GB/50GB will not be allocated instantly,
but will grow gradually as you add more stuff.


Install ROS
-----------

Once you are familiar with Linux, you can start installing ROS.
In general, we need to follow ROS
`installation tutorial <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>`_.
Main steps are

- Setup **locale**

   .. code-block:: bash

     locale  # check for UTF-8

     sudo apt update && sudo apt install locales
     sudo locale-gen en_US en_US.UTF-8
     sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
     sudo export LANG=en_US.UTF-8

     locale  # verify settings

- Add the repository to the list of sources that Ubuntu can query when looking for packages:

   .. code-block:: bash

     sudo apt install software-properties-common
     sudo add-apt-repository universe

- Set up the official ROS repository on Ubuntu so that the packages can be installed:

   .. code-block:: bash

     sudo apt update && sudo apt install curl -y
     export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
     curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
     sudo dpkg -i /tmp/ros2-apt-source.deb

- Install additional tools for ROS development:

   .. code-block:: bash

     sudo apt update && sudo apt install ros-dev-tools

- Update the system and install ROS 2.0 Jazzy.

   .. code-block:: bash

     sudo apt update
     sudo apt upgrade
     sudo apt install ros-jazzy-desktop

- The following snippet will find ROS commands and tools every time you login or open a new terminal window:

   .. code-block:: bash

     echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
     source ~/.bashrc

- Initialize rosdep and update:

   .. code-block:: bash

     sudo rosdep init
     rosdep update




Fixes to some common errors
----------------------------
- # represents a comment. Do not copy/paste the comments to the terminal when executing the instructions.

- If you see a red circle similar to the 'do not enter' sign on the top-right corner, click that circle, select show updates and click 'install now' before installing ROS to avoid any dependency issues.

- If there is no copy/paste in the VM, run the VM as administrator.

Learn from ROS Tutorials
---------------------------

Once you have ROS Noetic installed, we provide `the tutorial for ROS`_. You can also follow the tutorials
on `ROS wiki <https://docs.ros.org/en/jazzy/Tutorials.html>`_ and
`rclpy <https://docs.ros.org/en/jazzy/p/rclpy/>`_ documentation.

.. _the tutorial for ROS: https://ucr-robotics.readthedocs.io/en/latest/intro_ros.html

Have fun!


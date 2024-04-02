Reference Materials
===================


Navigation Stack
----------------

- `Main Page <http://wiki.ros.org/navigation>`_

- `Navigation Stack Tutorials <http://wiki.ros.org/navigation/Tutorials>`_

- `Robot Setup <http://wiki.ros.org/navigation/Tutorials/RobotSetup>`_

- `Using RViz with the Navigation Stack 
  <http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack>`_

- `Navigation Tuning Guide <http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide>`_


VS Code IDE
-----------

- The recommended IDE in Linux is `VS Code <https://code.visualstudio.com>`_.
  Just go to the official website, download the ``.deb`` file and install it. 

- After installation, you can go to Extensions and search for ROS. 
  Adding this extension can help you highlight the code and bring more convenience. 


Atom Editor
-----------

``gedit`` is the default editor available in Ubuntu OS,
which sometimes is hard to use.
Here I personally recommend a lightweight editor called `Atom <https://atom.io/>`_.
It has a couple good features, such as auto-completion, file system browser,
Tab key detection, and so on. 

- To install Atom, first add the official repository to the package list.

  .. code-block:: bash

    wget -qO - https://packagecloud.io/AtomEditor/atom/gpgkey | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=amd64] https://packagecloud.io/AtomEditor/atom/any/ any main" > /etc/apt/sources.list.d/atom.list'
    
- Then update your apt package list and install Atom.

  .. code-block:: bash
    
    sudo apt-get update
    sudo apt-get install atom

- After installation, you can open Atom by just typing atom in your terminal.

  .. code-block:: bash
    
    atom

- You can right click the Atom icon on taskbar and choose "Lock to Launcher",
  such that you can run it by just clicking the icon on taskbar next time.

- You can customize some settings in Atom. Go to the menu bar of Atom.
  Open settings by clicking on "edit" first, and then "preferences".
  Go to Editor, then you can change font size and tab length 
  (how many spaces you want to have when you press a Tab key).
  This is really useful especially when working with Python,
  since you have to indent each line of the block by the same whitespace,
  in order to indicate a block of code in Python.

- To use Atom for your project, just go to the menu bar, select "File",
  then select "Open Folder" or "Add Project Folder", and then open your ROS workspace or ROS package.
  With this, you can have a tree directory on the left side, and do not need
  to switch back and forth in the terminal or file manager.


USB WiFi Adapter
----------------

In order to have bi-directional communications between your laptop and the robot, 
we use a USB WiFi adapter binding your VM to a physical MAC address, 
rather than sharing the network of your host computer.

- Open a new terminal, download the driver and install it by the following commands.

  .. code-block:: bash

    cd ~/Downloads
    wget https://github.com/UCR-Robotics/ee144/raw/wifi-adapter-driver/RTL88x2BU_WiFi_linux_v5.3.1.zip
    unzip RTL88x2BU_WiFi_linux_v5.3.1.zip
    cd RTL88x2BU_... [press Tab key to complete]
    chmod +x install.sh
    sudo ./install.sh

- Restart your computer by one more command.

  .. code-block:: bash

    sudo reboot

- Plug in your adapter. 
  If you can see the flickering blue light on your adapter, 
  then you are good.

.. note::

  Sometimes the system upgrade may disable the linux driver.
  The solution would be to just install the driver again.


Teamviewer Remote Login
-----------------------

- `Teamviewer <https://www.teamviewer.com/en-us/>`_ is a remote login 
  (`VNC <https://en.wikipedia.org/wiki/Virtual_Network_Computing>`_) 
  software that can enable graphical user interface (GUI) remotely.
  Once setup, you can remote login to your robot with GUI, from any other computer and 
  and operating system. It looks just like you are working on that computer locally.

- To set up teamviewer on the onboard computer on a robot, first connect it to a monitor.
  This is necessary for installation and change settings in teamviewer later on.

- Open a new terminal, download the host-only version of teamviewer and install it.

  .. code-block:: bash

    cd ~/Downloads
    wget https://download.teamviewer.com/download/linux/teamviewer-host_amd64.deb
    sudo dpkg -i teamviewer-host_amd64.deb

.. note::

  If you do not have any graphics enabled on the current computer, you will get
  error messages in the installation. One error message could be like the following.

  .. code-block:: bash

    The following packages have unmet dependencies:
      qt56-teamviewer but it is not installable

  This is not really the absence of dependencies. 
  The actual reason is that you do not have proper graphics-related service/program/library enabled.
  The linux system will not launch some graphical programs if it knows that no monitor is connected.
  The solution could be using a dummy HDMI plug or connecting to a real monitor.
  The dummy HDMI plug can help with the installation, but cannot help with the settings in teamviewer later on.
  Therefore, here we can just connect to a real monitor.

- Launch teamviewer and go to the settings. We need to change two parts for future connection from other computers.

  .. code-block:: bash

    teamviewer

- Go to ``Extras``, and then ``Options``, change ``Incoming LAN connections`` to be ``accept exclusively``.
  With this, only the connection request from LAN is valid, which can help keep it safe from the outside world.

- Go to ``Security`` and set ``Personal password`` the same as your login password.

- Done. Later on you can remote login to this robot (onboard computer) from your Windows/MacOS laptop
  as long as you and the robot are in the same WiFi network. The partner ID is just the IP address of the robot,
  since we have already set up static IP for the robot.

.. note::

  When using remote login from other computer, it is better to keep dummy HDMI plug on the robot (host/onboard computer),
  because it can help simulate a monitor and enable some graphical tools. 
  Otherwise you may have some issues like fixed low resolution in display and whatnot.

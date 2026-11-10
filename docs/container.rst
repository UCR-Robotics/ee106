Container Setup
================

This project relies on the container provided by the UCR course-support repository:
`ucrcsedept/course-support <https://github.com/ucrcsedept/course-support>`_.

The container provides a pre-configured environment with all necessary software. If you install
the container, there should be no additional requirement for installing ROS.

For detailed installation instructions, refer to the official README:
`containers/cs265a <https://github.com/ucrcsedept/course-support/tree/main/containers/cs265a>`_.

Install `podman`
----------------

The instructions for installing `podman` can be found `here <https://podman.io/docs/installation>`_.

For Apple users, the binaries can be downloaded `here <https://podman.io/>_`.

Windows Users
-------------

`podman` on Windows runs inside WSL2. Open PowerShell **as Administrator** and enable WSL2, then restart
the computer:

   .. code-block:: powershell

     wsl.exe --install

After restarting, open a new PowerShell window and run the following commands. `podman-compose` requires
`Python <https://www.python.org/downloads/>`_ to be installed:

   .. code-block:: powershell

     winget install -e --id RedHat.Podman
     pip install podman-compose

Close and reopen PowerShell so the ``podman`` command is available.

Apple Users
-----------

You will need to first install `homebrew <https://brew.sh/>`_ if it's not installed. It's a package manager
that lets you install `podman` on the computer.

Once `homebrew` is installed, restart the terminal or open a new terminal window and run the following commands:

   .. code-block:: bash

     brew install podman
     brew install podman-compose


Steps to Set Up
---------------

1. Clone or download the repository:

   .. code-block:: bash

     git clone https://github.com/ucrcsedept/course-support

2. Navigate to the container directory:

   .. code-block:: bash

     cd course-support/containers/cs265a

3. Create a machine with some specifications. Note that these specifications will vary depending on the resources available on the computer:

   .. code-block:: bash

     podman machine init --cpus=4 --memory=8192 --disk-size=60 --rootful

  For windows users - ``wsl`` does not allow you to change the 

4. Change the properties of the machine. This is required because the simulations and assigments
   might require some compute power:

   .. code-block:: bash

     podman machine set --cpus=4        # Example 4 CPUs, please set it to a reasonable number according to the computer specifications
     podman machine set --memory=8192   # Example 8 GiB, please set it to a reasonable number according to the computer specifications

5. Start the machine:

   .. code-block:: bash

     podman machine start

6. Build the container:

   .. code-block:: bash

     podman compose build

7. Run the container:

   .. code-block:: bash

     podman compose up

Keep the terminal open while the container is running.

Accessing the Container
-----------------------

Once the container is running, open a browser and go to:

.. code-block:: bash

  http://127.0.0.1:6080/vnc.html

- Click **Connect**
- Enter password: ``password`` (default)

This will launch a Linux desktop environment where you can run applications such as Gazebo.

File Persistence
----------------

The container is non-persistent, meaning:

- Files outside ``/workspace`` will be lost after stopping the container
- Always save your work in:

  .. code-block:: bash

    /workspace

Restarting the Container
------------------------

To restart the container in a new session:

1. Start the Podman machine:

   .. code-block:: bash

     podman machine start

2. Navigate to the container directory:

   .. code-block:: bash

     cd C:\Users\YourName\Location\course-support\containers\cs265a

3. Run the container:

   .. code-block:: bash

     podman compose up

Keep the terminal open while the container is running.
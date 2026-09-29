Container Setup
================

This project relies on the container provided by the UCR course-support repository:
`ucrcsedept/course-support <https://github.com/ucrcsedept/course-support>`_.

The container provides a pre-configured environment with all necessary software.

For detailed installation instructions, refer to the official README:
`containers/ee106 <https://github.com/ucrcsedept/course-support/tree/main/containers/cs265a>`_.

Steps to Set Up
---------------

1. Clone or download the repository:

   .. code-block:: bash

     https://github.com/ucrcsedept/course-support

2. Navigate to the container directory:

   .. code-block:: bash

     containers/ee106

3. Build the container:

   .. code-block:: bash

     podman compose build

4. Run the container:

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

     cd C:\Users\YourName\Location\course-support\containers\ee106

3. Run the container:

   .. code-block:: bash

     podman compose up

Keep the terminal open while the container is running.
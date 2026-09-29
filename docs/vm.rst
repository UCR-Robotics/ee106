Virtual Machine Setup
=====================

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
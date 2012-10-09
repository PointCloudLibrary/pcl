.. _gpu_people:

Configuring your PC to use your Nvidia GPU with PCL
---------------------------------------------------
In this tutorial we will learn how to check if your PC is 
suitable for use with the GPU methods provided within PCL.
This tutorial has been tested on Ubuntu 11.04 and 12.04, let
us know on the user mailing list if you have tested this on other
distributions.

The explanation
---------------

In order to run the code you'll need a decent Nvidia GPU with Fermi or Kepler architecture you can check this by::

 $ lspci | grep nVidia

This should indicate which GPU you have on your system, if you don't have an Nvidia GPU, we're sorry, but you
won't be able to use PCL GPU.
The output of this you can compare with `this link <http://www.nvidia.co.uk/object/cuda_gpus_uk.html>`_  
on the Nvidia website, your card should mention compute capability of 2.x (Fermi) or 3.x (Kepler) or higher.
If you want to run with a GUI, you can also run::

 $ nvidia-settings

From either CLI or from your system settings menu. This should mention the same information.

First you need to test if your CPU is 32 or 64 bit, if it is 64-bit, it is preferred to work in this mode.
For this you can run::

  $ lscpu | grep op-mode

As a next step you will need a up to date version of the Cuda Toolkit. You can get this 
`here <http://developer.nvidia.com/cuda/cuda-downloads>`_, at the time of writing the
latest version was 4.2 and the beta release of version 5 was available as well.

First you will need to install the latest video drivers, download the correct one from the site
and run the install file, after this, download the toolkit and install it.

You can get the SDK, but for PCL this is needed, this provides you with general CUDA examples
and some scripts to test the performance of your CPU as well as your hardware specifications.


Tested Hardware
---------------
Please report us the hardware you have tested the following methods with.

+-----------------------+---------------------+----------------+
| Method                | Hardware            | Reported FPS   |
+=======================+=====================+================+
| Kinfu                 | GTX680              | 20-27          |
|                       | C2070               | 29             |
+-----------------------+---------------------+----------------+
| People Pose Detection | GTX680              | 20-23          |
|                       | C2070               | 10-20          |
+-----------------------+---------------------+----------------+



.. _gpu_install:

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
The output of this you can compare with `this link <http://www.nvidia.co.uk/object/cuda-parallel-computing-uk.html>`_  
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
At the moment of writing this was version 295.41, please choose the most up to date one::

  $ wget http://developer.download.nvidia.com/compute/cuda/4_2/rel/drivers/devdriver_4.2_linux_64_295.41.run

Make the driver executable::

 $ chmod +x devdriver_4.2_linux_64_295.41.run

Run the driver::

 $ sudo ./devdriver_4.2_linux_64_295.41.run

You need to run this script without your X-server running. You can shut your X-server down as follows:
Go to a terminal by pressing Ctrl-Alt-F1 and typing::

 $ sudo service gdm stop

Once you have installed you GPU device driver you will also need to install the CUDA Toolkit::

 $ wget http://developer.download.nvidia.com/compute/cuda/4_2/rel/toolkit/cudatoolkit_4.2.9_linux_64_ubuntu11.04.run
 $ chmod +x cudatoolkit_4.2.9_linux_64_ubuntu11.04.run
 $ sudo ./cudatoolkit_4.2.9_linux_64_ubuntu11.04.run
 
You can get the SDK, but for PCL this is not needed, this provides you with general CUDA examples
and some scripts to test the performance of your CPU as well as your hardware specifications.

CUDA only compiles with gcc 4.4 and lower, so if your default installed gcc is 4.5 or higher you'll need to install gcc 4.4:

 $ sudo apt-get install gcc-4.4

Now you need to force your gcc to use this version, you can remove the older version, the other option is to create a symlink in your home folder and include that in the beginning of your path:

 $ cd
 $ mkdir bin

Add 'export PATH=$HOME/bin:$PATH as the last line to your ~/.bashrc file.
Now create the symlinks in your bin folder:

 $ cd ~/bin
 $ ln -s <your_gcc_installation> c++
 $ ln -s <your_gcc_installation> cc
 $ ln -s <your_gcc_installation> g++
 $ ln -s <your_gcc_installation> gcc

If you use colorgcc these links all need to point to /usr/bin/colorgcc.

Now you can get the latest git master (or another one) of PCL and configure your
installation to use the CUDA functions.

Go to your PCL root folder and do::

 $ mkdir build; cd build
 $ ccmake ..

Press c to configure ccmake, press t to toggle to the advanced mode as a number of options
only appear in advanced mode. The latest CUDA algorithms are beeing kept in the GPU project, for
this the BUILD_GPU option needs to be on and the BUILD_gpu_<X> indicate the different
GPU subprojects.

.. image:: images/gpu/gpu_ccmake.png
    :width: 400 pt

Press c again to configure for you options, press g to generate the makefiles and to exit. Now
the makefiles have been generated successfully and can be executed by doing::

 $ make

If you want to install your PCL installation for everybody to use::

 $ make install

Now your installation is finished!

Tested Hardware
---------------
Please report us the hardware you have tested the following methods with.

+-----------------------+----------------------------------------------------------------------+----------------+
| Method                | Hardware                                                             | Reported FPS   |
+=======================+======================================================================+================+
| Kinfu                 | GTX680, Intel Xeon CPU E5620 @ 2.40Ghz x 8, 24Gb RAM                 | 20-27          |
+-----------------------+----------------------------------------------------------------------+----------------+
|                       | GTX480, Intel Xeon CPU W3550 @ 3.07GHz × 4, 7.8Gb RAM                | 40             |
+-----------------------+----------------------------------------------------------------------+----------------+
|                       | C2070, Intel Xeon CPU E5620 @ 2.40Ghz x 8, 24Gb RAM                  | 29             |
+-----------------------+----------------------------------------------------------------------+----------------+
| People Pose Detection | GTX680, Intel Xeon CPU E5620 @ 2.40Ghz x 8, 24Gb RAM                 | 20-23          |
+-----------------------+----------------------------------------------------------------------+----------------+
|                       | C2070, Intel Xeon CPU E5620 @ 2.40Ghz x 8, 24Gb RAM                  | 10-20          |
+-----------------------+----------------------------------------------------------------------+----------------+



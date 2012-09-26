.. _gpu_people:

Detecting people and their poses using PointCloud Library
---------------------------------------------------------

In this tutorial we will learn how detect a person in a pointcloud. This is based on work from Koen Buys, Cedric Cagniart and Anatoly Bashkeev.
This shows how to detect people with an Primesense device, the full version working on oni and pcd files can be found in trunk.

The code
--------

The full version of this code can be found in PCL trunk/gpu/people/tools, the following is a reduced version for the tutorial.
First, create a file, let's say, ``people_detect.cpp`` in your favorite editor, and place the following inside it:

.. literalinclude:: sources/gpu_people/people_detect.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece. Starting from the main routine.

.. literalinclude:: sources/gpu_people/people_detect.cpp
   :language: cpp
   :lines: 11-27

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/gpu_people/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do:

  $ ./people_detect


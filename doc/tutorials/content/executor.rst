.. _executor:

Using Executors
--------------------------------------------------

Executors help in providing fine-grained control to the programmer to choose where and how
a piece of program will execute. All functions supporting the executors have a uniform API
for running the program in multiple ways.

In this tutorial, we will learn how to manually create and pass executors to a FunctorFilter.

.. note::
   Not all functions support executors. Check the Code API for more info on which functions are supported.


The code
--------

First, create a file, let's say, ``executor.cpp``, and place the following inside it:

.. literalinclude:: sources/executor/executor.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

In the following lines, we create a Point Cloud structure, then fill the
input cloud using `CloudGenerator`. The generator uses 128 as a seed value to uniformly fill
the input cloud with a point cloud having width & height as 200. Each point is generated
with x,y & z coordinates in the range [-20, 20].

.. literalinclude:: sources/executor/executor.cpp
   :language: cpp
   :lines: 8-12

Then, we create two executors. The first one, `simple_executor`, is of type `inline_executor`
which runs code sequentially on the CPU. The second one, `parallel_executor`, is of type
`omp_executor`, which uses OMP to parallelize the execution. We set the max_threads that OMP
can utilize to 4. If unspecified explicitly by default, all the threads available in the system
will be used.

.. literalinclude:: sources/executor/executor.cpp
   :language: cpp
   :lines: 14-16


We then create two FunctorFilter instances. The first filter `radius_filter` is used to filter out
any points which are at a distance greater than the specified `radius_filter` from the origin.
The second filter, `positive_y_filter`, filters out all points having negative y coordinates.

.. literalinclude:: sources/executor/executor.cpp
   :language: cpp
   :lines: 18-39

Finally, we call `filter` with the respective executors.

For `radius_filter` we pass the `parallel_executor` which will create 4 OMP contexts and split
the filtering of points between them.


.. literalinclude:: sources/executor/executor.cpp
   :language: cpp
   :lines: 41-42

For `positive_y_filter` we pass the `simple_executor` which will simply filter the points in the existing
thread.

.. literalinclude:: sources/executor/executor.cpp
   :language: cpp
   :lines: 43-43

We can then visualize the final filtered point cloud using the `PCLVisualizer`.

.. literalinclude:: sources/executor/executor.cpp
   :language: cpp
   :lines: 45-57

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/executor/CMakeLists.txt
   :language: cmake
   :linenos:


After you have made the executable, you can run it. Simply do:

  $ ./executor

Below are the three visualizations of the point cloud at each phase.
The leftmost point cloud in the shape of a cube is the input cloud.
The center image shows a point cloud in the shape of a sphere. It is the
output after applying `radius_filter`.
The rightmost image shows a point cloud in the shape of a hemisphere. It is the
output after applying `positive_y_filter` to the point cloud output from `radius_filter`.

.. image:: images/executor_cube.png
   :width: 30%

.. image:: images/executor_sphere.png
   :width: 30%

.. image:: images/executor_hemisphere.png
   :width: 30%

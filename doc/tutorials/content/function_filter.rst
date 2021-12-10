.. _function_filter:

Removing outliers using a custom non-destructive condition
----------------------------------------------------------

This document demonstrates how to use the FunctionFilter class to remove points from a PointCloud that do not satisfy a custom criteria. This is a cleaner
and faster appraoch compared to ConditionalRemoval filter or a `custom Condition class <https://cpp-optimizations.netlify.app/pcl_filter/>`_.

.. note::
   Advanced users can use the FunctorFilter class that can provide a small but measurable speedup when used with a `lambda <https://en.cppreference.com/w/cpp/language/lambda>`_.

The code
--------

First, create a file, let's say, sphere_removal.cpp in you favorite editor, and place the following inside it:

.. literalinclude:: sources/function_filter/sphere_removal.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

In the following lines, we define the PointCloud structures, fill in the input cloud, and display its content to screen.

.. literalinclude:: sources/function_filter/sphere_removal.cpp
   :language: cpp
   :lines: 10-21

Then, we create the condition which a given point must satisfy so that it remains in our PointCloud. To do this we create a `std::function` which accepts a PointCloud by const reference and an index, and returns true only if the point lies inside a sphere. This is then used to build the filter

.. literalinclude:: sources/function_filter/sphere_removal.cpp
   :language: cpp
   :lines: 23-34

This last bit of code just applies the filter to our original PointCloud, and removes all of the points that do not satisfy the conditions we specified. Then it outputs all of the points remaining in the PointCloud.

.. literalinclude:: sources/function_filter/sphere_removal.cpp
   :language: cpp
   :lines: 36-42

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/function_filter/CMakeLists.txt
   :language: cmake
   :linenos:

After you have compiled the executable, you can run it. Simply do::

  $ ./sphere_removal

You will see something similar to::

  Cloud before filtering:
      -1.23392 1.81505 -0.968005
      -0.00934529 1.36497 0.158734
      0.488435 1.96851 -0.0534078
      1.27135 1.16404 -1.00462
      -0.249089 -0.0815883 1.13229
      0.448447 1.48914 1.78378
      1.14143 1.77363 1.68965
      1.08544 -1.01664 -1.13041
      1.1199 0.9951 -1.13308
      1.44268 -1.44434 -0.391739
  Cloud after filtering:
      -1.23392 1.81505 -0.968005
      -0.00934529 1.36497 0.158734
      0.488435 1.96851 -0.0534078
      1.27135 1.16404 -1.00462
      1.14143 1.77363 1.68965
      1.08544 -1.01664 -1.13041
      1.1199 0.9951 -1.13308
      1.44268 -1.44434 -0.391739

.. _concatenate_points:

Concatenate the points or the fields of two Point Clouds
--------------------------------------------------------

In this tutorial we will learn how to concatenating the points of two different
point clouds. The constraint imposed here is that the type and number of fields
in the two datasets have to be equal.

The code
--------

First, create a file, let's say, ``concatenate_points.cpp`` in your favorite
editor, and place the following code inside it:

.. literalinclude:: sources/concatenate_points/concatenate_points.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

In lines:

.. literalinclude:: sources/concatenate_points/concatenate_points.cpp
   :language: cpp
   :lines: 8-29

we define the three Point Clouds: two inputs (cloud_a and cloud_b), one output
(cloud_c), and fill in the data for the two input point clouds.

Then, lines:

.. literalinclude:: sources/concatenate_points/concatenate_points.cpp
   :language: cpp
   :lines: 31-37

display the content of cloud_a and cloud_b to screen.

In line:

.. literalinclude:: sources/concatenate_points/concatenate_points.cpp
   :language: cpp
   :lines: 39-41

we create cloud_c by concatenating the points of cloud_a and cloud_b together.

Finally:

.. literalinclude:: sources/concatenate_points/concatenate_points.cpp
   :language: cpp
   :lines: 43-45

is used to show the content of cloud_c.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/concatenate_points/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./concatenate_points

You will see something similar to::

  Cloud A: 
      0.352222 -0.151883 -0.106395
      -0.397406 -0.473106 0.292602
      -0.731898 0.667105 0.441304
      -0.734766 0.854581 -0.0361733
      -0.4607 -0.277468 -0.916762
  Cloud B: 
      0.183749 0.968809 0.512055
      -0.998983 -0.463871 0.691785
      0.716053 0.525135 -0.523004
  Cloud C: 
      0.352222 -0.151883 -0.106395 
      -0.397406 -0.473106 0.292602 
      -0.731898 0.667105 0.441304 
      -0.734766 0.854581 -0.0361733 
      -0.4607 -0.277468 -0.916762 
      0.183749 0.968809 0.512055 
      -0.998983 -0.463871 0.691785 
      0.716053 0.525135 -0.523004 


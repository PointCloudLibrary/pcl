.. _concatenate_fields:

Concatenate the fields of two Point Clouds
------------------------------------------

In this tutorial we will learn how to concatenating the fields (e.g.,
dimensions) of two different point clouds. The constraint imposed here is that
the number of points in the two datasets has to be equal.

The code
--------

First, create a file, let's say, ``concatenate_fields.cpp`` in your favorite
editor, and place the following code inside it:

.. literalinclude:: sources/concatenate_fields/concatenate_fields.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.


.. literalinclude:: sources/concatenate_fields/concatenate_fields.cpp
   :language: cpp
   :lines: 8-10

define the three Point Clouds: two inputs (cloud_a and cloud_b), one output
(cloud_c).

The lines:

.. literalinclude:: sources/concatenate_fields/concatenate_fields.cpp
   :language: cpp
   :lines: 12-30

fill in the data for the two input point clouds.

Then, lines:

.. literalinclude:: sources/concatenate_fields/concatenate_fields.cpp
   :language: cpp
   :lines: 32-38

display the content of cloud_a and cloud_b to screen.

In line:

.. literalinclude:: sources/concatenate_fields/concatenate_fields.cpp
   :language: cpp
   :lines: 40

we create cloud_c by concatenating the fields of cloud_a and cloud_b together.

Finally:

.. literalinclude:: sources/concatenate_fields/concatenate_fields.cpp
   :language: cpp
   :lines: 41-45

is used to show the content of cloud_c.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/concatenate_fields/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./concatenate_fields

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
      0.439387 0.56706 0.905417
      -0.579787 0.898706 -0.504929
  Cloud C: 
      0.352222 -0.151883 -0.106395 0.183749 0.968809 0.512055
      -0.397406 -0.473106 0.292602 -0.998983 -0.463871 0.691785
      -0.731898 0.667105 0.441304 0.716053 0.525135 -0.523004
      -0.734766 0.854581 -0.0361733 0.439387 0.56706 0.905417
      -0.4607 -0.277468 -0.916762 -0.579787 0.898706 -0.504929


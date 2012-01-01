.. _concatenate_clouds:

Concatenate the points of two Point Clouds
------------------------------------------

In this tutorial we will learn how to concatenating the points of two different
point clouds. The constraint imposed here is that the type and number of fields
in the two datasets has to be equal.  We will also learn how to concatenate the fields (e.g.,
dimensions) of two different point clouds. The constraint imposed here is that
the number of points in the two datasets has to be equal.

The code
--------

First, create a file, let's say, ``concatenate_clouds.cpp`` in your favorite
editor, and place the following code inside it:

.. literalinclude:: sources/concatenate_clouds/concatenate_clouds.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

In lines:

.. literalinclude:: sources/concatenate_clouds/concatenate_clouds.cpp
   :language: cpp
   :lines: 13-50

we define the five Point Clouds for use in concatenating clouds: three inputs (cloud_a, cloud_b and n_cloud_b), two outputs (cloud_c and p_n_cloud_c).  Then we fill in the data for the two input point clouds we are using (for points cloud_a and cloud_b, for fields cloud_a and n_cloud_b).

Then, lines:

.. literalinclude:: sources/concatenate_clouds/concatenate_clouds.cpp
   :language: cpp
   :lines: 51-61

display the content of cloud_a and either cloud_b or n_cloud_b (depending on the command line argument) to screen.

If we are trying to **concatenate points** then the code below:

.. literalinclude:: sources/concatenate_clouds/concatenate_clouds.cpp
   :language: cpp
   :lines: 66-67

creates cloud_c by concatenating the points of cloud_a and cloud_b together.

Otherwise if we are attempting to **concatenate fields** then the code below:

.. literalinclude:: sources/concatenate_clouds/concatenate_clouds.cpp
   :language: cpp
   :lines: 74-74

creates p_n_cloud_c by concatenating the fields of cloud_a and cloud_b together.

Finally either:

.. literalinclude:: sources/concatenate_clouds/concatenate_clouds.cpp
   :language: cpp
   :lines: 68-70

or

.. literalinclude:: sources/concatenate_clouds/concatenate_clouds.cpp
   :language: cpp
   :lines: 75-79

is used to show the content of cloud_c or p_n_cloud_c to the screen depending on if we concatenated the points or fields of the PointClouds.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/concatenate_clouds/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./concatenate_clouds -p

to concatenate points or do::

  $ ./concatenate_clouds -f

to concatenate fields.

You will see something similar to if concatenating points::

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

and similar to this if concatenating fields::

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


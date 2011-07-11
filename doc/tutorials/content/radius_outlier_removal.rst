.. _radius_outlier_removal:

Removing outliers using a RadiusOutlierRemoval filter
-----------------------------------------------------

This document demonstrates how to create and use a RadiusOutlierRemoval object that can be used to remove points from a PointCloud that do not have a given number of neighbors within a specific radius from their location.

The code
--------

First, create a file, let's say, radius_outlier_removal.cpp in your favorite editor, and place the following inside it:

.. literalinclude:: sources/radius_outlier_removal/radius_outlier_removal.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

In the following lines, we define the PointCloud structures, fill in the input cloud and display it's contents to the screen.

.. literalinclude:: sources/radius_outlier_removal/radius_outlier_removal.cpp
   :language: cpp
   :lines: 8-27

Then, we create the RadiusOutlierRemoval filter object, set it's parameters and apply it to our input cloud.  The radius of search is set to 0.8, and a point must have a minimum of 2 neighbors in that radius to be kept as part of the PointCloud.

.. literalinclude:: sources/radius_outlier_removal/radius_outlier_removal.cpp
   :language: cpp
   :lines: 29-35

This final block of code just displays the contents of the resulting PointCloud to the screen.

.. literalinclude:: sources/radius_outlier_removal/radius_outlier_removal.cpp
   :language: cpp
   :lines: 37-42

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/radius_outlier_removal/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./conditioinal_removal

You will see something similar to::

  Cloud before filtering: 
       0.352222 -0.151883 -0.106395
	   -0.397406 -0.473106 0.292602
	   -0.731898 0.667105 0.441304
	   -0.734766 0.854581 -0.0361733
	   -0.4607 -0.277468 -0.916762
  Cloud after filtering: 
	   -0.731898 0.667105 0.441304
	   -0.734766 0.854581 -0.0361733


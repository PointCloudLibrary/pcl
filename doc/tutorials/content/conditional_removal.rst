.. _conditional_removal:

Removing outliers using a ConditionalRemoval filter
---------------------------------------------------

This document demonstrates how to use the ConditionalRemoval filter to remove points from a PointCloud that do not satisfy a specific or multiple conditions.

The code
--------

First, create a file, let's say, conditional_removal.cpp in you favorite editor, and place the following inside it:

.. literalinclude:: sources/conditional_removal/conditional_removal.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

In the following Lines, we define the PointCloud structures, fill in the input cloud, and display it's content to screen.

.. literalinclude:: sources/conditional_removal/conditional_removal.cpp
   :language: cpp
   :lines: 8-27

Then, we create the condition which a given point must satisfy so that it remains in our PointCloud.  To do this we must add two comparisons to the condition, greater than 0.0, and less than 0.8. This condition is then used to build the filter. 

.. literalinclude:: sources/conditional_removal/conditional_removal.cpp
   :language: cpp
   :lines: 28-40

This last bit of code just applies the filter to our original PointCloud, and removes all of the points that do not satisfy the conditions we specified. Then it outputs all of the points remaining in the PointCloud.

.. literalinclude:: sources/conditional_removal/conditional_removal.cpp
   :language: cpp
   :lines: 42-50

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/conditional_removal/CMakeLists.txt
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
		-0.397406 -0.473106 0.292602
		-0.731898 0.667105 0.441304


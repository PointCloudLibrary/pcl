.. _BoxClipper3D:

Extracting points inside a box
------------------------------------

In this tutorial we will learn how to use the BoxClipper3D filter to extract a subset of 
points from a point cloud. In this example we will create a point cloud and some points will be inside the box and some on the outside. The way the BoxClipper3D works is counter intuitive. Instead of having to put the box somewhere and have the right inclination to fit the points we want to extract, we have to do the reverse operation on the cloud so the points we want to extract move inside the box. We will se how this works in this example.

The code
--------


Then, create a file, let's say, ``extract_points.cpp`` in your favorite
editor, and place the following inside it:

.. literalinclude:: sources/boxClipper3D/boxClipper3D_example.cpp
   :language: cpp
   :linenos:


The explanation
---------------

Here we define our simple point cloud with its inliers and outliers and we rotate the cloud 45 degree on its Z axis;

.. literalinclude:: sources/boxClipper3D/boxClipper3D_example.cpp
   :language: cpp
   :lines: 133-145

Now lets look at the magic.

.. literalinclude:: sources/boxClipper3D/boxClipper3D_example.cpp
   :language: cpp
   :lines: 23-46

We have to define three transformation: a translation, a rotation and a scaling factor;
.. literalinclude:: sources/boxClipper3D/boxClipper3D_example.cpp
   :language: cpp
   :lines: 27-31

If we think about the intuitive way: We have a point cloud that is rotated 45 degree on its Z axis.
Lets picture our self creating a box starting at (0,0,0). Then, we want to fit our inliers inside the box. So we have to translate the box by a vector of (1,1,1) and then rotate it of 45 degree on its Z axis and than scale the box to only fit the points we want. 

But as mentioned earlier the way the boxClipper3D works is the opposite. Therefore, we have to translate the cloud by a vector of (-1,-1,-1) and then rotate it of -45 degree on its Z axis so the points we want to extract fits inside the box. The box is 2x2x2 and it starts at (-1,-1,-1) and it ends at (1,1,1).

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/boxClipper3D/CMakeLists.txt
   :language: cmake
   :linenos: 

After you have made the executable, you can run it. Simply do::

  $ ./extract_points

You will see something similar to::

  nb inliers: 3


.. _range_image_creation:

How to create a range image from a point cloud
----------------------------------------------

This tutorial demonstrates how to create a range image from a point cloud and a given sensor position. The code creates an example point cloud of a rectangle floating in front of the observer.

The code
--------

First, create a file called, let's say, ``range_image_creation.cpp`` in your favorite
editor, and place the following code inside it:

.. literalinclude:: sources/range_image_creation/range_image_creation.cpp
   :language: cpp
   :linenos:

   
Explanation
-----------

Lets look at this in parts:

.. literalinclude:: sources/range_image_creation/range_image_creation.cpp
   :language: cpp
   :lines: 1-17

   
This includes the necessary range image header, starts the main and generates a point cloud that represents a rectangle.  


.. literalinclude:: sources/range_image_creation/range_image_creation.cpp
   :language: cpp
   :lines: 20-27

   
This part defines the parameters for the range image we want to create.

The angular resolution is supposed to be 1 degree, meaning the beams represented by neighboring pixels differ by one degree.

maxAngleWidth=360 and maxAngleHeight=180 mean that the range sensor we are simulating has a complete 360 degree view of the surrounding. You can always use this setting, since the range image will be cropped to only the areas where something was observed automatically. Yet you can save some computation by reducing the values. E.g. for a laser scanner with a 180 degree view facing forward, where no points behind the sensor can be observed, maxAngleWidth=180 is enough.

sensorPose defines the 6DOF position of the virtual sensor as the origin with roll=pitch=yaw=0.

coordinate_frame=CAMERA_FRAME tells the system that x is facing right, y downwards and the z axis is forward. An alternative would be LASER_FRAME, with x facing forward, y to the left and z upwards.

For noiseLevel=0 the range image is created using a normal z-buffer. Yet if you want to average over points falling in the same cell you can use a higher value. 0.05 would mean, that all points with a maximum distance of 5cm to the closest point are used to calculate the range.

If minRange is greater than 0, all points that are closer will be ignored.

borderSize greater than 0 will leave a border of unobserved points around the image when cropping it.


.. literalinclude:: sources/range_image_creation/range_image_creation.cpp
   :language: cpp
   :lines: 29-33

The remaining code creates the range image from the point cloud with the given parameters and outputs some information on the terminal.

The range image is derived from the PointCloud class and its points have the members x,y,z and range. There are three kinds of points. Valid points have a real range greater than zero. Unobserved points have x=y=z=NAN and range=-INFINITY. Far range points have x=y=z=NAN and range=INFINITY.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:


.. literalinclude:: sources/range_image_creation/CMakeLists.txt
   :language: cmake
   :linenos:
   
   
After you have made the executable, you can run it. Simply do::

  $ ./range_image_creation 

You should see the following::

  range image of size 42x36 with angular resolution 1deg/pixel and 1512 points

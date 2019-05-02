.. _project_inliers:

Projecting points using a parametric model
------------------------------------------

In this tutorial we will learn how to project points onto a parametric model
(e.g., plane, sphere, etc). The parametric model is given through a set of
coefficients -- in the case of a plane, through its equation: ax + by + cz + d
= 0.

The code
--------

First, create a file, let's say, ``project_inliers.cpp`` in your favorite
editor, and place the following inside it:

.. literalinclude:: sources/project_inliers/project_inliers.cpp
   :language: cpp
   :linenos:

   
The explanation
---------------

Now, let's break down the code piece by piece.

We first import the ModelCoefficients structure then the ProjectInliers filter.

.. literalinclude:: sources/project_inliers/project_inliers.cpp
   :language: cpp
   :lines: 4-5
   

We then create the point cloud structure, fill in the respective values, and
display the content on screen.

.. literalinclude:: sources/project_inliers/project_inliers.cpp
   :language: cpp
   :lines: 14-29

   
We fill in the ModelCoefficients values. In this case, we use a plane model,
with ax+by+cz+d=0, where a=b=d=0, and c=1, or said differently, the X-Y plane.

.. literalinclude:: sources/project_inliers/project_inliers.cpp
   :language: cpp
   :lines: 32-36

We create the ProjectInliers object and use the ModelCoefficients defined above
as the model to project onto. 


.. literalinclude:: sources/project_inliers/project_inliers.cpp
   :language: cpp
   :lines: 39-43

   
Finally we show the content of the projected cloud.


.. literalinclude:: sources/project_inliers/project_inliers.cpp
   :language: cpp
   :lines: 45-49

   
Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:


.. literalinclude:: sources/project_inliers/CMakeLists.txt
   :language: cmake
   :linenos:

   
After you have made the executable, you can run it. Simply do::

  $ ./project_inliers

You will see something similar to::

  Cloud before projection: 
      0.352222 -0.151883 -0.106395
      -0.397406 -0.473106 0.292602
      -0.731898 0.667105 0.441304
      -0.734766 0.854581 -0.0361733
      -0.4607 -0.277468 -0.916762
  Cloud after projection: 
      0.352222 -0.151883 0
      -0.397406 -0.473106 0
      -0.731898 0.667105 0
      -0.734766 0.854581 0
      -0.4607 -0.277468 0

A graphical display of the projection process is shown below.

.. image:: images/project_inliers_2.png

Note that the coordinate axes are represented as red (x), green (y), and blue
(z). The five points are represented with red as the points before projection
and green as the points after projection. Note that their z now lies on the X-Y
plane.

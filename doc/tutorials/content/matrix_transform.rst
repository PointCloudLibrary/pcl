.. _matrix_transform:

Using a matrix to transform a point cloud
-----------------------------------------

In this tutorial we will learn how to transform a point cloud using a 4x4 matrix.
We will apply a rotation and a translation to a loaded point cloud and display then
result.

This program is able to load one PCD or PLY file; apply a matrix transformation on it
and display the original and transformed point cloud. 

The code
--------

First, create a file, let's say, ``matrix_transform.cpp`` in your favorite
editor, and place the following code inside it:

.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :lines: 1-8

We include all the headers we will make use of.
**#include <pcl/registration/ia_ransac.h>** allows us to use **pcl::transformPointCloud** function.



.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :lines: 10-17

This function display the help in case the user didn't provide expected arguments.



.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :lines: 24-28

We parse the arguments on the command line, either using **-h** or **--help** will 
display the help. This terminates the program



.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :lines: 30-45

We look for .ply or .pcd filenames in the arguments. If not found; terminate the program.
The bool **file_is_pcd** will help us choose between loading PCD or PLY file.



.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :lines: 47-62

We now load the PCD/PLY file and check if the file was loaded successfuly. Otherwise terminate
the program.



.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :lines: 64-81

The interesting part: We define a 4x4 matrix for our transformation. We ititialize the matrix 
to indentity::

          |  1  0  0  0  |
      i = |  0  1  0  0  |
          |  0  0  1  0  |
          |  0  0  0  1  |

This means no transformation (null rotation and translation). We do not use the 
last row of the matrix. The first 3 rows and colums (top left) components are the rotation
matrix. The first 3 rows of the last column is the translation.

Here we defined a 45° (PI/4) rotation around the Z axis and a translation on the X axis.
This is the transformation we just defined ::

          |  cos(θ) -sin(θ)  0.0 |
      R = |  sin(θ)  cos(θ)  0.0 |
          |  0.0     0.0     1.0 |

      t = < 2.5, 0.0, 0.0 >


.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :lines: 83-89

Here we display the rotation matrix and the translation vector.



.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :lines: 91-92

Now we apply this matrix on the point cloud **source_cloud** and we save the result in the
 newly created **transformed_cloud**.



.. literalinclude:: sources/matrix_transform/matrix_transform.cpp
   :language: cpp
   :lines: 95-113

We then visualize the result using the **PCLVisualizer**. The original point cloud will be
displayed white and the transformed one in red. The coordoniates axis will be displayed.
We also set the background color of the visualizer and the point display size.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/matrix_transform/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./matrix_transform cube.ply

You will see something similar to this::

  ./matrix_transform cube.ply 
  [pcl::PLYReader] cube.ply:12: property 'list uint8 uint32 vertex_indices' of element 'face' is not handled
  [pcl::PLYReader] cube.ply:26: property 'float32 focal' of element 'camera' is not handled
  [pcl::PLYReader] cube.ply:27: property 'float32 scalex' of element 'camera' is not handled
  [pcl::PLYReader] cube.ply:28: property 'float32 scaley' of element 'camera' is not handled
  [pcl::PLYReader] cube.ply:29: property 'float32 centerx' of element 'camera' is not handled
  [pcl::PLYReader] cube.ply:30: property 'float32 centery' of element 'camera' is not handled
  [pcl::PLYReader] cube.ply:33: property 'float32 k1' of element 'camera' is not handled
  [pcl::PLYReader] cube.ply:34: property 'float32 k2' of element 'camera' is not handled
  
  This is the rotation matrix :
      |    0.7   -0.7    0.0 | 
  R = |    0.7    0.7    0.0 | 
      |    0.0    0.0    1.0 | 
  
  This is the translation vector :
  t = <    2.5,    0.0,    0.0 >
  
  Point cloud colors :	black	= original point cloud
  			red	= transformed point cloud

.. image:: images/matrix_transform/cube_big.png
  :height: 600

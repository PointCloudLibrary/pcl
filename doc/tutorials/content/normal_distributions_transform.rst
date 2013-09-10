.. _normal_distributions_transform:

How to use Normal Distributions Transform
-----------------------------------------

In this tutorial we will describe how to use the Normal Distributions Transform (NDT) algorithm to determine a rigid transformation between two large point clouds, both over 100,000 points.  The NDT algorithm is a registration algorithm that uses standard optimization techniques applied to statistical models of 3D points to determine the most probable registration between two point clouds.  For more information on the inner workings of the NDT algorithm, see Dr. Martin Magnusson's doctoral thesis, “The Three-Dimensional Normal Distributions Transform – an Efficient Representation for Registration, Surface Analysis, and Loop Detection.”

The code
--------

First, download the datasets `room_scan1.pcd
<https://raw.github.com/PointCloudLibrary/data/master/tutorials/room_scan1.pcd>`_ and `room_scan2.pcd
<https://raw.github.com/PointCloudLibrary/data/master/tutorials/room_scan2.pcd>`_ and save them to your disk.  These point clouds contain 360 degree scans of the same room from different perspectives.

Then, create a file in your favorite editor and place the following inside.  I used ``normal_distributions_transform.cpp`` for this tutorial.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's breakdown this code piece by piece.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 5-6

These are the required header files to use Normal Distributions Transform algorithm and a filter used to down sample the data.  The filter can be exchanged for other filters but I have found the approximate voxel filter to produce the best results.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 14-30

The above code loads the two pcd file into pcl::PointCloud<pcl::PointXYZ> boost shared pointers.  The input cloud will be transformed into the reference frame of the target cloud.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 32-39

This section filters the input cloud to improve registration time.  Any filter that downsamples the data uniformly can work for this section.  The target cloud does not need be filtered because voxel grid data structure used by the NDT algorithm does not use individual points, but instead uses the statistical data of the points contained in each of its data structures voxel cells.  

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 41-42

Here we create the NDT algorithm with the default values.  The internal data structures are not initialized until later.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 44-50


Next we need to modify some of the scale dependent parameters.  Because the NDT algorithm uses a voxelized data structure and More-Thuente line search, some parameters need to be scaled to fit the data set.  The above parameters seem to work well on the scale we are working with, size of a room, but they would need to be significantly decreased to handle smaller objects, such as scans of a coffee mug.  

The Transformation Epsilon parameter defines minimum, allowable,  incremental change of the transformation vector, [x, y, z, roll, pitch, yaw] in meters and radians respectively.  Once the incremental change dips below this threshold, the alignment terminates.  The Step Size parameter defines the maximum step length allowed by the More-Thuente line search.  This line search algorithm determines the best step length below this maximum value, shrinking the step length as you near the optimal solution.  Larger maximum step lengths will be able to clear greater distances in fewer iterations but run the risk of overshooting and ending up in an undesirable local minimum.  Finally, the Resolution parameter defines the voxel resolution of the internal NDT grid structure.  This structure is easily searchable and each voxel contain the statistical data, mean, covariance, etc., associated with the points it contains.  The statistical data is used to model the cloud as a set of multivariate Gaussian distributions and allows us to calculate and optimize the probability of the existence of points at any position within the voxel.  This parameter is the most scale dependent.  It needs to be large enough for each voxel to contain at least  6 points but small enough to uniquely describe the environment.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 52-53

This parameter controls the maximum number of iterations the optimizer can run.  For the most part, the optimizer will terminate on the Transformation Epsilon before hitting this limit but this helps prevent it from running for too long in the wrong direction.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 55-58

Here, we pass the point clouds to the NDT registration program.  The input cloud is the cloud that will be transformed and the target cloud is the reference frame to which the input cloud will be aligned.  When the target cloud is added, the NDT algorithm's internal data structure is initialized using the target cloud data.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 60-63

In this section of code, we create an initial guess about the transformation needed to align the point clouds.  Though the algorithm can be run without such an initial transformation, you tend to get better results with one, particularly if there is a large discrepancy between reference frames.  In robotic applications, such as the ones used to generate this data set, the initial transformation is usually generated using odometry data.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 65-70

Finally, we are ready to align the point clouds.  The resulting transformed input cloud is stored in the output cloud.  We then display the results of the alignment as well as the Euclidean fitness score, calculated as the sum of squared distances from the output cloud to the closest point in the target cloud.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 72-76

Immediately after the alignment process, the output cloud will contain a transformed version of the filtered input cloud because we passed the algorithm a filtered point cloud, as opposed to the original input cloud.  To obtain the aligned version of the original cloud, we extract the final transformation from the NDT algorithm and transform our original input cloud.  We can now save this cloud to file ``room_scan2_transformed.pcd`` for future use.

.. literalinclude:: sources/normal_distributions_transform/normal_distributions_transform.cpp
   :language: cpp
   :lines: 78-106

This next part is unnecessary but I like to visually see the results of my labors.  With PCL's  visualizer classes, this can be easily accomplished.  We first generate a visualizer with a black background.  Then we colorize our target and output cloud, red and green respectively, and load them into the visualizer.  Finally we start the visualizer and wait for the window to be closed.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/normal_distributions_transform/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./normal_distributions_transform

You should see results similar those below as well as a visualization of the aligned point clouds.  Happy Coding::

  Loaded 112586 data points from room_scan1.pcd
  Loaded 112624 data points from room_scan2.pcd
  Filtered cloud contains 12433 data points from room_scan2.pcd
  Normal Distributions Transform has converged:1 score: 0.638694

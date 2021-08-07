.. _statistical_outlier_removal:

Removing outliers using a StatisticalOutlierRemoval filter
----------------------------------------------------------

In this tutorial we will learn how to remove noisy measurements, e.g. outliers,
from a point cloud dataset using statistical analysis techniques.

.. raw:: html
  
  <iframe title="Removing outliers using a StatisticalOutlierRemoval filter" width="480" height="390" src="https://www.youtube.com/embed/RjQPp2_GRnI?rel=0" frameborder="0" allowfullscreen></iframe>

Background
----------

Laser scans typically generate point cloud datasets of varying point densities.
Additionally, measurement errors lead to sparse outliers which corrupt the
results even more.  This complicates the estimation of local point cloud
characteristics such as surface normals or curvature changes, leading to
erroneous values, which in turn might cause point cloud registration failures.
Some of these irregularities can be solved by performing a statistical analysis
on each point's neighborhood, and trimming those which do not meet a certain
criterion.  Our sparse outlier removal is based on the computation of the
distribution of point to neighbors distances in the input dataset. For each
point, we compute the mean distance from it to all its neighbors. By assuming
that the resulted distribution is Gaussian with a mean and a standard
deviation, all points whose mean distances are outside an interval defined by
the global distances mean and standard deviation can be considered as outliers
and trimmed from the dataset.

The following picture shows the effects of the sparse outlier analysis and
removal: the original dataset is shown on the left, while the resultant one on
the right. The graphic shows the mean k-nearest neighbor distances in a point
neighborhood before and after filtering.

.. image:: images/statistical_removal_2.jpg


The code
--------

First, download the dataset `table_scene_lms400.pcd
<https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd>`_
and save it somewhere to disk.

Then, create a file, let's say, ``statistical_removal.cpp`` in your favorite
editor, and place the following inside it:

.. literalinclude:: sources/statistical_removal/statistical_removal.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

The following lines of code will read the point cloud data from disk.

.. literalinclude:: sources/statistical_removal/statistical_removal.cpp
   :language: cpp
   :lines: 12-15

   
Then, a *pcl::StatisticalOutlierRemoval* filter is created. The number of
neighbors to analyze for each point is set to 50, and the standard deviation
multiplier to 1. What this means is that all points who have a distance larger
than 1 standard deviation of the mean distance to the query point will be
marked as outliers and removed. The output is computed and stored in
*cloud_filtered*.

.. literalinclude:: sources/statistical_removal/statistical_removal.cpp
   :language: cpp
   :lines: 20-25

   
The remaining data (inliers) is written to disk for later inspection. 

.. literalinclude:: sources/statistical_removal/statistical_removal.cpp
   :language: cpp
   :lines: 30-31

   
Then, the filter is called with the same parameters, but with the output
negated, to obtain the outliers (e.g., the points that were filtered).

.. literalinclude:: sources/statistical_removal/statistical_removal.cpp
   :language: cpp
   :lines: 33-34

   
And the data is written back to disk.

.. literalinclude:: sources/statistical_removal/statistical_removal.cpp
   :language: cpp
   :lines: 35
   

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/statistical_removal/CMakeLists.txt
   :language: cmake
   :linenos:
   
After you have made the executable, you can run it. Simply do::

  $ ./statistical_removal

You will see something similar to::

  Cloud before filtering: 
  header: 
  seq: 0
  stamp: 0.000000000
  frame_id: 
  points[]: 460400
  width: 460400
  height: 1
  is_dense: 0

  Cloud after filtering: 
  header: 
  seq: 0
  stamp: 0.000000000
  frame_id: 
  points[]: 429398
  width: 429398
  height: 1
  is_dense: 0


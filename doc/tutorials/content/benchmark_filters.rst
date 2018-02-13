.. _filterbenchmarking:

Filter Benchmarking
-------------------

This document introduces benchmarking concepts for filtering algorithms. By
*benchmarking* here we refer to the possibility of testing different
parameters for each filter algorithm on a specific point cloud in an **easy manner**. The goal is to find the best parameters of a certain filter that best describe the original point cloud without removing useful data.

Benchmarking Filter Algorithms
-------------------------------

To get rid of noisy data in a scan of a 3D scene or object, many filters could be applied to obtain the *cleanest* representation possible of the object or scene. These filters need to be tuned according to the characteristics of the raw data. A filter evaluation class can be implemented, similar to the **FeatureEvaluationFramework** to find these parameters.


1. Functionality
^^^^^^^^^^^^^^^^

The **FilterEvaluationFramework** object could be initialized by the following functions:

 * setInputCloud: *Load test cloud from .pcd file*;
 * setFilterTest: *Choose the filter algorithm to be tested*;
 * setParameters: *Specific to the Filter Algorithm*;
 * setThreshold: *A single or a range of threshold values for the evaluation metric*;


2. Filter Types and Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Provide test classes for all the existing filters implemented in PCL.

 * StatisticalOutlierRemoval: *meanK and StddevMulThresh*;
 * RadiusOutlierRemoval: *radiusSearch and MinNeighborsInRadius*;
 * VoxelGrid: *LeafSize*;
 * etc..

Users should be able to add their custom filter implementations to the framework.

3. Evaluation
^^^^^^^^^^^^^

This benchmark should be able to evaluate the filter's performance with the specified parameters. The Evaluation metrics should answer the following questions:

 * Did the filter remove useful data? (new holes)
 * Is the new filtered cloud a clear representation of the original? (same surface)
 * Computation Time?



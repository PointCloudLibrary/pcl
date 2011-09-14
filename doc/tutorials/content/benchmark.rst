.. _benchmarking:

Benchmarking 3D
---------------

This document introduces benchmarking concepts for 3D algorithms. By
*benchmarking* here we refer to the posibility of testing different
computational pipelines in an **easy manner**. The goal is to test their
reproductibility with respect to a particular problem of general interest.

Benchmarking Object Recognition
-------------------------------

For the general problem of Object Recognition (identification, categorization,
detection, etc -- all fall in the same category here), we identify the
following steps:


1. Training
^^^^^^^^^^^

Users should be able to acquire training data from different inputs, including
but not limited to:

 * full triangle meshes (CAD models);
 * 360-degree full point cloud models;
 * partial point cloud views:

   * in clutter;
   * cleanly segmented.


2. Keypoints
^^^^^^^^^^^^

Computing higher level representation from the object's appearance (texture + depth) should be done:

 * **densely** - at every point/vertex in the input data;
 * at certain **interest points** (i.e., keypoints).
 
The detected keypoint might also contain some meta-information required by some descriptors, like scale or orientation.

3. Descriptors
^^^^^^^^^^^^^^

A higher level representation as mentioned before will be herein represented by a **feature descriptor**. Feature descriptors can be:

 * 2D (two-dimensional) -- here we refer to those descriptors estimated solely from RGB texture data;
 * 3D (three-dimensional) -- here we refer to those descriptors estimated solely from XYZ/depth data;
 * a combination of the above.


In addtion, feature descriptors can be:

 * **local** - estimated only at a set of discrete keypoints, using the information from neighboring pixels/points;
 * **global**, or meta-local - estimated on entire objects or the entire input dataset.


4. Classification
^^^^^^^^^^^^^^^^^

The distribution of features should be classifiable into distinct, separable
classes. For local features, we identify two sets of techniques:

 * **bag of words**;
 * **voting**;
 * **supervised voting** (regression from the description to the relative 3D location, e.g. Hough forest).

For global features, any general purpose classification technique should work (e.g., SVMs, nearest neighbors, etc).

In addition to classification, a substep of it could be considered
**Registration**. Here we refine the classification results using iterative
closest point techniques for example.


5. Evaluation
^^^^^^^^^^^^^

This pipeline should be able to evaluate the algorithm's performance at
different tasks. Here are some requested tasks to support:

 * object id and pose
 * object id and segmentation
 * object id and bounding box
 * category and segmentation
 * category and bounding box


5.1 Metrics
"""""""""""

This pipeline should provide different metrics, since algorithms excel in
different areas. Here are some requested metrics:

 * precision-recall
 * time
 * average rank of correct id
 * area under curve of cumulative histogram of rank of correct id

Object Recognition API 
======================

Here we describe a proposed set of classes that could be easily extended and
used for the purpose of benchmarking object recognition tasks.


1. Training
^^^^^^^^^^^

2. Keypoints
^^^^^^^^^^^^

3. Descriptors
^^^^^^^^^^^^^^

4. Classification
^^^^^^^^^^^^^^^^^

5. Evaluation
^^^^^^^^^^^^^

The evaluation output needs to be one of the following:

 * object id
 * object pose
 * object category
 * object bounding box
 * object mask


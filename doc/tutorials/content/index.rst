.. toctree::
  
The following links describe a set of basic PCL tutorials. Please note that
their source codes may already be provided as part of the PCL regular releases,
so check there before you start copy & pasting the code. The list of tutorials
below is automatically generated from reST files located in our git repository.

.. note::

  Before you start reading, please make sure that you go through the higher-level overview documentation at http://www.pointclouds.org/documentation/, under **Getting Started**. Thank you.

As always, we would be happy to hear your comments and receive your
contributions on any tutorial.

Table of contents
-----------------

  * :ref:`basic_usage`
  * :ref:`advanced_usage`
  * :ref:`applications_tutorial`
  * :ref:`features_tutorial`
  * :ref:`filtering_tutorial`
  * :ref:`i_o`
  * :ref:`keypoints_tutorial`
  * :ref:`kdtree_tutorial`
  * :ref:`octree_tutorial`
  * :ref:`range_images`
  * :ref:`recognition_tutorial`
  * :ref:`registration_tutorial`
  * :ref:`sample_consensus`
  * :ref:`segmentation_tutorial`
  * :ref:`surface_tutorial`
  * :ref:`visualization_tutorial`
  * :ref:`gpu`

.. _basic_usage:

Basic Usage
-----------

  * :ref:`walkthrough`

     ======  ======
     |mi_0|  Title: **PCL Functionality Walkthrough**

             Author: *Razvan G. Mihalyi*

             Compatibility: > PCL 1.6

             Takes the reader through all of the PCL modules and offers basic explanations on their functionalities.
     ======  ======

     .. |mi_0| image:: images/pcl_logo.png
               :height: 75px


  * :ref:`basic_structures`

     ======  ======
     |mi_1|  Title: **Getting Started / Basic Structures**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             Presents the basic data structures in PCL and discusses their usage with a simple code example.
     ======  ======

     .. |mi_1| image:: images/pcl_logo.png
               :height: 75px

  * :ref:`using_pcl_pcl_config`

     ======  ======
     |mi_2|  Title: **Using PCL in your own project**

             Author: *Nizar Sallem*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to link your own project to PCL using cmake.
     ======  ======

     .. |mi_2| image:: images/pcl_logo.png
               :height: 75px

  * :ref:`compiling_pcl_posix`

     =======  ======
     |mi_11|  Title: **Compiling PCL from source on POSIX compliant systems**

              Author: *Victor Lamoine*

              Compatibility: > PCL 1.0

              In this tutorial, we will explain how to compile PCL from sources on POSIX/Unix systems.
     =======  ======

     .. |mi_11| image:: images/pcl_logo.png
               :height: 120px

  * :ref:`building_pcl`

     ======  ======
     |mi_3|  Title: **Explaining PCL's cmake options**

             Author: *Nizar Sallem*

             Compatibility: > PCL 1.0

             In this tutorial, we will explain the basic PCL cmake options, and ways to tweak them to fit your project.
     ======  ======

     .. |mi_3| image:: images/pcl_ccmake.png
               :height: 100px

  * :ref:`compiling_pcl_dependencies_windows`

     ======  ======
     |mi_4|  Title: **Compiling PCL's dependencies from source on Windows**

             Authors: *Alessio Placitelli* and *Mourad Boufarguine*

             Compatibility: > PCL 1.0

             In this tutorial, we will explain how to compile PCL's 3rd party dependencies from source on Microsoft Windows.
     ======  ======

     .. |mi_4| image:: images/windows_logo.png
               :height: 100px

  * :ref:`compiling_pcl_windows`

     ======  ======
     |mi_5|  Title: **Compiling PCL on Windows**

             Author: *Mourad Boufarguine*

             Compatibility: > PCL 1.0 

             In this tutorial, we will explain how to compile PCL on Microsoft Windows.
     ======  ======

     .. |mi_5| image:: images/windows_logo.png
               :height: 100px

  * :ref:`compiling_pcl_macosx`

     ======  ======
     |mi_6|  Title: **Compiling PCL and its dependencies from MacPorts and source on Mac OS X**

             Author: *Justin Rosen*

             Compatibility: > PCL 1.0

             This tutorial explains how to build the Point Cloud Library **from MacPorts and source** on Mac OS X platforms.
     ======  ======

     .. |mi_6| image:: images/macosx_logo.png
               :height: 100px

  * :ref:`installing_homebrew`

     ======  ======
     |mi_7|  Title: **Installing on Mac OS X using Homebrew**

             Author: *Geoffrey Biggs*

             Compatibility: > PCL 1.2

             This tutorial explains how to install the Point Cloud Library on Mac OS X using Homebrew. Both direct installation and compiling PCL from source are explained.
     ======  ======

     .. |mi_7| image:: images/macosx_logo.png
               :height: 100px

  * :ref:`using_pcl_with_eclipse`

     ======  ======
     |mi_8|  Title: **Using Eclipse as your PCL editor**

             Author: *Koen Buys*

             Compatibility: PCL git master

             This tutorial shows you how to get your PCL as a project in Eclipse.
     ======  ======

     .. |mi_8| image:: images/pcl_with_eclipse/eclipse.png
               :height: 100px

  * :ref:`generate_local_doc`

     =======  ======
     |mi_11|  Title: **Generate a local documentation for PCL**

              Author: *Victor Lamoine*

              Compatibility: PCL > 1.0

              This tutorial shows you how to generate and use a local documentation for PCL.
     =======  ======

     .. |mi_11| image:: images/pcl_logo.png
               :height: 75px

  * :ref:`matrix_transform`

     =======  ======
     |mi_10|  Title: **Using matrixes to transform a point cloud**

              Author: *Victor Lamoine*

              Compatibility: > PCL 1.5

              This tutorial shows you how to transform a point cloud using a matrix.
     =======  ======

     .. |mi_10| image:: images/matrix_transform/cube.png
               :height: 120px

.. _advanced_usage:

Advanced Usage
--------------

  * :ref:`adding_custom_ptype`

     ======  ======
     |au_1|  Title: **Adding your own custom PointT point type**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.9, < PCL 2.0

             This document explains what templated point types are in PCL, why do they exist, and how to create and use your own `PointT` point type.
     ======  ======

     .. |au_1| image:: images/pcl_logo.png
               :height: 75px

  * :ref:`writing_new_classes`

     ======  ======
     |au_2|  Title: **Writing a new PCL class**

             Author: *Radu B. Rusu, Luca Penasa*

             Compatibility: > PCL 0.9, < PCL 2.0

             This short guide is to serve as both a HowTo and a FAQ for writing new PCL classes, either from scratch, or by adapting old code.
     ======  ======

     .. |au_2| image:: images/pcl_logo.png
               :height: 75px

.. _features_tutorial:

Features
--------

  * :ref:`how_3d_features_work`

     ======  ======
     |fe_1|  Title: **How 3D features work**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             This document presents a basic introduction to the 3D feature estimation methodologies in PCL.
     ======  ======
     
     .. |fe_1| image:: images/good_features_small.jpg
               :height: 100px

  * :ref:`normal_estimation`
    
     ======  ======
     |fe_2|  Title: **Estimating Surface Normals in a PointCloud**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             This tutorial discusses the theoretical and implementation details of the surface normal estimation module in PCL.
     ======  ======
     
     .. |fe_2| image:: images/normal_estimation.png
               :height: 100px


  * :ref:`normal_estimation_using_integral_images`
    
     ======  ======
     |fe_3|  Title: **Normal Estimation Using Integral Images**

             Author: *Stefan Holzer*

             Compatibility: > PCL 1.0

             In this tutorial we will learn how to compute normals for an organized point cloud using integral images.
     ======  ======
     
     .. |fe_3| image:: images/normal_estimation_ii.png
               :height: 100px

  * :ref:`pfh_estimation`
    
     ======  ======
     |fe_4|  Title: **Point Feature Histograms (PFH) descriptors**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             This tutorial introduces a family of 3D feature descriptors called PFH (Point Feature Histograms) and discusses their implementation details from PCL's perspective.
     ======  ======
     
     .. |fe_4| image:: images/pfh_estimation.png
               :height: 100px

  * :ref:`fpfh_estimation`
    
     ======  ======
     |fe_5|  Title: **Fast Point Feature Histograms (FPFH) descriptors**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.3

             This tutorial introduces the FPFH (Fast Point Feature Histograms) 3D descriptor and discusses their implementation details from PCL's perspective.
     ======  ======
     
     .. |fe_5| image:: images/fpfh_estimation.jpg
               :height: 100px

  * :ref:`vfh_estimation`
    
     ======  ======
     |fe_6|  Title: **Estimating VFH signatures for a set of points**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.8

             This document describes the Viewpoint Feature Histogram (VFH) descriptor, a novel representation for point clusters for the problem of Cluster (e.g., Object) Recognition and 6DOF Pose Estimation.
     ======  ======
     
     .. |fe_6| image:: images/vfh_estimation.png
               :height: 100px
  
  * :ref:`narf_feature_extraction`
    
     ======  ======
     |fe_7|  Title: **How to extract NARF features from a range image**

             Author: *Bastian Steder*

             Compatibility: > 1.3

             In this tutorial, we will learn how to extract NARF features from a range image.
     ======  ======
     
     .. |fe_7| image:: images/narf_keypoint_extraction.png
               :height: 100px

  * :ref:`moment_of_inertia`

     ======  ======
     |fe_8|  Title: **Moment of inertia and eccentricity based descriptors**

             Author: *Sergey Ushakov*

             Compatibility: > PCL 1.7

             In this tutorial we will learn how to compute moment of inertia and eccentricity of the cloud. In addition to this we will learn how to extract AABB and OBB.
     ======  ======

     .. |fe_8| image:: images/moment_of_inertia.png
               :height: 100px

  * :ref:`rops_feature`

     ======  ======
     |fe_9|  Title: **RoPs (Rotational Projection Statistics) feature**

             Author: *Sergey Ushakov*

             Compatibility: > PCL 1.7

             In this tutorial we will learn how to compute RoPS feature.
     ======  ======

     .. |fe_9| image:: images/rops_feature.png
               :height: 100px

.. _filtering_tutorial:

Filtering
---------

  * :ref:`passthrough`
    
     ======  ======
     |fi_1|  Title: **Filtering a PointCloud using a PassThrough filter**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to remove points whose values fall inside/outside a user given interval along a specified dimension.
     ======  ======
     
     .. |fi_1| image:: images/passthrough.png
               :height: 100px

  * :ref:`voxelgrid`
    
     ======  ======
     |fi_2|  Title: **Downsampling a PointCloud using a VoxelGrid filter**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to downsample (i.e., reduce the number of points) a Point Cloud.
     ======  ======
     
     .. |fi_2| image:: images/voxel_grid.jpg
               :height: 100px
    

  * :ref:`statistical_outlier_removal` 
    
     ======  ======
     |fi_3|  Title: **Removing sparse outliers using StatisticalOutlierRemoval**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to remove sparse outliers from noisy data, using StatisticalRemoval.
     ======  ======
     
     .. |fi_3| image:: images/statistical_removal.jpg
               :height: 100px

  * :ref:`project_inliers`

     ======  ======
     |fi_4|  Title: **Projecting points using a parametric model**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to project points to a parametric model (i.e., plane).
     ======  ======
     
     .. |fi_4| image:: images/project_inliers.png
               :height: 100px

  * :ref:`extract_indices`

     ======  ======
     |fi_5|  Title: **Extracting indices from a PointCloud**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to extract a set of indices given by a segmentation algorithm.
     ======  ======
     
     .. |fi_5| image:: images/extract_indices.jpg
               :height: 100px

  * :ref:`remove_outliers` 
    
     ======  ======
     |fi_6|  Title: **Removing outliers using a Conditional or RadiusOutlier removal**

             Author: *Gabe O'Leary*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to remove outliers from noisy data, using ConditionalRemoval, RadiusOutlierRemoval.
     ======  ======
     
     .. |fi_6| image:: images/radius_outlier.png
               :height: 100px

.. _i_o:

I/O
---

  * :ref:`pcd_file_format`

     ======  ======
     |i_o0|  Title: **The PCD (Point Cloud Data) file format**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.9

             This document describes the PCD file format, and the way it is used inside PCL.
     ======  ======
     
     .. |i_o0| image:: images/PCD_icon.png
               :height: 100px

  * :ref:`reading_pcd`

     ======  ======
     |i_o1|  Title: **Reading Point Cloud data from PCD files**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to read a Point Cloud from a PCD file.
     ======  ======
     
     .. |i_o1| image:: images/read_pcd.jpg
               :height: 100px

  * :ref:`writing_pcd`

     ======  ======
     |i_o2|  Title: **Writing Point Cloud data to PCD files**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to write a Point Cloud to a PCD file.
     ======  ======
     
     .. |i_o2| image:: images/write_pcd.jpg
               :height: 100px

  * :ref:`concatenate_clouds`

     ======  ======
     |i_o3|  Title: **Concatenate the fields or points of two Point Clouds**

             Author: *Gabe O'Leary / Radu B. Rusu*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to concatenate both the fields and the point data of two Point Clouds.  When concatenating fields, one PointClouds contains only *XYZ* data, and the other contains *Surface Normal* information.
     ======  ======

     .. |i_o3| image:: images/concatenate_fields.jpg
               :height: 100px

  * :ref:`openni_grabber`

     ======  ======
     |i_o4|  Title: **Grabbing Point Clouds from an OpenNI camera**

             Author: *Nico Blodow*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to acquire point cloud data from an OpenNI camera.
     ======  ======

     .. |i_o4| image:: images/openni_grabber.png
               :height: 100px

  * :ref:`hdl_grabber`

     ======  ======
     |i_o5|  Title: **Grabbing Point Clouds from a Velodyne High Definition LiDAR (HDL)**

             Author: *Keven Ring*

             Compatibility: >= PCL 1.7

             In this tutorial, we will learn how to acquire point cloud data from a Velodyne HDL.
     ======  ======

     .. |i_o5| image:: images/hdl_grabber.png
               :height: 100px
               
  * :ref:`dinast_grabber`

     ======  ======
     |i_o6|  Title: **Grabbing Point Clouds from Dinast Cameras**

             Author: *Marco A. Gutierrez*

             Compatibility: >= PCL 1.7

             In this tutorial, we will learn how to acquire point cloud data from a Dinast camera.
     ======  ======

     .. |i_o6| image:: images/dinast_cyclopes.png
               :height: 100px

  * :ref:`ensenso_cameras`

     ======  ======
     |i_o7|  Title: **Grabbing point clouds from Ensenso cameras**

             Author: *Victor Lamoine*

             Compatibility: >= PCL 1.8.0

             In this tutorial, we will learn how to acquire point cloud data from an IDS-Imaging Ensenso camera.
     ======  ======

     .. |i_o7| image:: images/ensenso/ids.png
               :height: 165px

  * :ref:`david_sdk`

     ======  ======
     |i_o8|  Title: **Grabbing point clouds / meshes from davidSDK scanners**

             Author: *Victor Lamoine*

             Compatibility: >= PCL 1.8.0

             In this tutorial, we will learn how to acquire point cloud or mesh data from a davidSDK scanner.
     ======  ======

     .. |i_o8| image:: images/davidsdk/david.png
               :height: 70px

  * :ref:`depth_sense_grabber`

     ======  ======
     |i_o9|  Title: **Grabbing point clouds from DepthSense cameras**

             Author: *Sergey Alexandrov*

             Compatibility: >= PCL 1.8.0

             In this tutorial we will learn how to setup and use DepthSense cameras within PCL on both Linux and Windows platforms.
     ======  ======

     .. |i_o9| image:: images/creative_camera.jpg
               :height: 70px

.. _keypoints_tutorial:

Keypoints
---------

  * :ref:`narf_keypoint_extraction`
    
     ======  ======
     |kp_1|  Title: **How to extract NARF keypoints from a range image**

             Author: *Bastian Steder*

             Compatibility: > 1.3

             In this tutorial, we will learn how to extract NARF keypoints from a range image.
     ======  ======
     
     .. |kp_1| image:: images/narf_keypoint_extraction.png
               :height: 100px

.. _kdtree_tutorial:

KdTree
------

  * :ref:`kdtree_search`

     ======  ======
     |kd_1|  Title: **KdTree Search**

             Author: *Gabe O'Leary*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to search using the nearest neighbor method for k-d trees
     ======  ======
     
     .. |kd_1| image:: images/kdtree_search.png
               :height: 100px

.. _octree_tutorial:

Octree
------

  * :ref:`octree_compression`
  
     ======  ======
     |oc_1|  Title: **Point cloud compression**

             Author: *Julius Kammerl*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to compress a single point cloud and streams of point clouds.
     ======  ======
     
     .. |oc_1| image:: images/compression_tutorial.png
               :height: 100px

  * :ref:`octree_search`
  
     ======  ======
     |oc_2|  Title: **Octrees for spatial partitioning and neighbor search**

             Author: *Julius Kammerl*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to use octrees for spatial partitioning and nearest neighbor search.
     ======  ======
     
     .. |oc_2| image:: images/octree_img.png
               :height: 100px
               
  * :ref:`octree_change_detection`
  
     ======  ======
     |oc_3|  Title: **Spatial change detection on unorganized point cloud data**

             Author: *Julius Kammerl*

             Compatibility: > PCL 1.0

             In this tutorial, we will learn how to use octrees for detecting spatial changes within point clouds.
     ======  ======
     
     .. |oc_3| image:: images/changedetectionThumb.png
               :height: 100px

.. _range_images:

Range Images
------------

  * :ref:`range_image_creation`

     ======  ======
     |ri_1|  Title: **Creating Range Images from Point Clouds**

             Author: *Bastian Steder*

             Compatibility: > PCL 1.0
             
             This tutorial demonstrates how to create a range image from a point cloud and a given sensor position. 
     ======  ======

     .. |ri_1| image:: images/range_image_visualization.png
               :height: 100px

  * :ref:`range_image_border_extraction`

     ======  ======
     |ri_2|  Title: **Extracting borders from Range Images**

             Author: *Bastian Steder*

             Compatibility: > PCL 1.3
             
             This tutorial demonstrates how to extract borders (traversals from foreground to background) from a range image. 
     ======  ======

     .. |ri_2| image:: images/range_image_border_points.png
               :height: 100px

.. _recognition_tutorial:

Recognition
-----------

  * :ref:`correspondence_grouping`

     ======  ======
     |rc_1|  Title: **The PCL Recognition API**

             Author: *Tommaso Cavallari, Federico Tombari*

             Compatibility: > PCL 1.6

             This tutorial aims at explaining how to perform 3D Object Recognition based on the pcl_recognition module.
     ======  ======

     .. |rc_1| image:: images/correspondence_grouping/correspondence_grouping.jpg
               :height: 100px

  * :ref:`implicit_shape_model`

     ======  ======
     |rc_2|  Title: **Implicit Shape Model**

             Author: *Sergey Ushakov*

             Compatibility: > PCL 1.7

             In this tutorial we will learn how the Implicit Shape Model algorithm works and how to use it for finding objects centers.
     ======  ======

     .. |rc_2| image:: images/implicit_shape_model.png
               :height: 100px

  * :ref:`global_hypothesis_verification`

     ======  ======
     |rc_3|  Title: **Hypothesis Verification for 3D Object Recognition**

             Author: *Daniele De Gregorio, Federico Tombari*

             Compatibility: > PCL 1.7

             This tutorial aims at explaining how to do 3D object recognition in clutter by verifying model hypotheses in cluttered and  heavily occluded 3D scenes.
     ======  ======

     .. |rc_3| image:: images/global_hypothesis_verification/multiple.png
               :height: 100px

.. _registration_tutorial:

Registration
------------

  * :ref:`registration_api`

     ======  ======
     |re_1|  Title: **The PCL Registration API**

             Author: *Dirk Holz, Radu B. Rusu, Jochen Sprickerhof*

             Compatibility: > PCL 1.5

             In this document, we describe the point cloud registration API and its modules: the estimation and rejection of point correspondences, and the estimation of rigid transformations.
     ======  ======

     .. |re_1| image:: images/registration/registration_api.png
               :height: 100px

  * :ref:`iterative_closest_point`

     ======  ======
     |re_2|  Title: **How to use iterative closest point algorithm**

             Author: *Gabe O'Leary*

             Compatibility: > PCL 1.0

             This tutorial gives an example of how to use the iterative closest point algorithm to see if one PointCloud is just a rigid transformation of another PointCloud.
     ======  ======

     .. |re_2| image:: images/iterative_closest_point.gif
               :height: 100px

  * :ref:`pairwise_incremental_registration`

     ======  ======
     |re_3|  Title: **How to incrementally register pairs of clouds**

             Author: *Raphael Favier*

             Compatibility: > PCL 1.4

             This document demonstrates using the Iterative Closest Point algorithm in order to incrementally register a series of point clouds two by two.
     ======  ======

     .. |re_3| image:: images/iterative_closest_point.gif
               :height: 100px

  * :ref:`interactive_icp`

     ======  ======
     |re_7|  Title: **Interactive ICP**

             Author: *Victor Lamoine*

             Compatibility: > PCL 1.5

             This tutorial will teach you how to build an interactive ICP program
     ======  ======

     .. |re_7| image:: images/interactive_icp/monkey.png
               :height: 120px

  * :ref:`normal_distributions_transform`

     ======  ======
     |re_4|  Title: **How to use the Normal Distributions Transform algorithm**

             Author: *Brian Okorn*

             Compatibility: > PCL 1.6

             This document demonstrates using the Normal Distributions Transform algorithm to register two large point clouds.
     ======  ======

     .. |re_4| image:: images/normal_distributions_transform.gif
               :height: 100px

  * :ref:`in_hand_scanner`

     ======  ======
     |re_5|  Title: **How to use the In-hand scanner for small objects**

             Author: *Martin Saelzle*

             Compatibility: >= PCL 1.7

             This document shows how to use the In-hand scanner applications to obtain colored models of small objects with RGB-D cameras.
     ======  ======

     .. |re_5| image:: images/ihs_lion_model.png
               :height: 100px

  * :ref:`alignment_prerejective`

     ======  ======
     |re_6|  Title: **Robust pose estimation of rigid objects**

             Author: *Anders Glent Buch*

             Compatibility: >= PCL 1.7

             In this tutorial, we show how to find the alignment pose of a rigid object in a scene with clutter and occlusions.

     ======  ======

     .. |re_6| image:: images/alignment_prerejective_1.png
               :height: 100px


.. _sample_consensus:

Sample Consensus
----------------

  * :ref:`random_sample_consensus`
    
     ======  ======
     |sc_1|  Title: **How to use Random Sample Consensus model**

             Author: *Gabe O'Leary*

             Compatibility: > PCL 1.0

             In this tutorial we learn how to use a RandomSampleConsensus with a plane model to obtain the cloud fitting to this model.
     ======  ======
     
     .. |sc_1| image:: images/ransac_outliers_plane.png
               :height: 100px

.. _segmentation_tutorial:
  
Segmentation
------------

  * :ref:`planar_segmentation`
    
     ======  ======
     |se_1|  Title: **Plane model segmentation**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.3

             In this tutorial, we will learn how to segment arbitrary plane models from a given point cloud dataset.
     ======  ======
     
     .. |se_1| image:: images/planar_segmentation.jpg
               :height: 100px

  * :ref:`cylinder_segmentation`

     ======  ======
     |se_2|  Title: **Cylinder model segmentation**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 1.3

             In this tutorial, we will learn how to segment arbitrary cylindrical models from a given point cloud dataset.
     ======  ======
     
     .. |se_2| image:: images/cylinder_segmentation.jpg
               :height: 100px

  * :ref:`cluster_extraction`

     ======  ======
     |se_3|  Title: **Euclidean Cluster Extraction**

             Author: *Serkan Tuerker*

             Compatibility: > PCL 1.3

             In this tutorial we will learn how to extract Euclidean clusters with the ``pcl::EuclideanClusterExtraction`` class.
     ======  ======
     
     .. |se_3| image:: images/cluster_extraction.jpg
               :height: 100px

  * :ref:`region_growing_segmentation`

     ======  ======
     |se_4|  Title: **Region Growing Segmentation**

             Author: *Sergey Ushakov*

             Compatibility: >= PCL 1.7

             In this tutorial we will learn how to use region growing segmentation algorithm.
     ======  ======

     .. |se_4| image:: images/region_growing_segmentation.jpg
               :height: 100px

  * :ref:`region_growing_rgb_segmentation`

     ======  ======
     |se_5|  Title: **Color-based Region Growing Segmentation**

             Author: *Sergey Ushakov*

             Compatibility: >= PCL 1.7

             In this tutorial we will learn how to use color-based region growing segmentation algorithm.
     ======  ======

     .. |se_5| image:: images/region_growing_rgb_segmentation.jpg
               :height: 100px

  * :ref:`min_cut_segmentation`

     ======  ======
     |se_6|  Title: **Min-Cut Based Segmentation**

             Author: *Sergey Ushakov*

             Compatibility: >= PCL 1.7

             In this tutorial we will learn how to use min-cut based segmentation algorithm.
     ======  ======

     .. |se_6| image:: images/min_cut_segmentation.jpg
               :height: 100px

  * :ref:`conditional_euclidean_clustering`

     ======  ======
     |se_7|  Title: **Conditional Euclidean Clustering**

             Author: *Frits Florentinus*

             Compatibility: >= PCL 1.7

             This tutorial describes how to use the Conditional Euclidean Clustering class in PCL:
             A segmentation algorithm that clusters points based on Euclidean distance and a user-customizable condition that needs to hold.
     ======  ======

     .. |se_7| image:: images/conditional_euclidean_clustering.jpg
               :height: 100px

  * :ref:`don_segmentation`

     ======  ======
     |se_8|  Title: **Difference of Normals Based Segmentation**

             Author: *Yani Ioannou*

             Compatibility: >= PCL 1.7

             In this tutorial we will learn how to use the difference of normals feature for segmentation.
     ======  ======

     .. |se_8| image:: images/don_segmentation.png
               :height: 100px

  * :ref:`supervoxel_clustering`

     ======  ======
     |se_9|  Title: **Supervoxel Clustering**

             Author: *Jeremie Papon*

             Compatibility: >= PCL 1.8

             In this tutorial, we show to break a pointcloud into the mid-level supervoxel representation.
     ======  ======

     .. |se_9| image:: images/supervoxel_clustering_small.png
               :height: 100px

  * :ref:`progressive_morphological_filtering`

     =======  ======
     |se_10|  Title: **Progressive Morphological Filtering**

              Author: *Brad Chambers*

              Compatibility: >= PCL 1.8

              In this tutorial, we show how to segment a point cloud into ground and non-ground returns.
     =======  ======

     .. |se_10| image:: images/progressive_morphological_filter.png
               :height: 100px

  * :ref:`model_outlier_removal`

     =======  ======
     |se_11|  Title: **Model outlier removal**

              Author: *Timo Häckel*

              Compatibility: >= PCL 1.7.2

              This tutorial describes how to extract points from a point cloud using SAC models
     =======  ======

     .. |se_11| image:: images/pcl_logo.png
               :height: 75px
               
.. _surface_tutorial:

Surface
-------

  * :ref:`moving_least_squares`

     ======  ======
     |su_1|  Title: **Smoothing and normal estimation based on polynomial reconstruction**

             Author: *Zoltan-Csaba Marton, Alexandru E. Ichim*

             Compatibility: > PCL 1.6

             In this tutorial, we will learn how to construct and run a Moving Least Squares (MLS) algorithm to obtain smoothed XYZ coordinates and normals.
     ======  ======
     
     .. |su_1| image:: images/resampling.jpg
               :height: 100px

  * :ref:`hull_2d`

     ======  ======
     |su_2|  Title: **Construct a concave or convex hull polygon for a plane model**

             Author: *Gabe O'Leary, Radu B. Rusu*

             Compatibility: > PCL 1.0

             In this tutorial we will learn how to calculate a simple 2D concave or convex hull polygon for a set of points supported by a plane.
     ======  ======
     
     .. |su_2| image:: images/convex_hull_2d.jpg
               :height: 100px

  * :ref:`greedy_triangulation`

     ======  ======
     |su_3|  Title: **Fast triangulation of unordered point clouds**

             Author: *Zoltan-Csaba Marton*

             Compatibility: > PCL 1.0

             In this tutorial we will learn how to run a greedy triangulation algorithm on a PointCloud with normals to obtain a triangle mesh based on projections of the local neighborhood.
     ======  ======
     
     .. |su_3| image:: images/greedy_triangulation.png
               :height: 100px

  * :ref:`bspline_fitting`

     ======  ======
     |su_4|  Title: **Fitting trimmed B-splines to unordered point clouds**

             Author: *Thomas Mörwald*

             Compatibility: > PCL 1.7

             In this tutorial we will learn how to reconstruct a smooth surface from an unordered point-cloud by fitting trimmed B-splines.
     ======  ======
     
     .. |su_4| image:: images/bspline_bunny.png
               :height: 100px


.. _visualization_tutorial:

Visualization
-------------

  * :ref:`cloud_viewer`

     ======  ======
     |vi_1|  Title: **Visualizing Point Clouds**

             Author: *Ethan Rublee*

             Compatibility: > PCL 1.0

             This tutorial demonstrates how to use the pcl visualization tools.
     ======  ======

     .. |vi_1| image:: images/cloud_viewer.jpg
               :height: 100px

  * :ref:`range_image_visualization`

     ======  ======
     |vi_2|  Title: **Visualizing Range Images**

             Author: *Bastian Steder*

             Compatibility: > PCL 1.3

             This tutorial demonstrates how to use the pcl visualization tools for range images.
     ======  ======

     .. |vi_2| image:: images/range_image_visualization.png
               :height: 100px

  * :ref:`pcl_visualizer`

     ======  ======
     |vi_3|  Title: **PCLVisualizer**

             Author: *Geoffrey Biggs*

             Compatibility: > PCL 1.3

             This tutorial demonstrates how to use the PCLVisualizer class for powerful visualisation of point clouds and related data.
     ======  ======

     .. |vi_3| image:: images/pcl_visualizer_viewports.png
               :height: 100px
 
  * :ref:`pcl_plotter`

     ======  ======
     |vi_4|  Title: **PCLPlotter**

             Author: *Kripasindhu Sarkar*

             Compatibility: > PCL 1.7

             This tutorial demonstrates how to use the PCLPlotter class for powerful visualisation of plots, charts and histograms of raw data and explicit functions.
     ======  ======

     .. |vi_4| image:: images/pcl_plotter_comprational.png
               :height: 100px               

  * :ref:`visualization`

     ======  ======
     |vi_5|  Title: **PCL Visualization overview**

             Author: *Radu B. Rusu*

             Compatibility: >= PCL 1.0

             This tutorial will give an overview on the usage of the PCL visualization tools.
     ======  ======

     .. |vi_5| image:: images/visualization_small.png
               :height: 120px

  * :ref:`qt_visualizer`

     ======  ======
     |vi_6|  Title: **Create a PCL visualizer in Qt with cmake**

             Author: *Victor Lamoine*

             Compatibility: > PCL 1.5

             This tutorial shows you how to create a PCL visualizer within a Qt application.
     ======  ======

     .. |vi_6| image:: images/qt_visualizer/qt.png
               :height: 128px

  * :ref:`qt_colorize_cloud`

     ======  ======
     |vi_7|  Title: **Create a PCL visualizer in Qt to colorize clouds**

              Author: *Victor Lamoine*

              Compatibility: > PCL 1.5

              This tutorial shows you how to color point clouds within a Qt application.
     ======  ======

     .. |vi_7| image:: images/qt_visualizer/qt.png
               :height: 128px


.. _applications_tutorial:

Applications
------------

  * :ref:`template_alignment`

     ======  ======
     |ap_1|  Title: **Aligning object templates to a point cloud**

             Author: *Michael Dixon*

             Compatibility: > PCL 1.3

             This tutorial gives an example of how some of the tools covered in the previous tutorials can be combined to solve a higher level problem --- aligning a previously captured model of an object to some newly captured data.
     ======  ======

     .. |ap_1| image:: images/template_alignment_1.jpg
               :height: 100px

  * :ref:`vfh_recognition`
    
     ======  ======
     |ap_2|  Title: **Cluster Recognition and 6DOF Pose Estimation using VFH descriptors**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.8

             In this tutorial we show how the Viewpoint Feature Histogram (VFH) descriptor can be used to recognize similar clusters in terms of their geometry.
     ======  ======
     
     .. |ap_2| image:: images/vfh_recognition.jpg
               :height: 100px

  * :ref:`mobile_streaming`
    
     ======  ======
     |ap_3|  Title: **Point Cloud Streaming to Mobile Devices with Real-time Visualization**

             Author: *Pat Marion*

             Compatibility: > PCL 1.3

             This tutorial describes how to send point cloud data over the network from a desktop server to a client running on a mobile device.
     ======  ======
     
     .. |ap_3| image:: images/mobile_streaming_1.jpg
               :height: 100px
               
  * :ref:`ground_based_rgbd_people_detection`
     
     ======  ======
     |ap_5|  Title: **Detecting people on a ground plane with RGB-D data**

             Author: *Matteo Munaro*

             Compatibility: >= PCL 1.7

             This tutorial presents a method for detecting people on a ground plane with RGB-D data.
     ======  ======
     
     .. |ap_5| image:: images/ground_based_rgbd_people_detection/Index_photo.jpg
               :height: 120px

.. _gpu:

GPU
---

  * :ref:`gpu_install`
     
     ======  ======
     |gp_1|  Title: **GPU Installation**

             Author: *Koen Buys*

             Compatibility: PCL git master

             This tutorial explains how to configure PCL to use with a Nvidia GPU
     ======  ======
     
     .. |gp_1| image:: images/PCD_icon.png
               :height: 100px

  * :ref:`using_kinfu_large_scale`

     ======  ======
     |ap_4|  Title: **Using Kinfu Large Scale to generate a textured mesh**

             Author: *Francisco Heredia and Raphael Favier*

             Compatibility: PCL git master

             This tutorial demonstrates how to use KinFu Large Scale to produce a mesh from a room, and apply texture information in post-processing for a more appealing visual result.
     ======  ======

     .. |ap_4| image:: images/using_kinfu_large_scale.jpg
               :height: 100px

  * :ref:`gpu_people`
     
     ======  ======
     |gp_2|  Title: **People Detection**

             Author: *Koen Buys*

             Compatibility: PCL git master

             This tutorial presents a method for people and pose detection.
     ======  ======
     
     .. |gp_2| image:: images/gpu/people/c2_100.jpg
               :height: 100px

 ..
  * :ref:`normal_estimation_integral_images`
    Surface normal estimation
  * Range Image
    * :ref:`range_image_visualization`
      How to visualize a range image
    * :ref:`range_image_creation`
      How to create a range image from a point cloud
    * :ref:`range_image_border_extraction`
      How to extract borders from range images
    * :ref:`narf_keypoint`
      How to extract NARF keypoints from a range image
    * :ref:`narf_descriptor`
      How to extract NARF descriptors from points in a range images
    * :ref:`octree_search`
      Octrees for spatial partitioning and neighbor search.     


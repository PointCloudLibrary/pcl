.. toctree::
  
The following links describe a set of basic PCL tutorials. Please note that
their source codes may already be provided as part of the PCL tutorials. 

* I/O

  * :ref:`reading_pcd`

     ======  ======
     |i_o1|  Title: **Reading Point Cloud data from PCD files**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.5

             In this tutorial, we will learn how to read a Point Cloud from a PCD file.
     ======  ======
     
     .. |i_o1| image:: images/read_pcd.jpg
               :height: 100px

  * :ref:`writing_pcd`

     ======  ======
     |i_o2|  Title: **Writing Point Cloud data to PCD files**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.5

             In this tutorial, we will learn how to write a Point Cloud to a PCD file.
     ======  ======
     
     .. |i_o2| image:: images/write_pcd.jpg
               :height: 100px

  * :ref:`concatenate_fields`

     ======  ======
     |i_o3|  Title: **Concatenate the fields of two Point Clouds**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.5

             In this tutorial, we will learn how to concatenate the fields of two Point Clouds, one containing only *XYZ* data, and one containing *Surface Normal* information.
     ======  ======

     .. |i_o3| image:: images/concatenate_fields.jpg
               :height: 100px

  * :ref:`concatenate_points`

     ======  ======
     |i_o4|  Title: **Concatenate the points of two Point Clouds**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.5

             In this tutorial, we will learn how to concatenate the point data of two Point Clouds with the same fields.
     ======  ======

     .. |i_o4| image:: images/concatenate_data.jpg
               :height: 100px

* Filtering

  * :ref:`voxel_grid`
    
     ======  ======
     |fi_1|  Title: **Downsampling a PointCloud using a VoxelGrid filter**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.5

             In this tutorial, we will learn how to downsample (i.e., reduce the number of points) a Point Cloud.
     ======  ======
     
     .. |fi_1| image:: images/voxel_grid.jpg
               :height: 100px
    

  * :ref:`statistical_outlier_removal` 
    
     ======  ======
     |fi_2|  Title: **Removing outliers using a StatisticalOutlierRemoval filter**

             Author: *Radu B. Rusu*

             Compatibility: > PCL 0.5

             In this tutorial, we will learn how to remove sparse outliers from noisy data, using statistical analysis.
     ======  ======
     
     .. |fi_2| image:: images/statistical_removal.jpg
               :height: 100px

  * :ref:`passthrough`

    Filtering a PointCloud using a PassThrough filter

  * :ref:`project_inliers`

    Projecting points using a parametric model

  * :ref:`extract_indices`

    Extracting indices from a PointCloud

* Segmentation

  * :ref:`cylinder_segmentation`

    Cylinder model segmentation

  * :ref:`planar_segmentation`

    This tutorial walks you through writing planar model segmentation code.

  * SAC Model Coefficients
    Shows the model_coefficients used in SAC Segmentation.

* Surface

  * :ref:`normal_estimation_integral_images`

    Surface normal estimation

  * :ref:`greedy_triangulation`

    Run a greedy algorithm on a PointCloud with normals to obtain a triangle mesh based on projections of the local neighborhood

  * :ref:`convex_hull` 

    Construct a convex hull polygon for a planar model

  * :ref:`moving_least_squares`

    Run MLS to obtain smoothed XYZ coordinates and normals

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

  * :ref:`narf_descriptor_visualization`

    Visualization of how the NARF descriptor is calculated and of the descriptor distances to a marked point.


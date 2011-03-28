.. _single_compile_unit:

Single compilation units
------------------------

Even before reading [1]_, we noticed a great speed up in compile time for all
PCL libraries if instead of compiling N object files and linking them together,
we compile only one, and include all the sources of the N files in this main
source. If you peek at an older version of PCL, you might notice things along
the lines of:

.. code-block:: cpp
   :linenos:

    // Include the implementations instead of compiling them separately to speed up compile time
    #include "extract_indices.cpp"
    #include "passthrough.cpp"
    #include "project_inliers.cpp"
    #include "statistical_outlier_removal.cpp"
    #include "voxel_grid.cpp"

and in CMakeLists.txt:

.. code-block:: cmake
   :linenos:

    rosbuild_add_library (pcl_ros_filters
                          src/pcl_ros/filters/filter.cpp
                          # Compilation is much faster if we include all the following CPP files in filters.cpp
                          #src/pcl_ros/filters/passthrough.cpp
                          #src/pcl_ros/filters/project_inliers.cpp
                          #src/pcl_ros/filters/extract_indices.cpp
                          #src/pcl_ros/filters/statistical_outlier_removal.cpp
                          #src/pcl_ros/filters/voxel_grid.cpp
                         )

For more information on how/why this works, see [1]_.

.. [1] http://gamesfromwithin.com/physical-structure-and-c-part-2-build-times

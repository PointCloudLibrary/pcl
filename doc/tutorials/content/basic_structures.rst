.. _basic_structures:

Getting Started / Basic Structures
----------------------------------

The basic data type in PCL 1.x is a :pcl:`PointCloud<pcl::PointCloud>`. A
PointCloud is a C++ class which contains the following data fields:

  * :pcl:`width<pcl::PointCloud::width>` (int)

    Specifies the width of the point cloud dataset in the number of points. *width* has two meanings:

      * it can specify the total number of points in the cloud (equal with the number of elements in **points** -- see below) for unorganized datasets;
      * it can specify the width (total number of points in a row) of an organized point cloud dataset.


    .. note::

       An **organized point cloud** dataset is the name given to point clouds
       that resemble an organized image (or matrix) like structure, where the
       data is split into rows and columns. Examples of such point clouds
       include data coming from stereo cameras or Time Of Flight cameras. The
       advantages of an organized dataset is that by knowing the relationship
       between adjacent points (e.g. pixels), nearest neighbor operations are
       much more efficient, thus speeding up the computation and lowering the
       costs of certain algorithms in PCL.

    .. note::

       An **projectable point cloud** dataset is the name given to point clouds
       that have a correlation according to a pinhole camera model between the (u,v) index
       of a point in the organized point cloud and the actual 3D values. This correlation can be
       expressed in it's easiest form as: u = f*x/z and v = f*y/z

    Examples::

      cloud.width = 640; // there are 640 points per line

  * :pcl:`height<pcl::PointCloud::height>` (int)

    Specifies the height of the point cloud dataset in the number of points. *height* has two meanings:

      * it can specify the height (total number of rows) of an organized point cloud dataset;
      * it is set to **1** for unorganized datasets (*thus used to check whether a dataset is organized or not*).

      Example::

        cloud.width = 640; // Image-like organized structure, with 480 rows and 640 columns,
        cloud.height = 480; // thus 640*480=307200 points total in the dataset

      Example::

        cloud.width = 307200;
        cloud.height = 1; // unorganized point cloud dataset with 307200 points

  * :pcl:`points<pcl::PointCloud::points>` (std::vector<PointT>)

    Contains the data array where all the points of type **PointT** are stored. For example, for a cloud containing XYZ data, **points** contains a vector of *pcl::PointXYZ* elements::

      pcl::PointCloud<pcl::PointXYZ> cloud;
      std::vector<pcl::PointXYZ> data = cloud.points;

  * :pcl:`is_dense<pcl::PointCloud::is_dense>` (bool)

    Specifies if all the data in **points** is finite (true), or whether the XYZ values of certain points might contain Inf/NaN values (false).


  * :pcl:`sensor_origin_<pcl::PointCloud::sensor_origin_>` (Eigen::Vector4f)

    Specifies the sensor acquisition pose (origin/translation). This member is usually optional, and not used by the majority of the algorithms in PCL.

  * :pcl:`sensor_orientation_<pcl::PointCloud::sensor_orientation_>` (Eigen::Quaternionf)

    Specifies the sensor acquisition pose (orientation). This member is usually optional, and not used by the majority of the algorithms in PCL.


To simplify development, the :pcl:`PointCloud<pcl::PointCloud>` class contains
a number of helper member functions. For example, users don't have to check if
**height** equals 1 or not in their code in order to see if a dataset is
organized or not, but instead use :pcl:`PointCloud<pcl::PointCloud::isOrganized>`::
  
  if (!cloud.isOrganized ())
    ...


The **PointT** type is the primary point data type and describes what each
individual element of :pcl:`points<pcl::PointCloud::points>` holds. PCL comes
with a large variety of different point types, most explained in the
:ref:`adding_custom_ptype` tutorial.


Compiling your first code example
---------------------------------

Until we find the right minimal code example, please take a look at the
:ref:`using_pcl_pcl_config` and :ref:`writing_new_classes` tutorials to see how
to compile and write code for or using PCL.


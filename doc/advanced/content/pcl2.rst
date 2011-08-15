.. _pcl2:

PCL 2.x API consideration guide
-------------------------------

With the PCL 1.x API locked and a few releases already underway, it's time to
consider what the next generation of libraries should look like. This document
discusses a series of changes to the current API, from base classes to higher
level algorithms.

Major changes
=============

1.1 pcl::PointCloud
^^^^^^^^^^^^^^^^^^^

The :pcl:`PointCloud <pcl::PointCloud>` class represents the base class in PCL
for holding **nD** (n dimensional) data. 

The 1.x API includes the following data members:
 * :pcl:`Header <std_msgs::Header>` (coming from ROS)

   * **uint32_t** :pcl:`seq <std_msgs::Header::seq>` - a sequence number
   * **uint64_t** :pcl:`stamp <std_msgs::Header::stamp>` - a timestamp associated with the time when the data was acquired
   * **std::string** :pcl:`frame_id <std_msgs::Header::frame_id>` - a TF frame ID

 * **std::vector<T>** :pcl:`points <pcl::PointCloud::points>` - a std C++ vector of T data. T can be a structure of any of the types defined in `point_types.h`.

 * **uint32_t** :pcl:`width <pcl::PointCloud::width>` - the width (for organized datasets) of the data. Set to the number of points for unorganized data.
 * **uint32_t** :pcl:`height <pcl::PointCloud::height>` - the height (for organized datasets) of the data. Set to 1 for unorganized data.
 * **bool** :pcl:`is_dense <pcl::PointCloud::is_dense>` - true if the data contains only valid numbers (e.g., no NaN or -/+Inf, etc). False otherwise.

 * **Eigen::Vector4f** :pcl:`sensor_origin_ <pcl::PointCloud::sensor_origin_>` - the origin (pose) of the acquisition sensor in the current data coordinate system.
 * **Eigen::Quaternionf** :pcl:`sensor_orientation_ <pcl::PointCloud::sensor_orientation_>` - the origin (orientation) of hte acquisition sensor in the current data coordinate system.


Proposals for the 2.x API:

 * drop templating on point types, thus making :pcl:`PointCloud <pcl::PointCloud>` template free
 * drop the :pcl:`Header <std_msgs::Header>` structure, or consolidate all the above information (width, height, is_dense, sensor_origin, sensor_orientation) into a single struct
 * make sure we can access a slice of the data as a *2D image*, thus allowing fast 2D displaying, [u, v] operations, etc
 * implement channels (of a single type!) as data holders, e.g.:

   * cloud["xyz"] => gets all 3D x,y,z data
   * cloud["normals"] => gets all surface normal data
   * etc
 * Capability to construct point cloud types containing the necessary channels
   *at runtime*. This will be particularly useful for run-time configuration of
   input sensors and for reading point clouds from files, which may contain a
   variety of point cloud layouts not known until the file is opened.
 * Complete traits system to identify what data/channels a cloud stores at
   runtime, facilitating decision making in software that uses PCL. (e.g.
   generic component wrappers.)
 * Stream-based IO sub-system to allow developers to load a stream of point
   clouds and "play" them through their algorithm(s), as well as easily capture
   a stream of point clouds (e.g. from a Kinect). Perhaps based on
   Boost::Iostreams.
 * Given the experience on `libpointmatcher <https://github.com/ethz-asl/libpointmatcher>`_,
   I (Stéphane Magnenat) propose the following data structures::
     cloud = map<identifier, space>
     space = tuple<type, components_identifiers, data_matrix>
     components_identifiers = vector<identifier>
     data_matrix = Eigen matrix
     identifier = string with standardised naming (pos, normals, x, y, etc.)
     type = type of space, underlying scalar type + norm definition (float with euclidean norm, binary with manhattan norm, etc.)


1.2 PointTypes 
^^^^^^^^^^^^^^


1.3 GPU support
^^^^^^^^^^^^^^^
 #. Stop using Thrust containers (can't be constructed for user allocated memory, incompatibility alignment qualifier support, etc.)
 #. Implement own containers for data in GPU memory preferably with reference counting (like pcl::gpu::DeviceArray, or cv::gpu::GpuMat). 

     * DeviceArray for arbitrary binary data on GPU, DeviceArray_<T> for convenience.

 #. There should be two layers in PCL GPU part - host(main) and device(for advanced use):
    
     * Host is a main layer for calling GPU functionality. Users and non-GPU part of PCL will run GPU via this. 
       
       * It must have CUDA-independent interface, i.e. headers from it must be compiled with gcc, cl, etc. 
       * Algorithms receive input data uploaded to GPU(ex. DeviceArray_<T>) and output is in GPU memory as well. So that output from one algorithm can be passed as input to another algorithm (or even library) without downloading/uploading.
       * namespace pcl::cuda (can depend on float4, cudaEvent_t, etc.) or pcl::gpu (completely independent, ATI/Intel support in future?) namespaces. Do we need the second?
       
     * Device layer contains code that is built with for NVidia's compiler.        
     
       * pcl::device namespace, only headers.
       * It is mainly for internal usage, for sharing __device__ algorithms among GPU parts of PCL (like functors, traits, reduction, scans, etc.). 
       * Also users who develop for GPU can take advantages of this.
 
 #. Async. support??
 #. How about implementing cloud and other containers with flag that indicates where data is located? Can be CPU, GPU, other. Such approach allows to implement universal CPU/GPU interface with behaviour depending on data passed. 
 
    to be continued...
  
       
Minor changes
=============



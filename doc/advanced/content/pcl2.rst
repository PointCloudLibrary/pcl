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
 * :pcl:`PCLHeader <pcl::PCLHeader>` (coming from ROS)

   * **uint32_t** :pcl:`seq <pcl::PCLHeader::seq>` - a sequence number
   * **uint64_t** :pcl:`stamp <pcl::PCLHeader::stamp>` - a timestamp associated with the time when the data was acquired
   * **std::string** :pcl:`frame_id <pcl::PCLHeader::frame_id>` - a TF frame ID

 * **std::vector<T>** :pcl:`points <pcl::PointCloud::points>` - a std C++ vector of T data. T can be a structure of any of the types defined in `point_types.h`.

 * **uint32_t** :pcl:`width <pcl::PointCloud::width>` - the width (for organized datasets) of the data. Set to the number of points for unorganized data.
 * **uint32_t** :pcl:`height <pcl::PointCloud::height>` - the height (for organized datasets) of the data. Set to 1 for unorganized data.
 * **bool** :pcl:`is_dense <pcl::PointCloud::is_dense>` - true if the data contains only valid numbers (e.g., no NaN or -/+Inf, etc). False otherwise.

 * **Eigen::Vector4f** :pcl:`sensor_origin_ <pcl::PointCloud::sensor_origin_>` - the origin (pose) of the acquisition sensor in the current data coordinate system.
 * **Eigen::Quaternionf** :pcl:`sensor_orientation_ <pcl::PointCloud::sensor_orientation_>` - the origin (orientation) of hte acquisition sensor in the current data coordinate system.


Proposals for the 2.x API:

 * drop templating on point types, thus making :pcl:`PointCloud <pcl::PointCloud>` template free
 * drop the :pcl:`PCLHeader <pcl::PCLHeader>` structure, or consolidate all the above information (width, height, is_dense, sensor_origin, sensor_orientation) into a single struct
 * make sure we can access a slice of the data as a *2D image*, thus allowing fast 2D displaying, [u, v] operations, etc
 * make sure we can access a slice of the data as a subpoint cloud: only certain points are chosen from the main point cloud
 * implement channels (of a single type!) as data holders, e.g.:
   * cloud["xyz"] => gets all 3D x,y,z data
   * cloud["normals"] => gets all surface normal data
   * etc
 * internals should be hidden : only accessors (begin, end ...) are public, this facilitating the change of the underlying structure
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
   we (François Pomerleau and Stéphane Magnenat) propose the following data structures::
     cloud = map<space_identifier, space>
     space = tuple<type, components_identifiers, data_matrix>
     components_identifiers = vector<component_identifier>
     data_matrix = Eigen matrix
     space_identifier = string with standardised naming (pos, normals, color, etc.)
     component_identifier = string with standardised naming (x, y, r, g, b, etc.)
     type = type of space, underlying scalar type + distance definition (float with euclidean 2-norm distance, float representing gaussians with Mahalanobis distance, binary with manhattan distance, float with euclidean infinity norm distance, etc.)
   For instance, a simple point + color scenario could be::
     cloud = { "pos" => pos_space, "color" => color_space }
     pos_space = ( "float with euclidean 2-norm distance", { "x", "y", "z" }, [[(0.3,0,1.3) , ... , (1.2,3.1,2)], ... , [(1,0.3,1) , ... , (2,0,3.5)] )
     color_space = ( "uint8 with rgb distance", { "r", "g", "b" }, [[(0,255,0), ... , (128,255,32)] ... [(12,54,31) ... (255,0,192)]] )

1.2 PointTypes 
^^^^^^^^^^^^^^

  #. Eigen::Vector4f or Eigen::Vector3f ??
  
  #. Large points cause significant perfomance penalty for GPU. Let's assume that point sizes up to 16 bytes are suitable. This is some compromise between SOA and AOS. Structures like pcl::Normal (size = 32) is not desirable. SOA is better in this case.


1.3 GPU support
^^^^^^^^^^^^^^^
 #. Containers for GPU memory. pcl::gpu::DeviceMemory/DeviceMemory2D/DeviceArray<T>/DeviceArray2D<T> (Thrust containers are incinvinient).         
 
      * DeviceArray2D<T> is container for organized point cloud data (supports row alignment)
  
 #. PointCloud Channels for GPU memory. Say, with "_gpu" postfix.
 
     * cloud["xyz_gpu"] => gets channel with 3D x,y,z data allocated on GPU.     
     * GPU functions (ex. gpu::computeNormals) create new channel in cloud (ex. "normals_gpu") and write there. Users can preallocate the channel and data inside it in order to save time on allocations.
     * Users must manually invoke uploading/downloading data to/from GPU. This provides better understanding how much each operation costs.
          
 #. Two layers in GPU part:  host layer(nvcc-independent interface) and device(for advanced use, for sharing code compiled by nvcc):
 
     * namespace pcl::cuda (can depend on CUDA headers) or pcl::gpu (completely independent from CUDA, OpenCL support in future?).
     * namespace pcl::device for device layer, only headers.
      
 #. Async operation support???
     

1.4 Keypoints and features 
^^^^^^^^^^^^^^^^^^^^^^^^^^
 #. The name Feature is a bit misleading, since it has tons of meanings. Alternatives are Descriptor or FeatureDescription.
 #. In the feature description, there is no need in separate FeatureFromNormals class and setNormals() method, since all the required channels are contained in one input. We still need separate setSearchSurface() though.
 #. There exist different types of keypoints (corners, blobs, regions), so keypoint detector might return some meta-information besides the keypoint locations (scale, orientation etc.). Some channels of that meta-information are required by some descriptors. There are options how to deliver that information from keypoints to descriptor, but it should be easy to pass it if a user doesn't change anything. This interface should be uniform to allow for switching implementations and automated benchmarking. Still one might want to set, say, custom orientations, different from what detector returned. 
	
	to be continued...

1.5 Data slices
^^^^^^^^^^^^^^^
Anything involving a slice of data should use size_t for indices and not int. E.g the indices of the inliers in RANSAC, the focused points in RANSAC ...

1.6 RANSAC
^^^^^^^^^^
 * Renaming the functions and internal variables: everything should be named with _src and _tgt: we have confusing names like \indices_ and \indices_tgt_ (and no \indices_src_), setInputCloud and setInputTarget (duuh, everything is an input, it should be setTarget, setSource), in the code, a sample is named: selection, \model_ and samples. getModelCoefficients is confusing with getModel (this one should be getBestSample).
 * no const-correctness all over, it's pretty scary: all the get should be const, selectWithinDistance and so on too.
 * the getModel, getInliers function should not force you to fill a vector: you should just return a const reference to the internal vector: that could allow you to save a useless copy
 * some private members should be made protected in the sub sac models (like sac_model_registration) so that we can inherit from them.
 * the SampleConsensusModel should be independent from point clouds so that we can create our own model for whatever library. Then, the one used in the specialize models (like sac_model_registration and so on) should inherit from it and have constructors based on PointClouds like now. Maybe we should name those PclSampleConsensusModel or something (or have SampleConsensusModelBase and keep the naming for SampleConsensusModel).

Minor changes
=============

Concepts
========
See http://dev.pointclouds.org/issues/567.

References
----------
- `The Little Manual of API Design <www4.in.tum.de/~blanchet/api-design.pdf>`_

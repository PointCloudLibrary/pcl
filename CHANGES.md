# ChangeList

## *= 1.7.2 (10.09.2014) =*

* Added support for VTK6
  [[#363]](https://github.com/PointCloudLibrary/pcl/pull/363)
* Removed Google Test from the source tree and added it as a system dependency
  [[#731]](https://github.com/PointCloudLibrary/pcl/pull/731)
* Added support for QHull 2012 on non-Debian platforms
  [[#852]](https://github.com/PointCloudLibrary/pcl/pull/852)

### `libpcl_common:`

* Added `BearingAngleImage` class
  [[#198]](https://github.com/PointCloudLibrary/pcl/pull/198)
* Added `pcl::CPPFSignature` point type
  [[#296]](https://github.com/PointCloudLibrary/pcl/pull/296)
* Added `getRGBAVector4i()`, `getBGRVector3cMap()`, and `getBGRAVector4cMap()`
  to all point types containing RGB/RGBA fields
  [[#450]](https://github.com/PointCloudLibrary/pcl/pull/450)
* Added a family of "has field" functions to check presence of a particular
  field in a point type both at compile- and run-time
  [[#462]](https://github.com/PointCloudLibrary/pcl/pull/462)
* Added a function to copy data between points of different types
  [[#465]](https://github.com/PointCloudLibrary/pcl/pull/465)
* Added test macros for equality/nearness checks
  [[#499]](https://github.com/PointCloudLibrary/pcl/pull/499)
* Added `descriptorSize()` to all point types with descriptors
  [[#531]](https://github.com/PointCloudLibrary/pcl/pull/531)
* Added possibility to copy a cloud inside another one while interpolating
  borders
  [[#567]](https://github.com/PointCloudLibrary/pcl/pull/567)
* Added a function to determine the point of intersection of three non-parallel
  planes
  [[#571]](https://github.com/PointCloudLibrary/pcl/pull/571)
* Fixed a bug in HSV to RGB color conversion
  [[#581]](https://github.com/PointCloudLibrary/pcl/pull/581)
* Added a new `CentroidPoint` class
  [[#586]](https://github.com/PointCloudLibrary/pcl/pull/586)
* Templated intersection computation functions on scalar type
  [[#646]](https://github.com/PointCloudLibrary/pcl/pull/646)
* Templated functions in 'eigen.h' on scalar type
  [[#660]](https://github.com/PointCloudLibrary/pcl/pull/660)
* Added functions to transform points, vectors, lines, etc.
  [[#660]](https://github.com/PointCloudLibrary/pcl/pull/660)

### `libpcl_features:`

* Added a simple implementation of CPPF using normalised HSV values in the
  feature vector
  [[#296]](https://github.com/PointCloudLibrary/pcl/pull/296)
* Added `MomentOfInertiaEstimation` and `ROPSEstimation` features
  [[#319]](https://github.com/PointCloudLibrary/pcl/pull/319)
* Fixed a problem in `OURCVFHEstimation::computeRFAndShapeDistribution()`
  [[#738]](https://github.com/PointCloudLibrary/pcl/pull/738)
* Fixed undefined behavior in `OURCVFHEstimation::computeFeature()`
  [[#811]](https://github.com/PointCloudLibrary/pcl/pull/811)
* Fixed memory corruption error in OUR-CVFH
  [[#875]](https://github.com/PointCloudLibrary/pcl/pull/875)

### `libpcl_filters:`

* Added a function to set the minimum number of points required for a voxel to
  be used in `VoxelGrid`
  [[#434]](https://github.com/PointCloudLibrary/pcl/pull/434)
* Added `GridMinimum` filter
  [[#520]](https://github.com/PointCloudLibrary/pcl/pull/520)
* Added a morphological filter that operates on Z dimension
  [[#533]](https://github.com/PointCloudLibrary/pcl/pull/533)
* Added progressive morphological filter to extract ground returns
  [[#574]](https://github.com/PointCloudLibrary/pcl/pull/574)
* Added a filter to remove locally maximal points in the z dimension
  [[#577]](https://github.com/PointCloudLibrary/pcl/pull/577)
* Added an approximate version of the progressive morphological filter
  [[#665]](https://github.com/PointCloudLibrary/pcl/pull/665)
* Added `ModelOutlierRemoval` class that filters points in a cloud based on the
  distance between model and point
  [[#702]](https://github.com/PointCloudLibrary/pcl/pull/702)

### `libpcl_io:`

* Added experimental version of an OpenNI 2.x grabber
  [[#276]](https://github.com/PointCloudLibrary/pcl/pull/276)
  [[#843]](https://github.com/PointCloudLibrary/pcl/pull/843)
* Added support for IFS file format
  [[#354]](https://github.com/PointCloudLibrary/pcl/pull/354)
  [[#356]](https://github.com/PointCloudLibrary/pcl/pull/356)
* Added possibility to load `PCLPointCloud2` from OBJ files
  [[#363]](https://github.com/PointCloudLibrary/pcl/pull/363)
* Fixed loading and saving of PLY files
  [[#510]](https://github.com/PointCloudLibrary/pcl/pull/510)
  [[#579]](https://github.com/PointCloudLibrary/pcl/pull/579)
* Fixed race conditions in `PCDGrabber`
  [[#582]](https://github.com/PointCloudLibrary/pcl/pull/582)
* Fixed multi openni grabber buffer corruption
  [[#845]](https://github.com/PointCloudLibrary/pcl/pull/845)
* Fixed incompatibility with Boost 1.56 in `LZFImageWriter`
  [[#867]](https://github.com/PointCloudLibrary/pcl/pull/867)
* Fixed a bug in `PLYReader` which lead to deformation of point clouds when
  displayed in `CloudViewer` or `PCLVisualizer`
  [[#879]](https://github.com/PointCloudLibrary/pcl/pull/879)

### `libpcl_kdtree:`

* Fixed double memory free bug in `KdTreeFLANN`
  [[#618]](https://github.com/PointCloudLibrary/pcl/pull/618)

### `libpcl_keypoints:`

* Added a method `Keypoint::getKeypointsIndices ()`
  [[#318]](https://github.com/PointCloudLibrary/pcl/pull/318)
* Added keypoints based on Trajkovic and Hedley operator (2D and 3D versions)
  [[#409]](https://github.com/PointCloudLibrary/pcl/pull/409)

### `libpcl_octree:`

* Fixed a bug in `OctreePointCloudAdjacency::computeNeighbors()`
  [[#455]](https://github.com/PointCloudLibrary/pcl/pull/455)
* Accelerated `OctreePointCloudAdjacency` building by disabling dynamic key
  resizing
  [[#332]](https://github.com/PointCloudLibrary/pcl/pull/332)
* Fixed a bug with infinite points in `OctreePointCloudAdjacency`
  [[#723]](https://github.com/PointCloudLibrary/pcl/pull/723)

### `libpcl_people:`

* Added a possibility to define a transformation matrix for people tracker
  [[#606]](https://github.com/PointCloudLibrary/pcl/pull/606)

### `libpcl_recognition:`

* Allow PCL to be built against a system-wide installed metslib
  [[#299]](https://github.com/PointCloudLibrary/pcl/pull/299)
* Fixed a bug in `ObjRecRANSAC::addModel()`
  [[#269]](https://github.com/PointCloudLibrary/pcl/pull/269)
* Added `LINEMOD::loadTemplates()` (useful for object recognition systems that
  store templates for different objects in different files)
  [[#358]](https://github.com/PointCloudLibrary/pcl/pull/358)

### `libpcl_registration:`

* Fixed `SampleConsensusInitialAlignment::hasConverged()`
  [[#339]](https://github.com/PointCloudLibrary/pcl/pull/339)
* Added `JointIterativeClosestPoint`
  [[#344]](https://github.com/PointCloudLibrary/pcl/pull/344)
* Made correspondence rejectors to actually work with ICP
  [[#419]](https://github.com/PointCloudLibrary/pcl/pull/419)
* Added `GeneralizedIterativeClosestPoint6D` that integrates Lab color space
  information into the GICP algorithm
  [[#491]](https://github.com/PointCloudLibrary/pcl/pull/491)
* Fixed bugs and optimized `SampleConsensusPrerejective`
  [[#741]](https://github.com/PointCloudLibrary/pcl/pull/741)
* Fixed a bug in `TransformationEstimationSVDScale`
  [[#885]](https://github.com/PointCloudLibrary/pcl/pull/885)

### `libpcl_sample_consensus:`

* Unified `SampleConsensusModelNormalParallelPlane` with
  `SampleConsensusModelNormalPlane` to avoid code duplication
  [[#696]](https://github.com/PointCloudLibrary/pcl/pull/696)

### `libpcl_search:`

* `search::KdTree` can now be used with different KdTree implementations
  [[#81]](https://github.com/PointCloudLibrary/pcl/pull/81)
* Added a new interface to FLANN's multiple randomized trees for
  high-dimensional (feature) searches
  [[#435]](https://github.com/PointCloudLibrary/pcl/pull/435)
* Fixed a bug in the `Ptr` typdef in `KdTree`
  [[#820]](https://github.com/PointCloudLibrary/pcl/pull/820)

### `libpcl_segmentation:`

* Added `GrabCut` segmentation and a show-case application for 2D
  [[#330]](https://github.com/PointCloudLibrary/pcl/pull/330)
* Updated `RegionGrowingRGB::assembleRegion()` to speed up the algorithm
  [[#538]](https://github.com/PointCloudLibrary/pcl/pull/538)
* Fixed a bug with missing point infinity test in `RegionGrowing`
  [[#617]](https://github.com/PointCloudLibrary/pcl/pull/617)
* Fixed alignment issue in `SupervoxelClustering`
  [[#625]](https://github.com/PointCloudLibrary/pcl/pull/625)
* Added a curvature parameter to `Region3D` class
  [[#653]](https://github.com/PointCloudLibrary/pcl/pull/653)
* Fixed a minor bug in `OrganizedConnectedComponentSegmentation`
  [[#802]](https://github.com/PointCloudLibrary/pcl/pull/802)

### `libpcl_surface:`

* Fixed a bug in `EarClipping` where computation failed if all vertices have
  the same x or y component
  [[#130]](https://github.com/PointCloudLibrary/pcl/pull/130)
* Added support for unequal focal lengths along different axes in texture
  mapping
  [[#352]](https://github.com/PointCloudLibrary/pcl/pull/352)
* Speeded up bilateral upsampling
  [[#689]](https://github.com/PointCloudLibrary/pcl/pull/689)
* Reduced space usage in `MovingLeastSquares`
  [[#785]](https://github.com/PointCloudLibrary/pcl/pull/785)

### `libpcl_tracking:`

* Fixed Hue distance calculation in tracking `HSVColorCoherence`
  [[#390]](https://github.com/PointCloudLibrary/pcl/pull/390)
* Added pyramidal KLT tracking
  [[#587]](https://github.com/PointCloudLibrary/pcl/pull/587)

### `libpcl_visualization:`

* Added a new color handler `PointCloudColorHandlerRGBAField` that takes into
  account alpha channel
  [[#306]](https://github.com/PointCloudLibrary/pcl/pull/306)
* Fixed `PCLVisualizer` crashes on OS X
  [[#384]](https://github.com/PointCloudLibrary/pcl/pull/384)
* Added possibility to display texture on polygon meshes
  [[#400]](https://github.com/PointCloudLibrary/pcl/pull/400)
* Added ability to add and remove several coordinate systems
  [[#401]](https://github.com/PointCloudLibrary/pcl/pull/401)
* Added `ImageViewer::markPoints()`
  [[#439]](https://github.com/PointCloudLibrary/pcl/pull/439)
* Added `setWindowPosition()` and `setWindowName()` to `PCLPlotter`
  [[#457]](https://github.com/PointCloudLibrary/pcl/pull/457)
* Changed camera parameters display to be more user-friendly
  [[#544]](https://github.com/PointCloudLibrary/pcl/pull/544)
* Added `PCLVisualizer::updateCoordinateSystemPose()`
  [[#569]](https://github.com/PointCloudLibrary/pcl/pull/569)
* Fixed display of non-triangular meshes in `PCLVisualizer`
  [[#686]](https://github.com/PointCloudLibrary/pcl/pull/686)
* Added a capability to save and restore camera view in `PCLVisualizer`
  [[#703]](https://github.com/PointCloudLibrary/pcl/pull/703)
* Added `PCLVisualizer::getShapeActorMap()` function
  [[#725]](https://github.com/PointCloudLibrary/pcl/pull/725)
* Fixed undefined behavior when drawing axis in `PCLVisualizer`
  [[#762]](https://github.com/PointCloudLibrary/pcl/pull/762)
* Fixed HSV to RGB conversion in `PointCloudColorHandlerHSVField`
  [[#772]](https://github.com/PointCloudLibrary/pcl/pull/772)
* Fixed non-working key presses in visualization GUIs on Mac OS X systems
  [[#795]](https://github.com/PointCloudLibrary/pcl/pull/795)
* Fixed a bug in `PCLVisualizer::addCube()`
  [[#846]](https://github.com/PointCloudLibrary/pcl/pull/846)
* Fixed a bug in cone visualization and added possibility to set cone length
  [[#881]](https://github.com/PointCloudLibrary/pcl/pull/881)

### `PCL Tools:`

* Added a simple tool to compute Hausdorff distance between two point clouds
  [[#519]](https://github.com/PointCloudLibrary/pcl/pull/519)
* Updated `pcl_viewer` to use RGB color handler as default
  [[#556]](https://github.com/PointCloudLibrary/pcl/pull/556)
* Added a morphological tool `pcl_morph` to apply dilate/erode/open/close
  operations on the Z dimension
  [[#572]](https://github.com/PointCloudLibrary/pcl/pull/572)
* Added a tool `pcl_generate` to generate random clouds
  [[#599]](https://github.com/PointCloudLibrary/pcl/pull/599)
* Added a tool `pcl_grid_min` to find grid minimums
  [[#603]](https://github.com/PointCloudLibrary/pcl/pull/603)
* Added a tool `pcl_local_max` to filter out local maxima
  [[#604]](https://github.com/PointCloudLibrary/pcl/pull/604)
* Added optional depth image input to `pcl_png2pcd` converter
  [[#680]](https://github.com/PointCloudLibrary/pcl/pull/680)
* Fixed memory size calculation in `pcl_openni_pcd_recorder`
  [[#676]](https://github.com/PointCloudLibrary/pcl/pull/676)
* Added device ID parameter to `pcl_openni_pcd_recorder`
  [[#673]](https://github.com/PointCloudLibrary/pcl/pull/673)
* Added automatic camera reset on startup in `pcl_viewer`
  [[#693]](https://github.com/PointCloudLibrary/pcl/pull/693)
* Added a capability to save and restore camera view in `pcl_viewer`
  [[#703]](https://github.com/PointCloudLibrary/pcl/pull/703)
* Updated `pcl_pcd2png` tool to be able to paint pixels corresponding to
  infinite points with black. Added Glasbey lookup table to paint labels with
  a fixed set of highly distinctive colors.
  [[#767]](https://github.com/PointCloudLibrary/pcl/pull/767)
* Added `pcl_obj2pcd` tool
  [[#816]](https://github.com/PointCloudLibrary/pcl/pull/816)

### `PCL Apps:`

* Fixed disappearing cloud from selection in Cloud Composer
  [[#814]](https://github.com/PointCloudLibrary/pcl/pull/814)


## *= 1.7.1 (07.10.2013) =*

 * New pcl::io::savePNGFile() functions and pcd2png tool (deprecates organized_pcd_to_png).
 * Support for Intel Perceptual Computing SDK cameras.
 * New Dual quaternion transformation estimation algorithm.
 * Bugfixes.

## *= 1.7.0 (23.07.2013) =*

### `libpcl_common:`

* Added pcl::Intensity and pcl::Intensity8u point types
* Added pcl::RangeImageSpherical sub-class that is more suitable than pcl::RangeImage for some kinds of 360° range images (as discussed in [PCL-users] Range Image Projection)
* Added DefaultPointRepresentation<pcl::Narf36> to allow pcl::Narf36 to be used with pcl::search (#915)

### `PCL Apps:`

* Added cloud_composer app
* Added PCLModeler, with a tree view scene explorer and multiple render windows support 
* Added client app for the point cloud streaming server 
* Added new server app for point cloud streaming to mobile devices (pcl_openni_mobile_server)
* Added a new demo for the connected component segmentation. Includes a QT gui that allows various features to be toggled on/off. 
* Added SHOT estimator wrapper using OMP
* Added openni_organized_multi_plane_segmentation to demonstrate the OrganizedMultiPlaneSegmentation class. 

### `libpcl_recognition:`

* Added a new tutorial for "libpcl_recognition" for Correspondence Grouping by Tommaso Cavallari (#666)
* Added support for .LMT file loading (which are TARed files for PCD masks and SQMMT linemod templates)
* Changes in the computation of the modality to improve performance 
* Fixed a compilation error on windows; for some reason 'NULL' needs to be explicitly casted to the pointer type 
* Added a model library class used for maintaining the object models to be recognized. 
* Changed the interface to make it less confusing to use. 
* Added a couple useful overloads for "Houg###Grouping" and "GeometricConsistencyGrouping"
* Added CRHAlignment class. 
* Added Papazov HV method. 
* Fixed a bug in Poisson surface reconstruction that was causing the unit test to fail
* Added option for automatic selection of number of features in extractFeature
* Added a new greedy hypotheses verification method. 
* Added semi scale invariant linemod template detection 
* Fixed RF search radius in "Houg###Grouping" 
* Fixed some bugs in detection refinement along viewing direction  
* Fixed bug in LineRGBD::computeTransformedTemplatePoints (template point cloud's width and height fields were not set) 
* Converted uses of PointXYZRGB to PointXYZRGBA; converted std::cerr messages to PCL_DEBUG; minor reformatting to keep lines under 120 characters 
* Fixed some bugs related to bounding box computation and computation of transformed template point clouds 
* Added functionality to specify an object ID when loading templates in LineRGBD; 
* Added "GeometricConsistencyGrouping" clustering class 
* Added high level interface for RGBD version of linemod (not all parts are implemented yet) 
* Added SSE optimizations to improve detection speed 
* Added Ransac Correspondence Rejection into Houg###Grouping 
* Changed method for selecting features in color gradient modality 
* Added "CorrespondenceGrouping" abstract base class for correspondence grouping in pcl_recognitio
* Added cosine approximation in score computation 
* Added structure for hypotheses verification methods. Base abstract class 

### `libpcl_keypoints`

* Added implementation of the Intrinsic Shape Signature keypoint detector
* fixed crash caused by setNormals() in HarrisKeypoint3D (related to #745)

### `libpcl_outofcore:`

* Added support for PointCloud + gen LOD
* Added PointCloud2 support for outofcore queries via new "queryBBIncludes" method
* Added "copyPointCloud" support for PointCloud2 without indices
* Added feature: outofcore binary compressed pcd files to store point data on disk
* Bug fix: outofcore write buffer constant limitation fixed so outofcore_process will work with large 20M+ point TRCS data sets 
* Constants for write buffer changed to 2e12 to support insertion of very large point clouds until new serialization is implemented 
* Added getVoxelSideLength to octree_base for displaying of nodes in visualizer


### `libpcl_search:`

* FlannSearch: fixed wrong typedef (::Ptr would break if FlannDistance!=flann::L2) and compiler error
* Added new option in `FlannSearch`: FLANN KMeans Tree can now be uses as the search algorithm

### `libpcl_visualization:`

* Added specific methods to handle monochrome images represented by a PointCloud<Intensity> or a PointCloud<Intensity8u>
* Add area selection option to PCLVisualizer so user can get a whole area indexes
* Fix the ImageViewer shapes overlay using vtkContextItem so they now appear with transparent background

### `libpcl_tools:`

* Added a PNG to PCD converter

### `libpcl_io:`

* Added support for the Velodyne High Definition Laser (HDL)
* Add support for foo/bar vertex property in PLY file reading

## *= 1.6.0 (2012-07-15) = :: "About time" =*

The most notable overall changes are:


### `PCL Tools:`

* Added a tool for interfacing the marching cubes algorithms
* Added a PLY To PCD conversion tool
* Added a command line tool for transforming datasets based on their viewpoint
* Implemented feature #605: New octree visualizer example (Contributed by Raphael Favier. Thanks!)
* Updated "openni_save_image" to save both RGB and depth images as TIFF
* Added two command line tools for converting PCD data into PLY and VTK formats
* Fix for #504: Make ShapeContext and SpinImage uniform (thanks David!)

### `libpcl_common:`

* Fixed bug #658: Compiler error: undefined reference for getEulerAngles in pcl/common (thanks Gioia!)
* Moved towards a unified "eigen.h" header that only includes the Eigen headers that we need. 
* Added boost 1.48 and 1.49
* Added a default "PointRepresentation" type for "ShapeContext" (thanks Elizabeth!)
* Added a new "PlanarPolygon" class for representing 2D planar polygon regions
* Added "SetIfFieldExists" functor helper to copy data from a variable into a point field, if it exists
* Added a helper functor ("CopyIfFieldExists") for copying out specific data out of a PointT field. See "test_common.cpp" for usage examples
* Added point value initialization by default in constructors for "PointXYZI", "Normal", "PointXYZHSV", "PointXYZRGBL", and "PointXYZRGB"
* Updating transforms.hpp to ensure that point fields are copied when applying affine transform to a specific set of indices.
* Added error messages on failure of aux functions for PointCloud2, "pcl::concatenatePointCloud" and "pcl::concatenateFields"
* Fixed win32 compilation error on test_plane_intersection 
* Implemented plane intersection feature (feature #644) with a related unit test 
* Added kissfft library 
* Bugfix of eigen22 version for smallest eigenvalue/vector 
* Add specialization for pcl::RGB point type 
* Intensity field accessor moved from keypoints/sift to common to be shared by others 
* Fixed some valid usages of point traits (e.g. static_cast::type>(...) ) on GCC 4.4.3 by explicitly instantiating some assert template long before it should actually be needed. 
* Added ESF Histogram 640 to point types 

### `libpcl_filter:`

* Added command line option "-keep" to preserve the organized data structure after a "Passthrough" filter
* Implemented feature #663: flexible comparison for conditional_removal filter (Thanks Julian!)
* Fix for bug #672: ijk grid coordinates in VoxelGrid computed differently in different functions (thanks Nicholas!)
* Fix for #572: Example "Euclidean Cluster Extraction" crashes / bug in VoxelGrid filter
* Improved performance of 3x getFieldIndex and removed dependency on pcl_io
* Fixed and implemented feature #518: VoxelGrid<> performance improvement (roughly 50+ times) (thanks Radoslaw!)
* Fixed the visual studio compilation error described in feature #614 (thanks Remus!)
* Fixed bug #674 (Elements in leaf_layout_ in VoxelGrid are not reset between calls to VoxelGrid::filter), Thanks Nicholas for the patch!
* Work on issue 614: updated the PassThrough filter to derive from FilterIndices instead of Filter 
* Fix: use a makInfinite function to annihilate points in zero_padding cases 
* Add new class to handle convolution in 3D space for radial basis kernel 
* Modifying VoxelGridCovariance to allow for control over minimum number of points and eigen value inflation for singularity prevention. 

### `libpcl_visualization:`

* Overloaded "addRectangle" in "ImageViewer" with several useful methods that take as input 3D min-max points and organized datasets, as well as image masks to create 2D rectangles on screen
* Renamed "addBox" to "addFilledRectangle" in "ImageViewer"
* Added "addPlanarPolygon" methods for "ImageViewer" to display planar polygonal contours in the image
* Added new "addMask" methods to "ImageViewer" for displaying 2D image masks from a given "PointCloud<T>"
* "PCLVisualizer" now has a new "addCube" method for passing {x, y, z}_{min, max} directly to the screen (thanks Jeremie!)
* Patch for two cases of mismatched new/free cases for PCL<->VTK data exchange (thanks Peter!)
* Fixed a problem with "removeLayer" which was causing shapes to flicker on screen on sequences like: "removeLayer; addShape; spinOnce"
* Added "addLine" to "ImageViewer" for displaying 2D lines
* Added "PCLVisualizer::close" to close the interactor window and "PCLVisualizer::set{Size, Position}" to set the window size and position on screen added image viewer to the docs
* Added better support for 2D image visualization (via "pcl::visualization::ImageViewer"): add/remove layers with different transparency, add 2D shapes (rectangles, circles, points, etc)
* Added "wasStopped" to "ImageViewer" to check if the window has been closed
* Fixed an issue in pcd_viewer where the PointPicking callback wasn't functioning properly
* Added "setPosition" to "pcl::visualization::ImageViewer" for allowing the image viewer to be moved somewhere else on screen
* Added two additional "addPointCloud" helpers for directly displaying "sensor_msgs::PointCloud2" data 
* Added "addPointCloud" method for "sensor_msgs::PointCloud2" data (useful to bypass the conversion to XYZ for "pcd_viewer")
* Added the capability to remove a cloud when "removeShape" is called, to preserve API backward compatibility (a "PolygonMesh" is not treated as a "CloudActor" inside "PCLVisualizer")
* Fixed a bug where the scalars were not updated properly on "updatePointCloud" thus causing VTK warnings on the console
* Fixing issue #105 (PCLVisualizer::spinOnce don't work on Win32 system). Thanks Thibault and Alessio.
* Added "PointXYZRGBA" callbacks for "OpenNIGrabber" and "PCLVisualizer".
* Fixed two bugs where a segfault would occur in "addPolygonMesh" when the input cloud would be empty, as well as #563 : PCLVisualizer::addPolygonMesh crash (thanks Mourad!)
* Fixed a bug where changing the point color using "setPointCloudRenderingProperties" would not update the actor's colors on screen
* Fix for #532: "PCLVizualizer::addCoordinateSystem()" switch from "Eigen::Matrix4f" to "Eigen::Affine3f" (thanks Aurel!)
* Fix for #524: ctrl problem with pcd_viewer (thanks Mourad!)
* Adding opt in flag -use_vbos to pcl_visuzlier. It's still quite buggy, but shouldn't affect the visualizer unless this flag is passed. 
* Added vtkVertexBufferObject/Mapper and moved pcl_image_canvas_source2d 
* Added case handling where an actor might not have a valid viewpoint_transformation matrix resulting in a seg fault if pressing ALT+R 
* Fixed bug in displaying addCoordinateSystem
* Added method to visualize intensity gradients 
* Added zoom functionality for ALT + Scroll-Wheel 
* Merged openni_viewer_simple with openni_viewer with all the available options except XYZI 

### `libpcl_octree:`

* Fixed bug #693 - bounding box adaption affected change detection results
* Applied patch by Robert Huitl, Issue #651, define maxVoxelCount in octree raycasting
* Fixed OSX compiler warnings 
* Fixed bug #683 - octree depth changes during stream compression 
* Added new octree key class 
* Added bounding box checks in method isVoxelOccupiedAtPoint (octree pointcloud class) 
* Removed maxKeys limit in octree key generation method 
* Added range checks for integer keys in octree classes, extended octree key class 
* Fixed bug #620 (octree search fails if point cloud with indices is given) in octree pointcloud class 

### `libpcl_io:`

* Added "DepthImage" signals/callbacks for "PCDGrabber"
* Support for loading TAR-ed LMT files
* Added support for TAR-PCD files for "PCDGrabber". Simply use "tar cvf file.tar *.pcd" and use "PCDGrabber" on it afterwards
* Fixed a bug in the "PointCloud<MatrixXf>" feature estimation and I/O regarding the fields "count" property
* Added a "saveVTKFile" method helper for saving "sensor_msgs::PointCloud2" data
* Added support for reading PCD ascii and binary files (binary_compressed not implemented yet!) for pcl::PointCloud<Eigen::MatrixXf> datatypes.
* Implemented and tested a fix for feature request #558: Implement IO for PointCloud<MatrixXf>
* Adding missing openni_device files from install
* Fixed a bug in ply reader
* Fixed bug #683 - dropping empty point clouds during stream compression 
* Add functions to convert between PCL and VTK data structures. 
* Handle red_diffuse, blue_diffuse and green_diffuse vertex properties 
* Fix bug where normals were written before RGB 
* Bugfix for #627 - Wrong include path for ply header

### `libpcl_features:`

* Fixed bug #676 in PCL Feature Search (thanks Adam!)
* Fixes compilation problem on Windows due to size_t variable in omp loop.
* Implemented feature request #661: border management in integral image normal estimation
* Fixed PFHRGBEstimation bug, thanks Luis 
* Bug fix in SHOT feature, thanks wack 
* Fixed a bug which caused some normals to point in the wrong direction 
* Added Camera Roll histogram
* Fix bug in index used for normal selection 
* Added esf feature 
* Added setViewPoint functionality and useSensorOriginAsViewPoint method to reset the viewpoint 
* Fixed bug #576 - CVFH respect setIndices now  
* Fixed issue #550: Uninformative error in VFHEstimation (thanks David!)
* Fix #527: Bug in IntegralImageNormalEstimation (thanks Christoph!)
* Fixed #544: overflow in implicit constant conversion warnings (thanks David!)
* Modified SHOT omp so that the default computation of the reference frames
* SHOT: Fixed invalid access when keypoints have increased from the previous call.
* Setting sensor origin and orientation correctly for normal point cloud 
* Fixed a bug in estimating the orientation of the normal (view point wasn't initialized in the constructor) 
* Bug fix: if the cloud is not dense search for neighbours could raise an excpetion 
* Bug fix: SHOT accepts only radius search 

### `libpcl_segmentation:`

* Fixed a few issues in the new segmentation classes where some comparators didn't have the appropriate "Ptr" and "ConstPtr" well defined
* Fixed a bug where points passed onto the search method were not checked for NaN/Inf in "pcl::SegmentDifferences"
* Added a missing "setDistanceFromOrigin" to "SACSegmentationFromNormals" for "SACMODEL_NORMAL_PARALLEL_PLANE" (thanks A. Barral)
* Fix for #134 (Prism Extraction on Table top (flipping normal fails))
* Fixed a segmentation fault in "SACSegmentationFromNormals" caused by calling "segment" without passing the input XYZ or normal data
* Fixed a bug in organized connected component segmentation. Previously would crash if the first pixel had a valid depth (doesn't occur on kinect data). 
* Bugfix of issue # 675: Euclidean cluster extraction access violation. Thanks to Icy for helping find the solution 
* Fixed bug #673 in EuclideanClusterExtraction 
* Added new code for min cut segmentation 
* Fixed bug #681: member variable was not set in constructor (thanks Bhaskara!) 
* Added a threshold to MultiPlaneSegmentation for curvature, allowing us to discard smooth (but non-planar) regions. 
* Added optional projection for multi_plane_segmentation. 
* Added some new comparators for use with OrganizedConnectedComponents including RGB and edge aware 
* Added additional fucnction call for segmentAndRefine. 
* Added a comparator for doing euclidean clustering on organized point clouds. 
* Added fast SeededHueSegmentation implementations 
* Fixed segfault in multi plane refinement and functorized the comparison. 
* Improved MultiPlaneSegmentation to allow refinement of regions, and support different comparators. 
* New classes AutomatedSegmentation, AutomatedTreeSegmentation 
* Added OrganizedConnectedComponentSegmentation, which is a general class for use with organized point clouds  and can take user specified comparators. 
* Added OrganizedMultiPlaneSegmentation which returns all planes in an organized cloud, along with the PlaneCoefficientComparator needed to do this. 

### `libpcl_surface:`

* Fixed a grave bug where "setIndices" was not used in "ConvexHull"
* Added fix for #562: pcl::ConcaveHull crashes on an empty cloud (thanks Mourad!)
* Added "PCLSurfaceBase" base class for "MeshConstruction" and "SurfaceReconstruction", to unify the API around "setSearchMethod(&Search)" and "reconstruct(&PolygonMesh)"
* Added GPU accelerated convex hull. 7x for Ramesses dataset (700k points)
* Added a setDimension function to concave hull, so users can specify desired dimensionality of the resulting hull. If no dimension is specified, one will be automatically determined.
* Fixed bug #692 - MovingLeastSquares indices issues
* Added curve fitting and trimming of surfaces to examples/surface/example_nurbs_fitting_surface.cpp
* Added iterative fitting routines for curve fitting surface::on_nurbs::Triangulation - added convertion functions for nurbs curve to line-polygon - added convertion functions for nurbs surface and curve to PolyMesh 
* Added flag to enable/disable usage of UmfPack for fast solving of sparse systems of equations - added triangulation functions to convert ON_NurbsSurface to pcl::PolygonMesh 
* Added bug fix in ConcaveHull, thanks to summer.icecream
* Added marching cubes using RBF and Hoppe SDF
* Pushed new functions that perform texture-mapping on meshes.
* Fix: issue #646 (vtk_smoothing not copied)
* Added new functionalities to TextureMapping: Find occlusions based on raytracing in octrees, UV mapping based on Texture resolution and camera focal lenght.
* Relaxing dimensionality check threshold on concave_hull, so that 3D hulls should no longer be calculated on 2D input. 
* Added poisson filter

### `libpcl_keypoints:`

* Added combined Harris keypoint detector to detect corners in image, in 3D and combined ones
* Fix bug #691 (Compiler error: undefined reference for setMinimalDistance in pcl/keypoints/harris_keypoint2D)
* Fixed the #637 pitfall in harris3D and harris6D 
* Added implementation of 2D corner detectors 

### `libpcl_geometry:`

* Added a new PCL library for computational geometry
* Added bugfix to remove self intersecting polygons 
* Fixed some refinement corner cases for polygon approximation 
* Added line iterator class for iterating over e.g. an image/organized cloud in the pixel space.


### `libpcl_search:`

* Added NaN checks in "pcl::search::FlannSearch" 
* Skip infinite neighbor candidates in pcl::search::OrganizedNeighbor::radiusSearch 
* Added point projection method for organized search 

### `libpcl_tracking:`

* Fixed issue #514: SpinImages segfault when using setKSearch (thanks David!)
* gcc-4.7 compatibility issues in pcl::tracking fixed. Thanks to Rich Mattes

### `libpcl_sample_consensus:`

* Implemented feature #589: Add sac_model_cone and sac_model_normal_sphere (contributed by Stefan Schrandt. Thanks!)
* Fix for #616: pcl::ProgressiveSampleConsensus<PointT>::getRandomSamples unimplemented (thanks Mourad!)
* Fix for #498: Bug in setOptimizeCoefficients for pcl::SACMODEL_SPHERE.
* Fixed bug #648: code was checking for plane perpendicular to given axis instead of parallel 
* Applied patch for feature request #649: Locally constrained model generation for Sample Consensus 

### `libpcl_registration:`

* Fixed issue #619: Cannot call pcl::registration::TransformationValidationEuclidean::validateTransformation (thanks Raphael!)
* Added code for #592: correspondence estimation based on normal shooting (coded by Aravindhan. Thanks!)
* Fixed #533: WarpPointRigid missing EIGEN_MAKE_ALIGNED_OPERATOR_NEW (thanks Radoslaw!)
* Correct overlapping grid distribution in 2D NDT implementation


## *= 1.5.0 (2012-02-22) = :: "Better late than never" =*

The most notable overall changes are:

* new framework for PCL examples in the example/ folder
* added the ICCV 2011 tutorial code to the list of bundled applications
* a brand new PCL tracking library (`libpcl_tracking`)
* new, well-tested implementations for `OrganizedNeighbor` and `BruteForceSearch` in `libpcl_search`
* considerable speedups for: eigendecomposition, covariance matrix estimation, and normal estimation (now running at 30Hz on VGA data)
* a lot of bug fixes and code refactorizations and optimizations (thanks to everyone that contributed!)
* compilation now defaults to Eigen column major again, but all the code in PCL should compile and work against both row major and column major matrices.
* libpcl_range_image is now incorporated into pcl_common, no extra linking required.

### `libpcl_common`

* added two new methods to compute centroid and covariance at once. Both are faster than calculating first the centroid and afterwards the covariance matrix. The float versions of the new methods are twice as fast but less accurate, whereas the double versions are more accurate and about 35% faster.
* fixed a few issues in our ScopeTime class where the creation of the string as well as the cerr were contributing negatively to the time measurements
* added empty() to PointCloud<Eigen::MatrixXf> to return whether the matrix is empty or not
* added size to PointCloud<Eigen::MatrixXf> and fixed the constructor warnings exposed by -pedantic
* added two new Point Types: Axis and ReferenceFrame as they are used in the Reference Frame calculations
* added a helper functor (`CopyIfFieldExists`) for copying out specific data out of a PointT field. See `test_common.cpp` for usage examples
* implemented and tested a fix for feature request #558: Implement IO for PointCloud<MatrixXf>


### `libpcl_filters`

* refactored RandomSample to return sorted indices. In addition it now runs nearly twice as fast at it did previously.
* fixed and implemented feature #518: VoxelGrid<> performance improvement (roughly 50+ times)

### `libpcl_kdtree`

* removed unneeded mutices, added asserts instead of constant checks (the user is responsible for passing in valid data!) and sped up a few other things in KDTreeFLANN

### `libpcl_search`

* adapted use of flann::Matrix to new semantics of the 'stride' parameter, fixing incorrect results and a crash in FlannSearch
* changes that affect pcl_features, pcl_search, pcl_octree, pcl_registration, pcl_surface:

   * we used to have 3 "overloads" for nearestKSearch and 3 for radiusSearch that every class inheriting from pclk::KdTree or pcl::search::Search needed to implement. Because most of the children always had the same implementation (copy&paste) we moved it up in the base class, made the methods virtual, and only children that need to change that behavior will overwrite it.
   * the max_nn parameter from radiusSearch has been changed from int with a default value of -1 to an unsigned int with a default value of 0. Each radiusSearch implementation should check for 0 or >= max_points_in_cloud and branch accordingly (if needed).
   * const correctness is now implemented everywhere in all nearestKSearch and radiusSearch operations

* added an implementation of KdTreeFLANN for PointCloud<Eigen::MatrixXf> data structures
* FlannSearch: don't include flann/matrix.h, move all FLANN includes to impl/flann_search.hpp. Also adapt max_nn parameter in radius search to change r3962 (pass 0 to search for all neighbors instead of -1)
* completely new and shiny `BruteForceSearch` and `OrganizedNeighbor` classes


### `libpcl_octree`

* fixed issue #513 (bug in octree iterator)
* fixed octree bounding box adaption
* fixed range test in getPointByIndex method (octree)
* fixed bug #604 & removed compiler warnings in Visual Studio 10
* fixed numerical problem in octree pointcloud class when points lie very close to the bounding box bounds
* improved performance with std::deque in OctreeBreadthFirstIterator class
* added breadth-first octree iterator and depth-first octree iterator classes, restructured octree iterator classes, moved "getVoxelBounds" method from OctreeIterator class to OctreePointCloud class


### `libpcl_sample_consensus`

* added explicit checks + errors for NaN covariance matrices
* fixed a crash on 32bit platforms (thanks bharath)
* fix for #498: Bug in setOptimizeCoefficients for pcl::SACMODEL_SPHERE. The issue actually exposed a grave bug introduced with r2755, when we switched from CMinPack to Eigen. The fix touches all segmentation models in pcl_sample_consensus using Levenberg-Marquardt, and TransformationEstimationLM in libpcl_registration. Also added unit test to catch this in the future.

### `libpcl_io`

* fix row-major/column-major issues in I/O (#578 related)
* fixed #593 by removing unimplemented method declarations
* added a `saveVTKFile` method helper for saving `sensor_msgs::PointCloud2` data
* added support for reading PCD ascii and binary files (binary_compressed not implemented yet!) for pcl::PointCloud<Eigen::MatrixXf> datatypes.
* implemented and tested a fix for feature request #558: Implement IO for PointCloud<MatrixXf>
* added `PointXYZRGBA` callbacks for `OpenNIGrabber` and `PCLVisualizer`
* fixed issue #565
* fixed a few bugs in PCDReader regarding uint8/int8 data values and added a comprehensive unit test for all possible combinations of data types that we support in PCL
* added isValueFinite for checking whether a value T is finite or not, and rewrote the PCDReader bits of code that perform the is_dense checks
* integrated libply written by Ares Lagae as an independent component in io
* rely on ply library for PLY parsing, remove the old parser and use the freshly integrated libply, change the namespace ::ply to ::pcl::io::ply to avoid any potential issue


### `libpcl_surface`

* Updated convex hull.  It should now be somewhat faster, and allows users to optionally specify if the input is 2D or 3D, so the input dimension need not be calculated every time.
* fix for #562: pcl::ConcaveHull crashes on an empty cloud (thanks Mourad!)
* added option to ensure that triangle vertices are ordered in the positive direction around the normals in GreedyProjectionTriangulation
* fixed issue #489 in GreedyProjectionTriangulation
* fixed triangle vertex ordering issues in OrganizedFastMesh
* MarchingCubes now adheres to the new pcl_surface base class architecture
* added PCLSurfaceBase base class for MeshConstruction and SurfaceReconstruction, to unify the API around setSearchMethod(&Search) and reconstruct(&PolygonMesh)
* added marching_cubes_reconstruction tool

### `libpcl_features`

* fixed issue #550: Uninformative error in VFHEstimation (thanks David!)
* fix for #504: Make ShapeContext and SpinImage uniform (thanks David!). Note: this is an API breaking change, as `setInputCloudWithNormals`, which was deviating from the `FeatureEstimation` API is now replaced with patched `setInputCloud` and `setInputNormals`, thus no longer causing an error. However, we are confident with this change as the API was broken, so this is a bug fix.
* fixed a bug in the `PointCloud<MatrixXf>` feature estimation and I/O regarding the fields `count` property
* fixed an issue where the header stamp should only be copied from a PointCloud<T> into a PointCloud<MatrixXf> in PCL standalone, and not in PCL in ROS
* optimized RSD computation
* speeded up normal estimation in IntegralImageNormalEstimation
* renamed computeFeature (MatrixXf&) into computeFeatureEigen (MatrixXf&) and compute (MatrixXf&) into computeEigen (MatrixXf&) to keep the compiler happy, avoid multiple inheritance, and use clearer semantics between the two different output estimation methods
* fixed an issue where if radius would be defined for VFHEstimation, Feature::initCompute would error out
* fixed issue #539 (fpfh_estimation tutorial)
* fixed issue #544 (overflow in implicit constant conversion warnings)
* fixed issue #527 (bug in IntegralImageNormalEstimation)
* fixed issue #514 (SpinImages segfault when using setKSearch)

### `libpcl_registration`

* added TransformationValidation base class and TransformationValidationEuclidean as a measure to validate whether a final transformation obtained via TransformationEstimation when registering a pair of clouds was correct or not
* added getter/setter for maximum number of inner iterations to control number of optimizations in GeneralizedIterativeClosestPoint
* fixed issue #507
* fixed issue #533 (WarpPointRigid missing EIGEN_MAKE_ALIGNED_OPERATOR_NEW)
* fixed issue #464 (already defined function pointers)
* fixed a grave bug on 32bit architectures where MapAligned was used incorrectly

### `libpcl_segmentation`

* fixed a segmentation fault in SACSegmentationFromNormals caused by calling segment without passing the input XYZ or normal data
* added a missing `setDistanceFromOrigin` to `SACSegmentationFromNormals` for `SACMODEL_NORMAL_PARALLEL_PLANE` (thanks A. Barral)

### `libpcl_visualization`

* added `PointXYZRGBA` callbacks for `OpenNIGrabber` and `PCLVisualizer`
* fixed two bugs where a segfault would occur in `addPolygonMesh` when the input cloud would be empty, as well as #563 : PCLVisualizer::addPolygonMesh crash (thanks Mourad!)
* added the capability to remove a cloud when `removeShape` is called, to preserve API backward compatibility (a `PolygonMesh` is not treated as a `CloudActor` inside `PCLVisualizer`)
* fix #472: at startup the camera view pointis set to last added point cloud, but with the key combination ALT+r on can iterate through the view points of the visualized point clouds.
* added `addPointCloud` method for `sensor_msgs::PointCloud2` data (useful to bypass the conversion to XYZ for `pcd_viewer`)
* added `PCL_VISUALIZER_IMMEDIATE_RENDERING` for processing large datasets
* refactorized `pcd_viewer` to consume less memory by skipping the intermediate conversion to `PointCloud<PointXYZ>`, clearing the `PointCloud2` data once converted to VTK, and enabling intermediate mode
* added `setPosition` to `pcl::visualization::ImageViewer` for allowing the image viewer to be moved somewhere else on screen
* changed the `openni_image` viewer tool to display both RGB and depth images
* fixed a nasty bug where the scalars range were not correctly updated in the mapper on updatePointCloud thus causing weird VTK warnings at the console
* added setupInteractor to allow QVTK customizations
* pcd_grabber_viewer now shows the recorded images in PointXYZRGB clouds, and can use fps=0 and trigger each new frame manually by pressing SPACE
* fixed an BGR/RGB issue in image_viewer
* added setKeyboardModifier to change the default keyboard modifier from Alt to Shift or Ctrl so that in QVTK or other widgets, we can still use shortcuts like Mod+r, Mod+f, etc
* PCLVisualizer is now working with VTK compiled for cocoa, no X11 necessary on OS X
* build bundle executable for pcd_viewer if VTK is compiled with Cocoa. With that the keyboard events are handled by the application
* modified openni_passthrough to use QVTK instead of VTK
* added PCLVisualizer zoom in/out via /- [ alt]
* implemented feature #516 (pcd_viewer multiview with .vtk files)
* added showMonoImage for displaying a monochrome unsigned char 2D image in PCLVisualizer
* added setFullScreen and setWindowBorders methods to allow an user to change the full screen/border properties of the PCLVisualizer GUI
* fix for issue #519 (set the minimum number of cloud points to 1 in vtkLODActor)
* fix for issue #524 (ctrl problem with pcd_viewer)
* fix for issue #532 (PCLVizualizer::addCoordinateSystem() switch from Eigen::Matrix4f to Eigen::Affine3f)
* bug fix in pcd_grabber_viewer: now sorting the read .pcd file by name before showing them (was in random order before)
* fixed issue #525 (pcd_viewer did not handle nan values correctly in fields other than x, y, z)
* fixed a bug where changing the point color using setPointCloudRenderingProperties would not update the actor's colors on screen
* workaround for displaying FPS rate on PCLVisualizer - disabled backface culling because of known VTK issue

## *= 1.4.0 (2011-12-31) = :: "Happy New Year" =*

The most notable overall changes are:

* changed the default on the order of data in all Eigen matrices to be **row major** `-DEIGEN_DEFAULT_TO_ROW_MAJOR` (Eigen defaults to column major). Be careful if your code was using 1D index operators, as they might be broken! See http://eigen.tuxfamily.org/dox/TopicStorageOrders.html and http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2011/12/msg00062.html.

### `libpcl_common`

* added float union to the pointXYZRGBL type, in order to aid PCD_viewer compatibility
* bugfix: `pcl::computeCovarianceMatrixNormalized` did not normalize for each implementation
* fixed bug #421: `at(u,v)` should return a const reference
* added `InitFailedException` and `UnorganizedPointCloudException`. Created `PCL_EXCEPTION` helper macro
* added the GFPFH global descriptor point type
* added gaussian kernel class and utilities
* added `isTrivial` to `pcl::PointRepresentation` which allows avoiding copy operations in special cases, thus speeding up operations like `rep->isValid (p)`
* added `eigen33` for smallest eigenvalue/vector and only eigenvalues version
* added `PointCloudColorHandlerHSVField` implementation
* deleted wrong definition of density in `pcl::transformPointCloud`
* added copy constructor for PCLBase
* fixed `PointCloud`'s in range inserter: width and height were not updated
* added `PCL_MAJOR_VERSION` and `PCL_MINOR_VERSION` #defines per Ryan's request
* fixed the `getMatrixXf` advanced user API call to return the correct values now that we are forcing Eigen Matrices to be row major in 1.x
* moved the content of `win32_macros.h` into `pcl_macros.h`
* added an `isFinite` method to check for x/y/z = NaN in point_types.hpp
* moved internal headers from `pcl/ros/point_traits.h` into `pcl/point_traits.h` and `pcl/ros/for_each_type.h` to `pcl/for_each_type.h` (these headers are for internal use only and should have never been imported by user code - however pcl_ros in perception_pcl_unstable will need to be modified when following trunk after this revision, as that is the only external piece of code that should import these headers)
* fixed one of the constructors for `pcl::PointCloud<T> (cloud, indices)` that was incomplete
* no longer checking "frame_id" parameters in header for operator += in `pcl::PointCloud<T>`
* added operator + for `pcl::PointCloud<T>`
* improved doxygen documentation all around `libpcl_common`
* added new specialization for `pcl::PointCloud<Eigen::MatrixXf>` - for *advanced users only*! (expect the API to change in trunk if needed)
* moved `NdCopyPointEigenFunctor` and `NdCopyEigenPointFunctor` from `voxel_grid.h` to `point_cloud.h`
* added `CloudProperties` class for optional properties for `pcl::PointCloud<Eigen::MatrixXf>`, and moved `sensor_origin_` and `sensor_orientation_` there (as `properties.sensor_origin` and `properties.sensor_orientation`), and removed `header`, while keeping `header.stamp` as `properties.acquisition_time`
* fixed the copy constructor (vector<size_t> -> vector<int>) and added an implementation for the + operator
* added a general convolution class. Convolution handles point cloud convolution in rows, columns and horizontal directions. Convolution allows for 3 policies: zero padding (default), borders mirroring and borders duplicating through `PointCloudSpring` class
* refactorized `PointCorrespondence` and `Correspondence` into the same thing, per issue #458
* added `ChannelProperties` in preparation for 2.0
* fixed bug #445: Overloaded the setIndices method in PCLBase so that it can accept `boost::shared_ptr<const std::vector> >`
* disabled unit test for `getMatrixXfMap` in DEBUG mode, as it was failing due to eigen's strictness on aligned/unaligned data, and added const+non consts versions

### `libpcl_filters`

* fixed bug in `random_sample.cpp`; The `PointCloud2` filter wasn't working causing the unit test to fail
* crop_box filter now supports `PointT` and `PointCloud2` point clouds
* added a normal space sampling filter
* fixed bug #433: `pcl::CropBox` doesn't update `width` and `height` member of output point cloud
* fixed bug #423 `CropBox` + `VoxelGrid` filters, order changes behaviour (using openni grabber)
* add `CropHull` filter for filtering points based on a 2D or 3D convex or concave hull. The ray-polygon intersection test is used, which relys on closed polygons/surfaces
* added clipper3D interface and a draft plane_clipper3D implementation
* removed spurious old `ColorFilter` class (obsolete, and it was never implemented - the skeleton was just lurking around)
* better Doxygen documentation to `PassThrough`, `VoxelGrid` and `Filter`
* moved the `set/getFilterLimits`, `set/getFilterFieldName`, and `get/setFilterLimitsNegative` from `Filter` into `VoxelGrid` and `PassThrough`, as they were the only filters using it. The rest were not, thus leading to user confusion and a bloated API.

### `libpcl_kdtree`

* fixed bug #442: reversed template parameters preventing use of nearestKSearchT and radiusSearchT in `pcl::search::Search` and `pcl::kdtree:KdTree`

### `libpcl_search`

* fixed a bug in `OrganizedNeighbor` where one of the `radiusSearch` signatures was working only if the cloud is _not_ organized (#420 - thanks Hanno!)
* updated `pcl::search:OrganizedNeighbor` to be back functional, radiusSearch is working back, 4-6x faster then KdTree, Knearest still need to implement this
* changed `pcl::search::FlannSearch`: fixed style of class and tests, moved impl to hpp
* cleaning up `OrganizedNearestNeighbor` as well as implementing one of the missing `radiusSearch` methods
* improved documentation for libpcl_search and silenced all Doxygen warnings
* removed auto (it needs to be completely rewritten and was left in a very bad state)
* made sure all the output of the search method is consistently returning 0 in case the nearest neighbor search failed and not -1
* updated kdtree wrapper code for the latest FLANN release

### `libpcl_octree`

* added method `getVoxelBounds` to octree iterator & removed result vector reserves
* improved documentation for `libpcl_octree` and silenced all Doxygen warnings

### `libpcl_sample_consensus`

* fixed an omission of the sample size declaration for `SACMODEL_PARALLEL_LINES` (thanks Benergy)
* replaced rand () with boost random number generators and fixed all failing unit tests
* workaround to get rid of infinite loops if no valid model could be found

### `libpcl_io`

* refactorization and consistent code indentation + make sure the timestamp is not set if we use PCL in ROS
* fixed an older issue where the `is_dense` flag was not set appropriately when reading data from a binary file (thanks Updog!)
* fixed an issue in the `PLYWriter` class where the element camera was not correctly set thus leading to crashes in Meshlab (thanks Bruno!)
* the VTK library loads vertex colors in PLY files as RGB. Added that to the polymesh loader.
* fixed 2 bugs in writing `PolygonMesh` and unpacking RGB
* fixed a bug in the .OBJ exporter to strip the path from the material library filename (thanks Robert!)
* added support for exporting vertex normals to the .OBJ (`pcl::io::saveOBJFile`) file exporter (thanks Robert!)
* fixed a 32bit/64bit issue in pointcloud compression
* added method to write obj files from `PolygonMesh`
* added serial number support for Windows using mahisorns patch. Thanks to mahisorn
* overloaded callbacks for `OpenNIGrabber` to output `PointCloud<Eigen::MatrixXf>` datasets
* added the possibility to write binary compressed Eigen data to disk using two new methods: `generateHeaderEigen` and `writeBinaryCompressedEigen`. Performance improvements to 30Hz I/O
* fix for #463 (Missing Symbol rgb_focal_length_SXGA_)
* fix: rgb values need to be packed before saving them in PointCloud2 for `PLYWriter`
* added example code for accessing synchronized image x depth data
* added support for the Stanford range_grid element and obj_info for PLY files. If you chosse to use range_grid instead of camera then only valid vertices will be written down to the PLY file.

### `lipcl_keypoints`

* added refine method for `Harris3D` corner detector
* rewrote big parts of the NARF keypoint extraction. Hopfully fixing some stability issues. Unfortunately still pretty slow for high resolution point clouds.
* fixed bug #461 (SIFT Keypoint result cloud fields not complete); cleaned up the line-wrapping in the error/warning messages

### `libpcl_surface`

* fixed a bug in `MarchingCubes`'s `getIndexIn1D` which led to the result variable to overflow when the data_size was larger than something around 2^10 (thanks Robert!)
* reviewed and slightly modified mls implementation. Added `MovingLeastSquaresOMP` OpenMP implementation
* new architecture for the mesh processing algorithms using VTK: `MeshProcessing`
* overloaded `reconstruction` and `performReconstruction` in `SurfaceReconstruction` to output a `PointCloud<T>` and `vector<Vertices>` as well
* added a new class called `MeshConstruction` per API design decision to split the surface reconstruction methods into topology preserving (`EarClipping, `OrganizedFastMesh`, `GreedyProjectionTriangulation`) and the rest. The new class implements a `reconstruction/performReconstruction` for `PolygonMesh` (for backwards compatibility purposes) and a faster set of the same methods for `vector<Vertices>`
* refactorized `GreedyProjectionTriangulation` by making it inherit from the new `MeshConstruction` base class, and removed a lot of old buggy code
* overloaded `performReconstruction (PointCloud<T> &, vector<Vertices> &)` per the new `SurfaceReconstruction` API
* fixed a bug in `OrganizedFastMesh` where the x/y/z indices were assumed to be 0/1/2, and made it part of the `MeshConstruction` API
* optimizations for `OrganizedFastMesh`. Now in 30Hz+ flavor.
* fixed a segfault from last night's `EarClipping` refactorization and improved the efficiency of the algorithm considerably
* updated `ConvexHull` and `ConcaveHull` to inherit from the new `MeshConstruction` class
* renamed `mesh_processing.h` to `processing.h` and `performReconstruction/reconstruct` to `performProcessing/process` for the new `MeshProcessing` API


### `libpcl_features`

* fixed bug #439: 3DSC unit test fails on MacOS
* fixed `IntegralImage2Dim` : setting first line to zero was buggy producing undefined output data.
* fixing an issue in `PrincipalCurvaturesEstimation` where the *pc1* and *pc2* magnitudes were not normalized with respect to the neighborhood size, thus making comparisons of different neighborhoods impossible (thanks Steffen!)
* fixed `ShapeContext3DEstimation` computation and unit tests by switching from `stdlib.h`'s random () to Boost.
* fixed a bug in `IntensityGradient`: need to demean also intensity values, otherwise its assumed that the hyperplane goes always through origin which is not true
* overloaded `compute` in `pcl::Feature` to save the output in an `PointCloud<Eigen::MatrixXf>`. Added partial specialization on `Eigen::MatrixXf` to all features and implemented `computeFeature (PointCloud<MatrixXf> &output)`
* major doxygenization work
* added optimization checks for `is_dense` for some features, implemented NaN output for most (per http://dev.pointclouds.org/issues/457)
* added new unit tests for the Eigen::MatrixXf output
* fixing the MacOS GCC 4.2.1 compiler segfault by removing the OpenMP #pragma (related to http://gcc.gnu.org/bugzilla/show_bug.cgi?id=35364 ?).
* fixed bug #468: refactored to make floor explicit.
* added a `setUseInternalCache` method for `PFHEstimation` to use the internal cache on demand only. Other fixes to PFH/FPFH/PPF signatures (histogram sums were not adding up to 100.0 in some cases)
* major improvements for integral images + added template specialization for one dimensional integral images

### `libpcl_registration`

* fixed some bugs in ELCH, thanks Frits for pointing them out
* Big ELCH cleanup.

   * elch.h (API changes):
    - Use Boost bundled properties for LoopGraph, every vertex stores a
      pointer to the corresponding point cloud in graph[vertex].cloud.
    - loop_graph_ is now stored as a shared_ptr.
    - loop_start_ and loop_end_ are saved as ints for now (will change in
      the future).
    - loop_transform_ is not stored as a pointer (may change in the future).

   * elch.hpp:
    - Remove call to PCLBase::initCompute() as ELCH doesn't use a single
     input cloud or indices.
    - Change loopOptimizerAlgorithm to use start and end of the loop as
      stored in the object.
    - Adapt to API changes in elch.h.

* added 2D implementation of Normal Distributions Transform for registration (Biber, Strasser; 2003), including example tool program.
* reworked internal `Registration` API: Remove private `computeTransformation` without guess, make `computeTransformation` with guess abstract, Adapt all classes implementing `Registration` accordingly.
* implemented `SampleConsensusInitialAlignment` with initial guess (thanks Dennis Guse)
* cleaned up the `CorrespondenceRejector` API, per #375
* apply Mourad's patch for #454; add more doc comments


### `libpcl_visualization`


* fixed an issue in `pcl::visualization::PointPickingCallback` where `iren->GetMousePosition ()` didn't seem to work on some VTK versions/systems, so we replaced it instead with `iren->GetEventPosition ()` (#444 - thanks Adam!)
* fixed a bug in `PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2>` where one of the offsets for Z was incorrectly calculated thus leading to erroneous data (#404 - thanks Lucas!)
* set the lighting off on `setShapeRenderingProperties` when the user tries to control the color of the object precisely (maybe add a different lighting on/off property later?)
* revert `pcl::visualization::PCLVisualizer::setShapeRenderingProperties` vtk5.2, default in ubuntu LTS, doesn't support the SetLighting method on vtkActor
* fixed an issue in `PointCloudColorHandlerRGBField` where using an "rgba" field was causing an error in pcd_viewer (#428 - thanks Maurice!)
* added `PointCloudColorHandlerHSVField`
* fixed the `PointPickingCallback` behavior on Windows 7: `iren->GetShiftKey ()` returns 4 instead of 1 (thanks bepe)
* added `updateFeatureHistogram` functionality to `PCLHistogramVisualizer`, per #456 (thanks Asil!)
* added patch from Adam Stambler (Bug #396)
* added `removePolygonMesh` to make the API more user friendly (`addPolygonMesh` was already existing, but the polygon meshes were removed using `removeShape` until now)
* made the polygon meshes actually implement `CloudActor` structures internally, so that we can use different colors per vertex
* added `updatePolygonMesh` and improved the efficiency of `addPolygonMesh` for fast online rendering (i.e., produced via `OrganizedFastMesh`)

## *= 1.3.1 (2011-11-30) =*

* fixed bug #428: in `PointCloudColorHandlerRGBField` where using an "rgba" field was causing an error in `pcd_viewer` (thanks Maurice!)
* fixed bug #404: in `PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2>` where one of the offsets for Z was incorrectly calculated thus leading to erroneous data (thanks Lucas!)
fixed bug #420: in `OrganizedNeighbor` where one of the `radiusSearch` signatures was working only if the cloud is _not_ organized (thanks Hanno!)
* fixed bug #421: `toROSMsg` referenced the address of a temporary return
* fixed bug #422: SHOT estimation did not work with different surface than keypoints
* fixed bug #430: an infinite loop in SAC methods if no valid model could be found
* fixed bug #433: `CropBox` did not update width and height member of output point cloud
* added missing using declarations for shot
* added a missing include file to transformation_estimation_point_to_plane.h; 
* added transformation_estimation_point_to_plane_lls.h/hpp to the CMakeLists file
* fixed an issue where the `is_dense` flag was not set appropriately when reading data from a binary file (thanks Updog!)
* fixed a bug in the `PLYWriter` class where the element camera was not correctly set thus leading to crashes in Meshlab (thanks Bruno!)
* fixed a bug in `PrincipalCurvaturesEstimation` where the *pc1* and *pc2* magnitudes were not normalized with respect to the neighborhood size, thus making comparisons of different neighborhoods impossible (thanks Steffen!)
* set the lighting off on `setShapeRenderingProperties` when the user tries to control the color of the object precisely
fixed bug in `PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2>` where y_point_offset and z_point_offset were not incremented
* fixed a bug in tools/compute_cloud_error
* fixed a crash in test_ii_normals.cpp
* removed unnecessary header from point_correspondence.h
* fixed unnormalized covariance matrix in `computeCovarianceMatrixNormalized`
* added missing include in std_msgs/Header.h
* added error checking for radius search in `extractEuclideanClusters`
* fixed a bug in `PassThrough` where NaN values were not correctly replaced by the user-specified value
* fixed a bug in ply_io when writing PolygonMesh
* fixed a bug in ply_io when unpacking rgb values

## *= 1.3 (2011-10-31) = :: "Gaudi" =*

From 1.3 we are constructing the Changelist for each library separately, as they can in theory be mixed and matched with older versions (though we officially do not support this yet). The 
most notable overall changes are:

* **removed wxWidgets as a dependency from PCL** and implemented the Image visualization classes using VTK
* **removed cminpack as a dependency from PCL** and implemented the LM optimization functionality using Eigen
* added a new library called **PCL Search** (`libpcl_search`) that has a more uniform API for nearest neighbor searches and stripped the unneeded functionality from `libpcl_kdtree` and `libpcl_octree`. Search now wraps KdTree and Octree for NN calls. **This is a MINOR API BREAKING CHANGE**. To change your code switch from:
```
pcl::KdTree -> pcl::Search (if the purpose is to use one of its children for search)
pcl::KdTreeFLANN -> pcl::search::KdTree
pcl::OrganizedDataIndex -> pcl::search::OrganizedNeighbor
```
* **improved MacOS support**
* improved documentation and unit tests
* added lots of application examples and demos. Note: not all have a complete functionality... we will try to clean this up in 1.4 and future releases.

### Build system changes

* define PCL_ROOT environment variable using the NSIS installer
* removing test+python from the dependency graph
* fixed bug #374
* remove cminpack from PCLConfig.cmake cmake and 3rdparty (switched to Eigen's Levenberg-Marquardt implementation)
* update NSIS.template.in : use CPACK_PACKAGE_INSTALL_REGISTRY_KEY as the registry key
* added the capability to build an all-in-one installer
* fixed the build system to work 100% on Android and MacOS


### `libpcl_common`

* add overriding for operator[] to shorten code 
* add a `setIndices` method that computes indices of points in a region of interest
* add `demeanPointCloud` with indices as `PointIndices` as well
* added eigen allocator to correspondence vectors (`pcl::Correspondences`) and adapted all registration modules => be sure to use `pcl::Correspondences` instead of `std::vector<pcl::Correspondence>`
* fixed a few doxygen errors
* added simple stop watch for non-scoped function time measurements and made `ScodeTime` derived from `StopWatch`
* fixed a bug in `getRejectedQueryIndices`, wrong output when order of correspondences have been changed
* moved `getRejectedQueryIndices` to pcl/common/correspondence.h
* added more doxygen documentation to the registration components
* marked all `getRemainingCorrespondences`-functions as DEPRECATED, we sould replace them with purely stateless version outside the class body
* fixed a const missing in `PolynomialCalculationsT` (#388 - thanks Julian!)
* add `PCL_DEPRECATED` macro, closes #354.
* added `PointXYZHSV` type and the conversions for it
* added check for endianness for the Android platform


### `libpcl_search`

* BIG changes introduced - migration from `KdTree` to `pcl::Search`: moved `OrganizedDataIndex` and `OrganizedNeighbor` classes to `libpcl_search`
* added new templated methods for `nearestKSearch` and `radiusSearch` for situations when PointT is different than the one the KdTree object was created with (e.g., KdTree<PointT1> vs nearestKSearch (PointT2 &p...)
* added two new methods for `getApproximateIndices` where given a reference cloud of point type T1 we're trying to find the corresponding indices in a different cloud of point type T2
* refactorized a lot of code in search and octree to make it look more consistent with the rest of the API
* fixed a bug in octree_search which was semantically doing something bad: for each `radiusSearch`/`nearestKSearch`/`approxNearestSearch` call with a PointCloudConstPtr, the octree was getting recreated. Changed the API to be consistent with the rest of PCL (including pcl_search and pcl_kdtree) where we pass in a PointCloud instead of a PointCloudConstPtr which simply calls searchMethod (cloud.points[i], ...)
* minor code optimizations
* renamed organized_neighbor.h header in pcl_search (unreleased, therefore API changes OK!) to organized.h
* disabled the auto-search tuning as it wasn't really working. must clean this up further
* remove all `approxNearestSearch` methods from the base `pcl::Search` class as they did not belong there


### `libpcl_io`

* use `stringstream` instead of `atof` because of locale issues in IO (note that we can't use the `ifstream` directly because we have to check for the `nan` value)
* added locale independent PCD ASCII i/o
* implemented `setMapSynchronization` for `pcl::PCDWriter`. When set, `msync()` will be called before `munmap()` in the `pcl::PCDWriter::write*` calls, which guarantees data reliability. Though I/O performance is 300% better when unset, data corruption might occur on NFS systems, as indicated by http://www.pcl-developers.org/PCD-IO-consistency-on-NFS-msync-needed-td4885942.html.
* added new `writeBinaryCompressed` functionality for general purpose `sensor_msgs::PointCloud2` data (which is still our generic data container in PCL 1.x)
* added additional unit tests for binary_compressed
* fixed a grave bug in `PCDReader` (most likely introduced a few releases ago due to code refactorization) where the data was incorrectly copied if a PCD ASCII file had a field with multiple count elements (field.count) as first. Binary files are not affected by this bug. Added an unit test to catch this in the future.
* added functionality for `openni_grab_frame` (added optional command line options, optional output filename, chose output format)
* changed to new location of samplesconfig.xml for OpenNI
* added signal and slot blocking into grabber. Using blocking to skip first frame in `openni_grabber`, since it is corrupted
* added PLY format file support in binary and ascii mode (requires boost::iostreams library)

### `libpcl_keypoints`

* added 3D versions of Harris/Noble/Lowe/Tomasi and Curvature-based keypoint detection... scale space still missing
* work on making `SIFTKeypoint` more flexible by saving scale only when the output point type contains "scale" (the catch is that all point types must be correctly declared via our macros - see the modifications in test_keypoints.cpp). This allows us to use a `SIFTKeypoint<PointXYZRGB, PointXYZRGB>` and thus removes the constraint on using `copyPointCloud` afterwards.
* fixed an issue in `SIFTKeypoint` where width/height were not correctly set


### `libpcl_features`

* specialize std::vector for Eigen::Matrix4f (alignment issue with msvc 32bit) in `SHOTEstimation`
* added a faster (eigen-based) integral image calculation => sped up normal estimation to 15Hz
* added Unique Shape Context (USC) feature descriptor
* added Shape Context 3D feature descriptor
* fixed a bug in the normalization factor of VFH for the distance component (only affecting if set distance component is true)
* fixed a few bugs regarding Windows build introduced in earlier commits
* BIG changes introduced - migration from `KdTree` to `pcl::Search` 
* merged libpcl_range_image_border_extractor into libpcl_features. There's absolutely no reason why we should have 2 libraries generated from the features module.

### `libpcl_filters`

* added `FilterIndices` functionality #315
* added a `RandomSample` filter which makes use of indices #323
* added a new (very fast) class for data decimation: `ApproximateVoxelGrid`
* fix for #369 (StatisticalOutlierRemoval crash when input dataset is empty)
* implemented feature request #346

### `libpcl_octree`

* added function `genVoxelBounds` to octree pointcloud class
* added octree neighbor search class
* added octree-ray-tracing patch to octree_search class
* buxfix in octree ray traversal function `getIntersectedVoxelCentersRecursive`
* added unit test for `getIntersectedVoxelCentersRecursive`
* added method `getIntersectedVoxelIndices` for getting indices of intersected voxels and updated unit test
* refactorized a lot of code in search and octree to make it look more consistent with the rest of the API
* fixed a bug in octree_search which was semantically doing something bad: for each `radiusSearch`/`nearestKSearch`/`approxNearestSearch` call with a PointCloudConstPtr, the octree was getting recreated. Changed the API to be consistent with the rest of PCL (including pcl_search and pcl_kdtree) where we pass in a PointCloud instead of a PointCloudConstPtr which simply calls searchMethod (cloud.points[i], ...)
* minor code optimizations
* renamed organized_neighbor.h header in pcl_search (unreleased, therefore API changes OK!) to organized.h
* disabled the auto-search tuning as it wasn't really working. must clean this up further
* remove all `approxNearestSearch` methods from the base `pcl::Search` class as they did not belong there

### `libpcl_registration`

* fixed a minor bug in `CorrespondenceRejectorSampleConsensus`: `getRemainingCorrespondences` tried to nput_ although it should only use the given input correspondences
* added missing implementation for `TransformationEstimationLM` on correspondence vectors
* added eigen allocator to correspondence vectors (`pcl::Correspondences`) and adapted all registration modules --> be sure to use `pcl::Correspondences` instead of `std::vector<pcl::Correspondence>`
* fixing the API: a few left inconsistencies between `vector<Correspondence>` and `Correspondences`. The latter is to be preferred as it contains the Eigen aligned allocator.
* added new ELCH loop correction API (New pcl::registration::ELCH class (WIP), add Registration::Ptr typedef)
* added unit tests for the (new) registration API and all registration components
* Further cleaning up registration code and writing documentation.
  * fixed bug in getRejectedQueryIndices, wrong output when order of correspondences have been changed
  * moved getRejectedQueryIndices to pcl/common/correspondence.h
  * added more doxygen documentation to the registration components
  * marked all "getRemainingCorrespondences"-functions as DEPRECATED, we sould replace them with purely stateless version outside the class body
* Update: remove ciminpack dependency and rely on eigen for LM
* Fixed a bug in ICP-NL by modifying `WarpPointRigid` to preserve the value of the 4th coordinate when warping; Re-enabled missing unit tests for ICP and ICP-NL
* Added point-to-plane ICP
* added nr_iterations_ and max_iterations_ to the initializer list (were mising)
* Fixed bugs in `WarpPointRigid3D` and `TransformationEstimationLM`
* fixed a problem where if no correspondences would be found via `nearestKSearch`, the RANSAC-based rejection scheme would fail (thanks to James for reporting this)
* changed the default LM implementation to use L2 instead of L2SQR
* Added a new `TransformationEstimationPointToPlaneLLS` class that uses a Linear Least-Squares approximation to minimize the point-to-plane distance between two point clouds
* Added the ability to specify the error function to be minimized in SAC-IA (see Feature #362)


### `libpcl_sample_consensus`

* reimplemented the Levenberg Marquardt code that was using cminpack with Eigen, thus dropping the cminpack dependency for PCL

### `libpcl_surface`

* fixed bug in surface/mls: when no search interface and no tree is given, mls creates its' own tree, but didn'tupdate the search method to use 
* added citation for Rosie's work (http://dl.acm.org/citation.cfm?id=1839808)
* added some error checking for missing/ill formed inputs for `MarchingCubes`
* don't delete smart pointers (thanks Chytu: http://www.pcl-users.org/Seg-fault-while-using-VTK-Smoother-tp3380114p3381648.html)
* added computation of area and volume for convex hulls.

### `libpcl_segmentation`

* added a labeled cluster euclidean segmentation method

### `libpcl_visualization`

* fixed an issue where `saveScreenshot` was not correctly initialized with the proper renderer, thus not saving the data
* supporting fontsize in addText (per feature #365)
* fixing the interactor style by ignoring all combinations in `OnChar` that we cover in `OnKeyDown` 
* added `removeAllShapes` and `removeAllPointClouds` per #353
* renamed `deleteText3D` to `removeText3D` to consolidate the API
* fixing the API: a few left inconsistencies between `vector<Correspondence>` and `Correspondences`. The latter is to be preferred as it contains the Eigen aligned allocator.
* added patch from Lucas Walter to fix pcl::visualization::PCLVisualizer::removeAllPointClouds and removeAllShapes
* fixed a few doxygen errors
* cleaned up the `PCLHistogramVisualizer` API by moving to a vtk interactor instead of ours, and removed unnecessary calls such as `saveScreenShot` (never used with the histogram visualizer, and most likely buggy), `wasStopped` and `resetStoppedFlag` (never used).
* removed `PCLVisualizerInteractor` and switched back to a better default `vktWindowRenderInteractor` for VTK >= 5.6. Adjusted and fixed the internal API. No public API changes were made (except the removal of the `PCLVisualizerInteractor` class which was never meant to be used externally anyway).
* cleaned up and implemented the `ImageViewer` class properly and demonstrated it in a single thread via openni_viewer. The other tool (openni_viewer_simple) will not work due to `CloudViewer` starting its own thread and thus clashing on the same OpenGL context.
* fixed the correct order in initializer list for ImageViewer
* **removed wxWidgets completely** from the codebase
* added implementation for `markPoint` to `ImageViewer` to mimic older wxWidgets-based code
* fixed an issue in `PCLVisualizerInteractorStyle` where mouse events were not properly mapped in Qt (#389 - thanks Adam!)
* added an extra field of view to the camera paramers (#392 - thanks Adam!)
* small bugfix when the radius_sphere was different than 1 for `renderViewTesselatedSphere`



## *= 1.2 (2011-09-30) = :: "Enough small talk" (LM) =*

### Additions, improvements, and optimizations

* Eliminated the `transform.h` header file < *API breaking change* > (r2517)

  * the following functions have moved to `transforms.h`:

    * `transformXYZ` (renamed to `transformPoint`)
    * `getTransformedPointCloud` (replaced with `transformPointCloud`)

  * the following methods have been replaced with built-in Eigen methods:

    * `operator<<` (use `.matrix ()` )
    * `getRotationOnly` ( `.rotation ()` )
    * `getTranslation` ( `.translation ()` )
    * `getInverse` ( `.inverse ()` )

  * the following functions have move to `eigen.h`:

    * `getTransFromUnitVectorsZY`
    * `getTransFromUnitVectorsZY`
    * `getTransFromUnitVectorsXY`
    * `getTransFromUnitVectorsXY`
    * `getTransformationFromTwoUnitVectors`
    * `getTransformationFromTwoUnitVectors`
    * `getTransformationFromTwoUnitVectorsAndOrigin`
    * `getEulerAngles`
    * `getTranslationAndEulerAngles`
    * `getTransformation`
    * `getTransformation`
    * `saveBinary`
    * `loadBinary`

* Made major changes in pcl::registration (r2503)

  * all registration code now uses `TransformEstimation` objects (`TransformEstimationSVD` and `TransformEstimationLM` in particular) rather than the older `estimateRigidTransformationSVD` code. Each class inheriting from `pcl::Registration` can pass in a different estimator via `setTransformationEstimation`
  * simplified `TransformEstimationSVD` code
  * implemented `TransformEstimationLM` by moving away code from `IterativeClosestPointNonLinear` (which now uses the transformation object)
 
* replaced the `io/io.h` header file with `common/io.h` (for backwards compatibility, `io/io.h` will remain, but its use is deprecated)
* added unit test for `lineWithLineIntersection` (r2514)
* improved the VTK installation from source documentation for MacOS (r2589)
* updated the tutorials regarding usage of FindPCL.cmake vs. PCLConfig.cmake (r2567)
* added a new `PointCloud` constructor for copying a subset of points (r2562)
* made wxwidgets an optional dependency for visualization (r2559)
* added feature #334 (Enabling a library should enable all its library dependencies in CMake) implementation, (r2551)
* added an internal `estimateRigidTransformationSVD` method to `SampleConsensusModelRegistration` (r2502)
* added a `PCL_VISUALIZER_REPRESENTATION` property for `setShapeRenderingProperties` with three possible values:

  * `PCL_VISUALIZER_REPRESENTATION_POINTS` for representing data as points on screen 
  * `PCL_VISUALIZER_REPRESENTATION_WIREFRAME` for representing data as a surface wireframe on screen
  * `PCL_VISUALIZER_REPRESENTATION_SURFACE` for representing data as a filled surface on screen
    (r2500)

* optimized performance of `BoundaryEstimation` (approximately 25% faster) (r2497)
* added reference citation to `estimateRigidTransformationSVD` (r2492)
* added a new keypoint for uniformly sampling data over a 3D grid called `UniformSampling` (r2413)
* added a destructor to `VoxelGrid<sensor_msgs::PointCloud2>` (r2412)
* optimized the performance of `SampleConsensusModelLine` (r2404)
* changed the behavior of toc_print and toc from `pcl::console:TicToc` to return milliseconds (r2402)
* added a new model, `SAC_MODEL_STICK`, for 3D segmentation (r2400)
* added 2x point picking to `PCLVisualizer`; use Alt + Mouse Left click to select a pair of points and draw distances between them (r2388)
* added two new functions (`pcl::getMaxSegment`) for determining a maximum segment in a given set of points (r2386)
* moved filters/test/test_filters to test/test_filters (r2365)
* renamed the binary executable in the compression.rst tutorial (r2345)
* Updated library dependencies

  * removed the `libfeatures dependency` for `libpcl_surface (r2337)
  * removed the `libpcl_io dependency` from `libpcl_surface` (r2354)
  * removed `libpcl_io dependency` from `libpcl_keypoints` (r2364)
  * removed `libpcl_io dependency` for `libpcl_filters` (r2365)
  * removed `libpcl_io dependency` for `libpcl_registration` (r2372)

* added a new function, `pcl::concatenateFields` for concatenating the fields of two `sensor_msgs::PointCloud2` datasets (1933)
* added a new `countWithinDistance` method to `SampleConsensusModel` (r2326), and optimized RANSAC and RRANSAC by replacing calls to `selectWithinDistance` with `countWithinDistance` (r2327)
* added feature #330: new PCLVisualizer::addCube and pcl::visualization::createCube methods (r2322)
* added `correspondence.h` to the includes in `common/CMakeLists.txt` (r2260)
* implemented defaults for the search method, per http://www.pcl-developers.org/defaults-for-search-method-td4725514.html (r2219,r2220,r2222,r2225,r2226,r2228)
* exposed `pcl::PCLVisualizer::saveScreenshot (const std::string &filename)` (r2095)
* enabled `pcd_io.hpp` (r2085)
* added a new faster binary writer + cloud/indices (r2080)
* added `pcl::RGB` structure and switched bitshifting RGB code to use the `pcl::RGB` structure's members (r2077,r2078)
* added a new `IOException` class for read/write exceptions (r2068)
* added missing `set/getAngleThreshold` for `BoundaryEstimation`. Made class parameter protected. (r2067)
* added functions in `pcl::console` for parsing CSV arguments of arbitrary length (r2052)
* added new functionality to `OrganizedFastMesh` (r1996); Now support for modes: 

  * fixed triangle meshing (left and right) that create quads and always cut them in a fixed direction, 
  * adaptive meshing that cuts where possible and prefers larger differences in 'z' direction, as well as 
  * quad meshing 

* improved `OrganizedFastMesh`'s removal of unused points (r1996)

* CMake changes (r2592)

  * changed the install dir of PCLConfig.cmake in Windows
  * renamed CMAKE_INSTALL_DIR into PCLCONFIG_INSTALL_DIR
  * NSIS installer will add a key in Windows Registry at HKEY_LOCAL_MACHINE\Software\Kitware\CMake\Packages\PCL to help CMake find PCL (CMake >= 2.8.5) (NSIS.template.in)
  * reordered CMAKE_MODULE_PATH entries, for CMake to pick up our NSIS.template.in
  * fixed CPACK_PACKAGE_INSTALL_REGISTRY_KEY value

### Bug fixes

* fixed bugs in `PointCloud`

  * in `swap()`, the point data was swapped, but the width and height fields were not (r2562)
  * in `push_back()`, adding points did not update the width/height of the point cloud  (r2596)
  * in `insert()/erase()`, inserting or erasing points did not update the width/height of the point cloud (r2390)

* fixed bugs in `SampleConsensusModelRegistration`

  * if target_ wasn't given, would crash with a boost shared uninitialized error (r2501)
  * homogeneous coordinates were wrong (r2502)

* fixed a bug in `BoundaryEstimation` in case "angles" is empty (r2411)
* fixed a bug in `OrganizedFastMesh`'s adaptive cut triangulation; added a new unit test (r2138)
* fixed a bug in the openni_image viewer tool (r2511)
* fixed problems with Windows/MacOS ALT bindings in PCLVisualizer (r2558)
* fixed issues

  * #139 (`FPFHEstimation` for non-trivial indices) (r2528)
  * #303 (make `pcd_viewer` reset the camera viewpoint when no camera given) (r1915)
  * #338 (cylinder segmentation tutorial was referring to a different file) (r2396)
  * #339 (the direction of the normal was wrongly estimated when `setSearchSurface` was given) (r2395)
  * #340 (keep_organized_ bug in `ConditionalRemoval`) (r2433)
  * #342 (Allow QT to be used with PCL Visualizer) (r2489)
  * #343 (`empty()` member function of a point cloud is not const) (r2440)
  * #350 (Misspelling in `GreedyProjectionTriangulation::setMaximumSurfaceAgle()`) (r2556)

* added missing include in `correspondence_rejection.h` (r2393)
* corrected the headers included by `sample_consensus/sac_model.h` (r2550)
* removed duplicate content of NSIS.template.in (r2601)
* fixed various casting related compiler warnings (r2532)
* corrected typos in 

  * Windows tutorial (CMAKE_INSTALL_DIR => CMAKE_INSTALL_PREFIX) (r2595)
  * registration/icp.h documentation (reg => icp) (r2515)
  * several apps/tools console messages (wotks => works) (r2491)
  * how_features_work.rst tutorial (Muechen => Muenchen) (r2484)



## *= 1.1.1 (2011-08-31) = :: "Low Entropy" =*

* issues fixed: #224, #277, #288, #290, #291, #292, #293, #294, #295, #296, #297, #299, #302, #318, #319, #324, #325, #329
* Additional fixes:

  * fixed a segfault in PCLVisualizer::addPointCloudNormals
  * fixed PCLVisualizer hanging on 'q' press
  * fixed a bug in MLS
  * fixed a bug in test_io
  * fixed a bug in PointCloudColorHandlerGenericField
  * fixed a bug when writing chars to ASCII .PCD files
  * fixed several errors "writing new classes" tutorial
  * added missing parameter setter in the "concave hull" tutorial


## *= 1.1.0 (2011-07-18) = :: "Deathly Hallows" =*

* new 3D features: 

  * SHOT (Signature of Histograms of Orientations)
  * PPF (Point-Pair Features)
  * StatisticalMultiscaleInterestRegionExtraction
  * MultiscaleFeaturePersistence

* improved documentation: 

  * sample consensus model coefficients better explained
  * new tutorials (RadiusOutlierRemoval, ConditionalRemovalFilter, ConcatenateClouds, IterativeClosestPoint, KdTreeSearch, NARF Descriptor visualization, NARF keypoint extraction, ConcaveHull, PCLVisualizer demo)

* new surface triangulation methods: 

   * MarchingCubes, MarchingCubesGreedy
   * OrganizedFastMesh
   * SurfelSmoothing
   * SimplificationRemoveUnusedVertices

* new registration methods: 

   * PyramindFeatureMatching
   * CorrespondenceRejectorSampleConsensus

* new general purpose classes/methods: 

   * Synchronizer
   * distance norms
   * TextureMesh
   * PointCloud.{isOrganized, getMatrixXfMap)
   * SACSegmentation now works with PROSAC too
   * PCDViewer now reads VTK files
   * new Mouse and Keyboard events for PCLVisualizer
   * PCLVisualizer.{addText3D, addCoordinateSystem (Eigen::Matrix4f), deleteText3D, updatePointCloud, renderViewTesselatedSphere, resetCameraViewpoint, getCameras, getViewerPose}
   * ONIGrabber, DeviceONI
   * ImageRGB24, IRImage
   * generic FileReader + FileWriter
    
* optimizations: 

  * faster pipelinening by not recreating a fake set of indices everytime
  * rendering optimizations for PCLVisualizer
  * copyPointCloud is now faster and can copy the intersections of the fields in both input datasets
  * VoxelGrid is now ~20Hz for Kinect data

* new applications: 

  * VFH NN classification
  * 3D concave hulls
  * 3D convex hulls
  * ICP registration
  * Planar segmentation 
  * Stream compression
  * Range image viewer
  * Voxel Grid
  * IntegralImage normal estimation

* issues fixed: #75, #106, #118, #126, #132, #139, #156, #182, #184, #189, #197, #198, #199, #201, #202, #203, #210, #213, #211, #217, #223, #225, #228, #230, #231, #233, #234, #240, #247, #250, #251, #254, #255, #257, #258, #259, #261, #262, #264, #265, #266, #267, #268, #269, #273, #276, #278, #279, #281, #282, #283

## *= 1.0.1 (2011-06-29) = :: "First Steps" =*

*please note that version 1.0.0 had a flaw when creating ASCII pcd files. This version includes the tool pcd_convert_NaN_nan to fix this*

* added VTK file visualization to pcd_viewer
* hiding the cminpack/FLANN headers, thus reducing compile time for user code
* fixed `IntegralImageNormalEstimation`
* tutorial updates and fixes + new tutorials. Changed tutorial structure to split CPP files from RST text.
* better doxygen documentation for many functions
* fixed a bug in `ConditionalRemovalFilter` where the `keep_organized` condition was reversed
* removed `BorderDescription` and `Histogram<2>` from the list of explicit template instantiations
* added `PointXY` point registration macros
* added `ExtractIndicesSelf` unit test 
* fixed a lot of alignment issues on 32bit architectures
* PCD ascii files now have each individual line trimmed for trailing spaces
* internal changes for PCDReader/PCDWriter, where NAN data is represented as "nan"
* sped up compilation with MSVC by adding /MP for multiprocessor builds
* added a voxel grid command line tool filter
* issues fixed: #242, #207, #237, #215, #236, #226, #148, #214, #218, #216, #196, #219, #207, #194, #192, #183, #178, #154, #174, #145, #155, #122, #220
* added support for PathScale pathcc compiler
* added support for Intel icc C++ compiler
* added support for GCC 4.6 C++ compiler
* added preliminary support for Clang C++ compiler
* FindPCL.cmake and PCLConfig.cmake completed

## *= 1.0.0 (2011-05-11) = :: "A new beginning" =*

* completely standalone build system using CMake (not dependent on ROS anymore)
* tested on Win32/Win64, Linux (32/64) and MacOS (installers provided for all 3rd party dependencies and PCL)
* separated the entire codebase into multiple libraries that depend on each other. separate CMake declarations for each individual library.
* provide a FindPCL.cmake to ease integration
* many new unit tests + tutorials + improved Doxygen documentation (check http://www.pointclouds.org/documentation and http://docs.pointclouds.org)
* new liboctree interface for nearest neighbor/radius search with Octrees, change detection, and Point Cloud compression!
* added concave/convex hulls using QHull (dropped our old ConvexHull2D class)
* pcl::Registration improvements and bugfixes, sped up inner loops, added alignment methods with initial guess
* pcl::visualization bugfixes: multiple interactors are now possible, better threading support, sped up visualization (2-3x), addPolygonMesh helper methods
* new grabber interface for OpenNI-compatible camera interface: all 3 OpenNI cameras are supported (Kinect, Asus XTionPRO, PSDK)
* new grabber interface for streaming PCD files
* pcl::io bugfixes for PCD files using binary mode
* ROS_ macros are no longer valid, use PCL_ instead for printing to screen
* new PCA implementation, up to 4-5 times faster eigen decomposition code
* new CVFH feature, with higher performance than VFH for object recognition


# Pre 1.0 (copied from http://pcl.ros.org/ChangeList) - for historical purposes. 

The version numbers below belong to the *perception_pcl* stack in ROS, which coordinated both the release of PCL, as well as the release of the ROS PCL wrappers. From 1.0 onwards PCL got separated from ROS, and the release process of *perception_pcl* and PCL diverged.

* [[flann]]

 * Updated to version 1.6.8

* [[pcl]]

 * bug fix in _pcl::PassThrough_ filter, where the output data size was not set correctly when _keep_organized_ was set to false
 * bug fix in _pcl::GreedyProjectionTriangulation_, where point distances were incorrect, thus poroducing too many holes points during reconstruction ()
 * added more elaborate unit tests for surface reconstruction
 * Added possibility to restrict no of returned interest points to _pcl::NarfKeypoint_
 * Added parameter to control no of openmp threads to _pcl::NarfKeypoint_

* [[pcl_ros]]

 * added the _keep_organized_ option to _pcl_ros::Passthrough_ (#4627)

## *= 0.10.0 (2010-02-25) =*

* Re-versioned 0.9.9 to follow patch-versioning semantics.

## *= 0.9.9 (2010-02-22) :: "Almost there" edition =*

* [[pcl]]

 * removed _ConvexHull2D_ (*API breaking change!*)
    You need to change your code. From:
    ```
    pcl::ConvexHull2D<...> ...;
    ```
    to
    ```
    pcl::ConvexHull<...> ...;
    ```
 * added a new general purpose 2D/3D _ConvexHull_ class based on QHull
 * added a new general purpose 2D/3D _ConcaveHull_ class based on QHull
 * added helper _transformPointCloud_ method for cloud+indices as input
 * fixed: segfaults when ICP finds no correspondences (#4618)
 * improved the PCD I/O capabilities (for binary PCD) to deal with SSE padding
 * Added possibility to create a _RangeImagePlanar_ from a point cloud
 * Corrected is_dense to false in all range images.
 * Reimplemented big parts of the NARF keypoint extraction - should be more reliable now (and unfortunately slower) - uses polynomials to search maxima now.
 * Added helper classes for polynomial approximations to common
 * Added a new RANSAC-like algorithm: _PROSAC_ (much faster when there is a confidence for the matches)
 * Fixed normalization factor in the VFH's scale component.
 * Made MLS more flexible: output has all fields the input has, only with XYZ smoothed, and normals are provided separately (and optionally)
 * Added multi-scale calculation to NARF keypoint to make it faster again, fixed a bug in _BorderExtractor_ and fixed some issues in _RangeImagePlanar_.
 * Added functions in common to compute max distance from a point to a pointcloud. Fixed distance component of VFH, normalization is now also invariant to rotation about the roll axis.
 * Added _pcl::PointSurfel_ to known point types.
 * eigen-decomposition for symmetric positive-semi-definite 3x3 matrices: 1) bug fix so eigenvects are orthogonal, 2) is more robust for degenerated cases

* [[pcl_ros]]

  * _MovingLeastSquares_ nodelet improvements
  * Changed serialization of point clouds to ship the data as-is over the wire, padding included (#4754). Implemented subscriber-side optimizations to minimize memcpys. Will do one memcpy for the whole point cloud if the data layout is an exact match. 

## *= 0.9.0 (2011-02-08) :: "Darn the torpedoes" Edition =*

* [[pcl]]
 
 * optimizations for _dense_ dataset (no checking for _isfinite_ inside the loop unless _is_dense_ is false)
 * improved eigen decomposition _pcl::eigen33_ by better handling degenerate cases. Normal estimation should work better now.
 * more 32bit alignment fixes
 * improved Doyxgen documentation
 * added _POINT_CLOUD_REGISTER_POINT_STRUCT_ for _Narf36_
 * fixed a minor bug in poses_from_matches where a distance was not computed correctly
 * disabled TBB classes until 1.0 comes out
 * fixed a few bugs in _ICP_NL_ registration (thanks Mike!)
 * enforced const-ness for _pcl::ArrayXfMap_ and _pcl::VectorXfMap_, by creating _pcl::ArrayXfMapConst_ and _pcl::VectorXfMapConst_ for maps over const types
 * improved Windows support
 * fixed a bug in _StatisticalOutlierRemoval_ where the output data array was set incorrectly.
 * fixed a _boost::split_ bug in _pcd_io_, for cases when point data starts with spaces or tabs
 * finalized the surface reconstruction (_pcl::SurfaceReconstruction_) API
 * added a new method for surface reconstruction using grid projections (_pcl::GridProjection_)
 * added a new generalized field value filter (_pcl::ConditionalRemoval_)
 * cleaned up normal estimation through integral images
 * PCL rift.hpp and point_types.hpp fixed for Windows/VS2010
 * fixed all _is_dense_ occurances
 * *unmanged all _Eigen3::_ calls to _Eigen::_*
 * changed all _!isnan_ checks to _isfinite_ in order to catch INF/-INF values
 * added vtk_io tools for saving VTK data from _PolygonMesh_ structures
 * fixed _SACSegmentation::SACMODEL_CIRCLE2D:_ to accept _setRadius_ limits
 * use _SACSegmentation_ when the model doesn't fit _SACSegmentationFromNormals_ (assert fix)

* [[pcl_ros]]

 * added a mutex for configuration parameters and filter computation, to prevent changing parameters while the algorithm is running. Alternative: copy parameters before the compute loop.
 * pcd_to_pointcloud has been updated to take a private parameter "frame_id" which is used in publishing the cloud
 * unmangled all _Eigen3::_ calls to _Eigen::_
 * Document cloud to image conversion.

  * http://www.ros.org/wiki/pcl_ros/Tutorials/CloudToImage
  * Removed the tool from pcl , and moved to pcl_ros

## *= 0.8.0 (2011-01-27) :: Operation CloudMan =*

* [[flann]]

 * Updated to version 1.6.7

* [[pcl]]

 * improved doxygen documentation overall
 * added support for indices for _SampleConsensus_ models (previously broken)
 * fixed a grave bug in _ProjectInliers_, where row_step was not correctly set thus leading to (de)serialization issues in the new _ros::Subscriber<PointCloud<T> >_ scheme
 * _RangeImagePlanar_ can now also be created from a depth image.
 * Fixed a bug in _RangeImagePlanar_.
 * Fixed possible segmentation fault in _RangeImageBorderExtractor_.
 * _RangeImage_ now has _is_dense=false_, since there typically are NANs in there.
 * Added Correspondence as a structure representing correspondences/matches (similar to OpenCV's DMatch) containing query and match indices as well as the distance between them respective points.
 * Added _CorrespondenceEstimation_ for determining closest point correspondence, feature correspondences and reciprocal point correspondences.
 * Added _CorrespondenceRejection_ and derivations for rejecting correspondences, e.g., based on distance, removing 1-to-n correspondences, RANSAC-based outlier removal (+transformation estimation).
 * Further splitted up registration.h and added transformation estimation classes, e.g., for estimating rigid transformation using SVD.
 * Added ```sensor_msgs::Image image;  pcl::toROSMsg (cloud, image);```See tools/convert_pcd_image.cpp for a sample.
 * Added a new point type, _PointWithScale_, to store the output of _SIFTKeypoint_ detection.
 * Fixed a minor bug in the error-checking in _SIFTKeypoint::setScales(...)_
 * Fixed small bug in _MovingLeastSquares_
 * Fixed small bug in _GreedyProjectionTriangulation_
 * Added unit tests for _RSDEstimation_, _MovingLeastSquares_ and _GreedyProjectionTriangulation_
 * Fixed and improved multiple point to line distance calculations, and added it to distance.h and unit testing

* [[pcl_ros]]

 * Fixed a grave bug introduced in 0.7 where the _NullFilter_ approach doesn't work
 * Implemented the exact time synchronizer path for _PointCloudConcatenateDataSynchronizer_
 * replaced subscribers/publishers in surface, segmentation, and feature nodelets from _sensor_msgs::PointCloud2_ to _pcl::PointCloud<T>_, thus reducing complexity and getting rid of all the intermediate _from/toROSMsg_ (de)serializations
 * moved away from storing state in the nodelets. The right way to do it is: callback -> _PointCloud_ -> process -> publish, without storing the _PointCloud_ as a member into the base class
 * introduced a mandatory _emptyPublish_ which nodelets should use to send an empty result with the header frame_id and stamp equal with the input's.

## *= 0.7.1 (2011-01-09) =*

* [[flann]]

 * Makefile patch for LateX PDF build

* [[pcl]]

 * fixed _roslib/Header_ warnings by enabling _std_msgs/Header_ if ROS_VERSION > 1.3.0
 * working on PCL port to Android - mainly CMake scripts, and a few _#if ANDROID_ statements.
 * _pcl::VoxelGrid _now saves and gives access to the created voxel grid, with lots of helper functions

* [[pcl_ros]]

 * dropped _pcl_ros::Subscriber_ due to the changes introduced in 0.7

## *= 0.7 (2010-12-20) :: Special "FLANN is awesome" Edition =*

* [[eigen3]]

 * removing eigen3 as it's already present in geometry (unstable) and geometry_experimental (cturtle)

* [[pcl]]

 * added general purpose _getMinMax()_ method for n-dimensional histograms (r34915).
 * more 32bit architecture alignment fixes
 * fixing \r string splitting issues on Windows systems for _PCDReader_ (r34830).
 * Added new range image class RangeImagePlanar. E.g., for direct usage with kinect disparity images.
 * added the option to keep the data organized after filtering with _pcl::PassThrough_ and replace values in place rather than removing them (r34776)
 * removed boost::fusion macros
 * moving the Feature estimation architecture to the new PCL structure
 * moved _sorted__ as a parameter in the _KdTree_ base class
 * added _PCL_INSTANTIATE_ macros that easily instantiate a specific method/class given a set of point types
 * Better checks for _is_dense_ plus bugfixes.
 * Added RSD (Radius Signature Descriptor) feature.
 * Updated the entire PCL tree to use FLANN as a default _KdTree_
 * Optimized the _KdTreeFLANN::radiusSearch_ so it doesn't do any memory allocation if the indices and distances vectors passed in are pre-allocated to the point cloud size.
 * Changed _KdTree::radiusSearch_ method signature to return int (number of neighbors found) intead of bool
 * added new _pcl::View<T>_ class that holds a _PointCloud_, an _Image_, and a _CameraInfo_ message (r34575)
 * Moving the _Surface_ reconstruction framework to the new structure (r34547)
 * Moving the _Segmentation_ framework to the new structure (r34546)
 * Creating a libpcl_sample_consensus and moving the SAC methods and estimators to the new structure (r34545)
 * moving _Filter_ to the new structure (r34544)
 * Creating a libpcl_kdtree and moving the _KdTree_ to the new structure (r34543)
 * added support for PCD v0.7 files (+VIEWPOINT), minor API changes and consolidations
 * adding a sensor origin + orientation for _pcl::PointCloud<T>_ (r34519)
 * Improved _Boost_ CMake macros

* [[pcl_ros]]

 * greatly simplified the _PointCloudConcatenateDataSynchronizer_ nodelet (r34900).
 * _pcl::PointCloud<T>_ works natively with _ros::Publisher_ and _ros::Subscriber_. Separate _pcl_ros::Publisher_ and _pcl_ros::Subscriber_ are no longer necessary.

* [[flann]]

 * Updated to FLANN 1.6.6
 * Updated to FLANN 1.6.5.
 * Updated to FLANN 1.6.4
 * Updated to FLANN 1.6.3

* [[pcl_tf]]

 * Moved functionality to _pcl_ros_. RIP.

* [[ann]]

 * Removed ANN as a dependency. RIP.

* [[cminpack]]

 * updated cminpack to 1.1.1
 * fedora rosdep fixes
 * Patch for building and installing cminpack correctly (thanks to Nicholas)

## *= 0.6 (2010-12-01) :: Special "Happy Birthday Bastian!" Edition =*

* [[pcl]]

 * sort clusters in _ExtractEuclideanCluster_ in descending order before returning (r34402)
 * VoxelGrid patch for handling RGB colors (thanks to Yohei)
 * lots of work to make PCL compile in Windows (thanks to Stefan)
 * bumping up the ASCII precision from 5 to 7 on PCD I/O operations
 * moved PCL to the new stack: perception_pcl
 * header include cleanup for faster compiles
 * switched _StatisticalOutlierRemoval_ to FLANN
 * Updated kdtree_flann to use the C++ bindings from FLANN

* [[pcl_tutorials]]

 * Added new tutorial to visualize the NARF descriptor and look at descriptor distances.
 * fixed a problem that was preventing tutorials to be built on MacOS (r34287)

* [[flann]]

 * Updated to latest version of FLANN (1.6.2)
 * Updated to latest version of FLANN (1.6.1)
 * Updated to latest version of FLANN (1.6)

* [[cminpack]]

 * Updated _cminpack_ to version 1.1.0
 * Updated _cminpack_ to version 1.0.90 (r34303)
 * Fixed a portability issue where the library was copied as .so, while MacOS systems need .dylib, thanks to Cyril per #4596 (r34286)

* [[pcl_ros]]

 * Fixed a portability issue where uint was used without being defined on MacOS systems, thanks to Cyril per #4596 (r34285)
 * Patch for _ConvexHull2D_ to publish _PolygonStamped_ (thanks to Ryohei)

## *= 0.5 (2010-11-25) :: Special Thanksgiving Edition =*

* [[pcl]]

 * got rid of ROS_ASSERTS in favor of ROS_ERROR + return (true/false)
 * improvements on CMake build scripts and custom-made ROS header includes to make PCL work outside of ROS
 * pow() fixes for macos (#4568)
 * fixing a bug where if inliers are always 0, we get stuck in an (almost) infinite loop in Sample Consensus methods (r34191)
 * _SegmentDifferences_ is now optimized for null targets (simply copy the input)
 * Changed operator(x,y) in PointCloud to return references, both const and non-const.
 * set the default behavior of _ExtractIndices_ to return the complete point cloud if the input indices are empty and _negative_ is set to true
 * resize the clusters to 0 on failed _initCompute_ for _EuclideanClusterExtraction_
 * fixed all filtering methods to return empty datasets (points/data.size () = 0, width = height = 0) on failures and make sure that header (+fields for PointCloud2) are always copied correctly in all cases (r34099).
 * make sure header and fields are copied even if the indices are invalid for _ProjectInliers_
 * fixed error message typo in conversions.h
 * fixing the meaning of is_dense per #4446

* [[pcl_ros]]

 * added _min_inliers_ as the minimum number of inliers a model must have in order to be considered valid for _SACSegmentation_
 * added a _SegmentDifferences_ nodelet
 * fixed a grave bug in _Filter_ where the output TF frame was not always set correctly (r34159)
 * added _eps_angle_ as a dynamic reconfigure parameter to _SACSegmentation_
 * added error output on invalid input for all nodelets
 * Allow pcl_ros publishers to take shared pointers - fix #4574 (r34025)
 * change the default behavior of isValid from data.empty || width * height = 0, to width * height * step = data.size for _PCLNodelet_
 * make sure data is published even if empty
 * making sure publishing with a rate of 0 works for _pcd_to_pointcloud_
 * added TF transform capabilities to _bag_to_pcd_
 * Enforce that the TF frame and the timestamp are copied before publish

* [[pcl_tf]]

 * got rid of unneeded transforms (if _target_frame = input.header.frame_id_, simply return the input)

* [[pcl_visualization]]

 * removed -DNDEBUG which was causing some weird random bugs (-cam "..." was one of them) (r33984)
 * fixing an issue with VTK exports which lead to 3rd party user packages needing to know and explicitly import VTK (r33980)

## *= 0.4.2 (2010-11-01) =*

* [[pcl_ros]]

 * removed the vtk metapackage (replaced with findVTK macros)

## *= 0.4.1 (2010-11-01) =*

* [[pcl_ros]]

 * adding EIGEN_MAKE_ALIGNED_OPERATOR_NEW to all nodelets to make sure that we are aligned on 32bit architectures

## *= 0.4.0 (2010-10-30) =*

* [[pcl]]

 * changes needed for the eigen3 upgrade (_Eigen3::Transform3f_ -> _Eigen3::Affine3f_)
 * Added _SIFTKeypoint_ keypoint detection algorithm for use on xyz+intensity point clouds.
 * Fixed a bug in _Registration::FeatureContainer::isValid ()_ where the source and target feature clouds were required (incorrectly) to be the same size
 * added NARF features (descriptor) to features, updated NARF keypoints, made _RangeImageBorderExtractor_ derived from Feature.
 * added _set/get AllFields/AllData_ helper methods to _ProjectInliers_ (r33284)
 * added helper _bool getFilterLimitsNegative ()_ to _Filter_ (r33283)
 * added helper _set/get LocatorType_ methods to _EuclideanClusterExtraction_ (r33282)
 * added missing dependency on roscpp
 * added a check for insufficient number of points when fitting SAC models to avoid infinite loops (r33195)
 * added _makeShared ()_ to _pcl::PointCloud<T>_, and fixed the Ptr/ConstPtr typedefs (r33147)
 * fixed a bug in _SamplesConsensusModelRegistration_ where the sample selection could get stuck in an infinite loop
 * added line to line intersection routines (r33052)
 * corrected _transformPointCloudWithNormals_ in transforms.hpp to properly handle normals

* [[pcl_ros]]

 * switching rosrecord API to rosbag (r33602)
 * Complete refactorization of PCL_ROS nodelets. Got rid of multiple inheritance in favor of delegation. Using template specializations now the code compiles faster and GCC processes occupy less RAM during compilation.
 * new _PointCloudConcatenateDataSynchronizer_ nodelet: concatenates PointCloud2 messages from different topics (r33241)
 * added a _max_clusters_ int_t nodelet option to _EuclideanClusterExtraction_: The maximum number of clusters to extract (r33258)
 * added a _filter_limit_negative_ bool_t nodelet option to _Filter_. Set to true if we want to return the data outside [filter_limit_min; filter_limit_max] (r33257).
 * fixed bug in _PassThrough_ nodelet that prevented dynamic reconfigure from setting the _FilterFieldName_ properly.
 * added implementation of _MovingLeastSquares_ nodelet

* [[eigen3]]

 * upgraded to eigen3-beta2

* [[cminpack]]

 * Updated cminpack to 1.0.4 (r33419)

* [[pcl_tutorials]]

 * added/updated tutorials for range image creation, range image visualization, range image border extraction, NARF keypoint extraction and NARF descriptor calculation.

* [[point_cloud_converter]]

 * added missing dependency on roscpp (r33045)

## *= 0.3.3 (2010-10-04) =*

* [[pcl]]

 * fixed a bug in _ProjectInliers_ for 32bit architectures (r33035)
 * Added unit tests for _computeCovarianceMatrixNormalized_ (r32985)
 * making _PointCloud2_/_PointCloud_ typedefs consistent (r32989)
 * field W is no longer needed in _pcl::PointNormal_, as it was only used for SSE optimization. Same for _pcl::PointXYZW_, which means we can remove them (r32960)

* [[point_cloud_converter]]

 * Changed INFO to DEBUG on print statements

* [[pcl_tf]]

 * added code to check for TF exceptions

* [[pcl_ros]]

 * changes to consistently output/publish empty datasets + made sure all inputs are checked for validity in each nodelet before processing (r32993)
 * added dynamic_reconfigure TF input_frame/output_frame parameters to _pcl_ros::Filter_ (r32990)

## *= 0.3.2 (2010-09-28) =*

* [[pcl]]

 * fixed a bug in _ExtractPolygonalPrismData_ that got introduces in r31172
 * added new 3D Keypoint extraction base class and the NARF interest points.
 * added iterator, const_iterator, begin, end for _pcl::PointCloud_
 * fixed bug in radius search of kdtree, that occurred when indices_ is used
 * renamed sac_model_oriented_[line|plane] to *parallel_line and *perpendicular_plane to be more clear
 * added a new SAC model for finding planes that are parallel to a given axis. Useful for e.g. finding vertical planes (parallel to "up" axis)
 * added a smarter sample selection to _SampleConsensusModelRegistration_ that uses a minimum distance between samples based on the principal directions/magnitudes of the dataset (r32873)
 * enabling SSE optimizations by default for all libraries
 * normalized covariance matrix estimation _pcl::computeCovarianceMatrixNormalized_ (r32872)
 * Added methods to _pcl::Registration_ for finding point correspondences between pairs of feature clouds
 * fixed a bug in _pcl::SampleConsensusModelLine_ where the number of iterations was not correctly checked/set in _getSamples_
 * Added missing includes in icp_nl.h

* [[pcl_ros]]

 * changing the order of the members to allow nodelet unloading (r32941)
 * enabling SSE optimizations for all libraries (r32909)
 * enabling parameter setting via dynamic_reconfigure for _StatisticalOutlierRemoval_ filter (r32876)
 * set the default number of '''gcc''' build processes to 2, to avoid large memory allocation thus rendering the machine unusable during compilation

* [[flann]]

 * upgraded to FLANN 1.5

## *= 0.3.1 (2010-09-20) =*

* [[pcl]]

 * fixed a grave bug that caused 32bit architectures to exhibit errors regarding eigen alignment issues (32769)
 * fixed a bug with _isBoundaryPoint()_ and _pcl::BoundaryEstimation::computeFeature()_ relating to the use of different surface_ and input_clouds and/or non-default indices_, and added a new way of calling _isBoundaryPoint()_
 * fixed the problem of mls.hpp depending on eigen 2
 * added _computeCovarianceMatrixNormalized_ methods <<Ticket(ros-pkg 4171)>>
 * Renamed namespace of eigen3 to Eigen3 to prevent conflicts with eigen2

* [[laser_scan_geometry]]

 * fixed #4437 where PointCloud2 _projectLaser_ fails when laser intensity is off (32765)

* [[pcl_tf]]

 * fixed #4414 where _lookupTransform_ was being called without tf_listener object (32764)

## *= 0.3.0 (2010-09-05) =*

* [[pcl]]

 * _VectorAverage_ uses fast spezialized PCA method now for dimension=3
 * new version of _VoxelGrid_ filter that downsamples data faster and doesn't use as much memory (the previous version could easily cause std::bad_alloc exceptions)
 * fixed a major bug where: 1) PointCloud2 structures were being sent with extra unusued (padding) data (i.e., directly copied from _pcl::PointCloud<T>_); 2) writing PCD files in binary mode resulted in corrupted files (r32303)
 * fixed a bug where the fields were not cleared on _pcl::getFields (pcl::PointCloud<T>_) (r32293)
 * fixed a bug where _PCDWriter_ was writing to the same file instead of creating new ones
 * improvements in non linear registration (reduced the number of estimates from 7 to 6, SSE optimizations, etc)
 * reduced the number of operations for covariance matrix estimation from 18*N to 10*N+3 => big speedup increase together with _pcl::eigen33_ !
 * Big patch for PCL to work with Eigen3. Fixed alignment issues for Sample Consensus models. Added a faster (experimental) eigen decomposition method from Eigen3 trunk.
 * fixed a bug where the axis_ was not properly aligned on 32bit architectures thus causing an assert
 * Added ICP functionalities to range image, add new helper functions header file_io, minor fixes.

* [[pcl_tf]]

 * Added function to just transform a point cloud using an Eigen matrix.

* [[pcl_visualization]]

 * fixed a bug where NaN values invalidated certain color handlers
 * Range image example uses viewpoint information from file now

* [[eigen3]]

 * added Eigen3 library

## *= 0.2.9 (2010-08-27) =*

* [[pcl]]

 * added _copyPointCloud (sensor_msgs::PointCloud2 &in, indices, sensor_msgs::PointCloud2 &out)_

* [[pcl_visualization]]

 * fixed a bug in PCLVisualizer where NULL scalars were causing a segfault

* [[pcl_ros]]

 * changed the default max queue size to 3
 * added implementations for approximate time synchronizers
 * _ExtractPolygonalPrismData_ can now rotate the hull in the point cloud frame using TF

* [[pcl_tf]]

 * transforms.cpp add missing out.header.frame_id = target_frame;

* [[laser_scan_geometry]]

 * Max range handling from Hokuyo should work properly now.

## *= 0.2.8 (2010-08-24) =*

* [[pcl]]

 * added a Huber kernel to LM optimization
 * fixed a bug where the order of _setInputCloud_ and _setPointRepresentation_ triggered an assert condition (31938)
 * added a Sample Consensus model for outlier rejection during ICP-like registration
 * added implementations of the intensity-domain spin image and Rotation Invariant Feature Transform, as described in "A sparse texture representation using local affine regions," by Lazebnik et al. (PAMI 2005).
 * added a templated N-dimensional histogram point type
 * Updated code to work with point wrapper registration.

* [[pcl_visualization]]

 * scale the colors lookup table to the color handler limits (31942)
 * fixed a bug where the screenshot function was not using the last updated screen
 * _addPointCloudPrincipalCurvatures_ renders the principal curvatures of a point cloud on screen + set color properties

## *= 0.2.7 (2010-08-19) =*

* [[pcl]]

 * SSE improvements in *_PFHEstimation_
 * more SSE improvements in _computeCentroid_, _computeCovarianceMatrix_, etc
 * several improvements regarding SSE optimizations in _SACModelPlane_
 * make all data points that deal with XYZ SSE aligned (31780)
 * work on improving the VFH signatures (changed the signature from 263 to 308, added distances)

* [[pcl_visualization]]

 * fixed a silly X11 include bug, where Bool was defined globally therefore breaking BOOST_FOREACH
 * added font size property setter for text actors
 * new _PCLVisualizer::addText_ method for adding texts to screen (r31822)
 * implemented window position ordering and range min/max for _PCLHistogramVisualizer_ (r31805)
 * implemented a basic histogram visualizer for multi-dimensional count=1 features (e.g., VFH)

* [[pcl_tutorials]]

 * added a tutorial for online PointCloud2 message viewing

## *= 0.2.6 (2010-08-17) =*

* [[pcl]]

 * fixed another bug where internal matrices were not always cleared in FPFH/VFH estimation
 * fixed a few bugs regarding _sizeof(PointT)=16_ for _demeanPointCloud_ (r31762)
 * fixed a grave bun in FPFHOMP/VFH estimation where the matrices were not correctly initialized on consequent runs (r31757)
 * introduced PCL exceptions, and added checks for conversion and cloud.at (u, v) operations

* [[pcl_ros]]

 * fixed a bug where _SACSegmentation_ was accepting two parameters with the same name (distance_threshold vs model_threshold)
 * disable empty data publishing in _PCDReader_

* [[pcl_visualization]]

 * added lookup table to the renderer

* [[cminpack]]

 * enabled shared library by default

## *= 0.2.5 (2010-08-14) =*

* [[pcl]]

 * added a convenience method in _pcl::io::loadPCDFileHeader_ for loading up files fast and checking their fields, data types and sizes, etc (r31740)
 * added implementation of the Sample Consensus Initial Alignment (SAC-IA) algorithm described in "Fast Point Feature Histograms (FPFH) for 3D Registration," Rusu et al., (ICRA 2009)
 * moved the _estimateRigidTransformationSVD_ methods out of _IterativeClosestPoint_ and made them static functions defined in registration.h.

* [[pcl_visualization]]

 * fixed the minmax color handler visualization (r31710)
 * few _addPointCloud_ convenience method for geometry handlers + bugfix in pcd_viewer (the last field was added twice as a color handler when more than one geometry handler was available)
 * _addPointCloudNormals_ displays surface normals on screen (PCLVisualizer)
 * added -normals and -normals_scale options to pcd_viewer

* [[pcl_ros]]

 * grave bug fixed in _PointCloudConcatenateFieldsSynchronizer_ where data was not correctly copied (r31708)
 * _PCDReader_ nodelet uses a latched topic to publish data

## *= 0.2.4 (2010-08-10) =*

* [[pcl]]

 * pairwise registration tutorial now has on-screen visualization (r31698)
 * new laser scan geometry for LaserScan->PointCloud2 conversion
 * added general purpose transformation routines for sensor_msgs::PointCloud2 using TF (r31678)
 * added basic shapes (line, sphere, cylinder, polygon, etc), split rendering properties for shapes and point clouds (r31665)
 * fixed a few bugs regarding fields.count
 * small bug fix where triples on the parameter server didn't match with the value in dynamic reconfigure (VoxelGrid)
 * added a convenience method for conversion from PointCloud2 (xyz) to Eigen format
 * added a method to get the radius of a circumscribed circle
 * added methods for saving/loading camera parameters in PCL Visualizer (c, j, -cam)
 * added -multiview to pcd_viewer; PCLVisualizer can now create and use different viewports (r31600)

## *= 0.2.3 (2010-08-02) =*

* [[pcl]]

 * tiny bug fixes for cturtle

## *= 0.2.2 (2010-08-02) =*

* [[pcl]]

 * work to improve the PCL Visualization package, standalone pcd_visualizer completed
 * added cloud(u,v) accessors for organized datasets.
 * added constrained implementations (maximum distance between correspondences) for registration methods
 * added PCD file format v.6 which includes field count (r31367)
 * added a PointRepresentation class for spatial search methods that allows the user to specify what dimensions are important
 * added a maximum distance threshold for point correspondences in registration
 * fixed IterativeClosestPoint registration, added a L1 kernel to icp_nl, and unit tests (31283)
 * added a new segmentation method for obtaining the differences between two point clouds (r31281)

## *= 0.2.1 (2010-07-25) =*

* [[pcl]]

 * test build fixes
 * made BAGReader inherit directly from nodelet::Nodelet

## *= 0.2.0 (2010-07-24) =*

* [[pcl]]

 * added PCL_ROS package for all PCL_ROS interactions. Moved all nodelets and all ROS specific interfaces from PCL to PCL_ROS.
 * moved vectorAverage to common
 * added class to calculate transformation from corresponding points to common
 * added index remapping for the kdtrees so that NAN points are automatically ignored.
 * Split kdtrees in h and hpps
 * Added some new functionality to range images and fixed bugs in creation process.
 * separated the PCL tutorials from the library into a separate package (pcl_tutorials)
 * added a PCD Visualizer and implemented several point cloud visualization techniques (handlers) for geometry and color
 * added getFieldsList (PCL/io) for a pcl::PointCloud
 * added VTK as a dependency for visualization in an attempt to be compatible with older Linux distributions

## *= 0.1.9 (2010-06-28) =*

* [[pcl]]

 * added radius search for organized_data
 * fixed some bugs in the range image class
 * structural changes in range images related classes
 * split point_types.h in point_types.h and point_types.hpp and added output operators for all points
 * Added angles.h, norms.h, time.h, transform.h and common_headers.h (which includes all the others) to provide some basic functions for convenience.
 * Added border_extraction for range images
 * Added vectorAverage to features, which calculates mean and covariance matrix incrementally without storing added points.
 * Added macros.h, which provides some basic macros for convenience.
 * added an initial PCL Visualizer implementation
 * replaced rosrecord with rosbag for all PCL tools
 * added cluster_max size to Euclidean Clustering (r30210)
 * adding cylinder direction axis constraints + sphere min/max radius constraints for model segmentation (r30195)
 * fixed a grave bug which was preventing objects containing ANN trees to be used in multiple threads (r30194)
 * fixed a bug where patches for ANN 1.1.2 were not applied correctly (r30193)
 * all nodelets now have use_indices and max_queue_size checked at startup via PCLNodelet base class
 * switched from rosrecord to the rosbag API for BAGReader
 * enable max_queue_size parameter (default to 1) for all nodelets
 * set the indices/distances to 0 if the search failed for ANN trees (r30130)
 * PCD reader fix when an invalid file name is given (r30004)
 * added range image class (r29971)
 * added axis and eps_angle as ROS parameters for segmentation (r29938)
 * setting the maximum number of clusters that are to be published via max_clusters (r29935)
 * fixed a grave bug where SACModelOrientedPlane was performing an infinite loop inside selectWithinDistance (r29934)

## *= 0.1.8 (2010-05-24) =*

* [[pcl]]

 * simplified API in point_cloud_converter
 * new general purpose 3D point in 2D polygon check method (r29645)
 * added a getPointsInBox() method
 * fixed a bug where some points were not considered as being part of the polygonal prism due to polygonal bound checking errors (r29597)
 * added a PointXYZI type to represent intensity data.
 * updating flann to 1.5_pre2 (r29549)
 * added MovingLeastSquares tutorial (r29539)

## *= 0.1.7 (2010-05-13) =*

* [[pcl]]

 * getMinMax3D<T> methods now reside in common
 * fixed a major bug in VoxelGrid where centroids were not initialized correctly
 * fixed a bug where NdCopyEigenPointFunctor and NdCopyPointEigenFunctor were given incorrect values
 * reorganizing the library for faster compilation
 * fix for linker errors when linking together multiple translation units that include the same point type registration. Static tag name strings would be defined more than once. #4071 (r29435)

## *= 0.1.6 (2010-05-01) =*

* [[pcl]]

 * added parameter sets for Filter, VoxelGrid, ExtractIndices (r29229)
 * refactorized the dynamic reconfigure parametrization for nodelets
 * added tutorial/sample for cylinder segmentation via SAC (r29224)
 * introduced bad model handling in the sample consensus loop (r29218)
 * added the field name to the dynamic reconfigure parameter list for Filters (r29211)
 * improved NaN handling in kdtrees (r29204)
 * fixed a bug where PassThrough was incorrectly declared as a nodelet (r29202)
 * added tutorial/sample for the ConvexHull2D algorithm (r29198)
 * added set/get radius limits (to be extended), implemented radius limits for cylinder models, and better RANSAC model checking (r29194)
 * default input topic on pointcloud_to_pcd changed from "tilt_laser_cloud" to "input" (r29137)
 * CMakeLists split into individual files for better organization
 * fixed a couple of boost linking errors on some machines
 * transformed point_cloud_converter from a node into a library+node so it can be reused elsewhere
 * added a command line tool for concatenating PCD files (r29035)
 * added an implementation of the Viewpoint Feature Histogram (VFH) (r29032)

## *= 0.1.5 (2010-04-22) =*

* [[pcl]]

 * added tutorial/sample for the ExtractIndices filter (r28990)
 * added tutorial/sample for the VoxelGrid filter (r28986)
 * added tutorial/sample for the StatisticalOutlierRemoval filter (r28985)
 * added templated version of loadPCDFile (r28978) and PCDReader::read
 * added unit tests for the StatisticalOutlierRemoval filter (r28971)
 * added unit tests for the ProjectInliers filter (r28970)
 * added unit tests for the VoxelGrid filter (r28969)
 * added negative limits and fixed a bug where the centroid was not correctly initialized in VoxelGrid (r28967)
 * added unit tests for the PassThrough filter (r28966)
 * added unit tests for PCL filters: RadiusOutlierRemoval and ExtractIndices (r28961)
 * added extra unit tests for concatenate points/fields (r28959)
 * added tutorial/sample code for BAGReader (r28955)

## *= 0.1.4 (2010-04-21) =*

* [[pcl]]

 * added a few nodelet tests (talker, listener, pingpong)
 * added unit tests for BAGReader (r28924)
 * added an implementation for BAGReader (r28922)
 * added unit tests for PCDReader/PCDWriter
 * fixed a few bugs where certain include headers were not exposed properly
 * added an implementation of a statistical outlier removal filter (r28886)
 * added an implementation of radius outlier removal filter (r28878)
 * fixed a bug (#3378) where the output file name in several tools was incorrect (r28858)
 * added an implementation of non-linear ICP (28854)
 * fixed a bug where the min/max estimation was incorrect for sensor_msgs/PointCloud2 types in voxel_grid (r28846)

## *= 0.1.3 (2010-04-15) =*

* [[pcl]]

 * final transformation is now accessible after registration (r28797)
 * added filter_limit_negative to switch between inside/outside interval (r28796)
 * added =/+= operator to PointCloud<PointT> (r28795)
 * added a set of tutorials for IO, Segmentation, Filtering
 * fixed a very ugly bug regarding covariance matrices having 0 elements (r28789)
 * IterativeClosestPoint tested and works (r28706)
 * added set/get maximum distance for organized data index and virtualized the ANN radiusSeach (r28705)
 * added demeaning methods to registration/transforms.h
 * updated ANN library to 0.1.2
 * fixed a major bug in ExtractPolygonalPrismData where distances were incorrectly checked
 * added plane normal flip via viewpoint to ExtractPolygonalPrismData
 * added axis and eps_angle (set/get) to SACSegmentationFromNormals
 * added methods for transforming PointCloud datasets containing normals

## *= 0.1.2 (2010-04-05) =*

* [[pcl]]

 * added a first draft implementation of tf_pcl
 * added missing descriptions to PCL nodelets list
 * fixed a few bugs in voxel_grid where the offset was not incremented correctly (r28559)
 * added a new passthrough filter
 * fixed a few bugs regarding Eigen/Boost's make_shared (r28551)
 * solved numerical instability issue in MLS, preferred the solution using doubles over the one using SVD (r28502). MLS works great now.
 * simplified the registration API (still in progress)
 * changed point_cloud_converter to accept 2 input and 2 output topics, thus fixing the "subscribe when nothing is being published" problem (r28540)

## *= 0.1.1 (2010-03-29) =*

* [[pcl]]

 * fixed a very nasty bug in VoxelGrid where leaves were not properly initialized (r28404)
 * fixed extractEuclideanClusters (was not working correctly)
 * added min/max height for polygonal prism data extraction (r28325)
 * added a bag_to_pcd tool that doesn't need ROS to convert BAG files to PCD (r28469)
 * fixed a bug where PCL classes couldn't be used unless ros::init was called first
 * classes now inherit from PCLNodelet and PCLBase
 * setInputCloud and setIndices are defined in PCLBase now, and used everywhere
 * fixed a bug where kdtree structures were incorrectly created
 * added get/set helper methods everywhere
 * PointCloud2 structures can now contain NaN (Not a Number) values, so extra checks have been added in a lot of places
 * added new distance methods to sample consensus models
 * removed ROS_DEBUG statements in the feature constructors
 * indices can now be set via pcl::PointIndices too
 * fixed a bug where the surface was not getting re-initialized in Feature (r28405)
 * added explicit checks for invalid covariance matrices (r28306)
 * fixed a bug where indices are created to mimic the input point cloud data but never reset (r28306)
 * cleaned the C++ API documentation
 * initial MLS port (untested)

## *= 0.1.0 (2010-03-10) =*

* [[pcl]]

 * initial release

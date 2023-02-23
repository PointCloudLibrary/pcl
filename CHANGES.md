# ChangeList

## = 1.13.0 (10 December 2022) =

### Notable changes

**New features** *added to PCL*

* **[ci]** Add new CI to cover uncommon configurations, running weekly [[#5208](https://github.com/PointCloudLibrary/pcl/pull/5208)]
* **[common]** Added PointCloudXYZHSVtoXYZRGB function [[#5220](https://github.com/PointCloudLibrary/pcl/pull/5220)]
* **[visualization]** Add option to enable EDL rendering. [[#4941](https://github.com/PointCloudLibrary/pcl/pull/4941)]
* **[io]** Add empty point cloud support at pcd file. [[#5400](https://github.com/PointCloudLibrary/pcl/pull/5400)]
* **[filters]** FrustumCulling: allowing infinite far plane distance [[#5433](https://github.com/PointCloudLibrary/pcl/pull/5433)]
* **[filters]** FrustumCulling: asymmetrical Field of View [[#5438](https://github.com/PointCloudLibrary/pcl/pull/5438)]
* **[filters]** Add farthest point sampling filter [[#3723](https://github.com/PointCloudLibrary/pcl/pull/3723)]
* **[sample_consensus]** Adds Ellipse3D SacModel Class [[#5512](https://github.com/PointCloudLibrary/pcl/pull/5512)]

**Deprecation** *of public APIs, scheduled to be removed after two minor releases*

* **[gpu]** Use C++11 `std::atomic` instead of non-standard extensions [[#4807](https://github.com/PointCloudLibrary/pcl/pull/4807)]
* **[registration]** Use likelihood instead of probability in `ndt` [[#5073](https://github.com/PointCloudLibrary/pcl/pull/5073)]
* **[gpu]** Remove hand-rolled `static_assert` [[#4797](https://github.com/PointCloudLibrary/pcl/pull/4797)]
* Deprecate remaining three boost.h headers [[#5486](https://github.com/PointCloudLibrary/pcl/pull/5486)]

**Removal** *of the public APIs deprecated in previous releases*

* Remove deprecated code before PCL 1.13.0 release [[#5375](https://github.com/PointCloudLibrary/pcl/pull/5375)]

**Behavior changes** *in classes, apps, or tools*

* **[cmake]** Drop version from pkg-config files, now accessible as `pcl_{module}.pc` [[#4977](https://github.com/PointCloudLibrary/pcl/pull/4977)]

**API changes** *that did not go through the proper deprecation and removal cycle*

* **[filters]** applyFilter() made protected [[#4933](https://github.com/PointCloudLibrary/pcl/pull/4933)]

**ABI changes** *that are still API compatible*

* **[filters]** Add `PCL_MAKE_ALIGNED_OPERATOR_NEW` to CropBox for better Eigen support [[#4962](https://github.com/PointCloudLibrary/pcl/pull/4962)]
* **[features]** Add more configuration options to GRSDEstimation [[#4852](https://github.com/PointCloudLibrary/pcl/pull/4852)]
* **[sample_consensus]** Implement `SampleConsensusModelSphere<PointT>::projectPoints` properly [[#5095](https://github.com/PointCloudLibrary/pcl/pull/5095)]
* **[filters]** applyFilter() made protected [[#4933](https://github.com/PointCloudLibrary/pcl/pull/4933)]
* **[io]** Introduce buffer for texture coordinate indices in TextureMesh [[#4971](https://github.com/PointCloudLibrary/pcl/pull/4971)]
* **[filters]** Added region of interest for frustum culling [[#5136](https://github.com/PointCloudLibrary/pcl/pull/5136)]
* **[common]** constexpr constructors for point types [[#4646](https://github.com/PointCloudLibrary/pcl/pull/4646)]

### Changes grouped by module

#### CMake:

* **[behavior change]** Drop version from pkg-config files, now accessible as `pcl_{module}.pc` [[#4977](https://github.com/PointCloudLibrary/pcl/pull/4977)]
* Don't require boost in pkg-config [[#5110](https://github.com/PointCloudLibrary/pcl/pull/5110)]
* Link against atomic if needed (found on armel) [[#5117](https://github.com/PointCloudLibrary/pcl/pull/5117)]
* Add more error handling to some PCL CMake functions [[#5344](https://github.com/PointCloudLibrary/pcl/pull/5344)]
* Combine `PCL_SUBSUBSYS_DEPEND` with `PCL_SUBSYS_DEPEND` [[#5387](https://github.com/PointCloudLibrary/pcl/pull/5387)]
* Allow compilation with Boost 1.80 [[#5391](https://github.com/PointCloudLibrary/pcl/pull/5391)]
* Fix issue with `TARGET_PDB_FILE` [[#5396](https://github.com/PointCloudLibrary/pcl/pull/5396)]
* Fix append multiple BOOST_ALL_NO_LIB in preprocessor macro [[#5221](https://github.com/PointCloudLibrary/pcl/pull/5221)]
* Add define for static build google benchmark. [[#5492](https://github.com/PointCloudLibrary/pcl/pull/5492)]

#### libpcl_common:

* Fix division and subtraction operators [[#4909](https://github.com/PointCloudLibrary/pcl/pull/4909)]
* Add `PointXY` specific behavior to `transformPointCloud()` [[#4943](https://github.com/PointCloudLibrary/pcl/pull/4943)]
* Fix division by 0 width in `PointCloud` structured `assign` [[#5113](https://github.com/PointCloudLibrary/pcl/pull/5113)]
* RangeImage: add check before accessing lookup table [[#5149](https://github.com/PointCloudLibrary/pcl/pull/5149)]
* Fix build errors [[#5155](https://github.com/PointCloudLibrary/pcl/pull/5155)]
* **[new feature]** Added PointCloudXYZHSVtoXYZRGB function [[#5220](https://github.com/PointCloudLibrary/pcl/pull/5220)]
* **[ABI break]** constexpr constructors for point types [[#4646](https://github.com/PointCloudLibrary/pcl/pull/4646)]
* Add `bool`, `std::{u}int64_t` as possible point field types [[#4133](https://github.com/PointCloudLibrary/pcl/pull/4133)]

#### libpcl_cuda:

* Fix build errors [[#5155](https://github.com/PointCloudLibrary/pcl/pull/5155)]
* Remove `using namespace thrust` [[#5326](https://github.com/PointCloudLibrary/pcl/pull/5326)]
* Fix linking error for Kinfu [[#5327](https://github.com/PointCloudLibrary/pcl/pull/5327)]

#### libpcl_features:

* **[ABI break]** Add more configuration options to GRSDEstimation [[#4852](https://github.com/PointCloudLibrary/pcl/pull/4852)]
* Fix segfault executing multiscale feature persistence [[#5109](https://github.com/PointCloudLibrary/pcl/pull/5109)]
* Remove unnecessary member in SHOTEstimationBase [[#5434](https://github.com/PointCloudLibrary/pcl/pull/5434)]

#### libpcl_filters:

* **[ABI break]** Add `PCL_MAKE_ALIGNED_OPERATOR_NEW` to CropBox for better Eigen support [[#4962](https://github.com/PointCloudLibrary/pcl/pull/4962)]
* **[API break][ABI break]** applyFilter() made protected [[#4933](https://github.com/PointCloudLibrary/pcl/pull/4933)]
* Use `-FLT_MAX` instead of `FLT_MIN` for minimum value [[#5119](https://github.com/PointCloudLibrary/pcl/pull/5119)]
* **[ABI break]** Added region of interest for frustum culling [[#5136](https://github.com/PointCloudLibrary/pcl/pull/5136)]
* Fix missing definition "boost::optional" [[#5213](https://github.com/PointCloudLibrary/pcl/pull/5213)]
* Fix CropHull::applyFilter3D() [[#5255](https://github.com/PointCloudLibrary/pcl/pull/5255)]
* Fix UniformSampling filter by correcting distance calculation to voxel center [[#4328](https://github.com/PointCloudLibrary/pcl/pull/4328)]
* Fix error due to multiple declarations of template member function specializations in pyramid [[#3973](https://github.com/PointCloudLibrary/pcl/pull/3973)]
* Fix segfault of NDT for sparse clouds [[#5399](https://github.com/PointCloudLibrary/pcl/pull/5399)]
* **[new feature]** FrustumCulling: allowing infinite far plane distance [[#5433](https://github.com/PointCloudLibrary/pcl/pull/5433)]
* **[new feature]** FrustumCulling: asymmetrical Field of View [[#5438](https://github.com/PointCloudLibrary/pcl/pull/5438)]
* **[new feature]** Add farthest point sampling filter [[#3723](https://github.com/PointCloudLibrary/pcl/pull/3723)]
* PassThrough: add more checks for field type and name [[#5490](https://github.com/PointCloudLibrary/pcl/pull/5490)]

#### libpcl_gpu:

* **[deprecation]** Use C++11 `std::atomic` instead of non-standard extensions [[#4807](https://github.com/PointCloudLibrary/pcl/pull/4807)]
* **[deprecation]** Remove hand-rolled `static_assert` [[#4797](https://github.com/PointCloudLibrary/pcl/pull/4797)]
* Remove `using namespace thrust` [[#5326](https://github.com/PointCloudLibrary/pcl/pull/5326)]
* Fix linking error for Kinfu [[#5327](https://github.com/PointCloudLibrary/pcl/pull/5327)]

#### libpcl_io:

* **[ABI break]** Introduce buffer for texture coordinate indices in TextureMesh [[#4971](https://github.com/PointCloudLibrary/pcl/pull/4971)]
* Fix wrong header when saving PolygonMesh to ply file [[#5169](https://github.com/PointCloudLibrary/pcl/pull/5169)]
* Reimplement boost::split and optimize tokenization [[#5285](https://github.com/PointCloudLibrary/pcl/pull/5285)]
* Fixes Crash in pcl::PLYReader::amendProperty [[#5331](https://github.com/PointCloudLibrary/pcl/pull/5331)]
* Fix multiple memory corruption errors revealed by fuzzing [[#5342](https://github.com/PointCloudLibrary/pcl/pull/5342)]
* **[new feature]** Add empty point cloud support at pcd file. [[#5400](https://github.com/PointCloudLibrary/pcl/pull/5400)]
* Fix compile issues when compiling OpenNIDriver [[#5452](https://github.com/PointCloudLibrary/pcl/pull/5452)]
* PCDReader: remove fields with zero count instead of throwing exception while reading [[#4623](https://github.com/PointCloudLibrary/pcl/pull/4623)]
* PLYReader: Return empty handler if rgb doesn't exist when trying to add alpha value [[#5415](https://github.com/PointCloudLibrary/pcl/pull/5415)]

#### libpcl_keypoints:

* Fix HarrisKeypoint3D::refineCorners [[#5365](https://github.com/PointCloudLibrary/pcl/pull/5365)]
* Fix OpenMP compile issue under MSVC [[#5453](https://github.com/PointCloudLibrary/pcl/pull/5453)]

#### libpcl_octree:

* getSize: should return 0 when data_ is invalid [[#5352](https://github.com/PointCloudLibrary/pcl/pull/5352)]
* deleteTree: max_x_ was not reset [[#5353](https://github.com/PointCloudLibrary/pcl/pull/5353)]

#### libpcl_recognition:

* fix quantized normals' bin boundaries not consistent in different places in linemod [[#5464](https://github.com/PointCloudLibrary/pcl/pull/5464)]
* fix linemod binindex wrong range bug in surface normal modality [[#5444](https://github.com/PointCloudLibrary/pcl/pull/5444)]

#### libpcl_registration:

* Fix doxygen comment blocks in `ndt.h` [[#5080](https://github.com/PointCloudLibrary/pcl/pull/5080)]
* **[deprecation]** Use likelihood instead of probability in `ndt` [[#5073](https://github.com/PointCloudLibrary/pcl/pull/5073)]
* fix: use `similarity_threshold_squared_` instead of `cardinality_` in… [[#5236](https://github.com/PointCloudLibrary/pcl/pull/5236)]
* Fix of IterativeClosestPointWithNormals shared_ptr [[#4438](https://github.com/PointCloudLibrary/pcl/pull/4438)]
* print loss as debug for TransformationEstimationSVD and TransformationEstimationPointToPlaneLLS [[#5279](https://github.com/PointCloudLibrary/pcl/pull/5279)]
* add Scalar template variable to RegistrationVisualizer [[#5290](https://github.com/PointCloudLibrary/pcl/pull/5290)]
* add Scalar template variable to NormalDistributionsTransform [[#5291](https://github.com/PointCloudLibrary/pcl/pull/5291)]
* add Scalar template variable to GeneralizedIterativeClosestPoint [[#5312](https://github.com/PointCloudLibrary/pcl/pull/5312)]
* Fix segfault of NDT for sparse clouds [[#5399](https://github.com/PointCloudLibrary/pcl/pull/5399)]
* Fix can't compile getMeanPointDensity [[#5447](https://github.com/PointCloudLibrary/pcl/pull/5447)]
* GICP: correct matrix multiplication [[#5489](https://github.com/PointCloudLibrary/pcl/pull/5489)]

#### libpcl_sample_consensus:

* **[ABI break]** Implement `SampleConsensusModelSphere<PointT>::projectPoints` properly [[#5095](https://github.com/PointCloudLibrary/pcl/pull/5095)]
* **[new feature]** Adds Ellipse3D SacModel Class [[#5512](https://github.com/PointCloudLibrary/pcl/pull/5512)]

#### libpcl_surface:

* Fix mls voxel grid hashing out of bound [[#4973](https://github.com/PointCloudLibrary/pcl/pull/4973)]
* Fix nonsense code in pcl/surface/3rdparty/poisson4/sparse_matrix.hpp [[#5256](https://github.com/PointCloudLibrary/pcl/pull/5256)]
* GridProjection: scale output back to original size [[#5301](https://github.com/PointCloudLibrary/pcl/pull/5301)]
* Solve an internal compiler issue on MSVC 2022 within openNURBS [[#5463](https://github.com/PointCloudLibrary/pcl/pull/5463)]
* Fix segfault in mls::performUpsampling [[#5483](https://github.com/PointCloudLibrary/pcl/pull/5483)]

#### libpcl_visualization:

* Fix Bug  between addText3D and QVTKWidget [[#5054](https://github.com/PointCloudLibrary/pcl/pull/5054)]
* Change static to const since it-stability is not guaranteed [[#5147](https://github.com/PointCloudLibrary/pcl/pull/5147)]
* Add missing vtk library for context2d. [[#5160](https://github.com/PointCloudLibrary/pcl/pull/5160)]
* Fix problem with spin() and spinOnce() for X Window System [[#5252](https://github.com/PointCloudLibrary/pcl/pull/5252)]
* add Scalar template variable to RegistrationVisualizer [[#5290](https://github.com/PointCloudLibrary/pcl/pull/5290)]
* Fix PCLVisualizer::addPointCloudPrincipalCurvatures [[#5369](https://github.com/PointCloudLibrary/pcl/pull/5369)]
* **[new feature]** Add option to enable EDL rendering. [[#4941](https://github.com/PointCloudLibrary/pcl/pull/4941)]
* Support handling numpad +/- key event for visualizer [[#5468](https://github.com/PointCloudLibrary/pcl/pull/5468)]
* Fix usage of dangling pointer in PCLVisualizer::getUniqueCameraFile [[#5481](https://github.com/PointCloudLibrary/pcl/pull/5481)]
* point and area picking improvement for cloud names [[#5476](https://github.com/PointCloudLibrary/pcl/pull/5476)]
* Access to a potential null pointer in interactor_style (#5503) [[#5507](https://github.com/PointCloudLibrary/pcl/pull/5507)]

#### PCL Apps:

* fix vtk-qt crash on macos for manual_registration app [[#5432](https://github.com/PointCloudLibrary/pcl/pull/5432)]
* Fix pcd_video_player crash on OSX [[#5421](https://github.com/PointCloudLibrary/pcl/pull/5421)]

#### PCL Tutorials:

* Fix alignment prerejective tutorial [[#5363](https://github.com/PointCloudLibrary/pcl/pull/5363)]

#### PCL Tools:

* Add check in pcd_viewer.cpp for padding fields [[#5442](https://github.com/PointCloudLibrary/pcl/pull/5442)]

#### CI:

* Fix benchmark compilation issue on Ubuntu 21.10 [[#5165](https://github.com/PointCloudLibrary/pcl/pull/5165)]
* **[new feature]** Add new CI to cover uncommon configurations, running weekly [[#5208](https://github.com/PointCloudLibrary/pcl/pull/5208)]
* Add clang-tidy in a GitHub workflow [[#4636](https://github.com/PointCloudLibrary/pcl/pull/4636)]
* Update vcpkg to version 2022.07.25 on x86 windows to fix libharu hash value error. [[#5418](https://github.com/PointCloudLibrary/pcl/pull/5418)]
* Install openni2 in windows dockers [[#5459](https://github.com/PointCloudLibrary/pcl/pull/5459)]

## = 1.12.1 (2021.12.21) =

This minor release brings in a lot of enhancements in CMake thanks to @larshg and @SunBlack.
Enjoy a lot of bug-fixes and improvements in IO and Filters.

### Notable changes

**New features** *added to PCL*

* **[io]** Add a grabber for SICK 2D LiDAR: TiM [[#4429](https://github.com/PointCloudLibrary/pcl/pull/4429)]

**Deprecation** *of public APIs, scheduled to be removed after two minor releases*

* **[cuda][filters]** Add deprecation for filter getters with bool reference [[#4861](https://github.com/PointCloudLibrary/pcl/pull/4861)]
* **[filters]** Fix keep_organized behavior in CropHull filter [[#4855](https://github.com/PointCloudLibrary/pcl/pull/4855)]

**Behavior changes** *in classes, apps, or tools*

* **[registration]** Fix typo in the hessian representation of NDT [[#4889](https://github.com/PointCloudLibrary/pcl/pull/4889)]
* **[cmake]** Update PCLConfig.cmake.in to 3.10 for default policy.  [[#4996](https://github.com/PointCloudLibrary/pcl/pull/4996)]

**ABI changes** *that are still API compatible*

* **[ml]** Wrap QMatrix in namespace pcl to resolve linker conflict with Qt6 [[#4858](https://github.com/PointCloudLibrary/pcl/pull/4858)]

### Changes grouped by module

#### CMake:

* Add AVX for Linux & macos [[#4698](https://github.com/PointCloudLibrary/pcl/pull/4698)]
* Update cmake to 3.10 and add CUDA language support [[#4619](https://github.com/PointCloudLibrary/pcl/pull/4619)]
* Fix CUDA Compute Capability version detection [[#4900](https://github.com/PointCloudLibrary/pcl/pull/4900)]
* Update pcl_find_boost to allow compilation with Boost 1.77 and 1.78 [[#4972](https://github.com/PointCloudLibrary/pcl/pull/4972)] [[#5067](https://github.com/PointCloudLibrary/pcl/pull/5067)]
* Allow boost to be found by config files. [[#4952](https://github.com/PointCloudLibrary/pcl/pull/4952)]
* **[behavior change]** Update PCLConfig.cmake.in to 3.10 for default policy.  [[#4996](https://github.com/PointCloudLibrary/pcl/pull/4996)]
* Allow PCL to have non-static dependencies for static builds and vice-versa [[#4390](https://github.com/PointCloudLibrary/pcl/pull/4390)]
* Enhance finding of qhull [[#4923](https://github.com/PointCloudLibrary/pcl/pull/4923)]

#### libpcl_common:

* Fix: max_id size should be equal to histogram.size() - 1 [[#4934](https://github.com/PointCloudLibrary/pcl/pull/4934)]
* Remove casts, use more auto and uindex_t in conversions.h [[#4935](https://github.com/PointCloudLibrary/pcl/pull/4935)]
* Fix inaccurate covariance matrix computation [[#4983](https://github.com/PointCloudLibrary/pcl/pull/4983)]

#### libpcl_cuda:

* **[deprecation]** Add deprecation for filter getters with bool reference [[#4861](https://github.com/PointCloudLibrary/pcl/pull/4861)]

#### libpcl_features:

* Add `isNormalFinite` check in `ShapeContext3DEstimation` [[#4883](https://github.com/PointCloudLibrary/pcl/pull/4883)]

#### libpcl_filters:

* Clear the output indices in function `CropHull::applyFilters` [[#4851](https://github.com/PointCloudLibrary/pcl/pull/4851)]
* Fix unresolved linking to Convolution [[#4845](https://github.com/PointCloudLibrary/pcl/pull/4845)]
* **[deprecation]** Add deprecation for filter getters with bool reference [[#4861](https://github.com/PointCloudLibrary/pcl/pull/4861)]
* NormalSpaceSampling filter: add constructor to specify `extract_removed_indices`  [[#4846](https://github.com/PointCloudLibrary/pcl/pull/4846)]
* **[deprecation]** Fix keep_organized behavior in CropHull filter [[#4855](https://github.com/PointCloudLibrary/pcl/pull/4855)]
* Added reserve function before storing points in PointCloud in VoxelGr… [[#4938](https://github.com/PointCloudLibrary/pcl/pull/4938)]

#### libpcl_io:

* Higher flexibility regarding which PLY files can be read [[#4963](https://github.com/PointCloudLibrary/pcl/pull/4963)]
* **[new feature]** Add a grabber for SICK 2D LiDAR: TiM [[#4429](https://github.com/PointCloudLibrary/pcl/pull/4429)]

#### libpcl_keypoints:

* Bugfix: Number of OpenMP threads was not validated, ... [[#4863](https://github.com/PointCloudLibrary/pcl/pull/4863)]

#### libpcl_ml:

* **[ABI break]** Wrap QMatrix in namespace pcl to resolve linker conflict with Qt6 [[#4858](https://github.com/PointCloudLibrary/pcl/pull/4858)]

#### libpcl_registration:

* **[behavior change]** Fix typo in the hessian representation of NDT [[#4889](https://github.com/PointCloudLibrary/pcl/pull/4889)]
* Fix discretization bug in PPFRegistration [[#4975](https://github.com/PointCloudLibrary/pcl/pull/4975)]

#### libpcl_sample_consensus:

* Fix SampleConsensusModelCylinder.projectPoints and verify with test [[#4881](https://github.com/PointCloudLibrary/pcl/pull/4881)]

#### libpcl_search:

* Add missing include of hpp file in flann_search.h [[#4848](https://github.com/PointCloudLibrary/pcl/pull/4848)]

#### libpcl_surface:

* Improve logging in `multi_grid_octree_data.hpp` [[#4844](https://github.com/PointCloudLibrary/pcl/pull/4844)]
* Fix duplicate definition error in mls [[#5049](https://github.com/PointCloudLibrary/pcl/pull/5049)]

#### libpcl_visualization:

* Use pixel ratio to scale mouse events on HiDpi monitors [[#4411](https://github.com/PointCloudLibrary/pcl/pull/4411)]
* Remove declaration of updateCamera [[#4921](https://github.com/PointCloudLibrary/pcl/pull/4921)]

#### PCL Docs:

* Require sphinx>=3 to fix errors on readthedocs [[#5037](https://github.com/PointCloudLibrary/pcl/pull/5037)]

## = 1.12.0 (2021.07.07) =

PCL 1.12.0 enables custom index size and type, from `int16_t` to `uint64_t`, allowing
users to have as small or large clouds as they wish. 1.12 also comes with improved
support for VTK and CUDA, along with making existing functionality more user friendly.

This is all on top of the usual bug-fixes and performance improvements across the board

### Notable changes

**New features** *added to PCL*

* **[sample_consensus]** Add SIMD implementations to some countWithinDistance functions [[#3519](https://github.com/PointCloudLibrary/pcl/pull/3519)]
* **[io]** Enable Real Sense 2 grabber for all platforms [[#4471](https://github.com/PointCloudLibrary/pcl/pull/4471)]
* **[visualization]** add ellipsoid shape to pcl_visualizer [[#4531](https://github.com/PointCloudLibrary/pcl/pull/4531)]
* **[common]** Add `constexpr` to static member functions for point types, add macro for `if constexpr` [[#4735](https://github.com/PointCloudLibrary/pcl/pull/4735)]
* **[ci]** Use windows docker image in CI. [[#4426](https://github.com/PointCloudLibrary/pcl/pull/4426)]
* **[common]** Add pcl log stream macros [[#4595](https://github.com/PointCloudLibrary/pcl/pull/4595)]

**Deprecation** *of public APIs, scheduled to be removed after two minor releases*

* **[common]** Modify index type for vertices [[#4256](https://github.com/PointCloudLibrary/pcl/pull/4256)]
* **[gpu]** Add square distances to GPU knnSearch API [[#4322](https://github.com/PointCloudLibrary/pcl/pull/4322)]
* **[gpu]** Add square distances to ApproxNearestSearch [[#4340](https://github.com/PointCloudLibrary/pcl/pull/4340)]
* Deprecate unused ease-of-internal-use headers [[#4367](https://github.com/PointCloudLibrary/pcl/pull/4367)]
* **[registration]** Deprecate `TransformationEstimationDQ` in favor of `TransformationEstimationDualQuaternion` [[#4425](https://github.com/PointCloudLibrary/pcl/pull/4425)]
* **[segmentation]** Deprecate unused `max_label` in `extractLabeledEuclideanClusters` [[#4105](https://github.com/PointCloudLibrary/pcl/pull/4105)]
* **[surface]** MLS: correct typo in `principle` by using `principal` instead [[#4505](https://github.com/PointCloudLibrary/pcl/pull/4505)]
* Deprecate unneeded meta-headers [[#4628](https://github.com/PointCloudLibrary/pcl/pull/4628)]
* **[apps][tracking]** pyramidal klt: switch keypoints_status_ to int vector [[#4681](https://github.com/PointCloudLibrary/pcl/pull/4681)]
* Properly remove remaining items deprecated for version 1.12, deprecate `uniform_sampling.h` [[#4688](https://github.com/PointCloudLibrary/pcl/pull/4688)]
* **[recognition]** Add deprecation for incorrectly installed headers [[#4650](https://github.com/PointCloudLibrary/pcl/pull/4650)]

**Removal** *of the public APIs deprecated in previous releases*

* Remove deprecated items as scheduled in preparation of v1.12 (except `concatenatePointCloud`) [[#4341](https://github.com/PointCloudLibrary/pcl/pull/4341)]
* **[apps]** Remove unused code in persistence_utils.h [[#4500](https://github.com/PointCloudLibrary/pcl/pull/4500)]
* Properly remove remaining items deprecated for version 1.12, deprecate `uniform_sampling.h` [[#4688](https://github.com/PointCloudLibrary/pcl/pull/4688)]

**Behavior changes** *in classes, apps, or tools*

* **[registration]** Don't move, or copy ICP [[#4167](https://github.com/PointCloudLibrary/pcl/pull/4167)]
* **[common]** Fix PointXYZRGBA ctor, set A as 255 by default [[#4799](https://github.com/PointCloudLibrary/pcl/pull/4799)]

**API changes** *that did not go through the proper deprecation and removal cycle*

* **[common]** modify index type for PCLImage [[#4257](https://github.com/PointCloudLibrary/pcl/pull/4257)]
* **[registration]** Don't move, or copy ICP [[#4167](https://github.com/PointCloudLibrary/pcl/pull/4167)]
* **[kdtree]** KdTree: handle 0 or negative k for nearestKSearch [[#4430](https://github.com/PointCloudLibrary/pcl/pull/4430)]
* **[common]** Use `std::array` instead of C-array for ColorLUT [[#4489](https://github.com/PointCloudLibrary/pcl/pull/4489)]
* **[tracking]** Use SFINAE instead of relying on macro `PCL_TRACKING_NORMAL_SUPPORTED` [[#4643](https://github.com/PointCloudLibrary/pcl/pull/4643)]
* **[gpu]** Export and template extract clusters [[#4196](https://github.com/PointCloudLibrary/pcl/pull/4196)]
* **[common]** Added `namespace pcl` to free functions: `aligned_{malloc/free}` [[#4742](https://github.com/PointCloudLibrary/pcl/pull/4742)]

**ABI changes** *that are still API compatible*

* **[registration]** Refactoring and Bugfix of NDT [[#4180](https://github.com/PointCloudLibrary/pcl/pull/4180)]
* **[common]** modify index types for PCLPointCloud2 [[#4199](https://github.com/PointCloudLibrary/pcl/pull/4199)]
* **[common]** Modify index type for vertices [[#4256](https://github.com/PointCloudLibrary/pcl/pull/4256)]
* **[common]** Modify index type for PCLPointField [[#4228](https://github.com/PointCloudLibrary/pcl/pull/4228)]
* **[surface]** Enabled multithreading in Poisson surface reconstruction [[#4332](https://github.com/PointCloudLibrary/pcl/pull/4332)]
* **[io]** Allow file_io to read large point clouds depending on PCL config [[#4331](https://github.com/PointCloudLibrary/pcl/pull/4331)]
* **[sample_consensus]** Allow user to apply arbitrary constraint on models in sample consensus [[#4260](https://github.com/PointCloudLibrary/pcl/pull/4260)]
* **[tracking]** Use SFINAE instead of relying on macro `PCL_TRACKING_NORMAL_SUPPORTED` [[#4643](https://github.com/PointCloudLibrary/pcl/pull/4643)]
* **[features]** Move the init of static variables to library load time [[#4640](https://github.com/PointCloudLibrary/pcl/pull/4640)]
* **[octree]** Octree2BufBase: Fix bug that contents from previous buffer appear in current buffer [[#4642](https://github.com/PointCloudLibrary/pcl/pull/4642)]

### Changes grouped by module

#### CMake:

* Update `pcl_find_boost` to allow compilation with Boost 1.74 [[#4330](https://github.com/PointCloudLibrary/pcl/pull/4330)]
* Variable needs to be expanded when checking for `EXT_DEPS` [[#4353](https://github.com/PointCloudLibrary/pcl/pull/4353)]
* Update pcl_find_cuda.cmake to contain all supported architectures [[#4400](https://github.com/PointCloudLibrary/pcl/pull/4400)]
* Add support for VTK 9 [[#4262](https://github.com/PointCloudLibrary/pcl/pull/4262)]
* Refactor cmake find script of libusb [[#4483](https://github.com/PointCloudLibrary/pcl/pull/4483)]
* Add AVX for windows [[#4598](https://github.com/PointCloudLibrary/pcl/pull/4598)]
* Add SSE definitions for SSE 4.1 and 4.2 [[#4596](https://github.com/PointCloudLibrary/pcl/pull/4596)]

#### libpcl_common:

* **[ABI break]** modify index types for PCLPointCloud2 [[#4199](https://github.com/PointCloudLibrary/pcl/pull/4199)]
* **[API break]** modify index type for PCLImage [[#4257](https://github.com/PointCloudLibrary/pcl/pull/4257)]
* **[ABI break][deprecation]** Modify index type for vertices [[#4256](https://github.com/PointCloudLibrary/pcl/pull/4256)]
* **[ABI break]** Modify index type for PCLPointField [[#4228](https://github.com/PointCloudLibrary/pcl/pull/4228)]
* Allow PCL_DEPRECATED to detect and help remove deprecations before release [[#4336](https://github.com/PointCloudLibrary/pcl/pull/4336)]
* Allow conversion of PointCloud with more than 32-bit size rows/columns [[#4343](https://github.com/PointCloudLibrary/pcl/pull/4343)]
* Improve routing for `transformPointCloud` [[#4398](https://github.com/PointCloudLibrary/pcl/pull/4398)]
* Correct typo in `transformPlane` [[#4396](https://github.com/PointCloudLibrary/pcl/pull/4396)]
* **[API break]** Use `std::array` instead of C-array for ColorLUT [[#4489](https://github.com/PointCloudLibrary/pcl/pull/4489)]
* Set header in two toPCLPointCloud2 functions [[#4538](https://github.com/PointCloudLibrary/pcl/pull/4538)]
* Add more operators to `PointCloud` to prevent perf regression in refactoring [[#4397](https://github.com/PointCloudLibrary/pcl/pull/4397)]
* Make sure that organized point clouds are still organized after transformPointCloud [[#4488](https://github.com/PointCloudLibrary/pcl/pull/4488)]
* **[API break]** Added `namespace pcl` to free functions: `aligned_{malloc/free}` [[#4742](https://github.com/PointCloudLibrary/pcl/pull/4742)]
* **[new feature]** Add `constexpr` to static member functions for point types, add macro for `if constexpr` [[#4735](https://github.com/PointCloudLibrary/pcl/pull/4735)]
* Fix `PolygonMesh::concatenate` and its unit test [[#4745](https://github.com/PointCloudLibrary/pcl/pull/4745)]
* **[behavior change]** Fix PointXYZRGBA ctor, set A as 255 by default [[#4799](https://github.com/PointCloudLibrary/pcl/pull/4799)]
* Remove pseudo-template-instantiations in eigen.h to reduce compilation time [[#4788](https://github.com/PointCloudLibrary/pcl/pull/4788)]
* **[new feature]** Add pcl log stream macros [[#4595](https://github.com/PointCloudLibrary/pcl/pull/4595)]

#### libpcl_features:

* **[ABI break]** Move the init of static variables to library load time [[#4640](https://github.com/PointCloudLibrary/pcl/pull/4640)]
* Use correct cloud for checking finite-ness in fpfh [[#4720](https://github.com/PointCloudLibrary/pcl/pull/4720)]

#### libpcl_filters:

* Improve performance of median filter by using `nth_element` [[#4360](https://github.com/PointCloudLibrary/pcl/pull/4360)]
* Fix the covariance calculation as suggested by @zxd123 [[#4466](https://github.com/PointCloudLibrary/pcl/pull/4466)]
* Filters: fix wrong initialization of covariance in VoxelGridCovariance [[#4556](https://github.com/PointCloudLibrary/pcl/pull/4556)]
* Fix application of setMinimumPointsNumberPerVoxel for PCLPointCloud2 implementation of VoxelGrid [[#4389](https://github.com/PointCloudLibrary/pcl/pull/4389)]
* Adding tests for CropHull and using hull_cloud instead of input in getHullCloudRange [[#3976](https://github.com/PointCloudLibrary/pcl/pull/3976)]

#### libpcl_gpu:

* **[deprecation]** Add square distances to GPU knnSearch API [[#4322](https://github.com/PointCloudLibrary/pcl/pull/4322)]
* **[deprecation]** Add square distances to ApproxNearestSearch [[#4340](https://github.com/PointCloudLibrary/pcl/pull/4340)]
* **[API break]** Export and template extract clusters [[#4196](https://github.com/PointCloudLibrary/pcl/pull/4196)]
* Update support for CUDA arch in CMake and `convertSMVer2Cores` [[#4748](https://github.com/PointCloudLibrary/pcl/pull/4748)]
* Add ability to download contiguous chunk of memory to host using `Device{Array,Memory}` [[#4741](https://github.com/PointCloudLibrary/pcl/pull/4741)]
* Speeding up GPU clustering using smarter download strategy and memory allocations [[#4677](https://github.com/PointCloudLibrary/pcl/pull/4677)]

#### libpcl_io:

* **[ABI break]** Allow file_io to read large point clouds depending on PCL config [[#4331](https://github.com/PointCloudLibrary/pcl/pull/4331)]
* Improve PCD read performance (more than 50%) by reusing `istringstream` [[#4339](https://github.com/PointCloudLibrary/pcl/pull/4339)]
* **[new feature]** Enable Real Sense 2 grabber for all platforms [[#4471](https://github.com/PointCloudLibrary/pcl/pull/4471)]
* Throw error if the device bluffs about its capability [[#4141](https://github.com/PointCloudLibrary/pcl/pull/4141)]
* Fix crash in Dinast Grabber due to bad initialization of device handle [[#4484](https://github.com/PointCloudLibrary/pcl/pull/4484)]
* PLY face definition accepts uint fields as well [[#4492](https://github.com/PointCloudLibrary/pcl/pull/4492)]
* Prevent segfault in vtk2mesh [[#4581](https://github.com/PointCloudLibrary/pcl/pull/4581)]
* Prevent exception in PCDReader for misformed PCD files [[#4566](https://github.com/PointCloudLibrary/pcl/pull/4566)]
* Enable arbitary size Indices for Octree module [[#4350](https://github.com/PointCloudLibrary/pcl/pull/4350)]
* Fix addition of Carriage Return to PCD files. [[#4727](https://github.com/PointCloudLibrary/pcl/pull/4727)]
* Support Ensenso SDK 3.0 for ensenso_grabber [[#4751](https://github.com/PointCloudLibrary/pcl/pull/4751)]
* Specify no face elements in PLY files (from point cloud) to make them interoperable with VTK [[#4774](https://github.com/PointCloudLibrary/pcl/pull/4774)]

#### libpcl_kdtree:

* **[API break]** KdTree: handle 0 or negative k for nearestKSearch [[#4430](https://github.com/PointCloudLibrary/pcl/pull/4430)]

#### libpcl_ml:

* Fix un-initialized centroids bug (k-means) [[#4570](https://github.com/PointCloudLibrary/pcl/pull/4570)]

#### libpcl_octree:

* Enable arbitary size Indices for Octree module [[#4350](https://github.com/PointCloudLibrary/pcl/pull/4350)]
* Fix problems in octree search functions when using dynamic depth [[#4657](https://github.com/PointCloudLibrary/pcl/pull/4657)]
* **[ABI break]** Octree2BufBase: Fix bug that contents from previous buffer appear in current buffer [[#4642](https://github.com/PointCloudLibrary/pcl/pull/4642)]

#### libpcl_outofcore:

* Fix compile issue due to missing include under MSVC 2019 [[#4755](https://github.com/PointCloudLibrary/pcl/pull/4755)]

#### libpcl_recognition:

* **[deprecation]** Add deprecation for incorrectly installed headers [[#4650](https://github.com/PointCloudLibrary/pcl/pull/4650)]

#### libpcl_registration:

* **[ABI break]** Refactoring and Bugfix of NDT [[#4180](https://github.com/PointCloudLibrary/pcl/pull/4180)]
* **[API break][behavior change]** Don't move, or copy ICP [[#4167](https://github.com/PointCloudLibrary/pcl/pull/4167)]
* **[deprecation]** Deprecate `TransformationEstimationDQ` in favor of `TransformationEstimationDualQuaternion` [[#4425](https://github.com/PointCloudLibrary/pcl/pull/4425)]
* Fix force no recompute [[#4535](https://github.com/PointCloudLibrary/pcl/pull/4535)]
* Skip non-finite points for Pyramid Feature Matching [[#4711](https://github.com/PointCloudLibrary/pcl/pull/4711)]

#### libpcl_sample_consensus:

* **[ABI break]** Allow user to apply arbitrary constraint on models in sample consensus [[#4260](https://github.com/PointCloudLibrary/pcl/pull/4260)]
* Improve logging errors during sample consensus model registration [[#4381](https://github.com/PointCloudLibrary/pcl/pull/4381)]
* **[new feature]** Add SIMD implementations to some countWithinDistance functions [[#3519](https://github.com/PointCloudLibrary/pcl/pull/3519)]
* Faster sample consensus functions [[#4424](https://github.com/PointCloudLibrary/pcl/pull/4424)]
* Fix and improve MLESAC [[#4575](https://github.com/PointCloudLibrary/pcl/pull/4575)]
* Improve logging in module `sample_consensus` [[#4261](https://github.com/PointCloudLibrary/pcl/pull/4261)]

#### libpcl_search:

* Faster organized search [[#4496](https://github.com/PointCloudLibrary/pcl/pull/4496)]
* Add access to boxSearch [[#4282](https://github.com/PointCloudLibrary/pcl/pull/4282)]

#### libpcl_segmentation:

* **[deprecation]** Deprecate unused `max_label` in `extractLabeledEuclideanClusters` [[#4105](https://github.com/PointCloudLibrary/pcl/pull/4105)]
* Fix the dotproduct calculation in `extractEuclideanClusters` for smooth surfaces [[#4162](https://github.com/PointCloudLibrary/pcl/pull/4162)]
* Make euclidean clustering with normals faster [[#4551](https://github.com/PointCloudLibrary/pcl/pull/4551)]

#### libpcl_surface:

* **[ABI break]** Enabled multithreading in Poisson surface reconstruction [[#4332](https://github.com/PointCloudLibrary/pcl/pull/4332)]
* Add stdlib header for malloc in poisson (bugfix for gcc-5) [[#4376](https://github.com/PointCloudLibrary/pcl/pull/4376)]
* Always update counter and prevent overflow access in poisson4 octree [[#4316](https://github.com/PointCloudLibrary/pcl/pull/4316)]
* Add missing include to nurbs_solve_umfpack.cpp [[#4571](https://github.com/PointCloudLibrary/pcl/pull/4571)]
* **[deprecation]** MLS: correct typo in `principle` by using `principal` instead [[#4505](https://github.com/PointCloudLibrary/pcl/pull/4505)]

#### libpcl_visualization:

* Add support for VTK 9 [[#4262](https://github.com/PointCloudLibrary/pcl/pull/4262)]
* **[new feature]** add ellipsoid shape to pcl_visualizer [[#4531](https://github.com/PointCloudLibrary/pcl/pull/4531)]

#### PCL Apps:

* **[removal]** Remove unused code in persistence_utils.h [[#4500](https://github.com/PointCloudLibrary/pcl/pull/4500)]
* **[deprecation]** pyramidal klt: switch keypoints_status_ to int vector [[#4681](https://github.com/PointCloudLibrary/pcl/pull/4681)]

#### PCL Docs:

* Update documentation to be coherent with the style guide [[#4771](https://github.com/PointCloudLibrary/pcl/pull/4771)]

#### PCL Tutorials:

* Replace PassThrough with removeNaNFromPointCloud in 3 tutorials  [[#4760](https://github.com/PointCloudLibrary/pcl/pull/4760)]

#### PCL Tools:

* Fix virtual scanner [[#4730](https://github.com/PointCloudLibrary/pcl/pull/4730)]

#### CI:

* Make windows build on c:\ drive to fix out-of-disk-space errors [[#4382](https://github.com/PointCloudLibrary/pcl/pull/4382)]
* **[new feature]** Use windows docker image in CI. [[#4426](https://github.com/PointCloudLibrary/pcl/pull/4426)]

## = 1.11.1 (13.08.2020) =

Apart from the usual serving of bug-fixes and speed improvements, PCL 1.11.1 brings in
better support for CUDA, more uniform behavior in code as well as cmake, and an
experimental feature: `functor_filter`.

### Notable changes

**New features** *added to PCL*

* **[ci]** Add support for CUDA in CI [[#4101](https://github.com/PointCloudLibrary/pcl/pull/4101)]
* **[common]** Add always-unsigned index type `uindex_t` dependent on `index_t` [[#4205](https://github.com/PointCloudLibrary/pcl/pull/4205)]
* **[filters]** Add a filter accepting a functor to reduce boiler plate code for simple filters [[#3890](https://github.com/PointCloudLibrary/pcl/pull/3890)]

### Changes grouped by module

#### CMake:

* Update `pcl_find_boost` to allow compilation with Boost 1.73 and 1.72 [[#4080](https://github.com/PointCloudLibrary/pcl/pull/4080)]
* Fix NSIS Template for NSIS3 [[#4093](https://github.com/PointCloudLibrary/pcl/pull/4093)]
* Add option to choose `pcl::index_t` while compiling [[#4166](https://github.com/PointCloudLibrary/pcl/pull/4166)]
* Fix name warnings for PCAP, libusb and Ensenso. [[#4182](https://github.com/PointCloudLibrary/pcl/pull/4182)]
* Fixes compile error on MSVC by disabling Whole Program Optimization [[#4197](https://github.com/PointCloudLibrary/pcl/pull/4197)]

#### libpcl_common:

* Remove use of dynamic allocation in `GaussianKernel::convolve{Rows,Cols}` [[#4092](https://github.com/PointCloudLibrary/pcl/pull/4092)]
* Replace usage of `std::vector<int>` with `Indices` [[#3989](https://github.com/PointCloudLibrary/pcl/pull/3989)]
* Update `PointCloud` to conform to requirements of `ReversibleContainer` [[#3980](https://github.com/PointCloudLibrary/pcl/pull/3980)]
* **[new feature]** Add always-unsigned index type `uindex_t` dependent on `index_t` [[#4205](https://github.com/PointCloudLibrary/pcl/pull/4205)]
* Adding `data` member function to `PointCloud` [[#4216](https://github.com/PointCloudLibrary/pcl/pull/4216)]
* Switch `int` to `index_t` for field index variables in `pclBase`, add `pcl::UNAVAILABLE` for same [[#4211](https://github.com/PointCloudLibrary/pcl/pull/4211)]
* Added `resize` with 2 arguments to `PointCloud` [[#4225](https://github.com/PointCloudLibrary/pcl/pull/4225)]
* Adding `max_size` to PointCloud [[#4254](https://github.com/PointCloudLibrary/pcl/pull/4254)]
* Support multiple arguments in `pcl::utils::ignore()` [[#4269](https://github.com/PointCloudLibrary/pcl/pull/4269)]

#### libpcl_features:

* Fix feature histogram constructor bug [[#4234](https://github.com/PointCloudLibrary/pcl/pull/4234)]
* Fix regression in Organized Edge Detection (introduced in PCL 1.10.1) [[#4311](https://github.com/PointCloudLibrary/pcl/pull/4311)]

#### libpcl_filters:

* reduce computations involved in iterating over the pointcloud for `FastBilateral{OMP}` [[#4134](https://github.com/PointCloudLibrary/pcl/pull/4134)]
* **[new feature]** Add a filter accepting a functor to reduce boiler plate code for simple filters [[#3890](https://github.com/PointCloudLibrary/pcl/pull/3890)]
* Refatoring VoxelGridCovariance to make it multi-thread safe (and more) [[#4251](https://github.com/PointCloudLibrary/pcl/pull/4251)]

#### libpcl_gpu:

* Replace volatile shared memory with shfl_sync in KNNSearch [[#4306](https://github.com/PointCloudLibrary/pcl/pull/4306)]
* Fix octree radiusSearch [[#4146](https://github.com/PointCloudLibrary/pcl/pull/4146)]

#### libpcl_io:

* Better conversion of depth data + focal length into X,Y,Z point data [[#4128](https://github.com/PointCloudLibrary/pcl/pull/4128)]
* Add iterator include for  back_iterator to support VS2019 [[#4319](https://github.com/PointCloudLibrary/pcl/pull/4319)]

#### libpcl_outofcore:

* Fix compile issue in OutofcoreOctreeBase::setLODFilter after switching from boost::shared_ptr to std::shared_ptr [[#4081](https://github.com/PointCloudLibrary/pcl/pull/4081)]

#### libpcl_registration:

* Fix bug in NDT step interval convergence criteria [[#4181](https://github.com/PointCloudLibrary/pcl/pull/4181)]

#### libpcl_segmentation:

* Clean FLANN includes [[#4025](https://github.com/PointCloudLibrary/pcl/pull/4025)]
* Update angle between 2 planes to be the smallest angle between them [[#4161](https://github.com/PointCloudLibrary/pcl/pull/4161)]

#### libpcl_simulation:

* Don't prefix in shaders. [[#4209](https://github.com/PointCloudLibrary/pcl/pull/4209)]

#### libpcl_visualization:

* Fix error: misplaced deprecation attributes in `vtkVertexBufferObject{,Mapper}` [[#4079](https://github.com/PointCloudLibrary/pcl/pull/4079)]

#### PCL Docs:

* Fix typo in FPFH documentation, $$p_k$$ was used instead of $$p_i$$ [[#4112](https://github.com/PointCloudLibrary/pcl/pull/4112)]
* Fix typos in PFH estimation tutorial [[#4111](https://github.com/PointCloudLibrary/pcl/pull/4111)]

#### CI:

* **[new feature]** Add support for CUDA in CI [[#4101](https://github.com/PointCloudLibrary/pcl/pull/4101)]

## = 1.11.0 (11.05.2020) =

Starting with PCL 1.11, PCL uses `std::shared_ptr` and `std::weak_ptr` instead of the
boost smart pointers. The change leverages type aliases included with the 1.10.0
release. PCL 1.11 also introduces `pcl::index_t` which should be used for the size
of point types instead of `int`, `std::size_t`, etc. EOL for deprecated features
is also explicitly mentioned in the deprecation compile time warnings

### Notable changes

**New features** *added to PCL*

* **[common]** Provide dynamic and static pointer casts in namespace pcl to allow easy migration in future [[#3770](https://github.com/PointCloudLibrary/pcl/pull/3770)]
* **[common]** Add `ignore` function to remove Doxygen warnings for unused arguments [[#3942](https://github.com/PointCloudLibrary/pcl/pull/3942)]
* **[docs]** Generate TODO list [[#3937](https://github.com/PointCloudLibrary/pcl/pull/3937)]
* Force include order via clang-format [[#3909](https://github.com/PointCloudLibrary/pcl/pull/3909)]
* Change PCL smart pointers from `boost` to `std` [[#3750](https://github.com/PointCloudLibrary/pcl/pull/3750)]
* **[ci]** Add pipeline for building PCL's environment docker image [[#3843](https://github.com/PointCloudLibrary/pcl/pull/3843)]

**Deprecation** *of public APIs, scheduled to be removed after two minor releases*

* **[common]** Remove `#undef Success` in pcl_macros.h by extracting `PCL_MAKE_ALIGNED_OPERATOR_NEW` into memory.h [[#3654](https://github.com/PointCloudLibrary/pcl/pull/3654)]
* **[common]** Rename `point_traits.h` into `type_traits.h` [[#3698](https://github.com/PointCloudLibrary/pcl/pull/3698)]
* **[filters]** Deprecating functions in non-speclialized Passthrough filter [[#3888](https://github.com/PointCloudLibrary/pcl/pull/3888)]
* **[outofcore][registration]** Homogenize deprecation with PCL_DEPRECATED [[#3925](https://github.com/PointCloudLibrary/pcl/pull/3925)]
* **[cmake][visualization]** Deprecate legacy OpenGL backend of VTK [[#4065](https://github.com/PointCloudLibrary/pcl/pull/4065)]

**Removal** *of the public APIs deprecated in previous releases*

* **[io][recognition][tools]** Remove very old deprecated headers [[#3906](https://github.com/PointCloudLibrary/pcl/pull/3906)]
* **[docs]** Remove backup (and defunct) `CMakeLists.txt` [[#3915](https://github.com/PointCloudLibrary/pcl/pull/3915)]
* Remove use of VTK_EXCLUDE_STRSTREAM_HEADERS (unavailable since VTK 6.0.0) [[#3939](https://github.com/PointCloudLibrary/pcl/pull/3939)]

**Behavior changes** *in classes, apps, or tools*

* **[io]** Make grabbers move-only using `unique_ptr` [[#3626](https://github.com/PointCloudLibrary/pcl/pull/3626)]

**API changes** *that did not go through the proper deprecation and removal cycle*

* **[io]** Make grabbers move-only using `unique_ptr` [[#3626](https://github.com/PointCloudLibrary/pcl/pull/3626)]
* **[common]** Add `pcl::index_t`; move some type declarations from `pcl/pcl_macros.h` to `pcl/types.h` [[#3651](https://github.com/PointCloudLibrary/pcl/pull/3651)]
* **[filters]** Clean up `Filter` and `FilterIndices`, move `indices_`/`input_` from public to protected section [[#3726](https://github.com/PointCloudLibrary/pcl/pull/3726)]
* **[registration]** Better Generalized ICP optimizer gradient check management [[#3854](https://github.com/PointCloudLibrary/pcl/pull/3854)]
* Change PCL smart pointers from `boost` to `std` [[#3750](https://github.com/PointCloudLibrary/pcl/pull/3750)]
* **[registration]** Removing deprecated method `setInputCloud`  from public API [[#4026](https://github.com/PointCloudLibrary/pcl/pull/4026)]

**ABI changes** *that are still API compatible*

* **[filters]** NormalSpaceSampling - fix bucket assignment, remove use of raw distribution pointer, unit-test rewriting [[#3862](https://github.com/PointCloudLibrary/pcl/pull/3862)]
* **[io]** Add pcl::weak_ptr to have a single-switch move from boost:: to std:: weak pointers [[#3753](https://github.com/PointCloudLibrary/pcl/pull/3753)]

### Changes grouped by module

#### CMake:

* Add a stamp file to build documentation once [[#3819](https://github.com/PointCloudLibrary/pcl/pull/3819)]
* Fix compilation in OSX Catalina with OMP enabled [[#3721](https://github.com/PointCloudLibrary/pcl/pull/3721)]
* Show proper message in CMake config for default-off modules [[#3927](https://github.com/PointCloudLibrary/pcl/pull/3927)]
* **[deprecation]** Deprecate legacy OpenGL backend of VTK [[#4065](https://github.com/PointCloudLibrary/pcl/pull/4065)]

#### libpcl_2d:

* variable assigned a value which is never used [[#3857](https://github.com/PointCloudLibrary/pcl/pull/3857)]
* Fix issue with missing templating of `Keypoint`. Fixes coming from clang-doxy [[#3898](https://github.com/PointCloudLibrary/pcl/pull/3898)]

#### libpcl_common:

* **[deprecation]** Remove `#undef Success` in pcl_macros.h by extracting `PCL_MAKE_ALIGNED_OPERATOR_NEW` into memory.h [[#3654](https://github.com/PointCloudLibrary/pcl/pull/3654)]
* Fix issues with math defines on mingw-w64. [[#3756](https://github.com/PointCloudLibrary/pcl/pull/3756)]
* **[API break]** Add `pcl::index_t`; move some type declarations from `pcl/pcl_macros.h` to `pcl/types.h` [[#3651](https://github.com/PointCloudLibrary/pcl/pull/3651)]
* **[deprecation]** Rename `point_traits.h` into `type_traits.h` [[#3698](https://github.com/PointCloudLibrary/pcl/pull/3698)]
* Improve `PCL_DEPRECATED` macro to include scheduled removal version [[#3808](https://github.com/PointCloudLibrary/pcl/pull/3808)]
* Fix erroneous PCL version in deprecated message [[#3824](https://github.com/PointCloudLibrary/pcl/pull/3824)]
* Select OpenMP data sharing mode based on specific GCC versions [[#3823](https://github.com/PointCloudLibrary/pcl/pull/3823)]
* Define `PointIndices` based on the global `Indices` type alias [[#3822](https://github.com/PointCloudLibrary/pcl/pull/3822)]
* Fix warning C4067: unexpected tokens following preprocessor directive- expected a newline [[#3871](https://github.com/PointCloudLibrary/pcl/pull/3871)]
* **[new feature]** Provide dynamic and static pointer casts in namespace pcl to allow easy migration in future [[#3770](https://github.com/PointCloudLibrary/pcl/pull/3770)]
* **[new feature]** Add `ignore` function to remove Doxygen warnings for unused arguments [[#3942](https://github.com/PointCloudLibrary/pcl/pull/3942)]
* Fix excessive warnings on MSVC [[#3964](https://github.com/PointCloudLibrary/pcl/pull/3964)]
* Refactoring `PCL_DEPRECATED` macro [[#3945](https://github.com/PointCloudLibrary/pcl/pull/3945)]
* Correcting type mismatch [[#3967](https://github.com/PointCloudLibrary/pcl/pull/3967)]
* Removed empty file [[#4019](https://github.com/PointCloudLibrary/pcl/pull/4019)]

#### libpcl_filters:

* Clean up code duplication in `FilterIndices` derived classes [[#3807](https://github.com/PointCloudLibrary/pcl/pull/3807)]
* **[API break]** Clean up `Filter` and `FilterIndices`, move `indices_`/`input_` from public to protected section [[#3726](https://github.com/PointCloudLibrary/pcl/pull/3726)]
* **[deprecation]** Deprecating functions in non-speclialized Passthrough filter [[#3888](https://github.com/PointCloudLibrary/pcl/pull/3888)]
* **[ABI break]** NormalSpaceSampling - fix bucket assignment, remove use of raw distribution pointer, unit-test rewriting [[#3862](https://github.com/PointCloudLibrary/pcl/pull/3862)]
* Optimize VoxelGrid Filter [[#3853](https://github.com/PointCloudLibrary/pcl/pull/3853)]
* Fix error due to multiple declarations of template member function specializations in convolution [[#3971](https://github.com/PointCloudLibrary/pcl/pull/3971)]

#### libpcl_io:

* **[ABI break][API break][behavior change]** Make grabbers move-only using `unique_ptr` [[#3626](https://github.com/PointCloudLibrary/pcl/pull/3626)]
* **[removal]** Remove very old deprecated headers [[#3906](https://github.com/PointCloudLibrary/pcl/pull/3906)]
* **[ABI break]** Add pcl::weak_ptr to have a single-switch move from boost:: to std:: weak pointers [[#3753](https://github.com/PointCloudLibrary/pcl/pull/3753)]
* Use pcl::io::raw_read instead of direct call to POSIX function read [[#4062](https://github.com/PointCloudLibrary/pcl/pull/4062)]

#### libpcl_octree:

* Fix a memory leak in `OctreeBase::operator=` [[#3787](https://github.com/PointCloudLibrary/pcl/pull/3787)]

#### libpcl_outofcore:

* **[deprecation]** Homogenize deprecation with PCL_DEPRECATED [[#3925](https://github.com/PointCloudLibrary/pcl/pull/3925)]

#### libpcl_people:

* Missing include on windows [[#3791](https://github.com/PointCloudLibrary/pcl/pull/3791)]

#### libpcl_recognition:

* **[removal]** Remove very old deprecated headers [[#3906](https://github.com/PointCloudLibrary/pcl/pull/3906)]

#### libpcl_registration:

* **[API break]** Better Generalized ICP optimizer gradient check management [[#3854](https://github.com/PointCloudLibrary/pcl/pull/3854)]
* **[deprecation]** Homogenize deprecation with PCL_DEPRECATED [[#3925](https://github.com/PointCloudLibrary/pcl/pull/3925)]
* **[API break]** Removing deprecated method `setInputCloud`  from public API [[#4026](https://github.com/PointCloudLibrary/pcl/pull/4026)]

#### libpcl_sample_consensus:

* Better performance, error handling and other improvements in SAC classes [[#3642](https://github.com/PointCloudLibrary/pcl/pull/3642)]
* Use better types for indices: `int` -> `index_t`, `std::vector<int>` ->`Indices` [[#3835](https://github.com/PointCloudLibrary/pcl/pull/3835)]

#### libpcl_search:

* Use better types for indices: `int` -> `index_t`, `std::vector<int>` ->`Indices` [[#3840](https://github.com/PointCloudLibrary/pcl/pull/3840)]
* Add include for `pcl::isFinite` for compilation on VS2019 [[#4056](https://github.com/PointCloudLibrary/pcl/pull/4056)]

#### libpcl_segmentation:

* Check and warn user about missing normals in `OrganizedMultiPlaneSegmentation` [[#3861](https://github.com/PointCloudLibrary/pcl/pull/3861)]

#### libpcl_surface:

* Fix error due to multiple declarations of template member function specializations in Poisson4 [[#3972](https://github.com/PointCloudLibrary/pcl/pull/3972)]
* Reduce time taken in `TextureMapping::sortFacesByCamera` from `O(faces*points)` to `O(faces)` [[#3981](https://github.com/PointCloudLibrary/pcl/pull/3981)]

#### libpcl_visualization:

* Minor fix for converting unorganized PointClouds to VTK data [[#3988](https://github.com/PointCloudLibrary/pcl/pull/3988)]
* Fixes #4001 and #3452. [[#4017](https://github.com/PointCloudLibrary/pcl/pull/4017)]
* **[deprecation]** Deprecate legacy OpenGL backend of VTK [[#4065](https://github.com/PointCloudLibrary/pcl/pull/4065)]
* Fix rendering of points. [[#4067](https://github.com/PointCloudLibrary/pcl/pull/4067)]

#### PCL Docs:

* Fix Doxygen warnings unrelated to documentation -- Part 1 [[#3701](https://github.com/PointCloudLibrary/pcl/pull/3701)]
* update automatic code formatting info to clang-format [[#3845](https://github.com/PointCloudLibrary/pcl/pull/3845)]
* replace formula with math [[#3846](https://github.com/PointCloudLibrary/pcl/pull/3846)]
* Fix PCL_DEPRECATED usage in doxygen for a proper Deprecation List in documentation [[#3905](https://github.com/PointCloudLibrary/pcl/pull/3905)]
* **[removal]** Remove backup (and defunct) `CMakeLists.txt` [[#3915](https://github.com/PointCloudLibrary/pcl/pull/3915)]
* **[new feature]** Generate TODO list [[#3937](https://github.com/PointCloudLibrary/pcl/pull/3937)]
* Improve output to match the text in tutorial by adding a newline [[#4029](https://github.com/PointCloudLibrary/pcl/pull/4029)]
* Fix typo in region growing tutorial [[#4052](https://github.com/PointCloudLibrary/pcl/pull/4052)]
* Add information regarding header include order [[#4020](https://github.com/PointCloudLibrary/pcl/pull/4020)]

#### PCL Tutorials:

* Add a unit test for ICP and modernize tutorial code [[#3628](https://github.com/PointCloudLibrary/pcl/pull/3628)]

#### PCL Examples:

* Remove unnecessary includes in examples [[#4071](https://github.com/PointCloudLibrary/pcl/pull/4071)]

#### PCL Tools:

* **[removal]** Remove very old deprecated headers [[#3906](https://github.com/PointCloudLibrary/pcl/pull/3906)]

#### CI:

* Temporary fix for skipping of certain tests [[#3789](https://github.com/PointCloudLibrary/pcl/pull/3789)]
* Simplify Ubuntu CI using matrix strategy [[#3783](https://github.com/PointCloudLibrary/pcl/pull/3783)]
* Strip leading tag in commit message from changelog [[#3847](https://github.com/PointCloudLibrary/pcl/pull/3847)]
* Shift to clang-format-10 to resolve bug in clang-format-{7-9} [[#3895](https://github.com/PointCloudLibrary/pcl/pull/3895)]
* **[new feature]** Add pipeline for building PCL's environment docker image [[#3843](https://github.com/PointCloudLibrary/pcl/pull/3843)]
* Improve docker ci for other PRs [[#4051](https://github.com/PointCloudLibrary/pcl/pull/4051)]
* Remove unused variables from exceptions [[#4064](https://github.com/PointCloudLibrary/pcl/pull/4064)]
* Removing hardcoded version number from tutorial CI [[#4058](https://github.com/PointCloudLibrary/pcl/pull/4058)]

## *= 1.10.1 (18.03.2020) =*

### Notable changes

**Deprecation** *of public APIs, scheduled to be removed after two minor releases*

* **[common]** Deprecate several `PointWithViewpoint` ctors; make ctors more uniform in PCL point types [[#3597](https://github.com/PointCloudLibrary/pcl/pull/3597)]

**Removal** *of the public APIs deprecated in previous releases*

* **[common]** Remove deprecated checks for `USE_ROS` macro [[#3690](https://github.com/PointCloudLibrary/pcl/pull/3690)]

**Behavior changes** *in classes, apps, or tools*

* **[tools]** Continue on PCD load failure in `pcl_train_linemod_template` [[#3652](https://github.com/PointCloudLibrary/pcl/pull/3652)]

### Changes grouped by module

#### CMake:

* Fix CMake grouping of tools targets [[#3709](https://github.com/PointCloudLibrary/pcl/pull/3709)]
* Enable `PCL_ONLY_CORE_POINT_TYPES` in mingw builds [[#3694](https://github.com/PointCloudLibrary/pcl/pull/3694)]
* Downgrade RSSDK2 warning message to status [[#3683](https://github.com/PointCloudLibrary/pcl/pull/3683)]
* Fix test targets arguments for MSVC [[#3636](https://github.com/PointCloudLibrary/pcl/pull/3636)]
* Remove duplicate `/bigobj` for MSVC [[#3604](https://github.com/PointCloudLibrary/pcl/pull/3604)]

#### libpcl_common:

* Better PointType ctor and reduced warnings in `register_point_struct.h` [[#3732](https://github.com/PointCloudLibrary/pcl/pull/3732)]
* **[removal]** Remove deprecated checks for `USE_ROS` macro [[#3690](https://github.com/PointCloudLibrary/pcl/pull/3690)]
* Replace `dirent` with `boost::filesystem` [[#3676](https://github.com/PointCloudLibrary/pcl/pull/3676)]
* Fix code accidentally casting away const-ness [[#3648](https://github.com/PointCloudLibrary/pcl/pull/3648)]
* Fix compilation of CUDA code on Windows [[#3634](https://github.com/PointCloudLibrary/pcl/pull/3634)]
* Remove undefined behavior and add stricter checks in console arg parsing [[#3613](https://github.com/PointCloudLibrary/pcl/pull/3613)]
* **[deprecation]** Deprecate several `PointWithViewpoint` ctors; make ctors more uniform in PCL point types [[#3597](https://github.com/PointCloudLibrary/pcl/pull/3597)]

#### libpcl_cuda:

* Fix memory leaks in CUDA apps [[#3587](https://github.com/PointCloudLibrary/pcl/pull/3587)]
* Fix compilation of CUDA/GPU modules [[#3576](https://github.com/PointCloudLibrary/pcl/pull/3576)]

#### libpcl_features:

* Add precompiled `computeApproximateCovariances`; fix compilation error for the same [[#3711](https://github.com/PointCloudLibrary/pcl/pull/3711)]
* Fix vector initialization in `NormalEstimationOMP` [[#3614](https://github.com/PointCloudLibrary/pcl/pull/3614)]
* Fix indexing bug in `IntegralImageNormalEstimation` [[#3574](https://github.com/PointCloudLibrary/pcl/pull/3574)]

#### libpcl_filters:

* Set `is_dense` based on actual cloud contents in `removeNaNNormalsFromPointCloud()` [[#3685](https://github.com/PointCloudLibrary/pcl/pull/3685)]

#### libpcl_gpu:

* Fix compile error in KinFuLS `initRegistration` [[#3737](https://github.com/PointCloudLibrary/pcl/pull/3737)]
* Fix illegal memory acces in CUDA Octree builder [[#3627](https://github.com/PointCloudLibrary/pcl/pull/3627)]
* Fix compile issue in `people_app` [[#3618](https://github.com/PointCloudLibrary/pcl/pull/3618)]
* Fix compilation of CUDA/GPU modules [[#3576](https://github.com/PointCloudLibrary/pcl/pull/3576)]

#### libpcl_io:

* Fix `if/ifdef` WIN32 issues [[#3668](https://github.com/PointCloudLibrary/pcl/pull/3668)]
* Add `Grabber::toggle()` method [[#3615](https://github.com/PointCloudLibrary/pcl/pull/3615)]
* Close the correct file in `pcl::io::savePLYFileBinary` [[#3601](https://github.com/PointCloudLibrary/pcl/pull/3601)]
* Fix entropy range encoding in octree-based pointcloud compression [[#3579](https://github.com/PointCloudLibrary/pcl/pull/3579)]

#### libpcl_surface:

* Add default initialization of grid resolution in `MarchingCubes` [[#3718](https://github.com/PointCloudLibrary/pcl/pull/3718)]

#### PCL Apps:

* Fix `if/ifdef` WIN32 issues [[#3668](https://github.com/PointCloudLibrary/pcl/pull/3668)]

#### PCL Docs:

* Fix missing standard includes, reduce warnings in doxygen-enabled builds [[#3755](https://github.com/PointCloudLibrary/pcl/pull/3755)]

#### PCL Tutorials:

* Fix documentation for point cloud stream compression executable name [[#3693](https://github.com/PointCloudLibrary/pcl/pull/3693)]
* Fix segfault in NARF keypoint extraction tutorial [[#3623](https://github.com/PointCloudLibrary/pcl/pull/3623)]

#### PCL Tools:

* Use linemod member method to save templates [[#3691](https://github.com/PointCloudLibrary/pcl/pull/3691)]
* **[behavior change]** Continue on PCD load failure in `pcl_train_linemod_template` [[#3652](https://github.com/PointCloudLibrary/pcl/pull/3652)]
* Warn user about unorganized PCDs in `pcl_train_linemod_template` [[#3644](https://github.com/PointCloudLibrary/pcl/pull/3644)]


## *= 1.10.0 (19.01.2020) =*

Starting with PCL 1.10, to ensure compatibility with future PCL releases, please
use `pcl::make_shared` and the `Class::Ptr` + `Class::ConstPtr` type-alias
instead of using direct names like `{boost, std}::shared_ptr` or `{boost,
std}::make_shared`.  There is also `pcl::shared_ptr` which offers the same
abstraction for non-PCL types.

### `New Features:`

*Newly added functionalities.*

* **[sample_consensus]** Add parallel RANSAC implementation with OpenMP [[#3514](https://github.com/PointCloudLibrary/pcl/pull/3514)]
* **[registration]** Add linear least squares version of symmetric objective function for ICP [[#3390](https://github.com/PointCloudLibrary/pcl/pull/3390)]
* **[common]** Add concatenate operation for `PolygonMesh` [[#3316](https://github.com/PointCloudLibrary/pcl/pull/3316)]
* **[common]** Add `emplace[_back]` to `pcl::PointCloud` [[#3207](https://github.com/PointCloudLibrary/pcl/pull/3207)]
* **[cmake]** Add `format` compilation target that applies `clang-format` to whitelisted modules [[#3188](https://github.com/PointCloudLibrary/pcl/pull/3188)]
* **[common]** Add `pcl::make_shared` that automatically handles aligned allocations [[#3163](https://github.com/PointCloudLibrary/pcl/pull/3163)]
* **[modernization][cmake]** Enable C++ 14 flags [[#2690](https://github.com/PointCloudLibrary/pcl/pull/2690)]
* **[io]** Add RSSDK 2.0 (librealsense2) grabber [[#2214](https://github.com/PointCloudLibrary/pcl/pull/2214)]

### `Deprecated:`

*Deprecated code scheduled to be removed after two minor releases.*

* **[common]** Revert smart pointer type change in `PointCloud` and deprecate `getMapping()` [[#3563](https://github.com/PointCloudLibrary/pcl/pull/3563)]
* **[common]** Deprecate `getFields()` with output parameter in favor of overload with return value [[#3401](https://github.com/PointCloudLibrary/pcl/pull/3401)]
* **[recognition]** Refactor `MaskMap` and deprecate several of its methods [[#3399](https://github.com/PointCloudLibrary/pcl/pull/3399)]
* **[common]** Deprecate `getFieldIndex()`/`getFields()` with first argument as cloud [[#3365](https://github.com/PointCloudLibrary/pcl/pull/3365)]
* **[common]** Add `PCLPointCloud2::operator+=()` and update concatenation operation [[#3320](https://github.com/PointCloudLibrary/pcl/pull/3320)]
* **[segmentation]** Delete unused params in `OrganizedMultiPlaneSegmentation::refine()` [[#3302](https://github.com/PointCloudLibrary/pcl/pull/3302)]
* **[visualization]** Add new overload of `PointCloudColorHandler::getColor()` [[#3187](https://github.com/PointCloudLibrary/pcl/pull/3187)]
* **[modernization][io]** Add `registerCallback()` overload to grabbers to support assignment of `boost::function`s with templated signatures [[#3128](https://github.com/PointCloudLibrary/pcl/pull/3128)]
* **[surface]** Convert `MovingLeastSquaresOMP` into an alias template and deprecate [[#3119](https://github.com/PointCloudLibrary/pcl/pull/3119)]
* **[kdtree]** Remove unnecessary FLANN includes, deprecate "kdtree/flann.h" header [[#2993](https://github.com/PointCloudLibrary/pcl/pull/2993)]
* **[features]** Deprecate `computeRSD()` functions that take pointclouds by pointer [[#2827](https://github.com/PointCloudLibrary/pcl/pull/2827)]
* **[modernization]** Deprecate `pcl_isnan`, `pcl_isfinite`, and `pcl_isinf` in favor of `std` methods [[#2798](https://github.com/PointCloudLibrary/pcl/pull/2798), [#3457](https://github.com/PointCloudLibrary/pcl/pull/3457)]
* **[filters]** Restructure and add functionality to filters templated on `PCLPointCloud2` [[#3483](https://github.com/PointCloudLibrary/pcl/pull/3483), [#3500](https://github.com/PointCloudLibrary/pcl/pull/3500)]

### `Removed:`

*Removal of deprecated code.*

* **[segmentation]** Remove `SupervoxelClustering::getColoredVoxelCloud()` [[#3469](https://github.com/PointCloudLibrary/pcl/pull/3469)]
* **[io]** Remove FZ-API [[#2747](https://github.com/PointCloudLibrary/pcl/pull/2747)]

### `Behavioral changes:`

*Changes in the expected default behavior.*

* **[sample_consensus]** Set default min and max angle for SAC cone models [[#3466](https://github.com/PointCloudLibrary/pcl/pull/3466)]
* **[tools]** Do not discard data fields in `pcl_uniform_sampling` tool [[#3461](https://github.com/PointCloudLibrary/pcl/pull/3461)]
* **[common]** Disable colored output for non-interactive terminals [[#3310](https://github.com/PointCloudLibrary/pcl/pull/3310)]
* **[io]** Add support for device URI in `OpenNI2Grabber` constructor [[#3238](https://github.com/PointCloudLibrary/pcl/pull/3238)]
* **[common]** Enforce right-hand-rule on PCA eigenvectors [[#2946](https://github.com/PointCloudLibrary/pcl/pull/2946)]
* **[visualization]** Improve c/C command in `PCLVisualizer` [[#2926](https://github.com/PointCloudLibrary/pcl/pull/2926)]
* **[registration]** Fix ICP misbehavior in the "failure after maximum iterations" mode [[#2892](https://github.com/PointCloudLibrary/pcl/pull/2892)]
* **[common]** Initialize curvature in `PointNormal` default constructor [[#2674](https://github.com/PointCloudLibrary/pcl/pull/2674)]

### `API changes:`

*Changes to the API which didn't go through the proper deprecation and removal cycle.*

* **[gpu]** Replace `uint64_type` by `std::uint64_t` [[#3435](https://github.com/PointCloudLibrary/pcl/pull/3435)]
* **[modernization]** Migrate from `boost::tuple` to `std::tuple` [[#3250](https://github.com/PointCloudLibrary/pcl/pull/3250)]
* **[modernization]** Migrate `boost::function` to `std::function` [[#3079](https://github.com/PointCloudLibrary/pcl/pull/3079)]

### `ABI changes:`

*Changes that cause ABI incompatibility but are still API compatible.*

* **[modernization]** Migrate from `boost::unordered_map` to `std::unordered_map` [[#3107](https://github.com/PointCloudLibrary/pcl/pull/3107)]
* **[modernization]** Migrate to standard library type traits [[#3105](https://github.com/PointCloudLibrary/pcl/pull/3105)]
* **[visualization]** Minor refactoring of `pcl::visualization::Camera` and related functions [[#2901](https://github.com/PointCloudLibrary/pcl/pull/2901)]
* **[modernization]** Migrate from `boost::thread` to `std::thread` [[#3060](https://github.com/PointCloudLibrary/pcl/pull/3060), [#3094](https://github.com/PointCloudLibrary/pcl/pull/3094)]
* **[modernization]** Prefer using `Ptr` typedefs and migrate to `std` smart pointers in non-API code [[#2804](https://github.com/PointCloudLibrary/pcl/pull/2804), [#2821](https://github.com/PointCloudLibrary/pcl/pull/2821), [#2823](https://github.com/PointCloudLibrary/pcl/pull/2823), [#2929](https://github.com/PointCloudLibrary/pcl/pull/2929), [#3061](https://github.com/PointCloudLibrary/pcl/pull/3061), [#3141](https://github.com/PointCloudLibrary/pcl/pull/3141), [#3142](https://github.com/PointCloudLibrary/pcl/pull/3142), [#3217](https://github.com/PointCloudLibrary/pcl/pull/3217), [#3474](https://github.com/PointCloudLibrary/pcl/pull/3474), [#3482](https://github.com/PointCloudLibrary/pcl/pull/3482), [#3486](https://github.com/PointCloudLibrary/pcl/pull/3486), [#3489](https://github.com/PointCloudLibrary/pcl/pull/3489), [#3497](https://github.com/PointCloudLibrary/pcl/pull/3497)]
* **[modernization]** Migrate to `std` random number generators [[#2956](https://github.com/PointCloudLibrary/pcl/pull/2956), [#2962](https://github.com/PointCloudLibrary/pcl/pull/2962), [#3069](https://github.com/PointCloudLibrary/pcl/pull/3069)]
* **[filters]** Restructure and add functionality to filters templated on `PCLPointCloud2` [[#3483](https://github.com/PointCloudLibrary/pcl/pull/3483), [#3500](https://github.com/PointCloudLibrary/pcl/pull/3500)]

### `Migration to C++14 and code modernization:`

* Convert `boost::shared_ptr` to `pcl::shared_ptr` [[#3546](https://github.com/PointCloudLibrary/pcl/pull/3546)]
* Better indices in loops [[#3543](https://github.com/PointCloudLibrary/pcl/pull/3543)]
* Replace raw arrays with `std::vector`s in `RangeImageBorderExtractor` [[#3533](https://github.com/PointCloudLibrary/pcl/pull/3533)]
* Remove redundant calls to `std::string::c_str()` [[#3517](https://github.com/PointCloudLibrary/pcl/pull/3517)]
* Convert `pcl::int_t` to `std::int_t` [[#3422](https://github.com/PointCloudLibrary/pcl/pull/3422)]
* Remove deprecated `throw` specifier [[#3384](https://github.com/PointCloudLibrary/pcl/pull/3384)]
* Prefer `numeric_limits` from standard library [[#3360](https://github.com/PointCloudLibrary/pcl/pull/3360)]
* Add missing `std::move` or `const` reference for parameters [[#3253](https://github.com/PointCloudLibrary/pcl/pull/3253)]
* **[api]** Migrate from `boost::tuple` to `std::tuple` [[#3250](https://github.com/PointCloudLibrary/pcl/pull/3250)]
* Migrate from `boost::bind` to `std::bind` [[#3249](https://github.com/PointCloudLibrary/pcl/pull/3249)]
* Use transparent functors [[#3224](https://github.com/PointCloudLibrary/pcl/pull/3224)]
* Add `cloexec` option to `fopen` [[#3223](https://github.com/PointCloudLibrary/pcl/pull/3223)]
* **[deprecation]** Add `registerCallback()` overload to grabbers to support assignment of `boost::function`s with templated signatures [[#3128](https://github.com/PointCloudLibrary/pcl/pull/3128)]
* **[abi]** Migrate from `boost::unordered_map` to `std::unordered_map` [[#3107](https://github.com/PointCloudLibrary/pcl/pull/3107)]
* **[abi]** Migrate to standard library type traits [[#3105](https://github.com/PointCloudLibrary/pcl/pull/3105)]
* Remove dead stores [[#3095](https://github.com/PointCloudLibrary/pcl/pull/3095)]
* **[api]** Migrate `boost::function` to `std::function` [[#3079](https://github.com/PointCloudLibrary/pcl/pull/3079)]
* Remove redundant member field initialization [[#3070](https://github.com/PointCloudLibrary/pcl/pull/3070)]
* Prefer returning braced init list [[#3039](https://github.com/PointCloudLibrary/pcl/pull/3039)]
* Prefer `empty()` over `size()` when checking container state [[#3033](https://github.com/PointCloudLibrary/pcl/pull/3033)]
* Prefer combined assignment operators [[#3030](https://github.com/PointCloudLibrary/pcl/pull/3030)]
* Remove unnecessary `nullptr` checks before `delete` [[#2990](https://github.com/PointCloudLibrary/pcl/pull/2990)]
* Improve readability of string comparisons [[#2986](https://github.com/PointCloudLibrary/pcl/pull/2986)]
* Prefer `std::isnan` over `!=` comparison trick [[#2977](https://github.com/PointCloudLibrary/pcl/pull/2977)]
* Remove deprecated Boost Filesystem code [[#2966](https://github.com/PointCloudLibrary/pcl/pull/2966)]
* Migrate from `BOOST_STATIC_ASSERT` and `EIGEN_ASSERT` to `static_assert` [[#2951](https://github.com/PointCloudLibrary/pcl/pull/2951)]
* Prefer `std::log2` over custom implementation [[#2950](https://github.com/PointCloudLibrary/pcl/pull/2950)]
* Migrate from `boost::this_thread::sleep` to `std::this_thread::sleep_for` [[#2921](https://github.com/PointCloudLibrary/pcl/pull/2921)]
* Fix bug prone loop variables that are too small [[#2829](https://github.com/PointCloudLibrary/pcl/pull/2829)]
* Migrate from `boost::math::isnan` to `std::isnan` [[#2819](https://github.com/PointCloudLibrary/pcl/pull/2819)]
* Fix "variableScope" hints from CppCheck [[#2807](https://github.com/PointCloudLibrary/pcl/pull/2807)]
* Improve performance of finding single character in strings [[#2794](https://github.com/PointCloudLibrary/pcl/pull/2794)]
* Prefer using `bool` literals [[#2793](https://github.com/PointCloudLibrary/pcl/pull/2793)]
* Simplify boolean expressions [[#2790](https://github.com/PointCloudLibrary/pcl/pull/2790)]
* Prefer raw strings over escaped strings [[#2789](https://github.com/PointCloudLibrary/pcl/pull/2789)]
* Prefer `std::to_string` over `boost::lexical_cast` for integer conversions [[#2785](https://github.com/PointCloudLibrary/pcl/pull/2785)]
* Prefer `emplace_back` over `push_back` [[#2784](https://github.com/PointCloudLibrary/pcl/pull/2784)]
* Remove unnecessary void parameter [[#2780](https://github.com/PointCloudLibrary/pcl/pull/2780)]
* Use `=delete` to disable special members [[#2779](https://github.com/PointCloudLibrary/pcl/pull/2779)]
* Fix access to static members through instances [[#2776](https://github.com/PointCloudLibrary/pcl/pull/2776)]
* Remove usage of deprecated `register`storage class specifier [[#2761](https://github.com/PointCloudLibrary/pcl/pull/2761)]
* Remove redundant string initialization with `""` [[#2758](https://github.com/PointCloudLibrary/pcl/pull/2758)]
* Add `cloexec` option to `fopen` [[#2755](https://github.com/PointCloudLibrary/pcl/pull/2755)]
* Replace deprecated C library headers [[#2737](https://github.com/PointCloudLibrary/pcl/pull/2737)]
* Modernize code to use `override` [[#2728](https://github.com/PointCloudLibrary/pcl/pull/2728)]
* Prefer standard `[[deprecated]]` attribute [[#2699](https://github.com/PointCloudLibrary/pcl/pull/2699)]
* **[new-feature]** Enable C++ 14 flags [[#2690](https://github.com/PointCloudLibrary/pcl/pull/2690)]
* Revise arguments which were being passed by  value instead of as a reference [[#2668](https://github.com/PointCloudLibrary/pcl/pull/2668)]
* Fix "knownConditionTrueFalse" hints from CppCheck [[#2648](https://github.com/PointCloudLibrary/pcl/pull/2648)]
* Fix "unreadVariable" hints from CppCheck [[#2645](https://github.com/PointCloudLibrary/pcl/pull/2645)]
* Replace `hash_map` with `unordered_map` in 3rdparty/poisson4 [[#2640](https://github.com/PointCloudLibrary/pcl/pull/2640)]
* Prefer lambdas over binds [[#3136](https://github.com/PointCloudLibrary/pcl/pull/3136), [#3171](https://github.com/PointCloudLibrary/pcl/pull/3171), [#3173](https://github.com/PointCloudLibrary/pcl/pull/3173), [#3178](https://github.com/PointCloudLibrary/pcl/pull/3178), [#3189](https://github.com/PointCloudLibrary/pcl/pull/3189), [#3192](https://github.com/PointCloudLibrary/pcl/pull/3192), [#3199](https://github.com/PointCloudLibrary/pcl/pull/3199), [#3209](https://github.com/PointCloudLibrary/pcl/pull/3209), [#3231](https://github.com/PointCloudLibrary/pcl/pull/3231), [#3243](https://github.com/PointCloudLibrary/pcl/pull/3243), [#3254](https://github.com/PointCloudLibrary/pcl/pull/3254)]
* Prefer range-based for loops [[#2812](https://github.com/PointCloudLibrary/pcl/pull/2812), [#2834](https://github.com/PointCloudLibrary/pcl/pull/2834), [#2835](https://github.com/PointCloudLibrary/pcl/pull/2835), [#2836](https://github.com/PointCloudLibrary/pcl/pull/2836), [#2837](https://github.com/PointCloudLibrary/pcl/pull/2837), [#2838](https://github.com/PointCloudLibrary/pcl/pull/2838), [#2839](https://github.com/PointCloudLibrary/pcl/pull/2839), [#2840](https://github.com/PointCloudLibrary/pcl/pull/2840), [#2841](https://github.com/PointCloudLibrary/pcl/pull/2841), [#2842](https://github.com/PointCloudLibrary/pcl/pull/2842), [#2843](https://github.com/PointCloudLibrary/pcl/pull/2843), [#2844](https://github.com/PointCloudLibrary/pcl/pull/2844), [#2845](https://github.com/PointCloudLibrary/pcl/pull/2845), [#2846](https://github.com/PointCloudLibrary/pcl/pull/2846), [#2847](https://github.com/PointCloudLibrary/pcl/pull/2847), [#2848](https://github.com/PointCloudLibrary/pcl/pull/2848), [#2849](https://github.com/PointCloudLibrary/pcl/pull/2849), [#2850](https://github.com/PointCloudLibrary/pcl/pull/2850), [#2851](https://github.com/PointCloudLibrary/pcl/pull/2851), [#2853](https://github.com/PointCloudLibrary/pcl/pull/2853), [#2854](https://github.com/PointCloudLibrary/pcl/pull/2854), [#2855](https://github.com/PointCloudLibrary/pcl/pull/2855), [#2856](https://github.com/PointCloudLibrary/pcl/pull/2856), [#2857](https://github.com/PointCloudLibrary/pcl/pull/2857), [#2858](https://github.com/PointCloudLibrary/pcl/pull/2858), [#2859](https://github.com/PointCloudLibrary/pcl/pull/2859), [#2860](https://github.com/PointCloudLibrary/pcl/pull/2860), [#2887](https://github.com/PointCloudLibrary/pcl/pull/2887), [#3396](https://github.com/PointCloudLibrary/pcl/pull/3396), [#3557](https://github.com/PointCloudLibrary/pcl/pull/3557)]
* Prefer `nullptr` over 0 and `NULL` [[#3004](https://github.com/PointCloudLibrary/pcl/pull/3004), [#3005](https://github.com/PointCloudLibrary/pcl/pull/3005), [#3006](https://github.com/PointCloudLibrary/pcl/pull/3006), [#3007](https://github.com/PointCloudLibrary/pcl/pull/3007), [#3008](https://github.com/PointCloudLibrary/pcl/pull/3008), [#3009](https://github.com/PointCloudLibrary/pcl/pull/3009), [#3010](https://github.com/PointCloudLibrary/pcl/pull/3010), [#3011](https://github.com/PointCloudLibrary/pcl/pull/3011), [#3012](https://github.com/PointCloudLibrary/pcl/pull/3012), [#3013](https://github.com/PointCloudLibrary/pcl/pull/3013), [#3014](https://github.com/PointCloudLibrary/pcl/pull/3014), [#3015](https://github.com/PointCloudLibrary/pcl/pull/3015), [#3016](https://github.com/PointCloudLibrary/pcl/pull/3016), [#3017](https://github.com/PointCloudLibrary/pcl/pull/3017), [#3018](https://github.com/PointCloudLibrary/pcl/pull/3018), [#3019](https://github.com/PointCloudLibrary/pcl/pull/3019), [#3020](https://github.com/PointCloudLibrary/pcl/pull/3020), [#3021](https://github.com/PointCloudLibrary/pcl/pull/3021), [#3022](https://github.com/PointCloudLibrary/pcl/pull/3022), [#3023](https://github.com/PointCloudLibrary/pcl/pull/3023), [#3024](https://github.com/PointCloudLibrary/pcl/pull/3024), [#3025](https://github.com/PointCloudLibrary/pcl/pull/3025), [#3026](https://github.com/PointCloudLibrary/pcl/pull/3026), [#3027](https://github.com/PointCloudLibrary/pcl/pull/3027), [#3028](https://github.com/PointCloudLibrary/pcl/pull/3028), [#3029](https://github.com/PointCloudLibrary/pcl/pull/3029)]
* Migrate to `std::chrono` [[#2913](https://github.com/PointCloudLibrary/pcl/pull/2913), [#2919](https://github.com/PointCloudLibrary/pcl/pull/2919), [#3318](https://github.com/PointCloudLibrary/pcl/pull/3318)]
* **[abi]** Migrate from `boost::thread` to `std::thread` [[#3060](https://github.com/PointCloudLibrary/pcl/pull/3060), [#3094](https://github.com/PointCloudLibrary/pcl/pull/3094)]
* Migrate `mutex`, `lock` and `csv` to modernization [[#3063](https://github.com/PointCloudLibrary/pcl/pull/3063), [#3068](https://github.com/PointCloudLibrary/pcl/pull/3068), [#3074](https://github.com/PointCloudLibrary/pcl/pull/3074), [#3078](https://github.com/PointCloudLibrary/pcl/pull/3078), [#3084](https://github.com/PointCloudLibrary/pcl/pull/3084), [#3086](https://github.com/PointCloudLibrary/pcl/pull/3086), [#3088](https://github.com/PointCloudLibrary/pcl/pull/3088), [#3091](https://github.com/PointCloudLibrary/pcl/pull/3091), [#3093](https://github.com/PointCloudLibrary/pcl/pull/3093), [#3100](https://github.com/PointCloudLibrary/pcl/pull/3100)]
* Prefer `using` over `typedef` [[#3112](https://github.com/PointCloudLibrary/pcl/pull/3112), [#3113](https://github.com/PointCloudLibrary/pcl/pull/3113), [#3115](https://github.com/PointCloudLibrary/pcl/pull/3115), [#3117](https://github.com/PointCloudLibrary/pcl/pull/3117), [#3118](https://github.com/PointCloudLibrary/pcl/pull/3118), [#3121](https://github.com/PointCloudLibrary/pcl/pull/3121), [#3122](https://github.com/PointCloudLibrary/pcl/pull/3122), [#3123](https://github.com/PointCloudLibrary/pcl/pull/3123), [#3124](https://github.com/PointCloudLibrary/pcl/pull/3124), [#3125](https://github.com/PointCloudLibrary/pcl/pull/3125), [#3129](https://github.com/PointCloudLibrary/pcl/pull/3129), [#3130](https://github.com/PointCloudLibrary/pcl/pull/3130), [#3132](https://github.com/PointCloudLibrary/pcl/pull/3132), [#3134](https://github.com/PointCloudLibrary/pcl/pull/3134), [#3137](https://github.com/PointCloudLibrary/pcl/pull/3137), [#3138](https://github.com/PointCloudLibrary/pcl/pull/3138), [#3139](https://github.com/PointCloudLibrary/pcl/pull/3139), [#3144](https://github.com/PointCloudLibrary/pcl/pull/3144)]
* Prefer `std` math functions over C functions [[#3087](https://github.com/PointCloudLibrary/pcl/pull/3087), [#3236](https://github.com/PointCloudLibrary/pcl/pull/3236), [#3255](https://github.com/PointCloudLibrary/pcl/pull/3255), [#3256](https://github.com/PointCloudLibrary/pcl/pull/3256), [#3257](https://github.com/PointCloudLibrary/pcl/pull/3257), [#3258](https://github.com/PointCloudLibrary/pcl/pull/3258), [#3270](https://github.com/PointCloudLibrary/pcl/pull/3270), [#3271](https://github.com/PointCloudLibrary/pcl/pull/3271), [#3272](https://github.com/PointCloudLibrary/pcl/pull/3272), [#3280](https://github.com/PointCloudLibrary/pcl/pull/3280), [#3282](https://github.com/PointCloudLibrary/pcl/pull/3282), [#3287](https://github.com/PointCloudLibrary/pcl/pull/3287)]
* **[abi]** Prefer using `Ptr` typedefs and migrate to `std` smart pointers in non-API code [[#2804](https://github.com/PointCloudLibrary/pcl/pull/2804), [#2821](https://github.com/PointCloudLibrary/pcl/pull/2821), [#2823](https://github.com/PointCloudLibrary/pcl/pull/2823), [#2929](https://github.com/PointCloudLibrary/pcl/pull/2929), [#3061](https://github.com/PointCloudLibrary/pcl/pull/3061), [#3141](https://github.com/PointCloudLibrary/pcl/pull/3141), [#3142](https://github.com/PointCloudLibrary/pcl/pull/3142), [#3217](https://github.com/PointCloudLibrary/pcl/pull/3217), [#3474](https://github.com/PointCloudLibrary/pcl/pull/3474), [#3482](https://github.com/PointCloudLibrary/pcl/pull/3482), [#3486](https://github.com/PointCloudLibrary/pcl/pull/3486), [#3489](https://github.com/PointCloudLibrary/pcl/pull/3489), [#3497](https://github.com/PointCloudLibrary/pcl/pull/3497)]
* **[abi]** Migrate to `std` random number generators [[#2956](https://github.com/PointCloudLibrary/pcl/pull/2956), [#2962](https://github.com/PointCloudLibrary/pcl/pull/2962), [#3069](https://github.com/PointCloudLibrary/pcl/pull/3069)]
* **[deprecation]** Deprecate `pcl_isnan`, `pcl_isfinite`, and `pcl_isinf` in favor of `std` methods [[#2798](https://github.com/PointCloudLibrary/pcl/pull/2798), [#3457](https://github.com/PointCloudLibrary/pcl/pull/3457)]
* Add explicit `std::` prefix to standard types/functions [[#3265](https://github.com/PointCloudLibrary/pcl/pull/3265), [#3326](https://github.com/PointCloudLibrary/pcl/pull/3326), [#3327](https://github.com/PointCloudLibrary/pcl/pull/3327), [#3328](https://github.com/PointCloudLibrary/pcl/pull/3328), [#3354](https://github.com/PointCloudLibrary/pcl/pull/3354), [#3426](https://github.com/PointCloudLibrary/pcl/pull/3426), [#3434](https://github.com/PointCloudLibrary/pcl/pull/3434)]
* Remove `else` after `return` statement [[#3180](https://github.com/PointCloudLibrary/pcl/pull/3180), [#3181](https://github.com/PointCloudLibrary/pcl/pull/3181), [#3182](https://github.com/PointCloudLibrary/pcl/pull/3182), [#3183](https://github.com/PointCloudLibrary/pcl/pull/3183), [#3184](https://github.com/PointCloudLibrary/pcl/pull/3184), [#3185](https://github.com/PointCloudLibrary/pcl/pull/3185), [#3186](https://github.com/PointCloudLibrary/pcl/pull/3186)]
* Remove redundant `typename` keyword [[#2896](https://github.com/PointCloudLibrary/pcl/pull/2896), [#2897](https://github.com/PointCloudLibrary/pcl/pull/2897), [#2927](https://github.com/PointCloudLibrary/pcl/pull/2927)]
* Prefer `#pragma once` over `#ifndef` include guards [[#2617](https://github.com/PointCloudLibrary/pcl/pull/2617), [#2707](https://github.com/PointCloudLibrary/pcl/pull/2707)]
* Apply clang-format to white-listed modules [[#3343](https://github.com/PointCloudLibrary/pcl/pull/3343), [#3344](https://github.com/PointCloudLibrary/pcl/pull/3344), [#3356](https://github.com/PointCloudLibrary/pcl/pull/3356), [#3363](https://github.com/PointCloudLibrary/pcl/pull/3363), [#3393](https://github.com/PointCloudLibrary/pcl/pull/3393), [#3416](https://github.com/PointCloudLibrary/pcl/pull/3416)]
* Remove default constructors/destructors [[#3440](https://github.com/PointCloudLibrary/pcl/pull/3440), [#3454](https://github.com/PointCloudLibrary/pcl/pull/3454)]
* Fix various compiler warnings [[#2665](https://github.com/PointCloudLibrary/pcl/pull/2665), [#2775](https://github.com/PointCloudLibrary/pcl/pull/2775), [#2778](https://github.com/PointCloudLibrary/pcl/pull/2778), [#2781](https://github.com/PointCloudLibrary/pcl/pull/2781), [#2782](https://github.com/PointCloudLibrary/pcl/pull/2782), [#2822](https://github.com/PointCloudLibrary/pcl/pull/2822), [#2898](https://github.com/PointCloudLibrary/pcl/pull/2898), [#2907](https://github.com/PointCloudLibrary/pcl/pull/2907), [#3001](https://github.com/PointCloudLibrary/pcl/pull/3001), [#3075](https://github.com/PointCloudLibrary/pcl/pull/3075), [#3076](https://github.com/PointCloudLibrary/pcl/pull/3076), [#3153](https://github.com/PointCloudLibrary/pcl/pull/3153), [#3155](https://github.com/PointCloudLibrary/pcl/pull/3155), [#3208](https://github.com/PointCloudLibrary/pcl/pull/3208), [#3212](https://github.com/PointCloudLibrary/pcl/pull/3212), [#3214](https://github.com/PointCloudLibrary/pcl/pull/3214), [#3342](https://github.com/PointCloudLibrary/pcl/pull/3342), [#3345](https://github.com/PointCloudLibrary/pcl/pull/3345), [#3348](https://github.com/PointCloudLibrary/pcl/pull/3348), [#3366](https://github.com/PointCloudLibrary/pcl/pull/3366), [#3372](https://github.com/PointCloudLibrary/pcl/pull/3372), [#3375](https://github.com/PointCloudLibrary/pcl/pull/3375), [#3377](https://github.com/PointCloudLibrary/pcl/pull/3377), [#3385](https://github.com/PointCloudLibrary/pcl/pull/3385), [#3388](https://github.com/PointCloudLibrary/pcl/pull/3388), [#3409](https://github.com/PointCloudLibrary/pcl/pull/3409), [#3425](https://github.com/PointCloudLibrary/pcl/pull/3425), [#3427](https://github.com/PointCloudLibrary/pcl/pull/3427), [#3507](https://github.com/PointCloudLibrary/pcl/pull/3507), [#3509](https://github.com/PointCloudLibrary/pcl/pull/3509), [#3554](https://github.com/PointCloudLibrary/pcl/pull/3554), [#3555](https://github.com/PointCloudLibrary/pcl/pull/3555)]
* Prefer `std::size_t` in loops [[#3550](https://github.com/PointCloudLibrary/pcl/pull/3550), [#3556](https://github.com/PointCloudLibrary/pcl/pull/3556)]

### `Modules:`

#### `CMake:`

* Set compile features on imported PCL targets [[#3567](https://github.com/PointCloudLibrary/pcl/pull/3567)]
* Create CMake imported targets for header-only modules [[#3495](https://github.com/PointCloudLibrary/pcl/pull/3495)]
* Add `PCL_WARNINGS_ARE_ERRORS` CMake option and enable it in Ubuntu 16.04 CI job [[#3478](https://github.com/PointCloudLibrary/pcl/pull/3478)]
* Avoid using `VERSION_GREATER_EQUAL` to preserve compatibility with CMake 3.5 [[#3460](https://github.com/PointCloudLibrary/pcl/pull/3460)]
* Remove policy push/pop from "PCLConfig.cmake" file [[#3431](https://github.com/PointCloudLibrary/pcl/pull/3431)]
* Fix link-type keywords in linked libraries in "PCLConfig.cmake" with CMake >= 3.11 [[#3341](https://github.com/PointCloudLibrary/pcl/pull/3341)]
* Update prefix hints in GTest finder script [[#3331](https://github.com/PointCloudLibrary/pcl/pull/3331)]
* PCL All-in-one Installer: add process to add/remove VTK path [[#3322](https://github.com/PointCloudLibrary/pcl/pull/3322)]
* Add `surface` module to the list of dependencies of `examples` [[#3295](https://github.com/PointCloudLibrary/pcl/pull/3295)]
* Add missing install rule for "2d/kernel.hpp" header [[#3278](https://github.com/PointCloudLibrary/pcl/pull/3278)]
* **[new-feature]** Add `format` compilation target that applies `clang-format` to whitelisted modules [[#3188](https://github.com/PointCloudLibrary/pcl/pull/3188)]
* Use `COMPONENTS` when finding VTK to avoid linking agains unnecessary modules [[#3140](https://github.com/PointCloudLibrary/pcl/pull/3140)]
* Add thread library of the system by `Threads::Threads` instead of via `-pthread` flag [[#3102](https://github.com/PointCloudLibrary/pcl/pull/3102)]
* Fix `find_package_handle_standard_args` incorrect argument in GTest finder script [[#3098](https://github.com/PointCloudLibrary/pcl/pull/3098)]
* Fix compound target's target registration [[#3090](https://github.com/PointCloudLibrary/pcl/pull/3090)]
* Set `CMP0072` to `NEW` to prefer `GLVND` over legacy OpenGL modules [[#3066](https://github.com/PointCloudLibrary/pcl/pull/3066)]
* Improve parsing of `BUNDLE` option in `PCL_ADD_EXECUTABLE` [[#3064](https://github.com/PointCloudLibrary/pcl/pull/3064)]
* Change debug/release targets postfix on Windows [[#3055](https://github.com/PointCloudLibrary/pcl/pull/3055)]
* Add version info on Windows for DLLs/Exe [[#3054](https://github.com/PointCloudLibrary/pcl/pull/3054)]
* Modernize some CMake macros (Increases minimum required CMake version to 3.5) [[#3044](https://github.com/PointCloudLibrary/pcl/pull/3044)]
* Add newer Boost versions to `Boost_ADDITIONAL_VERSIONS` [[#3003](https://github.com/PointCloudLibrary/pcl/pull/3003)]
* Refactor `PCL_MAKE_PKGCONFIG` [[#2894](https://github.com/PointCloudLibrary/pcl/pull/2894)]
* Bump Eigen minimum version to 3.1 [[#2893](https://github.com/PointCloudLibrary/pcl/pull/2893)]
* Improve compatibility with latest VTK [[#2885](https://github.com/PointCloudLibrary/pcl/pull/2885)]
* PCL All-in-one Installer: add quiet flag to msiexec installs [[#2873](https://github.com/PointCloudLibrary/pcl/pull/2873)]
* Use `-mtune` instead of `-march` on non-x86 CPUs [[#2868](https://github.com/PointCloudLibrary/pcl/pull/2868)]
* Prevent adding `/MP` flag for MSVC in case any other language than C/CXX will be used (e.g. CUDA) [[#2862](https://github.com/PointCloudLibrary/pcl/pull/2862)]
* Add support for Visual Studio 2019 [[#2826](https://github.com/PointCloudLibrary/pcl/pull/2826)]
* Drop MSVC 2013 (and below) support [[#2817](https://github.com/PointCloudLibrary/pcl/pull/2817)]
* Drop GCC 4 support [[#2760](https://github.com/PointCloudLibrary/pcl/pull/2760)]
* Mark include directories of 3rd-party libraries as system includes [[#2733](https://github.com/PointCloudLibrary/pcl/pull/2733)]
* Fix Qt5 CMake issue [[#2719](https://github.com/PointCloudLibrary/pcl/pull/2719)]
* Reduce unnecessary nesting in "CMakeLists.txt" [[#2718](https://github.com/PointCloudLibrary/pcl/pull/2718)]
* Drop Qt4 support [[#2716](https://github.com/PointCloudLibrary/pcl/pull/2716)]
* Remove unnecessary `VTK_INCLUDE_DIRECTORIES` [[#2713](https://github.com/PointCloudLibrary/pcl/pull/2713)]
* Add `MSVC_MP` CMake option to adjust number of parallel build jobs [[#2705](https://github.com/PointCloudLibrary/pcl/pull/2705)]
* Set interface compile features to `cxx_std_14` on PCL targets [[#2697](https://github.com/PointCloudLibrary/pcl/pull/2697)]
*  Reorganize and beautify project generation for IDEs  [[#2691](https://github.com/PointCloudLibrary/pcl/pull/2691)]
* **[new-feature]** Enable C++ 14 flags [[#2690](https://github.com/PointCloudLibrary/pcl/pull/2690)]
* Remove vendored "FindGLEW.cmake" and adopt imported targets; rename "FindGTest.cmake" to prevent name clash [[#2679](https://github.com/PointCloudLibrary/pcl/pull/2679)]
* Add compound CMake targets for examples, tools, and apps [[#2673](https://github.com/PointCloudLibrary/pcl/pull/2673)]
* Set CMake policy `CMP0074` to `NEW` [[#2671](https://github.com/PointCloudLibrary/pcl/pull/2671)]
* Remove conditional code for no longer supported versions of CMake [[#2667](https://github.com/PointCloudLibrary/pcl/pull/2667)]
* Raise minimum required Boost version to 1.55; fix deprecated Boost endians [[#2801](https://github.com/PointCloudLibrary/pcl/pull/2801)]
* Modernize FLANN finder script [[#2740](https://github.com/PointCloudLibrary/pcl/pull/2740), [#2861](https://github.com/PointCloudLibrary/pcl/pull/2861), [#2905](https://github.com/PointCloudLibrary/pcl/pull/2905), [#2910](https://github.com/PointCloudLibrary/pcl/pull/2910), [#3157](https://github.com/PointCloudLibrary/pcl/pull/3157), [#3202](https://github.com/PointCloudLibrary/pcl/pull/3202), [#3220](https://github.com/PointCloudLibrary/pcl/pull/3220), [#3317](https://github.com/PointCloudLibrary/pcl/pull/3317)]

#### `libpcl_2d:`

* Fix `EIGEN_ALIGN16` positionining for point type `XYZHSV` [[#3237](https://github.com/PointCloudLibrary/pcl/pull/3237)]

#### `libpcl_common:`

* **[deprecation]** Revert smart pointer type change in `PointCloud` and deprecate `getMapping()` [[#3563](https://github.com/PointCloudLibrary/pcl/pull/3563)]
* Define `RangeImage` destructor as virtual [[#3477](https://github.com/PointCloudLibrary/pcl/pull/3477)]
* Refactor `pcl::eigen33()` to reduce potential errors (and remove bug) [[#3441](https://github.com/PointCloudLibrary/pcl/pull/3441)]
* Use generic detection idiom in `isFinite()` [[#3402](https://github.com/PointCloudLibrary/pcl/pull/3402)]
* **[deprecation]** Deprecate `getFields()` with output parameter in favor of overload with return value [[#3401](https://github.com/PointCloudLibrary/pcl/pull/3401)]
* Refactor polynomial calculations and remove memory leak [[#3392](https://github.com/PointCloudLibrary/pcl/pull/3392)]
* Removes deprecated usage of `getFields(cloud, fields)` [[#3374](https://github.com/PointCloudLibrary/pcl/pull/3374)]
* **[deprecation]** Deprecate `getFieldIndex()`/`getFields()` with first argument as cloud [[#3365](https://github.com/PointCloudLibrary/pcl/pull/3365)]
* Remove `vector.reserve()` from hot path in cloud concatenation [[#3361](https://github.com/PointCloudLibrary/pcl/pull/3361)]
* **[deprecation]** Add `PCLPointCloud2::operator+=()` and update concatenation operation [[#3320](https://github.com/PointCloudLibrary/pcl/pull/3320)]
* **[new-feature]** Add concatenate operation for `PolygonMesh` [[#3316](https://github.com/PointCloudLibrary/pcl/pull/3316)]
* Simplify the resize logic in `PCLBase` [[#3315](https://github.com/PointCloudLibrary/pcl/pull/3315)]
* **[behavior]** Disable colored output for non-interactive terminals [[#3310](https://github.com/PointCloudLibrary/pcl/pull/3310)]
* **[new-feature]** Add `emplace[_back]` to `pcl::PointCloud` [[#3207](https://github.com/PointCloudLibrary/pcl/pull/3207)]
* **[new-feature]** Add `pcl::make_shared` that automatically handles aligned allocations [[#3163](https://github.com/PointCloudLibrary/pcl/pull/3163)]
* **[behavior]** Enforce right-hand-rule on PCA eigenvectors [[#2946](https://github.com/PointCloudLibrary/pcl/pull/2946)]
* Fix a bug in `CentroidPoint` [[#2875](https://github.com/PointCloudLibrary/pcl/pull/2875)]
* **[behavior]** Initialize curvature in `PointNormal` default constructor [[#2674](https://github.com/PointCloudLibrary/pcl/pull/2674)]

#### `libpcl_cuda:`

* Remove code for CUDA below 7.5, update version checks [[#3152](https://github.com/PointCloudLibrary/pcl/pull/3152)]
* Add missing include to support CUDA 10.1 [[#2883](https://github.com/PointCloudLibrary/pcl/pull/2883)]
* Drop CUDA 7.0 (and below) support [[#2736](https://github.com/PointCloudLibrary/pcl/pull/2736)]

#### `libpcl_features:`

* Fix numerical issue in GASD estimation [[#3498](https://github.com/PointCloudLibrary/pcl/pull/3498)]
* **[deprecation]** Deprecate `computeRSD()` functions that take pointclouds by pointer [[#2827](https://github.com/PointCloudLibrary/pcl/pull/2827)]
* Fix data race in `NormalEstimationOMP` on Windows [[#2770](https://github.com/PointCloudLibrary/pcl/pull/2770)]

#### `libpcl_filters:`

* Merge pull request #3540 from facontidavide/fix_filter [[#3540](https://github.com/PointCloudLibrary/pcl/pull/3540)]
* Fix incorrect switch fallthrough in convolution operator [[#3429](https://github.com/PointCloudLibrary/pcl/pull/3429)]
* Use `size_t` for address computation in `CropBox` filter [[#3418](https://github.com/PointCloudLibrary/pcl/pull/3418)]
*  Fix a bug in removed index extraction in `UniformSampling` [[#3323](https://github.com/PointCloudLibrary/pcl/pull/3323)]
* Fix `CropBox` with indices; add new assertions for unit tests [[#3306](https://github.com/PointCloudLibrary/pcl/pull/3306)]
* Use fixed-size Eigen block expressions in `FrustumCulling` [[#2786](https://github.com/PointCloudLibrary/pcl/pull/2786)]
* **[abi][deprecation]** Restructure and add functionality to filters templated on `PCLPointCloud2` [[#3483](https://github.com/PointCloudLibrary/pcl/pull/3483), [#3500](https://github.com/PointCloudLibrary/pcl/pull/3500)]

#### `libpcl_gpu:`

* Fix building of KinFu Large Scale app with Visual Studio [[#3504](https://github.com/PointCloudLibrary/pcl/pull/3504)]
* **[api]** Replace `uint64_type` by `std::uint64_t` [[#3435](https://github.com/PointCloudLibrary/pcl/pull/3435)]
* Add OpenNI2 support to `kinfu_largescale` tool [[#3391](https://github.com/PointCloudLibrary/pcl/pull/3391)]
* Use `constexpr` in KinFu and KinFu large scale apps [[#3386](https://github.com/PointCloudLibrary/pcl/pull/3386)]
* Remove duplicated and unused "cutil_math.h" [[#3264](https://github.com/PointCloudLibrary/pcl/pull/3264)]
* Drop unnecessary __CUDA_ARCH__ checks [[#3154](https://github.com/PointCloudLibrary/pcl/pull/3154)]
* Remove code for CUDA below 7.5, update version checks [[#3152](https://github.com/PointCloudLibrary/pcl/pull/3152)]
* Remove unused timer routines in "NCV.cu" [[#3135](https://github.com/PointCloudLibrary/pcl/pull/3135)]
* Add sync variants to CUDA vote functions to support PTX/ARCH >= 7.0 [[#2981](https://github.com/PointCloudLibrary/pcl/pull/2981)]
* Fix `-debug` option parsing in `people_pcd_prob` tool [[#2656](https://github.com/PointCloudLibrary/pcl/pull/2656)]

#### `libpcl_io:`

* Make PLY parser more tolerant towards files not adhering to standard [[#3542](https://github.com/PointCloudLibrary/pcl/pull/3542)]
* Fix a bug in binary header generation for PCD files [[#3522](https://github.com/PointCloudLibrary/pcl/pull/3522)]
* **[behavior]** Add support for device URI in `OpenNI2Grabber` constructor [[#3238](https://github.com/PointCloudLibrary/pcl/pull/3238)]
* Include "pcl_config.h" before checking `HAVE_OPENNI2` [[#3191](https://github.com/PointCloudLibrary/pcl/pull/3191)]
* Print descriptive error if PLY file not found [[#3143](https://github.com/PointCloudLibrary/pcl/pull/3143)]
* **[deprecation]** Add `registerCallback()` overload to grabbers to support assignment of `boost::function`s with templated signatures [[#3128](https://github.com/PointCloudLibrary/pcl/pull/3128)]
* Fix preallocation of memory in PLY reader [[#2800](https://github.com/PointCloudLibrary/pcl/pull/2800)]
* Fix possible integer overflow while iterating `PointCloud` fields [[#2754](https://github.com/PointCloudLibrary/pcl/pull/2754)]
* **[removal]** Remove FZ-API [[#2747](https://github.com/PointCloudLibrary/pcl/pull/2747)]
* Improve loading of massive PLY files [[#2715](https://github.com/PointCloudLibrary/pcl/pull/2715)]
* **[new-feature]** Add RSSDK 2.0 (librealsense2) grabber [[#2214](https://github.com/PointCloudLibrary/pcl/pull/2214)]
* Fix callback signatures in some grabbers [[#3216](https://github.com/PointCloudLibrary/pcl/pull/3216), [#3225](https://github.com/PointCloudLibrary/pcl/pull/3225)]

#### `libpcl_kdtree:`

* **[deprecation]** Remove unnecessary FLANN includes, deprecate "kdtree/flann.h" header [[#2993](https://github.com/PointCloudLibrary/pcl/pull/2993)]

#### `libpcl_keypoints:`

* Removed all duplicated branches in `AgastKeypoint2D` [[#2657](https://github.com/PointCloudLibrary/pcl/pull/2657)]

#### `libpcl_recognition:`

* Use range-based for loops with `boost::filesystem::directory_iterator` [[#3432](https://github.com/PointCloudLibrary/pcl/pull/3432)]
* **[deprecation]** Refactor `MaskMap` and deprecate several of its methods [[#3399](https://github.com/PointCloudLibrary/pcl/pull/3399)]
* Add missing include in  "trimmed_icp.h" [[#3286](https://github.com/PointCloudLibrary/pcl/pull/3286)]
* Use I/O helper method in `pcl::LineRGBD` [[#2796](https://github.com/PointCloudLibrary/pcl/pull/2796)]

#### `libpcl_registration:`

* Make `Registration::hasConverged()` const-qualified [[#3456](https://github.com/PointCloudLibrary/pcl/pull/3456)]
* Fix debug message by reordering statements in GICP [[#3398](https://github.com/PointCloudLibrary/pcl/pull/3398)]
* **[new-feature]** Add linear least squares version of symmetric objective function for ICP [[#3390](https://github.com/PointCloudLibrary/pcl/pull/3390)]
* Fix the OpenMP errors/warnings in "ia_fpcs.hpp" [[#3389](https://github.com/PointCloudLibrary/pcl/pull/3389)]
* Fix `pcl::Registration::getFitnessScore()` documentation [[#3082](https://github.com/PointCloudLibrary/pcl/pull/3082)]
* **[behavior]** Fix ICP misbehavior in the "failure after maximum iterations" mode [[#2892](https://github.com/PointCloudLibrary/pcl/pull/2892)]

#### `libpcl_sample_consensus:`

* **[new-feature]** Add parallel RANSAC implementation with OpenMP [[#3514](https://github.com/PointCloudLibrary/pcl/pull/3514)]
* **[behavior]** Set default min and max angle for SAC cone models [[#3466](https://github.com/PointCloudLibrary/pcl/pull/3466)]
* Add `const` qualifier to multiple methods in SAC module [[#2970](https://github.com/PointCloudLibrary/pcl/pull/2970)]
* Fix regression in `pcl::SACSegmentation` line fitting [[#2767](https://github.com/PointCloudLibrary/pcl/pull/2767)]

#### `libpcl_segmentation:`

* **[removal]** Remove `SupervoxelClustering::getColoredVoxelCloud()` [[#3469](https://github.com/PointCloudLibrary/pcl/pull/3469)]
* **[deprecation]** Delete unused params in `OrganizedMultiPlaneSegmentation::refine()` [[#3302](https://github.com/PointCloudLibrary/pcl/pull/3302)]
* Add `noexcept` for `boost::checked_delete<>` friend [[#2942](https://github.com/PointCloudLibrary/pcl/pull/2942)]
* Fix `SupervoxelClustering` compilation problem in MSVC 2015.3 [[#2867](https://github.com/PointCloudLibrary/pcl/pull/2867)]

#### `libpcl_simulation:`

* Fix bug in `SimExample::write_depth_image_uint` trigerring `-Wtype-limits` warning [[#3430](https://github.com/PointCloudLibrary/pcl/pull/3430)]

#### `libpcl_stereo:`

* Merge pull request #3566 from kunaltyagi/stereo [[#3566](https://github.com/PointCloudLibrary/pcl/pull/3566)]

#### `libpcl_surface:`

* Fix undefined behaviour in `OctNode` [[#3561](https://github.com/PointCloudLibrary/pcl/pull/3561)]
* Fix memory leak in `TextureMapping` [[#3549](https://github.com/PointCloudLibrary/pcl/pull/3549)]
* Prevent `memset` for empty vector in 3rdparty/poisson4 [[#3537](https://github.com/PointCloudLibrary/pcl/pull/3537)]
* Remove incorrect tree update in `MarchingCubes` [[#3240](https://github.com/PointCloudLibrary/pcl/pull/3240)]
* **[deprecation]** Convert `MovingLeastSquaresOMP` into an alias template and deprecate [[#3119](https://github.com/PointCloudLibrary/pcl/pull/3119)]
* Fix a bug in `ConvexHull` when indices are used [[#2999](https://github.com/PointCloudLibrary/pcl/pull/2999)]
* Include "pcl_config.h" before testing `HAVE_QHULL` [[#2979](https://github.com/PointCloudLibrary/pcl/pull/2979)]
* Throw exceptions instead of `exit(0)` in Poisson surface reconstruction [[#2891](https://github.com/PointCloudLibrary/pcl/pull/2891)]
*  Add check for invalid plane coefficients in `MovingLeastSquares` [[#2805](https://github.com/PointCloudLibrary/pcl/pull/2805)]
* Fix the size of the lookup table in `BilateralUpsampling` [[#2749](https://github.com/PointCloudLibrary/pcl/pull/2749)]
* Replace `hash_map` with `unordered_map` in 3rdparty/poisson4 [[#2640](https://github.com/PointCloudLibrary/pcl/pull/2640)]

#### `libpcl_visualization:`

* Add a flag to disable window autoresizing in `ImageViewer` [[#3394](https://github.com/PointCloudLibrary/pcl/pull/3394)]
* **[deprecation]** Add new overload of `PointCloudColorHandler::getColor()` [[#3187](https://github.com/PointCloudLibrary/pcl/pull/3187)]
* **[behavior]** Improve c/C command in `PCLVisualizer` [[#2926](https://github.com/PointCloudLibrary/pcl/pull/2926)]
* Disable VTK warning display with OpenGL rendering backend [[#2912](https://github.com/PointCloudLibrary/pcl/pull/2912)]
* **[abi]** Minor refactoring of `pcl::visualization::Camera` and related functions [[#2901](https://github.com/PointCloudLibrary/pcl/pull/2901)]
* Fix a bug in `PCLVisualizer::setShapeRenderingProperties()` [[#2900](https://github.com/PointCloudLibrary/pcl/pull/2900)]
* Fix possible integer overflow while iterating `PointCloud` fields [[#2754](https://github.com/PointCloudLibrary/pcl/pull/2754)]
* Raise minimum VTK version to 6.2 [[#2685](https://github.com/PointCloudLibrary/pcl/pull/2685)]

#### `PCL Apps:`

* Fix memory leaks in OpenNI apps [[#3553](https://github.com/PointCloudLibrary/pcl/pull/3553)]
* Use range-based for loops with `boost::filesystem::directory_iterator` [[#3432](https://github.com/PointCloudLibrary/pcl/pull/3432)]
* Do not use deprecated function in `stereo_ground_segmentation` [[#3406](https://github.com/PointCloudLibrary/pcl/pull/3406)]
* Add missing `std::move` or `const` reference for parameters [[#3232](https://github.com/PointCloudLibrary/pcl/pull/3232)]
* Remove magic numbers from `organized_segmentation_demo` app [[#3108](https://github.com/PointCloudLibrary/pcl/pull/3108)]
* Add missing include `render_views_tesselated_sphere` app [[#2909](https://github.com/PointCloudLibrary/pcl/pull/2909)]
* Remove Qt version checks [[#2762](https://github.com/PointCloudLibrary/pcl/pull/2762)]
* Cleanup Qt includes in Modeler app [[#2756](https://github.com/PointCloudLibrary/pcl/pull/2756)]
* Cleanup Qt includes in CloudComposer app [[#2744](https://github.com/PointCloudLibrary/pcl/pull/2744)]
* Fix MSVC compile issues in CloudComposer app [[#2712](https://github.com/PointCloudLibrary/pcl/pull/2712)]
* Fix Modeler app with OpenGL2 rendering backend [[#2653](https://github.com/PointCloudLibrary/pcl/pull/2653)]

#### `PCL Docs:`

* Fix and improve documentation in `sample_consensus` module [[#3301](https://github.com/PointCloudLibrary/pcl/pull/3301)]
* Fix `pcl::Registration::getFitnessScore()` documentation [[#3082](https://github.com/PointCloudLibrary/pcl/pull/3082)]
* Fix source file name path stripping in Doxygen [[#2714](https://github.com/PointCloudLibrary/pcl/pull/2714)]

#### `PCL Tutorials:`

* Fix "CMakeLists.txt" in VFH tutorial [[#3449](https://github.com/PointCloudLibrary/pcl/pull/3449)]
* Update information about types with RGBA data in tutorial [[#3294](https://github.com/PointCloudLibrary/pcl/pull/3294)]
* Add a note about adjusting passthrough limits in template alignment tutorial [[#3193](https://github.com/PointCloudLibrary/pcl/pull/3193)]
* Add missing shell command in "Building PCL" tutorial [[#2996](https://github.com/PointCloudLibrary/pcl/pull/2996)]
* Fix cube rendering properties in "Moment of Inertia" tutorial [[#2880](https://github.com/PointCloudLibrary/pcl/pull/2880)]
* Migrate from `boost::math::iround` to `std::lround` [[#2818](https://github.com/PointCloudLibrary/pcl/pull/2818)]
* Link with `PCL_LIBRARIES`, not `PCL_***_LIBRARIES` [[#2799](https://github.com/PointCloudLibrary/pcl/pull/2799)]

#### `PCL Tests:`

* Move OMP version of `MovingLeastSquares` into a separate test case [[#3382](https://github.com/PointCloudLibrary/pcl/pull/3382)]
* Fix `CropBox` with indices; add new assertions for unit tests [[#3306](https://github.com/PointCloudLibrary/pcl/pull/3306)]
* Fix `find_package_handle_standard_args` incorrect argument in GTest finder script [[#3098](https://github.com/PointCloudLibrary/pcl/pull/3098)]
* Do not test buffers instantiated with `char` template argument [[#2980](https://github.com/PointCloudLibrary/pcl/pull/2980)]
* Remove `GTEST_USE_OWN_TR1_TUPLE` defines [[#2828](https://github.com/PointCloudLibrary/pcl/pull/2828)]
* Add a new test for `SampleConsensusModelLine` [[#2768](https://github.com/PointCloudLibrary/pcl/pull/2768)]
* Split `test_registration` into three binaries [[#2727](https://github.com/PointCloudLibrary/pcl/pull/2727)]

#### `PCL Tools:`

* **[behavior]** Do not discard data fields in `pcl_uniform_sampling` tool [[#3461](https://github.com/PointCloudLibrary/pcl/pull/3461)]
* Add missing includes in `ensenso_viewer.cpp` [[#3035](https://github.com/PointCloudLibrary/pcl/pull/3035)]
* Do not terminate `openni_viewer`/`openni2_viewer` if image viewer was not instantiated [[#2698](https://github.com/PointCloudLibrary/pcl/pull/2698)]

#### `CI:`

* Add `PCL_WARNINGS_ARE_ERRORS` CMake option and enable it in Ubuntu 16.04 CI job [[#3478](https://github.com/PointCloudLibrary/pcl/pull/3478)]
* Add a new Azure pipeline for Ubuntu 19.10 [[#3446](https://github.com/PointCloudLibrary/pcl/pull/3446)]
* Add formatting job on CI [[#3420](https://github.com/PointCloudLibrary/pcl/pull/3420)]
* Add Dockerfiles for images used on CI [[#3350](https://github.com/PointCloudLibrary/pcl/pull/3350)]
* Publish test results on Azure pipelines [[#2948](https://github.com/PointCloudLibrary/pcl/pull/2948)]
* Build tutorials on Azure Pipelines [[#2696](https://github.com/PointCloudLibrary/pcl/pull/2696)]

## *= 1.9.1 (26.11.2018) =*

### `Modules:`

#### `Uncategorized:`

* Add missing visualization section from the changelog generation. [[#2634]](https://github.com/PointCloudLibrary/pcl/pull/2634)

#### `CMake:`

* Fix development version check on DISSECT_VERSION. [[#2633]](https://github.com/PointCloudLibrary/pcl/pull/2633)
* Remove CMake policy CMP0054 setting. [[#2627]](https://github.com/PointCloudLibrary/pcl/pull/2627)
* PCLConfig.cmake - POP policy-stack before return() [[#2626]](https://github.com/PointCloudLibrary/pcl/pull/2626)
* Remove CMake config installation folder only when empty [[#2622]](https://github.com/PointCloudLibrary/pcl/pull/2622)
* Downgrade grabber dependency message level to STATUS. [[#2620]](https://github.com/PointCloudLibrary/pcl/pull/2620)
* Migrate CMake policy 0048 [[#2608]](https://github.com/PointCloudLibrary/pcl/pull/2608)
* Bump CMake minimum version to 3.1. [[#2605]](https://github.com/PointCloudLibrary/pcl/pull/2605)
* Bump version to 1.9.0-dev [[#2602]](https://github.com/PointCloudLibrary/pcl/pull/2602)
* Search in $EIGEN_ROOT first while looking for Eigen [[#2592]](https://github.com/PointCloudLibrary/pcl/pull/2592)

#### `libpcl_visualization:`

* address conflict between visualization and VTK head [[#2612]](https://github.com/PointCloudLibrary/pcl/pull/2612)

#### `PCL Tutorials:`

* Bump CMake minimum version to 3.1. [[#2605]](https://github.com/PointCloudLibrary/pcl/pull/2605)

#### `PCL Tests:`

* Refactor `SHOTEstimation` and `SHOTColorEstimation` tests [[#2636]](https://github.com/PointCloudLibrary/pcl/pull/2636)

#### `CI:`

* Set up azure pipelines for macOS High Sierra [[#2635]](https://github.com/PointCloudLibrary/pcl/pull/2635)
* Enable Windows builds on Azure Pipelines [[#2632]](https://github.com/PointCloudLibrary/pcl/pull/2632)
* Add Ubuntu16.04 based build on Azure Pipelines [[#2611]](https://github.com/PointCloudLibrary/pcl/pull/2611)
* Remove documentation job from Travis [[#2604]](https://github.com/PointCloudLibrary/pcl/pull/2604)


## *= 1.9.0 (06.11.2018) =*

### `New Features:`

*Newly added functionalities.*

* **[common][visualization]** Add Viridis color LUT [[#2420]](https://github.com/PointCloudLibrary/pcl/pull/2420)
* **[octree]** Implementation of the iterator 'OctreeLeafNodeBreadthIterator'. [[#2204]](https://github.com/PointCloudLibrary/pcl/pull/2204)
* **[octree]** Implementation of the iterator 'OctreeFixedDepthIterator'. [[#1983]](https://github.com/PointCloudLibrary/pcl/pull/1983)
* **[ci]** Enable Global Tests on Windows CI [[#2137]](https://github.com/PointCloudLibrary/pcl/pull/2137)
* **[features]** Add GASD global point cloud descriptor [[#1652]](https://github.com/PointCloudLibrary/pcl/pull/1652)
* **[visualization]** Add overload to `PCLVisualizer::addText3D()` that allows specifying text orientation [[#2038]](https://github.com/PointCloudLibrary/pcl/pull/2038)
* **[features]** FLARELocalReferenceFrameEstimation class added [[#1571]](https://github.com/PointCloudLibrary/pcl/pull/1571)
* **[surface][tools]** Add new mls projection method. Deprecated `MovingLeastSquares::setPolynomialFit()`. [[#1960]](https://github.com/PointCloudLibrary/pcl/pull/1960)

### `Deprecated:`

*Deprecated code scheduled to be removed after two minor releases.*

* **[octree]** Implementation of the iterator 'OctreeLeafNodeBreadthIterator'. [[#2204]](https://github.com/PointCloudLibrary/pcl/pull/2204)
* **[common][segmentation]** Provide proper EuclideanClusterComparator method depreciation. New Pragma macro. New Deprecated type. [[#2096]](https://github.com/PointCloudLibrary/pcl/pull/2096)
* **[io]** Add support pcl::PointXYZRGBA to pcl::VLPGrabber. Deprecate rgb signatures. [[#2102]](https://github.com/PointCloudLibrary/pcl/pull/2102)
* **[surface][tools]** Add new mls projection method. Deprecated `MovingLeastSquares::setPolynomialFit()`. [[#1960]](https://github.com/PointCloudLibrary/pcl/pull/1960)

### `Removed:`

*Removal of deprecated code.*

* **[filters][io][surface][visualization]** Removal of deprecated code in filters, io, surface and visualization modules [[#2077]](https://github.com/PointCloudLibrary/pcl/pull/2077)
* **[common]** Remove deprecated ros headers [[#2075]](https://github.com/PointCloudLibrary/pcl/pull/2075)
* **[registration]** Remove registration module deprecated methods [[#2076]](https://github.com/PointCloudLibrary/pcl/pull/2076)
* **[sample_consensus]** Remove deprecated functions and variables from SAC module [[#2071]](https://github.com/PointCloudLibrary/pcl/pull/2071)
* **[common]** Removal of PCA deprecated constructor [[#2070]](https://github.com/PointCloudLibrary/pcl/pull/2070)

### `Behavioral changes:`

*Changes in the expected default behavior.*

* **[common]** PointCloudDepthAndRGBtoXYZRGBA: initialize with the default alpha value (fix #2476) [[#2533]](https://github.com/PointCloudLibrary/pcl/pull/2533)
* **[octree]** Reverse octree's depth first iterator order [[#2332]](https://github.com/PointCloudLibrary/pcl/pull/2332)
* **[common]** `PointXYZRGBL` `label` field is now default constructed to 0 [[#2462]](https://github.com/PointCloudLibrary/pcl/pull/2462)
* **[io]** Fix PLYReader is_dense behavior [[#2133]](https://github.com/PointCloudLibrary/pcl/pull/2133)

### `API changes:`

*Changes to the API which didn't went through the proper deprecation and removal cycle.*

* **[octree]** Implementation of the iterator 'OctreeLeafNodeBreadthIterator'. [[#2204]](https://github.com/PointCloudLibrary/pcl/pull/2204)
* **[sample_consensus]** Const-qualify most of the methods in SAC model classes [[#2270]](https://github.com/PointCloudLibrary/pcl/pull/2270)
* **[simulation]** Use GLuint rather than size_t to represent OpenGL indices. [[#2238]](https://github.com/PointCloudLibrary/pcl/pull/2238)
* **[visualization]** Fix access specifier in `PointCloudColorHandlerRGBAField` [[#2226]](https://github.com/PointCloudLibrary/pcl/pull/2226)
* **[docs]** Misc. typos (cont.) [[#2215]](https://github.com/PointCloudLibrary/pcl/pull/2215)
* **[octree]** OctreeIterators special member revision [[#2108]](https://github.com/PointCloudLibrary/pcl/pull/2108)
* **[io]** Add support pcl::PointXYZRGBA to pcl::VLPGrabber. Deprecate rgb signatures. [[#2102]](https://github.com/PointCloudLibrary/pcl/pull/2102)
* **[surface][tools]** Add new mls projection method. Deprecated `MovingLeastSquares::setPolynomialFit()`. [[#1960]](https://github.com/PointCloudLibrary/pcl/pull/1960)
* **[surface]** Add ability to cache mls results [[#1952]](https://github.com/PointCloudLibrary/pcl/pull/1952)

### `ABI changes:`

*Changes that cause ABI incompatibility but are still API compatible.*

* **[surface]** Missing pcl::MovingLeastSquaresOMP declaration without /openmp [[#2324]](https://github.com/PointCloudLibrary/pcl/pull/2324)
* **[common][filters][surface]** Improved docstrings and error messages [[#2300]](https://github.com/PointCloudLibrary/pcl/pull/2300)
* **[common]** Modified `GlasbeyLUT` indexing type to `size_t` [[#2297]](https://github.com/PointCloudLibrary/pcl/pull/2297)
* **[octree]** Implementation of the iterator 'OctreeFixedDepthIterator'. [[#1983]](https://github.com/PointCloudLibrary/pcl/pull/1983)
* **[common][segmentation]** Provide proper EuclideanClusterComparator method depreciation. New Pragma macro. New Deprecated type. [[#2096]](https://github.com/PointCloudLibrary/pcl/pull/2096)
* **[gpu]** Allow specifying decimation step in convertToTsdfCloud [[#2099]](https://github.com/PointCloudLibrary/pcl/pull/2099)
* **[apps]** More warning suppression in pcl apps [[#2080]](https://github.com/PointCloudLibrary/pcl/pull/2080)
* **[io]** Removed unused member from ply_parser [[#2066]](https://github.com/PointCloudLibrary/pcl/pull/2066)
* **[filters]** Fixes remove_indices in UniformSampling [[#1902]](https://github.com/PointCloudLibrary/pcl/pull/1902)
* **[visualization]** Add accessor for current rendering framerate in PCLVisualizer [[#1974]](https://github.com/PointCloudLibrary/pcl/pull/1974)
* **[simulation]** Redo: Simulation: enable returning of organized point clouds [[#1687]](https://github.com/PointCloudLibrary/pcl/pull/1687)
* **[registration]** Added option to specify translation and rotation convergence deltas in ICP and NDT algorithms. [[#1724]](https://github.com/PointCloudLibrary/pcl/pull/1724)

### `Modules:`

#### `Uncategorized:`

* Change Log generation tool. Automates change log generation. [[#2396]](https://github.com/PointCloudLibrary/pcl/pull/2396)
* Compatibility reports generation script [[#2410]](https://github.com/PointCloudLibrary/pcl/pull/2410)
* Update logo [[#2547]](https://github.com/PointCloudLibrary/pcl/pull/2547)
* Never close stale issues/prs [[#2400]](https://github.com/PointCloudLibrary/pcl/pull/2400)
* Fix typos in the whole codebase [[#2217]](https://github.com/PointCloudLibrary/pcl/pull/2217)
* Fixed typo and rearragend items in the issue template [[#2197]](https://github.com/PointCloudLibrary/pcl/pull/2197)
* Change Stale daysTillClose to 100 years [[#2166]](https://github.com/PointCloudLibrary/pcl/pull/2166)
* set stale daysUntilClose to a really big number [[#2162]](https://github.com/PointCloudLibrary/pcl/pull/2162)
* Stale set up [[#2101]](https://github.com/PointCloudLibrary/pcl/pull/2101)

#### `CMake:`

* Fix checks for user-provided CXX flags [[#2579]](https://github.com/PointCloudLibrary/pcl/pull/2579)
* Fix FLANN path to lower case [[#2576]](https://github.com/PointCloudLibrary/pcl/pull/2576)
* Use pkg-config to find Flann [[#2563]](https://github.com/PointCloudLibrary/pcl/pull/2563)
* Update FindBoost versions [[#2558]](https://github.com/PointCloudLibrary/pcl/pull/2558)
* Add PCL_BUILD_WITH_QHULL_DYNAMIC_LINKING_WIN32 option [[#2552]](https://github.com/PointCloudLibrary/pcl/pull/2552)
* Fix app/CMakeLists to enable Apps under Windows [[#2550]](https://github.com/PointCloudLibrary/pcl/pull/2550)
* When configuring with WITH_DOCS, but Doxygen is not available, prevent generation. [[#2516]](https://github.com/PointCloudLibrary/pcl/pull/2516)
* CMake: Do not include test targets in PCLConfig.cmake [[#2458]](https://github.com/PointCloudLibrary/pcl/pull/2458)
* CMake Set temporarily the policy CMP0074 to OLD [[#2454]](https://github.com/PointCloudLibrary/pcl/pull/2454)
* prevent GCC flags propagating to NVCC [[#2430]](https://github.com/PointCloudLibrary/pcl/pull/2430)
* Mark visualization as an optional dependency of tools [[#2439]](https://github.com/PointCloudLibrary/pcl/pull/2439)
* Do not mark imported libraries as GLOBAL in PCLConfig [[#2435]](https://github.com/PointCloudLibrary/pcl/pull/2435)
* Intel fixes [[#2432]](https://github.com/PointCloudLibrary/pcl/pull/2432)
* Export `-march=native` for Clang and prevent it from being included during cross compilation. [[#2416]](https://github.com/PointCloudLibrary/pcl/pull/2416)
* Do not search for PCL components that have been found already [[#2428]](https://github.com/PointCloudLibrary/pcl/pull/2428)
* Move SSE compiler options to `PCL_COMPILE_OPTIONS`. Expose PCL as a CMake imported target. [[#2100]](https://github.com/PointCloudLibrary/pcl/pull/2100)
* Add Visual Studio compiler option /FS for Ninja build [[#2414]](https://github.com/PointCloudLibrary/pcl/pull/2414)
* Use rpath in the target's install name [[#2241]](https://github.com/PointCloudLibrary/pcl/pull/2241)
* Improve QHull finder script [[#2344]](https://github.com/PointCloudLibrary/pcl/pull/2344)
* Fix link order issue with boost [[#2236]](https://github.com/PointCloudLibrary/pcl/pull/2236)
* Mark found PCL component libraries and include dirs as advanced [[#2235]](https://github.com/PointCloudLibrary/pcl/pull/2235)
* Prevent search for disabled optional dependencies in targets. [[#2229]](https://github.com/PointCloudLibrary/pcl/pull/2229)
* Fix installation rules for ml module [[#2192]](https://github.com/PointCloudLibrary/pcl/pull/2192)
* Fix conditional branch of Visual C++ 2017 [[#2121]](https://github.com/PointCloudLibrary/pcl/pull/2121)
* Add *_USE_STATIC options to PCLConfig [[#2086]](https://github.com/PointCloudLibrary/pcl/pull/2086)
* Add search path suffixes for Vcpkg [[#2085]](https://github.com/PointCloudLibrary/pcl/pull/2085)
* Update finder scripts for Ensenso, OpenNI, and OpenNI2 [[#2061]](https://github.com/PointCloudLibrary/pcl/pull/2061)
* Fix PACKAGE to include cmake/Modules directory [[#2053]](https://github.com/PointCloudLibrary/pcl/pull/2053)
* Unifies Find scripts in PCLConfig [[#1421]](https://github.com/PointCloudLibrary/pcl/pull/1421)
* CUDA 9 Arch Flags [[#2047]](https://github.com/PointCloudLibrary/pcl/pull/2047)
* Suppress log when PCL_FIND_QUIETLY is turned on. [[#2032]](https://github.com/PointCloudLibrary/pcl/pull/2032)
* fix /MP option not generated for Visual Studio. [[#2031]](https://github.com/PointCloudLibrary/pcl/pull/2031)
* Generate pkgconfig for 2d module [[#1979]](https://github.com/PointCloudLibrary/pcl/pull/1979)
* Update Find Boost [[#1972]](https://github.com/PointCloudLibrary/pcl/pull/1972)
* Added CUDA compute capability 5.3 [[#1929]](https://github.com/PointCloudLibrary/pcl/pull/1929)
* Fix issue with finding pcl deployed out of path [[#1923]](https://github.com/PointCloudLibrary/pcl/pull/1923)
* Add new gtest path [[#1920]](https://github.com/PointCloudLibrary/pcl/pull/1920)

#### `libpcl_2d:`

* Avoid huge index jumps in `RandomSample`. Remove `io` dependency from `2d`. [[#2141]](https://github.com/PointCloudLibrary/pcl/pull/2141)
* Fix header names [[#2079]](https://github.com/PointCloudLibrary/pcl/pull/2079)
* Generate pkgconfig for 2d module [[#1979]](https://github.com/PointCloudLibrary/pcl/pull/1979)

#### `libpcl_common:`

* Fix docstrings [[#2591]](https://github.com/PointCloudLibrary/pcl/pull/2591)
* Throw an early exception to prevent divide by zero error (#2481) [[#2530]](https://github.com/PointCloudLibrary/pcl/pull/2530)
* Relax requirements in eigen22d test. Always provide a normalized result in `pcl::transformPlane`. [[#2503]](https://github.com/PointCloudLibrary/pcl/pull/2503)
* **[behavior]** PointCloudDepthAndRGBtoXYZRGBA: initialize with the default alpha value (fix #2476) [[#2533]](https://github.com/PointCloudLibrary/pcl/pull/2533)
* Throw `UnorganizedPointCloudException` in `PointCloud::at` [[#2521]](https://github.com/PointCloudLibrary/pcl/pull/2521)
* Add missing const specifier for getters in `PCLBase`. [[#2502]](https://github.com/PointCloudLibrary/pcl/pull/2502)
* swap the header in pcl::PointCloud::swap [[#2499]](https://github.com/PointCloudLibrary/pcl/pull/2499)
* Add header guard and copyright info to polynomial_calculations.hpp [[#2500]](https://github.com/PointCloudLibrary/pcl/pull/2500)
* Add `header` to the print output of `PointCloud` [[#2498]](https://github.com/PointCloudLibrary/pcl/pull/2498)
* Fix force recalculation option in `BivariatePolynomialT::calculateGradient` [[#2479]](https://github.com/PointCloudLibrary/pcl/pull/2479)
* Fix various errors and typos in the docstrings and tutorials [[#2486]](https://github.com/PointCloudLibrary/pcl/pull/2486)
* Fix a bug in `PointRGBtoI` color conversion [[#2475]](https://github.com/PointCloudLibrary/pcl/pull/2475)
* Provide `operator<<` for `Intensity32u` point type [[#2467]](https://github.com/PointCloudLibrary/pcl/pull/2467)
* **[behavior]** `PointXYZRGBL` `label` field is now default constructed to 0 [[#2462]](https://github.com/PointCloudLibrary/pcl/pull/2462)
* Add some missing eigen alignment operators [[#2433]](https://github.com/PointCloudLibrary/pcl/pull/2433)
* Intel fixes [[#2432]](https://github.com/PointCloudLibrary/pcl/pull/2432)
* **[new-feature]** Add Viridis color LUT [[#2420]](https://github.com/PointCloudLibrary/pcl/pull/2420)
* Remove malloc header to restore builds on BSDs [[#2374]](https://github.com/PointCloudLibrary/pcl/pull/2374)
* Add support for multiple extensions in `parse_file_extension_argument ()`. [[#2347]](https://github.com/PointCloudLibrary/pcl/pull/2347)
* Improve speed of `transformPointCloud/WithNormals()` functions [[#2247]](https://github.com/PointCloudLibrary/pcl/pull/2247)
* Add RGB constructor that takes R, G, and B components [[#2329]](https://github.com/PointCloudLibrary/pcl/pull/2329)
* **[abi]** Improved docstrings and error messages [[#2300]](https://github.com/PointCloudLibrary/pcl/pull/2300)
* **[abi]** Modified `GlasbeyLUT` indexing type to `size_t` [[#2297]](https://github.com/PointCloudLibrary/pcl/pull/2297)
* Add GASDSignatures to `PCL_POINT_TYPES` and `PCL_FEATURE_POINTTYPES` macros. [[#2295]](https://github.com/PointCloudLibrary/pcl/pull/2295)
* [PARSE] Constness of the API [[#2224]](https://github.com/PointCloudLibrary/pcl/pull/2224)
* Fix two "unreachable code" warnings in `pca.hpp` [[#2219]](https://github.com/PointCloudLibrary/pcl/pull/2219)
* Fix covariance calculation in PCA [[#2130]](https://github.com/PointCloudLibrary/pcl/pull/2130)
* **[abi][deprecation]** Provide proper EuclideanClusterComparator method depreciation. New Pragma macro. New Deprecated type. [[#2096]](https://github.com/PointCloudLibrary/pcl/pull/2096)
* **[removal]** Remove deprecated ros headers [[#2075]](https://github.com/PointCloudLibrary/pcl/pull/2075)
* Suppress (maybe) uninitialized warning [[#2073]](https://github.com/PointCloudLibrary/pcl/pull/2073)
* **[removal]** Removal of PCA deprecated constructor [[#2070]](https://github.com/PointCloudLibrary/pcl/pull/2070)
* [gcc] fixes -Wimplicit-fallthrough: common/io.h [[#2041]](https://github.com/PointCloudLibrary/pcl/pull/2041)
* Include pcl/point_cloud.h and pcl/point_types.h headers. [[#1962]](https://github.com/PointCloudLibrary/pcl/pull/1962)
* Add test for macro _USE_MATH_DEFINES. [[#1956]](https://github.com/PointCloudLibrary/pcl/pull/1956)
* instantiate: remove duplicate macro definition. Fixes #1924. [[#1925]](https://github.com/PointCloudLibrary/pcl/pull/1925)

#### `libpcl_cuda:`

* add support for latest Turing gpu and cuda 10 [[#2560]](https://github.com/PointCloudLibrary/pcl/pull/2560)
* Fix compilation issues with CUDA 9.1 [[#2212]](https://github.com/PointCloudLibrary/pcl/pull/2212)
* Fix some CUDA 9 related errors [[#2181]](https://github.com/PointCloudLibrary/pcl/pull/2181)
* Added CUDA compute capability 5.3 [[#1929]](https://github.com/PointCloudLibrary/pcl/pull/1929)

#### `libpcl_features:`

*  Solve issues with failing features tests [[#2544]](https://github.com/PointCloudLibrary/pcl/pull/2544)
* Update the OpenMP implementations of normal and FPFH estimation [[#2278]](https://github.com/PointCloudLibrary/pcl/pull/2278)
* Make `MomentOfInertia` instantiations consistent with the rest of the library [[#2266]](https://github.com/PointCloudLibrary/pcl/pull/2266)
* Docstring corrections [[#2143]](https://github.com/PointCloudLibrary/pcl/pull/2143)
* Improve Doxygen comments for HistogramInterpolationMethod [[#2111]](https://github.com/PointCloudLibrary/pcl/pull/2111)
* **[new-feature]** Add GASD global point cloud descriptor [[#1652]](https://github.com/PointCloudLibrary/pcl/pull/1652)
* Suppress (maybe) uninitialized warning [[#2073]](https://github.com/PointCloudLibrary/pcl/pull/2073)
* **[new-feature]** FLARELocalReferenceFrameEstimation class added [[#1571]](https://github.com/PointCloudLibrary/pcl/pull/1571)
* fix missing include file: from_meshes.h is using pcl::Vertices in it [[#2009]](https://github.com/PointCloudLibrary/pcl/pull/2009)
* Typo [[#1968]](https://github.com/PointCloudLibrary/pcl/pull/1968)

#### `libpcl_filters:`

* Corrections to CovarianceSampling class and corresponding test [[#2512]](https://github.com/PointCloudLibrary/pcl/pull/2512)
* Add the missing const modifier in `Filter::getRemovedIndices`. [[#2523]](https://github.com/PointCloudLibrary/pcl/pull/2523)
* Add const modifiers to getters of pcl::PassThrough [[#2524]](https://github.com/PointCloudLibrary/pcl/pull/2524)
* Add const specifiers for getters in VoxelGrid. [[#2526]](https://github.com/PointCloudLibrary/pcl/pull/2526)
* Copy the pose info from the input cloud to the output cloud in NaN removal functions [[#2522]](https://github.com/PointCloudLibrary/pcl/pull/2522)
* Fix misc. typos in tutorials and docstrings [[#2529]](https://github.com/PointCloudLibrary/pcl/pull/2529)
* Fix various errors and typos in the docstrings and tutorials [[#2486]](https://github.com/PointCloudLibrary/pcl/pull/2486)
* Add some missing eigen alignment operators [[#2433]](https://github.com/PointCloudLibrary/pcl/pull/2433)
* Add PointNormal to ExtractIndices Instantiate Types [[#2389]](https://github.com/PointCloudLibrary/pcl/pull/2389)
* **[abi]** Improved docstrings and error messages [[#2300]](https://github.com/PointCloudLibrary/pcl/pull/2300)
* Public access to `VoxelGrid` boost pointer. [[#2205]](https://github.com/PointCloudLibrary/pcl/pull/2205)
* Add const qualifiers to getters in `filter_indices.h` [[#2193]](https://github.com/PointCloudLibrary/pcl/pull/2193)
* Avoid huge index jumps in `RandomSample`. Remove `io` dependency from `2d`. [[#2141]](https://github.com/PointCloudLibrary/pcl/pull/2141)
* **[removal]** Removal of deprecated code in filters, io, surface and visualization modules [[#2077]](https://github.com/PointCloudLibrary/pcl/pull/2077)
* Suppress unused parameter warning [[#2074]](https://github.com/PointCloudLibrary/pcl/pull/2074)
* Suppress sign compare warnings [[#2068]](https://github.com/PointCloudLibrary/pcl/pull/2068)
* Transformation Fix for BoxClipper3D [[#1961]](https://github.com/PointCloudLibrary/pcl/pull/1961)
* **[abi]** Fixes remove_indices in UniformSampling [[#1902]](https://github.com/PointCloudLibrary/pcl/pull/1902)
* Inherit StatisticalOutlierRemoval<PCLPointCloud2> from FilterIndices [[#1663]](https://github.com/PointCloudLibrary/pcl/pull/1663)

#### `libpcl_gpu:`

* Remove sm_72 from CUDA 9.0 [[#2567]](https://github.com/PointCloudLibrary/pcl/pull/2567)
* Fix compilation issues with CUDA 9.1 [[#2212]](https://github.com/PointCloudLibrary/pcl/pull/2212)
* Fix compilation error in `gpu_people` code [[#2199]](https://github.com/PointCloudLibrary/pcl/pull/2199)
* Fix some CUDA 9 related errors [[#2181]](https://github.com/PointCloudLibrary/pcl/pull/2181)
* **[abi]** Allow specifying decimation step in convertToTsdfCloud [[#2099]](https://github.com/PointCloudLibrary/pcl/pull/2099)
* Fix the incorrect include directory. [[#2024]](https://github.com/PointCloudLibrary/pcl/pull/2024)
* need to include instantiate.hpp to use PCL_INSTANTIATE [[#1943]](https://github.com/PointCloudLibrary/pcl/pull/1943)
* Added CUDA compute capability 5.3 [[#1929]](https://github.com/PointCloudLibrary/pcl/pull/1929)
* Fix issue #1674 [[#1926]](https://github.com/PointCloudLibrary/pcl/pull/1926)

#### `libpcl_io:`

* Suppress miscelanious warnings [[#2556]](https://github.com/PointCloudLibrary/pcl/pull/2556)
* vtk2mesh: Add parsing support to the new RGBA scalar field added in vtk8 [[#2492]](https://github.com/PointCloudLibrary/pcl/pull/2492)
* Fix various errors and typos in the docstrings and tutorials [[#2486]](https://github.com/PointCloudLibrary/pcl/pull/2486)
* Improved obj file parsing efficiency. Make parsing robust against situations where there are more normals than points. Added unit tests. [[#2450]](https://github.com/PointCloudLibrary/pcl/pull/2450)
* `pcl::PCDReader::readHeader()` change `nr_points` type to `size_t` to avoid possible `int32` overflow [[#2408]](https://github.com/PointCloudLibrary/pcl/pull/2408)
* Fix raw_fallocate for Android and deal with unsupported filesystems. [[#2363]](https://github.com/PointCloudLibrary/pcl/pull/2363)
* Add low_level_io.h to the header list of the io module [[#2356]](https://github.com/PointCloudLibrary/pcl/pull/2356)
* Created header for low level I/O helpers. Fix for `::posix_fallocate` on Mac OSX [[#2354]](https://github.com/PointCloudLibrary/pcl/pull/2354)
* Added warnings when the input data is too large for compressed pcds [[#2323]](https://github.com/PointCloudLibrary/pcl/pull/2323)
* Allocate disk space with posix_fallocate before mmapping. [[#2325]](https://github.com/PointCloudLibrary/pcl/pull/2325)
* Fix cmake warning: Logical block closes with mis-matching arguments [[#2320]](https://github.com/PointCloudLibrary/pcl/pull/2320)
* Added PCL_IO_ENABLE_MAND_LOCKING cmake flag. [[#2315]](https://github.com/PointCloudLibrary/pcl/pull/2315)
* Added missing 8 bytes to compressed binary pcd length. [[#2281]](https://github.com/PointCloudLibrary/pcl/pull/2281)
* Remove useless size check in PLYReader::endHeaderCallback() [[#2246]](https://github.com/PointCloudLibrary/pcl/pull/2246)
* **[behavior]** Fix PLYReader is_dense behavior [[#2133]](https://github.com/PointCloudLibrary/pcl/pull/2133)
* `EnsensoGrabber` `uint` is undefined in Visual studio. [[#2223]](https://github.com/PointCloudLibrary/pcl/pull/2223)
* Add protection from invalid WIDTH values in PCD reader [[#2195]](https://github.com/PointCloudLibrary/pcl/pull/2195)
* `PLYReader` Cast cloud point step as 64-bit integer [[#2161]](https://github.com/PointCloudLibrary/pcl/pull/2161)
* `OpenNI2Device` Add device sensor check for IR and depth modesetting [[#2150]](https://github.com/PointCloudLibrary/pcl/pull/2150)
* Adds a check for when CreateFileMappingA fails [[#2146]](https://github.com/PointCloudLibrary/pcl/pull/2146)
* `PCDWriter`changed `toff` to `size_t` in `writeBinaryCompressed` [[#2144]](https://github.com/PointCloudLibrary/pcl/pull/2144)
* Prevent POINTS field parsing before point_step is specified [[#2131]](https://github.com/PointCloudLibrary/pcl/pull/2131)
* Check COUNT value specified in PCD files [[#2126]](https://github.com/PointCloudLibrary/pcl/pull/2126)
* Prevent mmapping more than the original PCD file size [[#2125]](https://github.com/PointCloudLibrary/pcl/pull/2125)
* **[api][deprecation]** Add support pcl::PointXYZRGBA to pcl::VLPGrabber. Deprecate rgb signatures. [[#2102]](https://github.com/PointCloudLibrary/pcl/pull/2102)
* **[removal]** Removal of deprecated code in filters, io, surface and visualization modules [[#2077]](https://github.com/PointCloudLibrary/pcl/pull/2077)
* Suppress strict alias warning [[#2072]](https://github.com/PointCloudLibrary/pcl/pull/2072)
* Suppress unused parameter warnings [[#2067]](https://github.com/PointCloudLibrary/pcl/pull/2067)
* **[abi]** Removed unused member from ply_parser [[#2066]](https://github.com/PointCloudLibrary/pcl/pull/2066)
* Suppress control reaches end of non-void function in io.h [[#2057]](https://github.com/PointCloudLibrary/pcl/pull/2057)
* Modify STRICT_ALIGN because macro expansion w/defined is undefined [[#2043]](https://github.com/PointCloudLibrary/pcl/pull/2043)
* Add necessary boost headers to pcl/io to build in CUDA mode [[#2025]](https://github.com/PointCloudLibrary/pcl/pull/2025)
* Fix MSVC compile issue related with ssize_t [[#2027]](https://github.com/PointCloudLibrary/pcl/pull/2027)
* Adds in-memory PCD serialization/deserialization; de-duplicates PCDReader::readHeader(). (take #2) [[#1986]](https://github.com/PointCloudLibrary/pcl/pull/1986)

#### `libpcl_kdtree:`

* Consistent ptr typedefs for kd tree flann [[#1607]](https://github.com/PointCloudLibrary/pcl/pull/1607)

#### `libpcl_keypoints:`

* Add `TrajkovicKeypoint2D/3D` to CMake build [[#2179]](https://github.com/PointCloudLibrary/pcl/pull/2179)

#### `libpcl_ml:`

* Fix installation rules for ml module [[#2192]](https://github.com/PointCloudLibrary/pcl/pull/2192)

#### `libpcl_octree:`

* **[behavior]** Reverse octree's depth first iterator order [[#2332]](https://github.com/PointCloudLibrary/pcl/pull/2332)
* Fix various errors and typos in the docstrings and tutorials [[#2486]](https://github.com/PointCloudLibrary/pcl/pull/2486)
* Make test conditions consistent with `OctreePointCloudSearch::boxSearch()` implementation. [[#2457]](https://github.com/PointCloudLibrary/pcl/pull/2457)
* `octree_key.h` suppress GCC 8 ignored-qualifier warning [[#2405]](https://github.com/PointCloudLibrary/pcl/pull/2405)
* **[api][deprecation][new-feature]** Implementation of the iterator 'OctreeLeafNodeBreadthIterator'. [[#2204]](https://github.com/PointCloudLibrary/pcl/pull/2204)
* **[abi][new-feature]** Implementation of the iterator 'OctreeFixedDepthIterator'. [[#1983]](https://github.com/PointCloudLibrary/pcl/pull/1983)
* Octree Iterator begin/end method and added tests [[#2174]](https://github.com/PointCloudLibrary/pcl/pull/2174)
* Remove parametrization of end iterators [[#2168]](https://github.com/PointCloudLibrary/pcl/pull/2168)
* Fix docstrings in octree test [[#2173]](https://github.com/PointCloudLibrary/pcl/pull/2173)
* **[api]** OctreeIterators special member revision [[#2108]](https://github.com/PointCloudLibrary/pcl/pull/2108)
* Remove unused variable from octree_viewer [[#2069]](https://github.com/PointCloudLibrary/pcl/pull/2069)
* Silence compile warning by removing superfluous call to std::max() [[#2014]](https://github.com/PointCloudLibrary/pcl/pull/2014)
* [OCTREE] Add bounding box checks in isVoxelOccupiedAtPoint() and deleteVoxelAtPoint() [[#1976]](https://github.com/PointCloudLibrary/pcl/pull/1976)

#### `libpcl_outofcore:`

* Explictly use mt19937 random generator for boost 1.67. [[#2338]](https://github.com/PointCloudLibrary/pcl/pull/2338)
* Fixed queryBBIncludes_subsample [[#1988]](https://github.com/PointCloudLibrary/pcl/pull/1988)

#### `libpcl_people:`

* Misleading indentation [[#2034]](https://github.com/PointCloudLibrary/pcl/pull/2034)

#### `libpcl_recognition:`

* Relax threshold in Hough3DGrouping test [[#2507]](https://github.com/PointCloudLibrary/pcl/pull/2507)
* Add some missing eigen alignment operators [[#2433]](https://github.com/PointCloudLibrary/pcl/pull/2433)
* Setting the resolution of the occupancy grid [[#2273]](https://github.com/PointCloudLibrary/pcl/pull/2273)
* Inline helper function gcCorrespSorter() [[#2248]](https://github.com/PointCloudLibrary/pcl/pull/2248)
* Misleading indentation [[#2034]](https://github.com/PointCloudLibrary/pcl/pull/2034)

#### `libpcl_registration:`

* Remove std::binary_function from Registration [[#2599]](https://github.com/PointCloudLibrary/pcl/pull/2599)
* Suppress miscelanious warnings [[#2556]](https://github.com/PointCloudLibrary/pcl/pull/2556)
* Relax precision requirements on TransformationEstimationLM test. [[#2497]](https://github.com/PointCloudLibrary/pcl/pull/2497)
* Relax rejector threshold in JointIterativeClosestPoint test. [[#2496]](https://github.com/PointCloudLibrary/pcl/pull/2496)
* Add some missing eigen alignment operators [[#2433]](https://github.com/PointCloudLibrary/pcl/pull/2433)
* Remove explicit initialization of `update_visualizer_` in `Registration`  [[#2423]](https://github.com/PointCloudLibrary/pcl/pull/2423)
* **[removal]** Remove registration module deprecated methods [[#2076]](https://github.com/PointCloudLibrary/pcl/pull/2076)
* Suppress (maybe) uninitialized warning [[#2073]](https://github.com/PointCloudLibrary/pcl/pull/2073)
* Remove unreachable code in DefaultConvergenceCriteria [[#1967]](https://github.com/PointCloudLibrary/pcl/pull/1967)
* **[abi]** Added option to specify translation and rotation convergence deltas in ICP and NDT algorithms. [[#1724]](https://github.com/PointCloudLibrary/pcl/pull/1724)

#### `libpcl_sample_consensus:`

* Revise direction test in SampleConsensusModelParallelLine/RANSAC to be consistent with set tolerance. [[#2491]](https://github.com/PointCloudLibrary/pcl/pull/2491)
* Fix error in SampleConsensusModelLine::isSampleGood [[#2488]](https://github.com/PointCloudLibrary/pcl/pull/2488)
* **[api]** Const-qualify most of the methods in SAC model classes [[#2270]](https://github.com/PointCloudLibrary/pcl/pull/2270)
* **[removal]** Remove deprecated functions and variables from SAC module [[#2071]](https://github.com/PointCloudLibrary/pcl/pull/2071)

#### `libpcl_search:`

* Correct testPoint for organized nearest neighbor search [[#2198]](https://github.com/PointCloudLibrary/pcl/pull/2198)

#### `libpcl_segmentation:`

* Add some missing eigen alignment operators [[#2433]](https://github.com/PointCloudLibrary/pcl/pull/2433)
* Add setter/getter for search method in ConditionalEuclideanClustering [[#2437]](https://github.com/PointCloudLibrary/pcl/pull/2437)
* Increase threshold for expected value in test_non_linear [[#2424]](https://github.com/PointCloudLibrary/pcl/pull/2424)
* Avoid infinite recursion in getPointCloudDifference [[#2402]](https://github.com/PointCloudLibrary/pcl/pull/2402)
* Correct setting of is_dense flag in SegmentDifferences. Deprecate unused parameter in method. [[#2380]](https://github.com/PointCloudLibrary/pcl/pull/2380)
* Dereference shared_ptr, fix for GCC8 [[#2299]](https://github.com/PointCloudLibrary/pcl/pull/2299)
* **[abi][deprecation]** Provide proper EuclideanClusterComparator method depreciation. New Pragma macro. New Deprecated type. [[#2096]](https://github.com/PointCloudLibrary/pcl/pull/2096)
* Removed normal related accessors and types from EuclideanClusterComparator [[#1542]](https://github.com/PointCloudLibrary/pcl/pull/1542)

#### `libpcl_simulation:`

* Add `const` qualifiers to `RangeLikelihood` getters. [[#2411]](https://github.com/PointCloudLibrary/pcl/pull/2411)
* **[api]** Use GLuint rather than size_t to represent OpenGL indices. [[#2238]](https://github.com/PointCloudLibrary/pcl/pull/2238)
* Support both RGB and RGBA colors in mesh loading [[#2036]](https://github.com/PointCloudLibrary/pcl/pull/2036)
* **[abi]** Redo: Simulation: enable returning of organized point clouds [[#1687]](https://github.com/PointCloudLibrary/pcl/pull/1687)
* Simulation: more access to camera parameters [[#1650]](https://github.com/PointCloudLibrary/pcl/pull/1650)

#### `libpcl_surface:`

* Fixed memory leak in Poisson's BSplineData [[#2572]](https://github.com/PointCloudLibrary/pcl/pull/2572)
* Suppress miscelanious warnings [[#2556]](https://github.com/PointCloudLibrary/pcl/pull/2556)
* Add some missing eigen alignment operators [[#2433]](https://github.com/PointCloudLibrary/pcl/pull/2433)
* Make pcl::MovingLeastSquares thread-safe [[#2418]](https://github.com/PointCloudLibrary/pcl/pull/2418)
* **[abi]** Missing pcl::MovingLeastSquaresOMP declaration without /openmp [[#2324]](https://github.com/PointCloudLibrary/pcl/pull/2324)
* **[abi]** Improved docstrings and error messages [[#2300]](https://github.com/PointCloudLibrary/pcl/pull/2300)
* opennurbs: fix `ON_Curve::EvaluatePoint` calculation [[#2185]](https://github.com/PointCloudLibrary/pcl/pull/2185)
* **[removal]** Removal of deprecated code in filters, io, surface and visualization modules [[#2077]](https://github.com/PointCloudLibrary/pcl/pull/2077)
* Suppress (maybe) uninitialized warning [[#2073]](https://github.com/PointCloudLibrary/pcl/pull/2073)
* Suppress sign compare warnings [[#2068]](https://github.com/PointCloudLibrary/pcl/pull/2068)
* Fix incorrect Ptr/ConstPtr typedefs in MovingLeastSquaresOMP [[#2055]](https://github.com/PointCloudLibrary/pcl/pull/2055)
* Replace float indices with Eigen Index [[#2017]](https://github.com/PointCloudLibrary/pcl/pull/2017)
* **[api][deprecation][new-feature]** Add new mls projection method. Deprecated `MovingLeastSquares::setPolynomialFit()`. [[#1960]](https://github.com/PointCloudLibrary/pcl/pull/1960)
* Avoid phantom surfces in marching cubes hoppe [[#1874]](https://github.com/PointCloudLibrary/pcl/pull/1874)
* **[api]** Add ability to cache mls results [[#1952]](https://github.com/PointCloudLibrary/pcl/pull/1952)

#### `libpcl_visualization:`

* Made PCLVisualizerInteractorStyle::CloudActorMapPtr public. [[#2542]](https://github.com/PointCloudLibrary/pcl/pull/2542)
* **[new-feature]** Add Viridis color LUT [[#2420]](https://github.com/PointCloudLibrary/pcl/pull/2420)
* Update sha1 header inclusion for Boost 1.68+ [[#2373]](https://github.com/PointCloudLibrary/pcl/pull/2373)
* Second change VTK version check in addTextureMesh [[#2322]](https://github.com/PointCloudLibrary/pcl/pull/2322)
* Include missing `vtkTexture.h`. Follow up on #2291 [[#2316]](https://github.com/PointCloudLibrary/pcl/pull/2316)
* Change VTK version check in `addTextureMesh` [[#2311]](https://github.com/PointCloudLibrary/pcl/pull/2311)
* Remove depreciated VTK function MapDataArrayToMultiTextureAttribute [[#2291]](https://github.com/PointCloudLibrary/pcl/pull/2291)
* Warn and skip adding normal cloud if it is empty [[#2265]](https://github.com/PointCloudLibrary/pcl/pull/2265)
* **[api]** Fix access specifier in `PointCloudColorHandlerRGBAField` [[#2226]](https://github.com/PointCloudLibrary/pcl/pull/2226)
* Check if color handler returns false [[#2209]](https://github.com/PointCloudLibrary/pcl/pull/2209)
* Handle VTK legacy function (follow up to #2112) [[#2165]](https://github.com/PointCloudLibrary/pcl/pull/2165)
* Travis merge test jobs. Expose VTK include directory for the visualization module. [[#2163]](https://github.com/PointCloudLibrary/pcl/pull/2163)
* Small optimizations and fixes in PCLVisualizer [[#2112]](https://github.com/PointCloudLibrary/pcl/pull/2112)
* Add another variant to `getPointCloudRenderingProperties()`  [[#2142]](https://github.com/PointCloudLibrary/pcl/pull/2142)
* Restrict accepted types in `PCLVisualizer::addLine` signature [[#2134]](https://github.com/PointCloudLibrary/pcl/pull/2134)
* Typo fix in error message. [[#2132]](https://github.com/PointCloudLibrary/pcl/pull/2132)
* Fix add/remove of 3D text without custom viewport [[#2110]](https://github.com/PointCloudLibrary/pcl/pull/2110)
* Add `octree` dependency to `visualization` [[#2115]](https://github.com/PointCloudLibrary/pcl/pull/2115)
* Fix pcd_viewer color handling when invalid fields are present in pcd [[#2105]](https://github.com/PointCloudLibrary/pcl/pull/2105)
* Add pcl visualizer interactor null guards [[#2104]](https://github.com/PointCloudLibrary/pcl/pull/2104)
* Add remove text3d [[#2088]](https://github.com/PointCloudLibrary/pcl/pull/2088)
* Replace auto_ptr with scoped_ptr [[#2037]](https://github.com/PointCloudLibrary/pcl/pull/2037)
* **[removal]** Removal of deprecated code in filters, io, surface and visualization modules [[#2077]](https://github.com/PointCloudLibrary/pcl/pull/2077)
* Suppress sign compare warnings [[#2068]](https://github.com/PointCloudLibrary/pcl/pull/2068)
* **[new-feature]** Add overload to `PCLVisualizer::addText3D()` that allows specifying text orientation [[#2038]](https://github.com/PointCloudLibrary/pcl/pull/2038)
* Add new constructors to PCLVisualizer [[#2004]](https://github.com/PointCloudLibrary/pcl/pull/2004)
* Fix bug in PointCloudGeometryHandlerCustom [[#2001]](https://github.com/PointCloudLibrary/pcl/pull/2001)
* Add missing PCL_EXPORTS in pcl_visualizer.h [[#1995]](https://github.com/PointCloudLibrary/pcl/pull/1995)
* **[abi]** Add accessor for current rendering framerate in PCLVisualizer [[#1974]](https://github.com/PointCloudLibrary/pcl/pull/1974)
* Allow changing LUT properties of a shape actor [[#1668]](https://github.com/PointCloudLibrary/pcl/pull/1668)
* Fixed spelling and grammar errors [[#1959]](https://github.com/PointCloudLibrary/pcl/pull/1959)
* Fixed error in documentation. [[#1957]](https://github.com/PointCloudLibrary/pcl/pull/1957)

#### `PCL Apps:`

* Suppress miscelanious warnings [[#2556]](https://github.com/PointCloudLibrary/pcl/pull/2556)
* Fix 3d_rec_framework compilation error [[#2495]](https://github.com/PointCloudLibrary/pcl/pull/2495)
* Fix compilation issue in point cloud editor. [[#2490]](https://github.com/PointCloudLibrary/pcl/pull/2490)
* `demean_cloud` correct usage message. [[#2443]](https://github.com/PointCloudLibrary/pcl/pull/2443)
* Do not use deprecated method in openni_mls_smoothing app [[#2421]](https://github.com/PointCloudLibrary/pcl/pull/2421)
* add windows.h for includes GL/gl.h; handle cancel for denoiseWidget. [[#2267]](https://github.com/PointCloudLibrary/pcl/pull/2267)
* Add missing dependecy to apps [[#2251]](https://github.com/PointCloudLibrary/pcl/pull/2251)
* Suppress the final set of warnings in pcl apps [[#2082]](https://github.com/PointCloudLibrary/pcl/pull/2082)
* **[abi]** More warning suppression in pcl apps [[#2080]](https://github.com/PointCloudLibrary/pcl/pull/2080)

#### `PCL Docs:`

* Fix misc. typos in tutorials and docstrings [[#2529]](https://github.com/PointCloudLibrary/pcl/pull/2529)
* Fix various errors and typos in the docstrings and tutorials [[#2486]](https://github.com/PointCloudLibrary/pcl/pull/2486)
* Docstring typos' corrections. [[#2449]](https://github.com/PointCloudLibrary/pcl/pull/2449)
* `demean_cloud` correct usage message. [[#2443]](https://github.com/PointCloudLibrary/pcl/pull/2443)
* Set IMAGE_PATH explicitly in Doxygen config [[#2442]](https://github.com/PointCloudLibrary/pcl/pull/2442)
* Switch to using client-based search in Doxygen [[#2391]](https://github.com/PointCloudLibrary/pcl/pull/2391)
* **[api]** Misc. typos (cont.) [[#2215]](https://github.com/PointCloudLibrary/pcl/pull/2215)
* doc: misc. typos [[#2213]](https://github.com/PointCloudLibrary/pcl/pull/2213)
* Add url to API/ABI compatibity report [[#2116]](https://github.com/PointCloudLibrary/pcl/pull/2116)
* Improve Doxygen comments for HistogramInterpolationMethod [[#2111]](https://github.com/PointCloudLibrary/pcl/pull/2111)
* Update organized.h [[#1965]](https://github.com/PointCloudLibrary/pcl/pull/1965)
* Typo [[#1968]](https://github.com/PointCloudLibrary/pcl/pull/1968)
* Fixed spelling and grammar errors [[#1959]](https://github.com/PointCloudLibrary/pcl/pull/1959)
* Fixed error in documentation. [[#1957]](https://github.com/PointCloudLibrary/pcl/pull/1957)

#### `PCL Tutorials:`

* Fix dataset link in conditional euclidean clustering tutorial [[#2546]](https://github.com/PointCloudLibrary/pcl/pull/2546)
* Fix dead links in Walkthrough tutorial [[#2532]](https://github.com/PointCloudLibrary/pcl/pull/2532)
* Simplify explanation of PointXYZ structure in "Writing PCD" tutorial [[#2534]](https://github.com/PointCloudLibrary/pcl/pull/2534)
* Fix misc. typos in tutorials and docstrings [[#2529]](https://github.com/PointCloudLibrary/pcl/pull/2529)
* Fix a dead link to Radu Rusu's dissertation in the tutorial. [[#2508]](https://github.com/PointCloudLibrary/pcl/pull/2508)
* Fix various errors and typos in the docstrings and tutorials [[#2486]](https://github.com/PointCloudLibrary/pcl/pull/2486)
* Fix link to Institut Maupertuis's ensenso_extrinsic_calibration repo [[#2447]](https://github.com/PointCloudLibrary/pcl/pull/2447)
* Add settings for hypothesis verification [[#2274]](https://github.com/PointCloudLibrary/pcl/pull/2274)
* Fix ICP tutorial [[#2244]](https://github.com/PointCloudLibrary/pcl/pull/2244)
* Fix error in example code for estimate set of surface for a subset of points in the input dataset [[#2203]](https://github.com/PointCloudLibrary/pcl/pull/2203)
* Update message about legacy point cloud types in tutorial [[#2175]](https://github.com/PointCloudLibrary/pcl/pull/2175)
* Add descriptor unpacking to GASD tutorial [[#2167]](https://github.com/PointCloudLibrary/pcl/pull/2167)
* Fix convert to `Eigen::Map<const Eigen::Vector3f>` from Normal of `pcl::PointXYZINormal` [[#2128]](https://github.com/PointCloudLibrary/pcl/pull/2128)
* Fix the tutorial qt_visualizer compilation issue: qt4 -> qt5. [[#2051]](https://github.com/PointCloudLibrary/pcl/pull/2051)
* Fix several documentation typos [[#2020]](https://github.com/PointCloudLibrary/pcl/pull/2020)
* Replace literal include of wrong CMakeLists file with correct script [[#1971]](https://github.com/PointCloudLibrary/pcl/pull/1971)
* Update Ensenso tutorial for Ensenso X devices [[#1933]](https://github.com/PointCloudLibrary/pcl/pull/1933)

#### `PCL Examples:`

* Suppress strict alias warning [[#2072]](https://github.com/PointCloudLibrary/pcl/pull/2072)
* Suppress (maybe) uninitialized warning [[#2073]](https://github.com/PointCloudLibrary/pcl/pull/2073)
* Fix CPC/LCCP segmentation examples for VTK 7.1 [[#2063]](https://github.com/PointCloudLibrary/pcl/pull/2063)

#### `PCL Tests:`

* Corrections to Windows unit tests. [[#2596]](https://github.com/PointCloudLibrary/pcl/pull/2596)
* Relax eigen22f test criteria [[#2553]](https://github.com/PointCloudLibrary/pcl/pull/2553)
*  Solve issues with failing features tests [[#2544]](https://github.com/PointCloudLibrary/pcl/pull/2544)
* Relax requirements in eigen22d test. Always provide a normalized result in `pcl::transformPlane`. [[#2503]](https://github.com/PointCloudLibrary/pcl/pull/2503)
* Enable tests_2d and tests_io in AppVeyor. [[#2505]](https://github.com/PointCloudLibrary/pcl/pull/2505)
* Relax threshold in Hough3DGrouping test [[#2507]](https://github.com/PointCloudLibrary/pcl/pull/2507)
* Relax precision requirements on TransformationEstimationLM test. [[#2497]](https://github.com/PointCloudLibrary/pcl/pull/2497)
* Relax rejector threshold in JointIterativeClosestPoint test. [[#2496]](https://github.com/PointCloudLibrary/pcl/pull/2496)
* vtk2mesh: Add parsing support to the new RGBA scalar field added in vtk8 [[#2492]](https://github.com/PointCloudLibrary/pcl/pull/2492)
* Revise direction test in SampleConsensusModelParallelLine/RANSAC to be consistent with set tolerance. [[#2491]](https://github.com/PointCloudLibrary/pcl/pull/2491)
* Make test conditions consistent with `OctreePointCloudSearch::boxSearch()` implementation. [[#2457]](https://github.com/PointCloudLibrary/pcl/pull/2457)
* Increase threshold for expected value in test_non_linear [[#2424]](https://github.com/PointCloudLibrary/pcl/pull/2424)
* Replace floating point numerals when using `boost::posix_time`. Fix compatibility with Boost 1.67. [[#2422]](https://github.com/PointCloudLibrary/pcl/pull/2422)
* Cleanup and improve unit test coverage for transformPointCloud functions [[#2245]](https://github.com/PointCloudLibrary/pcl/pull/2245)
* Fixes and new assertion macro in "pcl_tests.h" [[#2237]](https://github.com/PointCloudLibrary/pcl/pull/2237)
* Add new gtest path [[#1920]](https://github.com/PointCloudLibrary/pcl/pull/1920)

#### `PCL Tools:`

* Allow the `pcl_uniform_sampling` tool to deal with several formats (PCD, PLY and VTK) as input or output point cloud [[#2348]](https://github.com/PointCloudLibrary/pcl/pull/2348)
* mesh_sampling tool: Add support for colors [[#2257]](https://github.com/PointCloudLibrary/pcl/pull/2257)
* Error message on non-recognized feature names [[#2178]](https://github.com/PointCloudLibrary/pcl/pull/2178)
* Suppress sign compare warnings [[#2068]](https://github.com/PointCloudLibrary/pcl/pull/2068)
* [OCTREE] Compute accurately the centroids of octree in 'pcl_octree_viewer' [[#1981]](https://github.com/PointCloudLibrary/pcl/pull/1981)
* **[api][deprecation][new-feature]** Add new mls projection method. Deprecated `MovingLeastSquares::setPolynomialFit()`. [[#1960]](https://github.com/PointCloudLibrary/pcl/pull/1960)
* [OCTREE] Fix pcl_octree_viewer [[#1973]](https://github.com/PointCloudLibrary/pcl/pull/1973)
* [OCTREE] Remove a useless field in octree_viewer. [[#1980]](https://github.com/PointCloudLibrary/pcl/pull/1980)

#### `CI:`

* Disable Travis email notifications. Update PCL logo endpoint. [[#2535]](https://github.com/PointCloudLibrary/pcl/pull/2535)
* Migrate Travis to the new travis-ci.com platform [[#2538]](https://github.com/PointCloudLibrary/pcl/pull/2538)
* Enable tests_2d and tests_io in AppVeyor. [[#2505]](https://github.com/PointCloudLibrary/pcl/pull/2505)
* Fix docs on Travis CI. [[#2441]](https://github.com/PointCloudLibrary/pcl/pull/2441)
* Disable SSE flags in AppVeyor. [[#2438]](https://github.com/PointCloudLibrary/pcl/pull/2438)
* Split (yet again) Travis test job into two and tweak timings in building apps [[#2182]](https://github.com/PointCloudLibrary/pcl/pull/2182)
* **[new-feature]** Enable Global Tests on Windows CI [[#2137]](https://github.com/PointCloudLibrary/pcl/pull/2137)
* Travis merge test jobs. Expose VTK include directory for the visualization module. [[#2163]](https://github.com/PointCloudLibrary/pcl/pull/2163)
* Remove unnecessary PPAs and packages from Travis [[#2153]](https://github.com/PointCloudLibrary/pcl/pull/2153)
* AppVeyor - Change to simple style of specify triplet [[#2135]](https://github.com/PointCloudLibrary/pcl/pull/2135)
* Initial Appveyor CI integration [[#2083]](https://github.com/PointCloudLibrary/pcl/pull/2083)
* Change Travis to use pip3 for installing sphinx [[#2124]](https://github.com/PointCloudLibrary/pcl/pull/2124)
* [TRAVIS] Enable the build of apps. [[#2012]](https://github.com/PointCloudLibrary/pcl/pull/2012)
* [TRAVIS] Enable the build of tools. [[#2007]](https://github.com/PointCloudLibrary/pcl/pull/2007)
* Disable tools build in CI. [[#2003]](https://github.com/PointCloudLibrary/pcl/pull/2003)


## *= 1.8.1 (08.08.2017) =*

* Replaced `make_shared` invocations on aligned allocated vars
  [[#1405]](https://github.com/PointCloudLibrary/pcl/pull/1405)
* Created an issue template for bug reporting
  [[#1637]](https://github.com/PointCloudLibrary/pcl/pull/1637)
* PCL logo image is now locally available
  [[#1677]](https://github.com/PointCloudLibrary/pcl/pull/1677)
* Updated the Windows all in one installer for MSVC15
  [[#1762]](https://github.com/PointCloudLibrary/pcl/pull/1762)
* Added compile support to VTK 7.1
  [[#1770]](https://github.com/PointCloudLibrary/pcl/pull/1770)
* Fixed badges markup in README.md
  [[#1873]](https://github.com/PointCloudLibrary/pcl/pull/1873)
* Replaced C-style `sqrtf` with `std::sqrt`
  [[#1901]](https://github.com/PointCloudLibrary/pcl/pull/1901)

### `CMake:`

* Tweaks to PCL_DEFINITIONS behavior (to be **deprecated** in future
  versions)
  [[#1478]](https://github.com/PointCloudLibrary/pcl/pull/1478)
* VTK directory can now be manually specified during configuration
  [[#1605]](https://github.com/PointCloudLibrary/pcl/pull/1605)
* Updated the find Boost cmake macro to support the latest versions plus 
  exported definitions now give priority to finding the same Boost version 
  PCL was compiled with.
  [[#1630]](https://github.com/PointCloudLibrary/pcl/pull/1630)
* Corrected PCL_ROOT in PCLConfig.cmake
  [[#1678]](https://github.com/PointCloudLibrary/pcl/pull/1678)
* Removed automatic override of VTK_LIBRARIES
  [[#1760]](https://github.com/PointCloudLibrary/pcl/pull/1760)
* Updated find boost versions
  [[#1788]](https://github.com/PointCloudLibrary/pcl/pull/1788)
  [[#1855]](https://github.com/PointCloudLibrary/pcl/pull/1855)
  [[#1856]](https://github.com/PointCloudLibrary/pcl/pull/1856)
* Updated CUDA compute capabilities
  [[#1789]](https://github.com/PointCloudLibrary/pcl/pull/1789)
* Extend linking of `delayimp.lib` to all MSVC version
  [[#1823]](https://github.com/PointCloudLibrary/pcl/pull/1823)
* Removal of `MSVCxx` variables
  [[#1830]](https://github.com/PointCloudLibrary/pcl/pull/1830)
* Fixed path link to Documents of Windows Start-Menu
  [[#1857]](https://github.com/PointCloudLibrary/pcl/pull/1857)
* Fixed CPack for Documents
  [[#1858]](https://github.com/PointCloudLibrary/pcl/pull/1858)
* Fixed bug present when Ensenso SDK path included spaces
  [[#1875]](https://github.com/PointCloudLibrary/pcl/pull/1875)
* `-D_FORCE_INLINES` definition added for CUDA targets to prevent
  issues between old versions of the CUDA Toolkit and new versions
  of gcc
  [[#1900]](https://github.com/PointCloudLibrary/pcl/pull/1900)
* Implemented new versioning scheme for PCL, employing the suffix
  `-dev` in between releases.
  [[#1905]](https://github.com/PointCloudLibrary/pcl/pull/1905)
* Corrected search paths for Eigen on Windows
  [[#1912]](https://github.com/PointCloudLibrary/pcl/pull/1912)
* SSE definitions are now exported and cleanup of Eigen's
  definitions
  [[#1917]](https://github.com/PointCloudLibrary/pcl/pull/1917)
* Added support to dynamic linking against FLANN on Windows
  [[#1919]](https://github.com/PointCloudLibrary/pcl/pull/1919)
* Add new search path for GTest to the finder script
  [[#1920]](https://github.com/PointCloudLibrary/pcl/pull/1920)
* Fix discovery of PCL deployed out of install path
  [[#1923]](https://github.com/PointCloudLibrary/pcl/pull/1923)


### `libpcl_2d:`

* Removed the non-free lena-grayscale-png image :( 
  [[#1676]](https://github.com/PointCloudLibrary/pcl/pull/1676)
* 2d library is no longer generated since it contained no symbols
  [[#1679]](https://github.com/PointCloudLibrary/pcl/pull/1679)

### `libpcl_common:`

* Changed default alpha value to 255 on all RGB(A) point types
  [[#1385]](https://github.com/PointCloudLibrary/pcl/pull/1385)
* Fixed an issue preventing aligned memory allocation on 32-bit Windows
  systems
  [[#1665]](https://github.com/PointCloudLibrary/pcl/pull/1665)
* Fixed compile error on test_common on MSVC
  [[#1689]](https://github.com/PointCloudLibrary/pcl/pull/1689)
* Fixed parallel plane test condition on `pcl::planeWithPlaneIntersection`
  [[#1698]](https://github.com/PointCloudLibrary/pcl/pull/1698)
* Fixed endless loop condition in `compute3DCentroid`
  [[#1704]](https://github.com/PointCloudLibrary/pcl/pull/1704)
* `toPCLPointCloud2` is not resilient to an empty pointcloud input
  [[#1723]](https://github.com/PointCloudLibrary/pcl/pull/1723)
* Normal accumulator `normalized()` is now resilient to a 0 filled vector
  [[#1728]](https://github.com/PointCloudLibrary/pcl/pull/1728)
* Defined additional types in `PointCloud` to ensure STL container
  compatibility
  [[#1741]](https://github.com/PointCloudLibrary/pcl/pull/1741)
* Aligned malloc now works on Android as well
  [[#1774]](https://github.com/PointCloudLibrary/pcl/pull/1774)
* Added missing include to boost shared_ptr in vertices
  [[#1790]](https://github.com/PointCloudLibrary/pcl/pull/1790)
* Prevent incorrect copy of adjacent point in `fromPCLPointCloud2()`
  [[#1813]](https://github.com/PointCloudLibrary/pcl/pull/1813)
* Restored `Eigen::umeyama` for Eigen 3.3+
  [[#1820]](https://github.com/PointCloudLibrary/pcl/pull/1820)
  [[#1887]](https://github.com/PointCloudLibrary/pcl/pull/1887)
* Fixed type in deprecation messages
  [[#1878]](https://github.com/PointCloudLibrary/pcl/pull/1878)
* Improved support for mingw aligned allocation
  [[#1904]](https://github.com/PointCloudLibrary/pcl/pull/1904)
* Added test for macro `_USE_MATH_DEFINES` to avoid warnings
  [[#1956]](https://github.com/PointCloudLibrary/pcl/pull/1956)

### `libpcl_cuda:`

* Fixed macro definitions for the Windows platform
  [[#1568]](https://github.com/PointCloudLibrary/pcl/pull/1568)

### `libpcl_features:`

* NormalEstimation[OMP] and FPFHEstimation[OMP] are now instantiated for
  the same types as the non OMP variants.
  [[#1642]](https://github.com/PointCloudLibrary/pcl/pull/1642)
* Prevention of the addition of duplicate keys in `PFHEstimation`
  [[#1701]](https://github.com/PointCloudLibrary/pcl/pull/1701)
* Bug fixes in OUR-CVFH
  [[#1827]](https://github.com/PointCloudLibrary/pcl/pull/1827)
* Fixed incorrect initialization of SHOT
  [[#1859]](https://github.com/PointCloudLibrary/pcl/pull/1859)
  [[#1876]](https://github.com/PointCloudLibrary/pcl/pull/1876)

### `libpcl_filters:`

* ExtractIndices filter now aborts prematurely and prints error verbose
  in case it detects an index which exceeds the size on the input data
  [[#1670]](https://github.com/PointCloudLibrary/pcl/pull/1670)
* Potential reduction of computational time of `ModelOutlierRemoval`
  [[#1735]](https://github.com/PointCloudLibrary/pcl/pull/1735)
* Improved code readability in CropBox
  [[#1817]](https://github.com/PointCloudLibrary/pcl/pull/1817)

### `libpcl_gpu:`

* Added support to NVidia Pascal GPUs
  [[#1824]](https://github.com/PointCloudLibrary/pcl/pull/1824)
* Fixed compilation error in KinfuLS
  [[#1872]](https://github.com/PointCloudLibrary/pcl/pull/1872)
* Fixed CUDA architecture check
  [[#1872]](https://github.com/PointCloudLibrary/pcl/pull/1872)

### `libpcl_io:`

* RGB values are now always saved as uint32 on PCD files
  [[#1385]](https://github.com/PointCloudLibrary/pcl/pull/1385)
* Fixed find RealSense macro and compilation error with RealSenseGrabber 
  on Windows
  [[#1560]](https://github.com/PointCloudLibrary/pcl/pull/1560)
* Unified verbose on OctreePointCloudCompression
  [[#1569]](https://github.com/PointCloudLibrary/pcl/pull/1569)
* Improved performance on saving PLY, OBJ and VTK files
  [[#1580]](https://github.com/PointCloudLibrary/pcl/pull/1580)
* Added support to the transparency property `Tr` on pcl::MTLReader
  and fixed issue with parsing of the material's properties.
  [[#1599]](https://github.com/PointCloudLibrary/pcl/pull/1599)
* Fixed function signature mismatch in auto_io
  [[#1625]](https://github.com/PointCloudLibrary/pcl/pull/1625)
* Fix `ASCIIReader::setInputFields` interface
  [[#1690]](https://github.com/PointCloudLibrary/pcl/pull/1690)
* Adopted pcl_isnan in test_buffers to prevent compilation problems on
  MSVC12
  [[#1694]](https://github.com/PointCloudLibrary/pcl/pull/1694)
* Fixed incorrect laser number test condition in VLP Grabber
  [[#1697]](https://github.com/PointCloudLibrary/pcl/pull/1697)
* Fixed bug verbose output of compression statistics
  [[#1749]](https://github.com/PointCloudLibrary/pcl/pull/1749)
* Fixed a bug in the parsing of PLY headers
  [[#1750]](https://github.com/PointCloudLibrary/pcl/pull/1750)
* Replacement of `boost::math::isnan` by `pcl_isnan`
  [[#1766]](https://github.com/PointCloudLibrary/pcl/pull/1766)
* Binary files written by `PCDWriter` now have the same permissions
  as the ASCII ones
  [[#1779]](https://github.com/PointCloudLibrary/pcl/pull/1779)
* Fixed ODR violation when compiling with both OpenNI and OpenNI2
  [[#1818]](https://github.com/PointCloudLibrary/pcl/pull/1818)
* PLYReader now also accepts the property `vertex_index`
  [[#1847]](https://github.com/PointCloudLibrary/pcl/pull/1847)
* Fixed bug in return value of `pcl_converter`
  [[#1903]](https://github.com/PointCloudLibrary/pcl/pull/1903)


### `libpcl_keypoints:`

* Fixed memory leak in `ISSKeypoint3D`
  [[#1815]](https://github.com/PointCloudLibrary/pcl/pull/1815)

### `libpcl_octree:`

* Fixed unexpected octree boundaries' reduction
  [[#1532]](https://github.com/PointCloudLibrary/pcl/pull/1532)
  [[#1906]](https://github.com/PointCloudLibrary/pcl/pull/1906)
* Fixed octree precompilation mechanism
  [[#1639]](https://github.com/PointCloudLibrary/pcl/pull/1639)
  [[#1916]](https://github.com/PointCloudLibrary/pcl/pull/1916)
* Fixed invalid cast in `OctreePointCloudVoxelCentroid`
  [[#1700]](https://github.com/PointCloudLibrary/pcl/pull/1700)

### `libpcl_recognition:`

* LineMOD bug fixes
  [[#1835]](https://github.com/PointCloudLibrary/pcl/pull/1835)
* Removed redundant definition of point types
  [[#1836]](https://github.com/PointCloudLibrary/pcl/pull/1836)

### `libpcl_registration:`

* Fixed GICP behavior when a guess is provided
  [[#989]](https://github.com/PointCloudLibrary/pcl/pull/989)
* Fixed compilation issues in NDT 2D with Eigen 3.3
  [[#1821]](https://github.com/PointCloudLibrary/pcl/pull/1821)
* NDT 2D state is now properly initialized
  [[#1731]](https://github.com/PointCloudLibrary/pcl/pull/1731)

### `libpcl_sample_consensus:`

* Improved error verbose in
  `SampleConsensusModelPlane::optimizeModelCoefficient`
  [[#1811]](https://github.com/PointCloudLibrary/pcl/pull/1811)

### `libpcl_segmentation:`

* Fixed bug in organized multiplane segmentation refine function where label 
  indices were not being updated correctly
  [[#1502]](https://github.com/PointCloudLibrary/pcl/pull/1502)
* Corrected function signature in lccp segmentation
  [[#1761]](https://github.com/PointCloudLibrary/pcl/pull/1761)
* Fixed bug in boundary checking in Organized Connected Component
  Segmentation
  [[#1800]](https://github.com/PointCloudLibrary/pcl/pull/1800)
* Clarified documentation in Super Voxel Clustering
  [[#1804]](https://github.com/PointCloudLibrary/pcl/pull/1804)
* Fixed bug causing unnecessary computation in Region Growing
  [[#1882]](https://github.com/PointCloudLibrary/pcl/pull/1882)

### `libpcl_surface:`

* Double pass mean and covariance estimation are now employed in
  `ConcaveHull::reconstruct`
  [[#1567]](https://github.com/PointCloudLibrary/pcl/pull/1567)
* GP3 bug fixes
  [[#1850]](https://github.com/PointCloudLibrary/pcl/pull/1850)
  [[#1879]](https://github.com/PointCloudLibrary/pcl/pull/1879)
* Fixed buggy index cast in bilateral upsampling
  [[#1914]](https://github.com/PointCloudLibrary/pcl/pull/1914)


### `libpcl_visualization:`

* Fixed bug in addPointCloudNormals which was ignoring view point information
  [[#1504]](https://github.com/PointCloudLibrary/pcl/pull/1504)
* Fixed bug camera FOV computation in PCLVisualizerInteractorStyle
  [[#1611]](https://github.com/PointCloudLibrary/pcl/pull/1611)
* Fixed a MSVC compilation error with the colormap LUT
  [[#1635]](https://github.com/PointCloudLibrary/pcl/pull/1635)
* Abort prematurely when the camera file cannot be opened on
  `PCLVisualizerInteractorStyle`
  [[#1776]](https://github.com/PointCloudLibrary/pcl/pull/1776)
* Fix to `addText3D`
  [[#1805]](https://github.com/PointCloudLibrary/pcl/pull/1805)
* Added some exception guards in OpenNI and OpenNI2 Viewer tools
  [[#1862]](https://github.com/PointCloudLibrary/pcl/pull/1862)

### `PCL Apps:`

* Fixed bug in point cloud editor app which allowed to select points behind
  the camera
  [[#1539]](https://github.com/PointCloudLibrary/pcl/pull/1539)
* Explicitly define OpenGL headers to fix build on Ubuntu arm64
  [[#1715]](https://github.com/PointCloudLibrary/pcl/pull/1715)
* Replaced the use of `slot` and `signals` keywords in QT apps for
  their `Q_*` counterparts to present name clashes with Boost Signals
  [[#1898]](https://github.com/PointCloudLibrary/pcl/pull/1898)

### `PCL Docs:`

* Fix docs generation on Windows
  [[#1717]](https://github.com/PointCloudLibrary/pcl/pull/1717)

### `PCL Tests:`

* Modularized the build of unit tests.
  [[#1768]](https://github.com/PointCloudLibrary/pcl/pull/1768)
* Removed invalid test condition on test_common_io
  [[#1884]](https://github.com/PointCloudLibrary/pcl/pull/1884)

### `PCL Tools:`

* `mesh2pcd` has now an option to explicitly disable visualization
  [[#1768]](https://github.com/PointCloudLibrary/pcl/pull/1768)
* `mesh_sampling` has now an option to explicitly disable visualization
  [[#1769]](https://github.com/PointCloudLibrary/pcl/pull/1769)
* Mesh sampling now has an option to include normal information
  [[#1795]](https://github.com/PointCloudLibrary/pcl/pull/1795)
* Fixed incorrect return value in pcl_converter
  [[#1903]](https://github.com/PointCloudLibrary/pcl/pull/1903)

### `PCL Tutorials:`

* Fixed a crash in the pcl_visualizer tutorial triggered in interactive
  mode
  [[#1631]](https://github.com/PointCloudLibrary/pcl/pull/1631)
* Fixed hyperlink in narf keypoint extraction
  [[#1777]](https://github.com/PointCloudLibrary/pcl/pull/1777)
* Typo corrections in random sample consensus
  [[#1865]](https://github.com/PointCloudLibrary/pcl/pull/1865)
* Updated matrix transform tutorial and added cube.ply mesh
  [[#1894]](https://github.com/PointCloudLibrary/pcl/pull/1894)
  [[#1897]](https://github.com/PointCloudLibrary/pcl/pull/1897)
* Updated Ensenso tutorial for Ensenso X devices
  [[#1933]](https://github.com/PointCloudLibrary/pcl/pull/1933)

### `CI:`

* Applied a workaround to a regression bug introduced by doxylink
  in the docs build job
  [[#1784]](https://github.com/PointCloudLibrary/pcl/pull/1784)
* Build jobs refactoring
  [[#1768]](https://github.com/PointCloudLibrary/pcl/pull/1768)
* Enable ccache to speed up builds in CI
  [[#1892]](https://github.com/PointCloudLibrary/pcl/pull/1892)

## *= 1.8.0 (14.06.2016) =*

* Added missing `Eigen::aligned_allocator` in vectors and maps that contain
  vectorizable Eigen where appropriate
  [[#1034]](https://github.com/PointCloudLibrary/pcl/pull/1034)
  [[#1052]](https://github.com/PointCloudLibrary/pcl/pull/1052)
  [[#1068]](https://github.com/PointCloudLibrary/pcl/pull/1068)
  [[#1182]](https://github.com/PointCloudLibrary/pcl/pull/1182)
  [[#1497]](https://github.com/PointCloudLibrary/pcl/pull/1497)
* Fixed compilation errors/warning when compiling in C++11 mode
  [[#1179]](https://github.com/PointCloudLibrary/pcl/pull/1179)
* Added a configuration option to choose between Qt4 and Qt5; the default is
  changed to be Qt5
  [[#1217]](https://github.com/PointCloudLibrary/pcl/pull/1217)
* Improved compatibility with recent Eigen versions
  [[#1261]](https://github.com/PointCloudLibrary/pcl/pull/1261)
  [[#1298]](https://github.com/PointCloudLibrary/pcl/pull/1298)
  [[#1316]](https://github.com/PointCloudLibrary/pcl/pull/1316)
  [[#1369]](https://github.com/PointCloudLibrary/pcl/pull/1369)
* Added support for VTK compiled with OpenGL2 backend (was introduced in VTK
  6.3, became default in VTK 7.0)
  [[#1534]](https://github.com/PointCloudLibrary/pcl/pull/1534)

### `libpcl_common:`

* Added `copy_all_fields` option to the family of transformPointCloudXXX()
  functions
  [[#805]](https://github.com/PointCloudLibrary/pcl/pull/805)
* Added a color lookup table consisting of 256 colors structured in a maximally
  discontinuous manner (Glasbey colors)
  [[#849]](https://github.com/PointCloudLibrary/pcl/pull/849)
* Added a helper class `EventFrequency` to measure frequency of a certain event
  [[#850]](https://github.com/PointCloudLibrary/pcl/pull/850)
* Added a new `UniqueShapeContext1960` point type
  [[#856]](https://github.com/PointCloudLibrary/pcl/pull/856)
* Added a function `transformPointWithNormal()`
  [[#908]](https://github.com/PointCloudLibrary/pcl/pull/908)
* Fixed index-out-of-range error in `copyPointCloud()` for empty clouds
  [[#933]](https://github.com/PointCloudLibrary/pcl/pull/933)
* Fixed errors when compiling library with Boost 1.56 and Qt4
  [[#938]](https://github.com/PointCloudLibrary/pcl/pull/938)
* Created a new point type `PointXYZLNormal` with position, normal, and label
  fields
  [[#962]](https://github.com/PointCloudLibrary/pcl/pull/962)
* Created a new point type `PointDEM` to represent Digital Elevation Maps
  [[#1021]](https://github.com/PointCloudLibrary/pcl/pull/1021)
* Fixed angle convexity calculation for parallel and anti-parallel normals,
  where a rounding error occasionally caused NaN angles in `getAngle3D()`
  [[#1035]](https://github.com/PointCloudLibrary/pcl/pull/1035)
* Fixed undefined behavior when using multiple instances of `TimeTrigger`
  [[#1074]](https://github.com/PointCloudLibrary/pcl/pull/1074)
* Fixed starvation bug in `TimeTrigger` on Windows with Boost < 1.55
  [[#1086]](https://github.com/PointCloudLibrary/pcl/pull/1086)
* Removed unnecessary mutex locking in `TimeTrigger::registerCallback`
  [[#1087]](https://github.com/PointCloudLibrary/pcl/pull/1087)
* Updated PCL exception types to have nothrow copy constructor and copy
  assignment operator
  [[#1119]](https://github.com/PointCloudLibrary/pcl/pull/1119)
* Fixed a bug with `PCA` not triggering recomputation when input indices are
  changed
  [[#1167]](https://github.com/PointCloudLibrary/pcl/pull/1167)
* Added missing XYZ coordinate copying in `PointXYZRGBAtoXYZHSV` and
  `PointXYZRGBtoXYZHSV` conversion functions
  [[#1273]](https://github.com/PointCloudLibrary/pcl/pull/1273)
* Added `const` qualifiers where appropriate in point type conversion functions
  [[#1274]](https://github.com/PointCloudLibrary/pcl/pull/1274)
* Fixed assignment operator in `PCA`
  [[#1328]](https://github.com/PointCloudLibrary/pcl/pull/1328)
* Added `PointWithRange` to the list of core point types
  [[#1352]](https://github.com/PointCloudLibrary/pcl/pull/1352)
* Fixed a bug in `getMaxDistance()` (this affected computation of OUR-CVFH
  features)
  [[#1449]](https://github.com/PointCloudLibrary/pcl/pull/1449)
* Added `operator==` to `PCLHeader` class
  [[#1508]](https://github.com/PointCloudLibrary/pcl/pull/1508)

### `libpcl_features:`

* Fixed default parameters of the USC descriptor to match the values proposed in
  the original paper
  [[#856]](https://github.com/PointCloudLibrary/pcl/pull/856)
* Fixed the L1 normalization of the `ROPSEstimation` feature
  [[#993]](https://github.com/PointCloudLibrary/pcl/pull/993)
* Fixed default angle step in `ROPSEstimation`
  [[#1000]](https://github.com/PointCloudLibrary/pcl/pull/1000)
* Fixed a bug in `CRHEstimation` where internal spatial data vector was not
  zero-initialized
  [[#1151]](https://github.com/PointCloudLibrary/pcl/pull/1151)
* Updated `NormalEstimation` to mark cloud as non-dense when normal computation
  fails
  [[#1239]](https://github.com/PointCloudLibrary/pcl/pull/1239)
* Added new functions to compute approximate surface normals on a mesh and
  approximate covariance matrices
  [[#1262]](https://github.com/PointCloudLibrary/pcl/pull/1262)
* Fixed histogram computation in `computePointPFHRGBSignature()`
  [[#1331]](https://github.com/PointCloudLibrary/pcl/pull/1331)
* Fixed wrong erasing order in feature cache in `PFHEstimation`
  [[#1335]](https://github.com/PointCloudLibrary/pcl/pull/1335)

### `libpcl_filters:`

* Improved `RadiusOutlierRemoval` performance by using nearest-K search when the
  input point cloud is dense
  [[#709]](https://github.com/PointCloudLibrary/pcl/pull/709)
* Fixed the signature of `BoxClipper3D::clipPlanarPolygon3D()`
  [[#911]](https://github.com/PointCloudLibrary/pcl/pull/911)
* Updated base `Filter` class to allow using same point cloud as input and
  output (effective for every filtering algorithm)
  [[#1042]](https://github.com/PointCloudLibrary/pcl/pull/1042)
* Improved `CropBox` performance by caching the result of transform matrix
  identity test
  [[#1210]](https://github.com/PointCloudLibrary/pcl/pull/1210)
* Updated `PassThrough` filter to write a user-supplied value in place of bad
  points
  [[#1290]](https://github.com/PointCloudLibrary/pcl/pull/1290)
* Fixed handling of color fields in `VoxelGrid` centroid computation
  [[#1415]](https://github.com/PointCloudLibrary/pcl/pull/1415)
* Updated `ExtractIndices` (for `PCLPointCloud2` cloud type) to respect
  `keep_organized_` flag
  [[#1462]](https://github.com/PointCloudLibrary/pcl/pull/1462)
* Fixed OpenMP support on MSVC in `Convolution3D`
  [[#1527]](https://github.com/PointCloudLibrary/pcl/pull/1527)
* BugFix: Filters used applyFilter twice.
  [[#1572]](https://github.com/PointCloudLibrary/pcl/pull/1572)

### `libpcl_gpu:`

* Added a function `hasShifted()` in KinFu large scale
  [[#944]](https://github.com/PointCloudLibrary/pcl/pull/944)
* Fixed empty "View3D" window bug when using registration mode with `-pcd` flag
  in KinFu app
  [[#1018]](https://github.com/PointCloudLibrary/pcl/pull/1018)
* Fixed uninitialized loop variable in `PeoplePCDApp::convertProbToRGB()`
  [[#1104]](https://github.com/PointCloudLibrary/pcl/pull/1104)
* Fixed compilation errors in `gpu_people`
  [[#1194]](https://github.com/PointCloudLibrary/pcl/pull/1194)
* Fixed compilation error in `kinfu_large_scale` with CUDA ≥ 6.0
  [[#1225]](https://github.com/PointCloudLibrary/pcl/pull/1225)
* Fixed volume size computation in `kinfu_large_scale`
  [[#1233]](https://github.com/PointCloudLibrary/pcl/pull/1233)
* Fixed sporadical out-of-bounds memory accesses in `kinfu_large_scale` kernels
  [[#1263]](https://github.com/PointCloudLibrary/pcl/pull/1263)
* Fixed `plot_camera_poses.m` script in KinFu project
  [[#1311]](https://github.com/PointCloudLibrary/pcl/pull/1311)
* Fixed runtime exceptions related to `--viz` flag in KinFu
* Fix compilation on Mac OSX
  [[#1586]](https://github.com/PointCloudLibrary/pcl/pull/1586)

### `libpcl_io:`

* Added a grabber for IDS Imaging Ensenso devices
  [[#888]](https://github.com/PointCloudLibrary/pcl/pull/888)
* Updated `RobotEyeGrabber` class to handle new packet format
  [[#982]](https://github.com/PointCloudLibrary/pcl/pull/982)
* Fixed focal length computation in `OpenNI2Grabber`
  [[#992]](https://github.com/PointCloudLibrary/pcl/pull/992)
* Updated `OpenNIGrabber` to use depth camera parameters instead of color camera
  parameters for point reprojection
  [[#994]](https://github.com/PointCloudLibrary/pcl/pull/994)
* Made PCD reader case insensitive with respect to nan/NaN/NAN values
  [[#1004]](https://github.com/PointCloudLibrary/pcl/pull/1004)
* Added support for saving normal and curvature fields in `savePLYFile` and
  `savePLYFileBinary`
  [[#1057]](https://github.com/PointCloudLibrary/pcl/pull/1057)
  [[#1058]](https://github.com/PointCloudLibrary/pcl/pull/1058)
* Fixed alpha value of bad points in `OpenNIGrabber`
  [[#1090]](https://github.com/PointCloudLibrary/pcl/pull/1090)
* Fixed a bug in `OpenNIGrabber` destructor where wrong callback handle was
  unregistered
  [[#1094]](https://github.com/PointCloudLibrary/pcl/pull/1094)
* Fixed a bug in `PCDGrabber` destructor
  [[#1127]](https://github.com/PointCloudLibrary/pcl/pull/1127)
* Fixed point coordinate computation in `HDLGrabber`
  [[#1130]](https://github.com/PointCloudLibrary/pcl/pull/1130)
* Improved the PLY parser to work around some issues on Mac OSX
  [[#1165]](https://github.com/PointCloudLibrary/pcl/pull/1165)
* Added a family of data buffer classes useful for temporal filtering in
  grabbers
  [[#1212]](https://github.com/PointCloudLibrary/pcl/pull/1212)
* Added a grabber for davidSDK devices
  [[#1216]](https://github.com/PointCloudLibrary/pcl/pull/1216)
* Added a grabber and viewer for DepthSense SDK devices
  [[#1230]](https://github.com/PointCloudLibrary/pcl/pull/1230)
* Fixed stride computation and alpha values in
  `OpenNI2Grabber::convertToXYZRGBPointCloud()`
  [[#1248]](https://github.com/PointCloudLibrary/pcl/pull/1248)
* Changed type and semantics of return values in polygon saving functions based
  on VTK
  [[#1279]](https://github.com/PointCloudLibrary/pcl/pull/1279)
* Moved implementations of `pcl::io::load()` and `pcl::io::save()` to a new file
  "io/auto_io.h"
  [[#1294]](https://github.com/PointCloudLibrary/pcl/pull/1294)
* Fixed compilation of `OpenNI2Grabber` on _msvc14_
  [[#1310]](https://github.com/PointCloudLibrary/pcl/pull/1310)
* Added a callback signal for the filename of grabbed PCD file in `PCDGrabber`
  [[#1354]](https://github.com/PointCloudLibrary/pcl/pull/1354)
* Added support for both 'CRLF' and 'LF' line endings in PLY reader
  [[#1370]](https://github.com/PointCloudLibrary/pcl/pull/1370)
* Updated OpenNI2 grabber to support devices without color stream
  [[#1372]](https://github.com/PointCloudLibrary/pcl/pull/1372)
* Updated `PCDWriter` to not fail when the filesystem does not support setting
  file permissions
  [[#1374]](https://github.com/PointCloudLibrary/pcl/pull/1374)
* Fixed a bug in `MTLReader` reading function
  [[#1380]](https://github.com/PointCloudLibrary/pcl/pull/1380)
* Removed `PXCGrabber` (superseded by `DepthSenseGrabber`)
  [[#1395]](https://github.com/PointCloudLibrary/pcl/pull/1395)
* Added a grabber and viewer for RealSense SDK devices
  [[#1401]](https://github.com/PointCloudLibrary/pcl/pull/1401)
* Updated `loadPLYFile()` to support NaN values
  [[#1433]](https://github.com/PointCloudLibrary/pcl/pull/1433)
* Fixed parsing of `char` and `uchar` scalars in PLY files
  [[#1443]](https://github.com/PointCloudLibrary/pcl/pull/1443)
* Fixed ASCII file support in `savePolygonFile*` functions
  [[#1445]](https://github.com/PointCloudLibrary/pcl/pull/1445)
* Added a grabber and viewer for Velodyne VLP
  [[#1452]](https://github.com/PointCloudLibrary/pcl/pull/1452)
* Fix compilation when WITH_VTK=OFF
  [[#1585]](https://github.com/PointCloudLibrary/pcl/pull/1585)

### `libpcl_keypoints:`

* Fixed invalid array allocation in `ISSKeypoint3D`
  [[#1022]](https://github.com/PointCloudLibrary/pcl/pull/1022)
* Removed superfluous parameter in 'TrajkovicKeypoint3D::getNormals()'
  [[#1096]](https://github.com/PointCloudLibrary/pcl/pull/1096)
* Moved `UniformSampling` to the `filters` module
  [[#1411]](https://github.com/PointCloudLibrary/pcl/pull/1411)
* Fixed OpenMP support in `HarrisKeypoint2D`
  [[#1501]](https://github.com/PointCloudLibrary/pcl/pull/1501)
* Updated `SIFTKeypoint` to preserve point cloud viewpoint
  [[#1508]](https://github.com/PointCloudLibrary/pcl/pull/1508)

### `libpcl_octree:`

* Added `const` qualifiers in `OctreePointCloud::getVoxelBounds()`
  [[#1016]](https://github.com/PointCloudLibrary/pcl/pull/1016)
* Updated `Octree` iterator to use `unsigned long`s in key computations to
  reduce chance of overflows
  [[#1297]](https://github.com/PointCloudLibrary/pcl/pull/1297)
* Fixed compilation of `OctreePointCloudOccupancy` on _gcc_
  [[#1461]](https://github.com/PointCloudLibrary/pcl/pull/1461)

### `libpcl_outofcore:`

* Fixed compilation errors with C++11 standard
  [[#1386]](https://github.com/PointCloudLibrary/pcl/pull/1386)

### `libpcl_people:`

* Fixed undefined behavior in `HOG` (use `new`/`delete` consistently)
  [[#1099]](https://github.com/PointCloudLibrary/pcl/pull/1099)

### `libpcl_recognition:`

* Fixed multiple includes in `recognition` module
  [[#1109]](https://github.com/PointCloudLibrary/pcl/pull/1109)
  [[#1110]](https://github.com/PointCloudLibrary/pcl/pull/1110)
* Fixed "index out of bounds" error in `LineRGBD::refineDetectionsAlongDepth()`
  [[#1117]](https://github.com/PointCloudLibrary/pcl/pull/1117)
* Fixed a memory leak in `LINEMOD::detectTemplatesSemiScaleInvariant()`
  [[#1184]](https://github.com/PointCloudLibrary/pcl/pull/1184)

### `libpcl_registration:`

* Updated `GeneralizedIterativeClosestPoint` to return _transformed_ input point
  cloud after alignment
  [[#887]](https://github.com/PointCloudLibrary/pcl/pull/887)
* Fixed a problem with multiple definition of `setInputFeatureCloud` and
  `nearestNeighborSearch` symbols in `PPFRegistration`
  [[#891]](https://github.com/PointCloudLibrary/pcl/pull/891)
  [[#907]](https://github.com/PointCloudLibrary/pcl/pull/907)
* Added an implementation of the algorithm "4-Points Congruent Sets for Robust
  Surface Registration"
  [[#976]](https://github.com/PointCloudLibrary/pcl/pull/976)
* Added an implementation of the algorithm "Keypoint-based 4-Points Congruent
  Sets – Automated marker-less registration of laser scans"
  [[#979]](https://github.com/PointCloudLibrary/pcl/pull/979)
* Fixed compilation of `pcl_registration` module with MSVC2010
  [[#1014]](https://github.com/PointCloudLibrary/pcl/pull/1014)
* Removed wrong error normalization in `SampleConsensusPrerejective`
  [[#1037]](https://github.com/PointCloudLibrary/pcl/pull/1037)
* Added a new `IncrementalRegistration` class that allows to register a stream
  of clouds where each cloud is aligned to the previous cloud
  [[#1202]](https://github.com/PointCloudLibrary/pcl/pull/1202)
  [[#1451]](https://github.com/PointCloudLibrary/pcl/pull/1451)
* Fixed a wrong typedef for `KdTreeReciprocalPtr`
  [[#1204]](https://github.com/PointCloudLibrary/pcl/pull/1204)
* Added support for externally computed covariance matrices in
  `GeneralizedIterativeClosestPoint`
  [[#1262]](https://github.com/PointCloudLibrary/pcl/pull/1262)
* Fixed initialization of source and target covariances in
  `GeneralizedIterativeClosestPoint6D`
  [[#1304]](https://github.com/PointCloudLibrary/pcl/pull/1304)
* Added a new `MetaRegistration` class that allows to register a stream of
  clouds where each cloud is aligned to the conglomerate of all previous clouds
  [[#1426]](https://github.com/PointCloudLibrary/pcl/pull/1426)
* Fixed segmentation fault occurring in `CorrespondenceRejectorSurfaceNormal`
  [[#1536]](https://github.com/PointCloudLibrary/pcl/pull/1536)
* Use aligned allocator in vectors of MatchingCandidate
  [[#1552]](https://github.com/PointCloudLibrary/pcl/pull/1552)

### `libpcl_sample_consensus:`

* Fixed behavior of `SACMODEL_PARALLEL_LINE` to match the name (instead of
  searching for lines perpendicular to a given axis)
  [[#969]](https://github.com/PointCloudLibrary/pcl/pull/969)
* Added `getClassName()` function to all SAC models
  [[#1071]](https://github.com/PointCloudLibrary/pcl/pull/1071)
* Improved performance of `SampleConsensusModel::computeVariance()` by up to 10
  times
  [[#1285]](https://github.com/PointCloudLibrary/pcl/pull/1285)
* Fixed assignment operators for `SacModelCone` and `SacModelCylinder`
  [[#1299]](https://github.com/PointCloudLibrary/pcl/pull/1299)
* Refactored SAC models to store expected model and sample sizes in a protected
  member field; this deprecated `SAC_SAMPLE_SIZE` map
  [[#1367]](https://github.com/PointCloudLibrary/pcl/pull/1367)
  [[#1396]](https://github.com/PointCloudLibrary/pcl/pull/1396)

### `libpcl_search:`

* Fixed potential segfault in `OrganizedNeighbor::estimateProjectionMatrix()`
  [[#1176]](https://github.com/PointCloudLibrary/pcl/pull/1176)

### `libpcl_segmentation:`

* Added implementation of `LCCP` segmentation algorithm
  [[#718]](https://github.com/PointCloudLibrary/pcl/pull/718)
  [[#1287]](https://github.com/PointCloudLibrary/pcl/pull/1287)
  [[#1389]](https://github.com/PointCloudLibrary/pcl/pull/1389)
* Made `SupervoxelClustering` fully deterministic and made some internal
  refactoring
  [[#912]](https://github.com/PointCloudLibrary/pcl/pull/912)
* Moved specializations of `OctreePointCloudAdjacency::VoxelData` class from
  header to implementation files
  [[#919]](https://github.com/PointCloudLibrary/pcl/pull/919)
* Deprecated `SupervoxelClustering::getColoredCloud()`
  [[#941]](https://github.com/PointCloudLibrary/pcl/pull/941)
* Fixed a regression in `ExtractPolygonalPrismData`; both explicitly and
  implicitly closed polygons are supported again
  [[#1044]](https://github.com/PointCloudLibrary/pcl/pull/1044)
* Added an overload of `setConditionFunction()` in
  `ConditionalEuclideanClustering` that takes `boost::function`
  [[#1050]](https://github.com/PointCloudLibrary/pcl/pull/1050)
* Updated `SupervoxelClustering` to use the depth dependent transform by
  default only if the input cloud is organized; added a function to force use
  of the transform, and removed corresponding parameter from the constructor
  [[#1115]](https://github.com/PointCloudLibrary/pcl/pull/1115)
* Substituted hard-coded label point type with template parameter in
  `OrganizedConnectedComponentSegmentation`
  [[#1264]](https://github.com/PointCloudLibrary/pcl/pull/1264)
* Added an implementation of supervoxel graph partitioning algorithm described
  in "Constrained Planar Cuts - Object Partitioning for Point Clouds"
  [[#1278]](https://github.com/PointCloudLibrary/pcl/pull/1278)
* Fixed crashes in `ApproximateProgressiveMorphologicalFilter` in the case of
  non-default cell size
  [[#1293]](https://github.com/PointCloudLibrary/pcl/pull/1293)
* Fixed a bug in `RegionGrowing::validatePoint()`
  [[#1327]](https://github.com/PointCloudLibrary/pcl/pull/1327)
* Fixed return value of `SupervoxelClustering::getSeedResolution()`
  [[#1339]](https://github.com/PointCloudLibrary/pcl/pull/1339)

### `libpcl_stereo:`

* Added a new `DisparityMapConverter` class for converting disparity maps into
  point clouds
  [[#1021]](https://github.com/PointCloudLibrary/pcl/pull/1021)
* Added a new `DigitalElevationMapBuilder` class for building Digital Elevation
  Maps from disparity maps
  [[#1021]](https://github.com/PointCloudLibrary/pcl/pull/1021)

### `libpcl_surface:`

* Updated `TextureMapping` to not use hard-coded point types
  [[#929]](https://github.com/PointCloudLibrary/pcl/pull/929)
* Added a new function `getHullPointIndices` in concave and convex hull classes
  to retrieve indices of points that form the computed hull
  [[#1213]](https://github.com/PointCloudLibrary/pcl/pull/1213)
* Added several functions and parameters to the `OrganizedFastMesh` class
  [[#1262]](https://github.com/PointCloudLibrary/pcl/pull/1262)
* Added missing `PCL_EXPORTS` attributes for OpenNURBS classes
  [[#1315]](https://github.com/PointCloudLibrary/pcl/pull/1315)
* Fixed memory leak in `MeshSmoothingLaplacianVTK`
  [[#1424]](https://github.com/PointCloudLibrary/pcl/pull/1424)

### `libpcl_tracking:`

* Improved OMP 2.0 compatibility of `PyramidalKLTTracker`
  [[#1214]](https://github.com/PointCloudLibrary/pcl/pull/1214)
  [[#1223]](https://github.com/PointCloudLibrary/pcl/pull/1223)
* Fixed occasional segfault in `KLDAdaptiveParticleFilterOMPTracker`
  [[#1392]](https://github.com/PointCloudLibrary/pcl/pull/1392)

### `libpcl_visualization:`

* Added a new `PointCloudColorHandler` for "label" field
  [[#849]](https://github.com/PointCloudLibrary/pcl/pull/849)
* Fixed `setSize()` and `setPosition()` functions in `PCLVisualizer`
  [[#923]](https://github.com/PointCloudLibrary/pcl/pull/923)
* Fixed an issue with `PCLVisualizer` producing empty screenshots on some system
  configurations
  [[#956]](https://github.com/PointCloudLibrary/pcl/pull/956)
* Added a new function `removeAllCoordinateSystems()` in `PCLVisualizer`
  [[#965]](https://github.com/PointCloudLibrary/pcl/pull/965)
* Made `PCLVisualizer::addPointCloudPrincipalCurvatures()` templated on point
  and normal type
  [[#965]](https://github.com/PointCloudLibrary/pcl/pull/965)
* Fixed a minor bug in `PCLVisualizer::updatePolygonMesh()`
  [[#977]](https://github.com/PointCloudLibrary/pcl/pull/977)
* Fixed a minor bug in `ImageViewer::addMask()`
  [[#990]](https://github.com/PointCloudLibrary/pcl/pull/990)
* Fixed opacity handling in `ImageViewer`
  [[#995]](https://github.com/PointCloudLibrary/pcl/pull/995)
* Fixed a bug with `ImageViewer` not displaying anything with VTK 6
  [[#1009]](https://github.com/PointCloudLibrary/pcl/pull/1009)
* Updated `ImageViewer` to work around a bug in VTK 6.1
  [[#1017]](https://github.com/PointCloudLibrary/pcl/pull/1017)
* Fixed an Eigen-related compilation error in `PCLVisualizer::renderView()`
  [[#1019]](https://github.com/PointCloudLibrary/pcl/pull/1019)
* Fixed wrong axis flipping in `PCLVisualizer::renderView()`
  [[#1026]](https://github.com/PointCloudLibrary/pcl/pull/1026)
* Fixed a bug in `renderViewTesselatedSphere` when generated vertices were not
  guaranteed to stay on the unit sphere
  [[#1043]](https://github.com/PointCloudLibrary/pcl/pull/1043)
* Fixed misaligned context items in `ImageViewer`
  [[#1049]](https://github.com/PointCloudLibrary/pcl/pull/1049)
* Fixed opacity handling for layered rectangles of context items in
  `ImageViewer`
  [[#1051]](https://github.com/PointCloudLibrary/pcl/pull/1051)
* Fixed a regression in `RenderViewsTesselatedSphere::generateViews()` related
  to handling of multiple VTK versions
  [[#1056]](https://github.com/PointCloudLibrary/pcl/pull/1056)
  [[#1067]](https://github.com/PointCloudLibrary/pcl/pull/1067)
  [[#1072]](https://github.com/PointCloudLibrary/pcl/pull/1072)
* Updated `PCLVisualizer` to use `PointCloudColorHandlerRGBAField` for
  `PointXYZRGBA` clouds by default
  [[#1064]](https://github.com/PointCloudLibrary/pcl/pull/1064)
* Fixed a bug in `PointCloudColorHandlerLabelField` where red and blue channels
  were swapped
  [[#1076]](https://github.com/PointCloudLibrary/pcl/pull/1076)
* Updated `PCLPlotter` to ignore NaN values in histogram computation
  [[#1120]](https://github.com/PointCloudLibrary/pcl/pull/1120)
  [[#1126]](https://github.com/PointCloudLibrary/pcl/pull/1126)
* Fixed initial size of the `PCLVisualizer` window
  [[#1125]](https://github.com/PointCloudLibrary/pcl/pull/1125)
* Changed default representation of all shapes in `PCLVisualizer` to "surface"
  [[#1132]](https://github.com/PointCloudLibrary/pcl/pull/1132)
* Added a check for model coefficients size in functions that add shapes to
  `PCLVisualizer`
  [[#1142]](https://github.com/PointCloudLibrary/pcl/pull/1142)
* Added an option to switch between static/optimal color assignment in
  `PointCloudColorHandlerLabelField`
  [[#1156]](https://github.com/PointCloudLibrary/pcl/pull/1156)
* Added `PCLVisualizer::contains()` to check if a cloud, shape, or coordinate
  axes with given id already exist
  [[#1181]](https://github.com/PointCloudLibrary/pcl/pull/1181)
* Improved shape visualization by enabling shading
  [[#1211]](https://github.com/PointCloudLibrary/pcl/pull/1211)
* Improved 'u' key functionality in `PCLVisualizer`
  [[#1241]](https://github.com/PointCloudLibrary/pcl/pull/1241)
  [[#1321]](https://github.com/PointCloudLibrary/pcl/pull/1321)
  [[#1323]](https://github.com/PointCloudLibrary/pcl/pull/1323)
* Fixed potential crashes in `PCLVisualizer` by always checking result of
  `vtkSafeDownCast` calls
  [[#1245]](https://github.com/PointCloudLibrary/pcl/pull/1245)
* Updated `addPointCloud()` to use `PointCloudColorHandlerRGBField` when the
  cloud has color field
  [[#1295]](https://github.com/PointCloudLibrary/pcl/pull/1295)
  [[#1325]](https://github.com/PointCloudLibrary/pcl/pull/1325)
* Updated `PCLVisualizer` not to disable shading when changing shape's color
  [[#1300]](https://github.com/PointCloudLibrary/pcl/pull/1300)
* Fixed behavior of `PCLVisualizer::wasStopped()` with VTK6 on OSX
  [[#1436]](https://github.com/PointCloudLibrary/pcl/pull/1436)
* Improve pointcloud visualization with colormaps
  [[#1581]](https://github.com/PointCloudLibrary/pcl/pull/1581)

### `PCL Apps:`

* Fixed compilation of `point_cloud_editor` with Qt5
  [[#935]](https://github.com/PointCloudLibrary/pcl/pull/935)
* Fixed compilation of `dominant_plane_segmentation` and `manual_registration`
  with Boost 1.57
  [[#1062]](https://github.com/PointCloudLibrary/pcl/pull/1062)
  [[#1063]](https://github.com/PointCloudLibrary/pcl/pull/1063)

### `PCL Examples:`

* Updated supervoxel clustering example
  [[#915]](https://github.com/PointCloudLibrary/pcl/pull/915)
* Fixes for MS Visual Studio 2013
  [[#1526]](https://github.com/PointCloudLibrary/pcl/pull/1526)

### `PCL Tools:`

* Added support for point label visualization in `pcl_viewer`
  [[#849]](https://github.com/PointCloudLibrary/pcl/pull/849)
* Added support for absolute positioning of visualized point clouds in
  `pcl_viewer`
  [[#1154]](https://github.com/PointCloudLibrary/pcl/pull/1154)
* Fixed PLY file loading in `pcl_mesh_sampling` tool
  [[#1155]](https://github.com/PointCloudLibrary/pcl/pull/1155)
* Added loop distance (`-D`) and loop count (`-c`) parameters to the LUM tool
  [[#1291]](https://github.com/PointCloudLibrary/pcl/pull/1291)
* Fixed in-place filtering with `VoxelGrid` in `mesh_sampling` tool
  [[#1366]](https://github.com/PointCloudLibrary/pcl/pull/1366)
* Added a tool to convert OBJ files to PLY format
  [[#1375]](https://github.com/PointCloudLibrary/pcl/pull/1375)
* Added a universal mesh/cloud converted tool to convert between OBJ, PCD, PLY,
  STL, and VTK files
  [[#1442]](https://github.com/PointCloudLibrary/pcl/pull/1442)

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
* Declare `const InterestPoint&` explicitly
  [[#1541]](https://github.com/PointCloudLibrary/pcl/pull/1541)

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
* Adds MLS instantiation for input type PointXYZRGBNormal
  [[#1545]](https://github.com/PointCloudLibrary/pcl/pull/1545)

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
* Added iterative fitting routines for curve fitting surface::on_nurbs::Triangulation - added conversion functions for nurbs curve to line-polygon - added conversion functions for nurbs surface and curve to PolyMesh 
* Added flag to enable/disable usage of UmfPack for fast solving of sparse systems of equations - added triangulation functions to convert ON_NurbsSurface to pcl::PolygonMesh 
* Added bug fix in ConcaveHull, thanks to summer.icecream
* Added marching cubes using RBF and Hoppe SDF
* Pushed new functions that perform texture-mapping on meshes.
* Fix: issue #646 (vtk_smoothing not copied)
* Added new functionalities to TextureMapping: Find occlusions based on raytracing in octrees, UV mapping based on Texture resolution and camera focal length.
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
* add `CropHull` filter for filtering points based on a 2D or 3D convex or concave hull. The ray-polygon intersection test is used, which relies on closed polygons/surfaces
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
* rewrote big parts of the NARF keypoint extraction. Hopefully fixing some stability issues. Unfortunately still pretty slow for high resolution point clouds.
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
* marked all `getRemainingCorrespondences`-functions as DEPRECATED, we should replace them with purely stateless version outside the class body
* fixed a const missing in `PolynomialCalculationsT` (#388 - thanks Julian!)
* add `PCL_DEPRECATED` macro, closes #354.
* added `PointXYZHSV` type and the conversions for it
* added check for endianness for the Android platform


### `libpcl_search`

* BIG changes introduced - migration from `KdTree` to `pcl::Search`: moved `OrganizedDataIndex` and `OrganizedNeighbor` classes to `libpcl_search`
* added new templated methods for `nearestKSearch` and `radiusSearch` for situations when PointT is different than the one the KdTree object was created with (e.g., KdTree<PointT1> vs nearestKSearch (PointT2 &p...)
* added two new methods for `getApproximateIndices` where given a reference cloud of point type T1 we're trying to find the corresponding indices in a different cloud of point type T2
* refactorized a lot of code in search and octree to make it look more consistent with the rest of the API
* fixed a bug in octree_search which was semantically doing something bad: for each `radiusSearch`/`nearestKSearch`/`approxNearestSearch` call with a PointCloudConstPtr, the octree was getting recreated. Changed the API to be consistent with the rest of PCL (including pcl_search and pcl_kdtree) where we pass in a PointCloud instead of a PointCloudConstPtr which simply calls searchMethod (cloud[i], ...)
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
* fixed a bug in octree_search which was semantically doing something bad: for each `radiusSearch`/`nearestKSearch`/`approxNearestSearch` call with a PointCloudConstPtr, the octree was getting recreated. Changed the API to be consistent with the rest of PCL (including pcl_search and pcl_kdtree) where we pass in a PointCloud instead of a PointCloudConstPtr which simply calls searchMethod (cloud[i], ...)
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
  * marked all "getRemainingCorrespondences"-functions as DEPRECATED, we should replace them with purely stateless version outside the class body
* Update: remove ciminpack dependency and rely on eigen for LM
* Fixed a bug in ICP-NL by modifying `WarpPointRigid` to preserve the value of the 4th coordinate when warping; Re-enabled missing unit tests for ICP and ICP-NL
* Added point-to-plane ICP
* added nr_iterations_ and max_iterations_ to the initializer list (were missing)
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
 * fixed all _is_dense_ occurrences
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
 * Further split up registration.h and added transformation estimation classes, e.g., for estimating rigid transformation using SVD.
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
 * Changed _KdTree::radiusSearch_ method signature to return int (number of neighbors found) instead of bool
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

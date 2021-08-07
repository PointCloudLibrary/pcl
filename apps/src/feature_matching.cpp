#include <pcl/common/transforms.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/extract_indices.h> // for ExtractIndices
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/memory.h> // for pcl::dynamic_pointer_cast

#include <string>
#include <vector>

template <typename FeatureType>
class ICCVTutorial {
public:
  ICCVTutorial(
      pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI>::Ptr keypoint_detector,
      typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor,
      pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal>::Ptr surface_reconstructor,
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target);

  /**
   * \brief starts the event loop for the visualizer
   */
  void
  run();

protected:
  /**
   * \brief remove plane and select largest cluster as input object
   * \param input the input point cloud
   * \param segmented the resulting segmented point cloud containing only points of the
   * largest cluster
   */
  void
  segmentation(const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
               const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& segmented) const;

  /**
   * \brief Detects key points in the input point cloud
   * \param input the input point cloud
   * \param keypoints the resulting key points. Note that they are not necessarily a
   * subset of the input cloud
   */
  void
  detectKeypoints(const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints) const;

  /**
   * \brief extract descriptors for given key points
   * \param input point cloud to be used for descriptor extraction
   * \param keypoints locations where descriptors are to be extracted
   * \param features resulting descriptors
   */
  void
  extractDescriptors(typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
                     const typename pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints,
                     typename pcl::PointCloud<FeatureType>::Ptr features);

  /**
   * \brief find corresponding features based on some metric
   * \param source source feature descriptors
   * \param target target feature descriptors
   * \param correspondences indices out of the target descriptors that correspond
   * (nearest neighbor) to the source descriptors
   */
  void
  findCorrespondences(typename pcl::PointCloud<FeatureType>::Ptr source,
                      typename pcl::PointCloud<FeatureType>::Ptr target,
                      std::vector<int>& correspondences) const;

  /**
   * \brief  remove non-consistent correspondences
   */
  void
  filterCorrespondences();

  /**
   * \brief calculate the initial rigid transformation from filtered corresponding
   * keypoints
   */
  void
  determineInitialTransformation();

  /**
   * \brief calculate the final rigid transformation using ICP over all points
   */
  void
  determineFinalTransformation();

  /**
   * \brief reconstructs the surface from merged point clouds
   */
  void
  reconstructSurface();

  /**
   * \brief callback to handle keyboard events
   * \param event object containing information about the event. e.g. type (press,
   * release) etc.
   * \param cookie user defined data passed during registration of the callback
   */
  void
  keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* cookie);

private:
  pcl::visualization::PCLVisualizer visualizer_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_;
  pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI>::Ptr keypoint_detector_;
  typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor_;
  pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal>::Ptr surface_reconstructor_;
  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source_;
  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target_;
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_segmented_;
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_segmented_;
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_transformed_;
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_registered_;
  typename pcl::PolygonMesh surface_;
  typename pcl::PointCloud<FeatureType>::Ptr source_features_;
  typename pcl::PointCloud<FeatureType>::Ptr target_features_;
  std::vector<int> source2target_;
  std::vector<int> target2source_;
  pcl::CorrespondencesPtr correspondences_;
  Eigen::Matrix4f initial_transformation_matrix_;
  Eigen::Matrix4f transformation_matrix_;
  bool show_source2target_;
  bool show_target2source_;
  bool show_correspondences;
};

template <typename FeatureType>
ICCVTutorial<FeatureType>::ICCVTutorial(
    pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI>::Ptr keypoint_detector,
    typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor,
    pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal>::Ptr surface_reconstructor,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target)
: source_keypoints_(new pcl::PointCloud<pcl::PointXYZI>())
, target_keypoints_(new pcl::PointCloud<pcl::PointXYZI>())
, keypoint_detector_(std::move(keypoint_detector))
, feature_extractor_(feature_extractor)
, surface_reconstructor_(std::move(surface_reconstructor))
, source_(std::move(source))
, target_(std::move(target))
, source_segmented_(new pcl::PointCloud<pcl::PointXYZRGB>)
, target_segmented_(new pcl::PointCloud<pcl::PointXYZRGB>)
, source_transformed_(new pcl::PointCloud<pcl::PointXYZRGB>)
, source_registered_(new pcl::PointCloud<pcl::PointXYZRGB>)
, source_features_(new pcl::PointCloud<FeatureType>)
, target_features_(new pcl::PointCloud<FeatureType>)
, correspondences_(new pcl::Correspondences)
, show_source2target_(false)
, show_target2source_(false)
, show_correspondences(false)
{
  visualizer_.registerKeyboardCallback(
      &ICCVTutorial::keyboard_callback, *this, nullptr);

  segmentation(source_, source_segmented_);
  segmentation(target_, target_segmented_);

  detectKeypoints(source_segmented_, source_keypoints_);
  detectKeypoints(target_segmented_, target_keypoints_);

  extractDescriptors(source_segmented_, source_keypoints_, source_features_);
  extractDescriptors(target_segmented_, target_keypoints_, target_features_);

  findCorrespondences(source_features_, target_features_, source2target_);
  findCorrespondences(target_features_, source_features_, target2source_);

  filterCorrespondences();

  determineInitialTransformation();
  determineFinalTransformation();

  reconstructSurface();
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::segmentation(
    const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& source,
    const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& segmented) const
{
  std::cout << "segmentation..." << std::flush;
  // fit plane and keep points above that plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);

  seg.setInputCloud(source);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(source);
  extract.setIndices(inliers);
  extract.setNegative(true);

  extract.filter(*segmented);
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);
  std::cout << "OK" << std::endl;

  std::cout << "clustering..." << std::flush;
  // euclidean clustering
  typename pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(segmented);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
  clustering.setClusterTolerance(0.02); // 2cm
  clustering.setMinClusterSize(1000);
  clustering.setMaxClusterSize(250000);
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(segmented);
  clustering.extract(cluster_indices);

  if (!cluster_indices.empty()) // use largest cluster
  {
    std::cout << cluster_indices.size() << " clusters found";
    if (cluster_indices.size() > 1)
      std::cout << " Using largest one...";
    std::cout << std::endl;
    typename pcl::IndicesPtr indices(new pcl::Indices);
    *indices = cluster_indices[0].indices;
    extract.setInputCloud(segmented);
    extract.setIndices(indices);
    extract.setNegative(false);

    extract.filter(*segmented);
  }
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::detectKeypoints(
    const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints) const
{
  std::cout << "keypoint detection..." << std::flush;
  keypoint_detector_->setInputCloud(input);
  keypoint_detector_->compute(*keypoints);
  std::cout << "OK. keypoints found: " << keypoints->size() << std::endl;
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::extractDescriptors(
    typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
    const typename pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints,
    typename pcl::PointCloud<FeatureType>::Ptr features)
{
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  kpts->points.resize(keypoints->size());

  pcl::copyPointCloud(*keypoints, *kpts);

  typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, FeatureType>::Ptr
      feature_from_normals = pcl::dynamic_pointer_cast<
          pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, FeatureType>>(
          feature_extractor_);

  feature_extractor_->setSearchSurface(input);
  feature_extractor_->setInputCloud(kpts);

  if (feature_from_normals) {
    std::cout << "normal estimation..." << std::flush;
    typename pcl::PointCloud<pcl::Normal>::Ptr normals(
        new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(
        new pcl::search::KdTree<pcl::PointXYZRGB>));
    normal_estimation.setRadiusSearch(0.01);
    normal_estimation.setInputCloud(input);
    normal_estimation.compute(*normals);
    feature_from_normals->setInputNormals(normals);
    std::cout << "OK" << std::endl;
  }

  std::cout << "descriptor extraction..." << std::flush;
  feature_extractor_->compute(*features);
  std::cout << "OK" << std::endl;
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::findCorrespondences(
    typename pcl::PointCloud<FeatureType>::Ptr source,
    typename pcl::PointCloud<FeatureType>::Ptr target,
    std::vector<int>& correspondences) const
{
  std::cout << "correspondence assignment..." << std::flush;
  correspondences.resize(source->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<FeatureType> descriptor_kdtree;
  descriptor_kdtree.setInputCloud(target);

  // Find the index of the best match for each keypoint, and store it in
  // "correspondences_out"
  const int k = 1;
  pcl::Indices k_indices(k);
  std::vector<float> k_squared_distances(k);
  for (int i = 0; i < static_cast<int>(source->size()); ++i) {
    descriptor_kdtree.nearestKSearch(*source, i, k, k_indices, k_squared_distances);
    correspondences[i] = k_indices[0];
  }
  std::cout << "OK" << std::endl;
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::filterCorrespondences()
{
  std::cout << "correspondence rejection..." << std::flush;
  std::vector<std::pair<unsigned, unsigned>> correspondences;
  for (std::size_t cIdx = 0; cIdx < source2target_.size(); ++cIdx)
    if (target2source_[source2target_[cIdx]] == static_cast<int>(cIdx))
      correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));

  correspondences_->resize(correspondences.size());
  for (std::size_t cIdx = 0; cIdx < correspondences.size(); ++cIdx) {
    (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
    (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
  }

  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
  rejector.setInputSource(source_keypoints_);
  rejector.setInputTarget(target_keypoints_);
  rejector.setInputCorrespondences(correspondences_);
  rejector.getCorrespondences(*correspondences_);
  std::cout << "OK" << std::endl;
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::determineInitialTransformation()
{
  std::cout << "initial alignment..." << std::flush;
  pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr
      transformation_estimation(
          new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI,
                                                             pcl::PointXYZI>);

  transformation_estimation->estimateRigidTransformation(
      *source_keypoints_,
      *target_keypoints_,
      *correspondences_,
      initial_transformation_matrix_);

  pcl::transformPointCloud(
      *source_segmented_, *source_transformed_, initial_transformation_matrix_);
  std::cout << "OK" << std::endl;
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::determineFinalTransformation()
{
  std::cout << "final registration..." << std::flush;
  pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registration(
      new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
  registration->setInputSource(source_transformed_);
  // registration->setInputSource (source_segmented_);
  registration->setInputTarget(target_segmented_);
  registration->setMaxCorrespondenceDistance(0.05);
  registration->setRANSACOutlierRejectionThreshold(0.05);
  registration->setTransformationEpsilon(0.000001);
  registration->setMaximumIterations(1000);
  registration->align(*source_registered_);
  transformation_matrix_ = registration->getFinalTransformation();
  std::cout << "OK" << std::endl;
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::reconstructSurface()
{
  std::cout << "surface reconstruction..." << std::flush;
  // merge the transformed and the target point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>);
  *merged = *source_registered_;
  *merged += *target_segmented_;

  // apply grid filtering to reduce amount of points as well as to make them uniform
  // distributed
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
  voxel_grid.setInputCloud(merged);
  voxel_grid.setLeafSize(0.002f, 0.002f, 0.002f);
  voxel_grid.setDownsampleAllData(true);
  voxel_grid.filter(*merged);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::copyPointCloud(*merged, *vertices);

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> normal_estimation;
  normal_estimation.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(
      new pcl::search::KdTree<pcl::PointXYZRGB>));
  normal_estimation.setRadiusSearch(0.01);
  normal_estimation.setInputCloud(merged);
  normal_estimation.compute(*vertices);

  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree->setInputCloud(vertices);

  surface_reconstructor_->setSearchMethod(tree);
  surface_reconstructor_->setInputCloud(vertices);
  surface_reconstructor_->reconstruct(surface_);
  std::cout << "OK" << std::endl;
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::run()
{
  visualizer_.spin();
}

template <typename FeatureType>
void
ICCVTutorial<FeatureType>::keyboard_callback(
    const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.keyUp()) {
    switch (event.getKeyCode()) {
    case '1':
      if (!visualizer_.removePointCloud("source_points")) {
        visualizer_.addPointCloud(source_, "source_points");
      }
      break;

    case '2':
      if (!visualizer_.removePointCloud("target_points")) {
        visualizer_.addPointCloud(target_, "target_points");
      }
      break;

    case '3':
      if (!visualizer_.removePointCloud("source_segmented")) {
        visualizer_.addPointCloud(source_segmented_, "source_segmented");
      }
      break;

    case '4':
      if (!visualizer_.removePointCloud("target_segmented")) {
        visualizer_.addPointCloud(target_segmented_, "target_segmented");
      }
      break;

    case '5':
      if (!visualizer_.removePointCloud("source_keypoints")) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color(
            source_keypoints_, 0, 0, 255);
        visualizer_.addPointCloud(
            source_keypoints_, keypoint_color, "source_keypoints");
      }
      break;

    case '6':
      if (!visualizer_.removePointCloud("target_keypoints")) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color(
            target_keypoints_, 255, 0, 0);
        visualizer_.addPointCloud(
            target_keypoints_, keypoint_color, "target_keypoints");
      }
      break;

    case '7':
      if (!show_source2target_)
        visualizer_.addCorrespondences<pcl::PointXYZI>(
            source_keypoints_, target_keypoints_, source2target_, "source2target");
      else
        visualizer_.removeCorrespondences("source2target");

      show_source2target_ = !show_source2target_;
      break;

    case '8':
      if (!show_target2source_)
        visualizer_.addCorrespondences<pcl::PointXYZI>(
            target_keypoints_, source_keypoints_, target2source_, "target2source");
      else
        visualizer_.removeCorrespondences("target2source");

      show_target2source_ = !show_target2source_;
      break;

    case '9':
      if (!show_correspondences)
        visualizer_.addCorrespondences<pcl::PointXYZI>(
            source_keypoints_, target_keypoints_, *correspondences_, "correspondences");
      else
        visualizer_.removeCorrespondences("correspondences");
      show_correspondences = !show_correspondences;
      break;

    case 'i':
    case 'I':
      if (!visualizer_.removePointCloud("transformed"))
        visualizer_.addPointCloud(source_transformed_, "transformed");
      break;

    case 'r':
    case 'R':
      if (!visualizer_.removePointCloud("registered"))
        visualizer_.addPointCloud(source_registered_, "registered");
      break;

    case 't':
    case 'T':
      visualizer_.addPolygonMesh(surface_, "surface");
      break;
    }
  }
}

int
main(int argc, char** argv)
{
  if (argc < 6) {
    // clang-format off
    pcl::console::print_info ("Syntax is: %s <source-pcd-file> <target-pcd-file> <keypoint-method> <descriptor-type> <surface-reconstruction-method>\n", argv[0]);
    pcl::console::print_info("available <keypoint-methods>: 1 = Sift3D\n");
    pcl::console::print_info("                              2 = Harris3D\n");
    pcl::console::print_info("                              3 = Tomasi3D\n");
    pcl::console::print_info("                              4 = Noble3D\n");
    pcl::console::print_info("                              5 = Lowe3D\n");
    pcl::console::print_info("                              6 = Curvature3D\n\n");
    pcl::console::print_info("available <descriptor-types>: 1 = FPFH\n");
    pcl::console::print_info("                              2 = SHOTRGB\n");
    pcl::console::print_info("                              3 = PFH\n");
    pcl::console::print_info("                              4 = PFHRGB\n\n");
    pcl::console::print_info("available <surface-methods>:  1 = Greedy Projection\n");
    pcl::console::print_info("                              2 = Marching Cubes\n");
    // clang-format on

    return 1;
  }
  pcl::console::print_info("== MENU ==\n");
  pcl::console::print_info("1 - show/hide source point cloud\n");
  pcl::console::print_info("2 - show/hide target point cloud\n");
  pcl::console::print_info("3 - show/hide segmented source point cloud\n");
  pcl::console::print_info("4 - show/hide segmented target point cloud\n");
  pcl::console::print_info("5 - show/hide source key points\n");
  pcl::console::print_info("6 - show/hide target key points\n");
  pcl::console::print_info("7 - show/hide source2target correspondences\n");
  pcl::console::print_info("8 - show/hide target2source correspondences\n");
  pcl::console::print_info("9 - show/hide consistent correspondences\n");
  pcl::console::print_info("i - show/hide initial alignment\n");
  pcl::console::print_info("r - show/hide final registration\n");
  pcl::console::print_info("t - show/hide triangulation (surface reconstruction)\n");
  pcl::console::print_info("h - show visualizer options\n");
  pcl::console::print_info("q - quit\n");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile(argv[1], *source);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile(argv[2], *target);

  int keypoint_type = atoi(argv[3]);
  int descriptor_type = atoi(argv[4]);
  int surface_type = atoi(argv[5]);

  pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI>::Ptr keypoint_detector;

  if (keypoint_type == 1) {
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift3D =
        new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
    sift3D->setScales(0.01f, 3, 2);
    sift3D->setMinimumContrast(0.0);
    keypoint_detector.reset(sift3D);
  }
  else {
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>* harris3D =
        new pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>(
            pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS);
    harris3D->setNonMaxSupression(true);
    harris3D->setRadius(0.01f);
    harris3D->setRadiusSearch(0.01f);
    keypoint_detector.reset(harris3D);
    switch (keypoint_type) {
    case 2:
      harris3D->setMethod(
          pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS);
      break;

    case 3:
      harris3D->setMethod(
          pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::TOMASI);
      break;

    case 4:
      harris3D->setMethod(
          pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::NOBLE);
      break;

    case 5:
      harris3D->setMethod(
          pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE);
      break;

    case 6:
      harris3D->setMethod(
          pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::CURVATURE);
      break;
    default:
      pcl::console::print_error(
          "unknown key point detection method %d\n expecting values between 1 and 6",
          keypoint_type);
      exit(1);
      break;
    }
  }

  pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal>::Ptr surface_reconstruction;

  if (surface_type == 1) {
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>* gp3 =
        new pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>;

    // Set the maximum distance between connected points (maximum edge length)
    gp3->setSearchRadius(0.025);

    // Set typical values for the parameters
    gp3->setMu(2.5);
    gp3->setMaximumNearestNeighbors(100);
    gp3->setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3->setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3->setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3->setNormalConsistency(false);
    surface_reconstruction.reset(gp3);
  }
  else if (surface_type == 2) {
    pcl::MarchingCubes<pcl::PointXYZRGBNormal>* mc =
        new pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal>;
    mc->setIsoLevel(0.001f);
    mc->setGridResolution(50, 50, 50);
    surface_reconstruction.reset(mc);
  }
  else {
    pcl::console::print_error(
        "unknown surface reconstruction method %d\n expecting values between 1 and 2",
        surface_type);
    exit(1);
  }

  switch (descriptor_type) {
  case 1: {
    pcl::Feature<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr feature_extractor(
        new pcl::
            FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
    feature_extractor->setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(
        new pcl::search::KdTree<pcl::PointXYZRGB>));
    feature_extractor->setRadiusSearch(0.05);
    ICCVTutorial<pcl::FPFHSignature33> tutorial(
        keypoint_detector, feature_extractor, surface_reconstruction, source, target);
    tutorial.run();
  } break;

  case 2: {
    pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344>* shot =
        new pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344>;
    shot->setRadiusSearch(0.04);
    pcl::Feature<pcl::PointXYZRGB, pcl::SHOT1344>::Ptr feature_extractor(shot);
    ICCVTutorial<pcl::SHOT1344> tutorial(
        keypoint_detector, feature_extractor, surface_reconstruction, source, target);
    tutorial.run();
  } break;

  case 3: {
    pcl::Feature<pcl::PointXYZRGB, pcl::PFHSignature125>::Ptr feature_extractor(
        new pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>);
    feature_extractor->setKSearch(50);
    ICCVTutorial<pcl::PFHSignature125> tutorial(
        keypoint_detector, feature_extractor, surface_reconstruction, source, target);
    tutorial.run();
  } break;

  case 4: {
    pcl::Feature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr feature_extractor(
        new pcl::
            PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>);
    feature_extractor->setKSearch(50);
    ICCVTutorial<pcl::PFHRGBSignature250> tutorial(
        keypoint_detector, feature_extractor, surface_reconstruction, source, target);
    tutorial.run();
  } break;

  default:
    pcl::console::print_error(
        "unknown descriptor type %d\n expecting values between 1 and 4",
        descriptor_type);
    exit(1);
    break;
  }

  return 0;
}

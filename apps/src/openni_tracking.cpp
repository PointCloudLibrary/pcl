/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/normal_coherence.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/tracking.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/format.hpp>

#include <mutex>
#include <thread>

#define FPS_CALC_BEGIN                                                                 \
  static double duration = 0;                                                          \
  double start_time = pcl::getTime();

// clang-format off
#define FPS_CALC_END(_WHAT_)                                                           \
  {                                                                                    \
    double end_time = pcl::getTime();                                                  \
    static unsigned count = 0;                                                         \
    if (++count == 10) {                                                               \
      std::cout << "Average framerate(" << _WHAT_ << "): "                             \
                << double(count) / double(duration) << " Hz" << std::endl;             \
      count = 0;                                                                       \
      duration = 0.0;                                                                  \
    }                                                                                  \
    else {                                                                             \
      duration += end_time - start_time;                                               \
    }                                                                                  \
  }
// clang-format on

using namespace pcl::tracking;
using namespace std::chrono_literals;

template <typename PointType>
class OpenNISegmentTracking {
public:
  using RefPointType = pcl::PointXYZRGBA;
  using ParticleT = ParticleXYZRPY;

  using Cloud = pcl::PointCloud<PointType>;
  using RefCloud = pcl::PointCloud<RefPointType>;
  using RefCloudPtr = RefCloud::Ptr;
  using RefCloudConstPtr = RefCloud::ConstPtr;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;
  using ParticleFilter = ParticleFilterTracker<RefPointType, ParticleT>;
  using CoherencePtr = ParticleFilter::CoherencePtr;
  using KdTree = pcl::search::KdTree<PointType>;
  using KdTreePtr = typename KdTree::Ptr;
  OpenNISegmentTracking(const std::string& device_id,
                        int thread_nr,
                        double downsampling_grid_size,
                        bool use_convex_hull,
                        bool visualize_non_downsample,
                        bool visualize_particles,
                        bool use_fixed)
  : viewer_("PCL OpenNI Tracking Viewer")
  , device_id_(device_id)
  , new_cloud_(false)
  , ne_(thread_nr)
  , counter_(0)
  , use_convex_hull_(use_convex_hull)
  , visualize_non_downsample_(visualize_non_downsample)
  , visualize_particles_(visualize_particles)
  , downsampling_grid_size_(downsampling_grid_size)
  {
    KdTreePtr tree(new KdTree(false));
    ne_.setSearchMethod(tree);
    ne_.setRadiusSearch(0.03);

    std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
    if (use_fixed) {
      ParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr tracker(
          new ParticleFilterOMPTracker<RefPointType, ParticleT>(thread_nr));
      tracker_ = tracker;
    }
    else {
      KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr tracker(
          new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>(thread_nr));
      tracker->setMaximumParticleNum(500);
      tracker->setDelta(0.99);
      tracker->setEpsilon(0.2);
      ParticleT bin_size;
      bin_size.x = 0.1f;
      bin_size.y = 0.1f;
      bin_size.z = 0.1f;
      bin_size.roll = 0.1f;
      bin_size.pitch = 0.1f;
      bin_size.yaw = 0.1f;
      tracker->setBinSize(bin_size);
      tracker_ = tracker;
    }

    tracker_->setTrans(Eigen::Affine3f::Identity());
    tracker_->setStepNoiseCovariance(default_step_covariance);
    tracker_->setInitialNoiseCovariance(initial_noise_covariance);
    tracker_->setInitialNoiseMean(default_initial_mean);
    tracker_->setIterationNum(1);

    tracker_->setParticleNum(400);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal(false);
    // setup coherences
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence =
        ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr(
            new ApproxNearestPairPointCloudCoherence<RefPointType>());

    DistanceCoherence<RefPointType>::Ptr distance_coherence(
        new DistanceCoherence<RefPointType>);
    coherence->addPointCoherence(distance_coherence);

    HSVColorCoherence<RefPointType>::Ptr color_coherence(
        new HSVColorCoherence<RefPointType>);
    color_coherence->setWeight(0.1);
    coherence->addPointCoherence(color_coherence);

    pcl::search::Octree<RefPointType>::Ptr search(
        new pcl::search::Octree<RefPointType>(0.01));
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(0.01);
    tracker_->setCloudCoherence(coherence);
  }

  bool
  drawParticles(pcl::visualization::PCLVisualizer& viz)
  {
    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
    if (particles) {
      if (visualize_particles_) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& point : particles->points) {
          particle_cloud->points.emplace_back(point.x, point.y, point.z);
        }

        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(
              particle_cloud, 250, 99, 71);
          if (!viz.updatePointCloud(particle_cloud, blue_color, "particle cloud"))
            viz.addPointCloud(particle_cloud, blue_color, "particle cloud");
        }
      }
      return true;
    }
    PCL_WARN("no particles\n");
    return false;
  }

  void
  drawResult(pcl::visualization::PCLVisualizer& viz)
  {
    ParticleXYZRPY result = tracker_->getResult();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);
    // move a little bit for better visualization
    transformation.translation() += Eigen::Vector3f(0.0f, 0.0f, -0.005f);
    RefCloudPtr result_cloud(new RefCloud());

    if (!visualize_non_downsample_)
      pcl::transformPointCloud<RefPointType>(
          *(tracker_->getReferenceCloud()), *result_cloud, transformation);
    else
      pcl::transformPointCloud<RefPointType>(
          *reference_, *result_cloud, transformation);

    {
      pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color(
          result_cloud, 0, 0, 255);
      if (!viz.updatePointCloud(result_cloud, red_color, "resultcloud"))
        viz.addPointCloud(result_cloud, red_color, "resultcloud");
    }
  }

  void
  viz_cb(pcl::visualization::PCLVisualizer& viz)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!cloud_pass_) {
      std::this_thread::sleep_for(1s);
      return;
    }

    if (new_cloud_ && cloud_pass_downsampled_) {
      CloudPtr cloud_pass;
      if (!visualize_non_downsample_)
        cloud_pass = cloud_pass_downsampled_;
      else
        cloud_pass = cloud_pass_;

      if (!viz.updatePointCloud(cloud_pass, "cloudpass")) {
        viz.addPointCloud(cloud_pass, "cloudpass");
        viz.resetCameraViewpoint("cloudpass");
      }
    }

    if (new_cloud_ && reference_) {
      bool ret = drawParticles(viz);
      if (ret) {
        drawResult(viz);

        // clang-format off
        // draw some texts
        viz.removeShape("N");
        viz.addText((boost::format("number of Reference PointClouds: %1%") %
                     tracker_->getReferenceCloud()->size()).str(),
                    10, 20, 20, 1.0, 1.0, 1.0, "N");

        viz.removeShape("M");
        viz.addText((boost::format("number of Measured PointClouds:  %1%") %
                     cloud_pass_downsampled_->size()).str(),
                    10, 40, 20, 1.0, 1.0, 1.0, "M");

        viz.removeShape("tracking");
        viz.addText((boost::format("tracking:        %f fps") % (1.0 / tracking_time_)).str(),
                    10, 60, 20, 1.0, 1.0, 1.0, "tracking");

        viz.removeShape("downsampling");
        viz.addText((boost::format("downsampling:    %f fps") % (1.0 / downsampling_time_)).str(),
                    10, 80, 20, 1.0, 1.0, 1.0, "downsampling");

        viz.removeShape("computation");
        viz.addText((boost::format("computation:     %f fps") % (1.0 / computation_time_)).str(),
                    10, 100, 20, 1.0, 1.0, 1.0, "computation");

        viz.removeShape("particles");
        viz.addText((boost::format("particles:     %1%") %
                     tracker_->getParticles()->size()).str(),
                    10, 120, 20, 1.0, 1.0, 1.0, "particles");
        // clang-format on
      }
    }
    new_cloud_ = false;
  }

  void
  filterPassThrough(const CloudConstPtr& cloud, Cloud& result)
  {
    FPS_CALC_BEGIN;
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 10.0);
    pass.setKeepOrganized(false);
    pass.setInputCloud(cloud);
    pass.filter(result);
    FPS_CALC_END("filterPassThrough");
  }

  void
  euclideanSegment(const CloudConstPtr& cloud,
                   std::vector<pcl::PointIndices>& cluster_indices)
  {
    FPS_CALC_BEGIN;
    pcl::EuclideanClusterExtraction<PointType> ec;
    KdTreePtr tree(new KdTree());

    ec.setClusterTolerance(0.05); // 2cm
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    FPS_CALC_END("euclideanSegmentation");
  }

  void
  gridSample(const CloudConstPtr& cloud, Cloud& result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime();
    pcl::VoxelGrid<PointType> grid;
    grid.setLeafSize(float(leaf_size), float(leaf_size), float(leaf_size));
    grid.setInputCloud(cloud);
    grid.filter(result);
    double end = pcl::getTime();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }

  void
  gridSampleApprox(const CloudConstPtr& cloud, Cloud& result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime();
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize(static_cast<float>(leaf_size),
                     static_cast<float>(leaf_size),
                     static_cast<float>(leaf_size));
    grid.setInputCloud(cloud);
    grid.filter(result);
    double end = pcl::getTime();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }

  void
  planeSegmentation(const CloudConstPtr& cloud,
                    pcl::ModelCoefficients& coefficients,
                    pcl::PointIndices& inliers)
  {
    FPS_CALC_BEGIN;
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);
    FPS_CALC_END("planeSegmentation");
  }

  void
  planeProjection(const CloudConstPtr& cloud,
                  Cloud& result,
                  const pcl::ModelCoefficients::ConstPtr& coefficients)
  {
    FPS_CALC_BEGIN;
    pcl::ProjectInliers<PointType> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(result);
    FPS_CALC_END("planeProjection");
  }

  void
  convexHull(const CloudConstPtr& cloud,
             Cloud&,
             std::vector<pcl::Vertices>& hull_vertices)
  {
    FPS_CALC_BEGIN;
    pcl::ConvexHull<PointType> chull;
    chull.setInputCloud(cloud);
    chull.reconstruct(*cloud_hull_, hull_vertices);
    FPS_CALC_END("convexHull");
  }

  void
  normalEstimation(const CloudConstPtr& cloud, pcl::PointCloud<pcl::Normal>& result)
  {
    FPS_CALC_BEGIN;
    ne_.setInputCloud(cloud);
    ne_.compute(result);
    FPS_CALC_END("normalEstimation");
  }

  void
  tracking(const RefCloudConstPtr& cloud)
  {
    double start = pcl::getTime();
    FPS_CALC_BEGIN;
    tracker_->setInputCloud(cloud);
    tracker_->compute();
    double end = pcl::getTime();
    FPS_CALC_END("tracking");
    tracking_time_ = end - start;
  }

  void
  addNormalToCloud(const CloudConstPtr& cloud,
                   const pcl::PointCloud<pcl::Normal>::ConstPtr&,
                   RefCloud& result)
  {
    result.width = cloud->width;
    result.height = cloud->height;
    result.is_dense = cloud->is_dense;
    for (const auto& pt : *cloud) {
      RefPointType point;
      point.x = pt.x;
      point.y = pt.y;
      point.z = pt.z;
      point.rgba = pt.rgba;
      result.push_back(point);
    }
  }

  void
  extractNonPlanePoints(const CloudConstPtr& cloud,
                        const CloudConstPtr& cloud_hull,
                        Cloud& result)
  {
    pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
    pcl::PointIndices::Ptr inliers_polygon(new pcl::PointIndices());
    polygon_extract.setHeightLimits(0.01, 10.0);
    polygon_extract.setInputPlanarHull(cloud_hull);
    polygon_extract.setInputCloud(cloud);
    polygon_extract.segment(*inliers_polygon);
    {
      pcl::ExtractIndices<PointType> extract_positive;
      extract_positive.setNegative(false);
      extract_positive.setInputCloud(cloud);
      extract_positive.setIndices(inliers_polygon);
      extract_positive.filter(result);
    }
  }

  void
  removeZeroPoints(const CloudConstPtr& cloud, Cloud& result)
  {
    for (const auto& point : *cloud) {
      if (!(std::abs(point.x) < 0.01 && std::abs(point.y) < 0.01 &&
            std::abs(point.z) < 0.01) &&
          !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
        result.push_back(point);
    }

    result.width = result.size();
    result.height = 1;
    result.is_dense = true;
  }

  void
  extractSegmentCluster(const CloudConstPtr& cloud,
                        const std::vector<pcl::PointIndices>& cluster_indices,
                        const int segment_index,
                        Cloud& result)
  {
    pcl::PointIndices segmented_indices = cluster_indices[segment_index];
    for (const auto& index : segmented_indices.indices) {
      PointType point = (*cloud)[index];
      result.push_back(point);
    }
    result.width = result.size();
    result.height = 1;
    result.is_dense = true;
  }

  void
  cloud_cb(const CloudConstPtr& cloud)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    double start = pcl::getTime();
    FPS_CALC_BEGIN;
    cloud_pass_.reset(new Cloud);
    cloud_pass_downsampled_.reset(new Cloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    filterPassThrough(cloud, *cloud_pass_);
    if (counter_ < 10) {
      gridSample(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
    }
    else if (counter_ == 10) {
      cloud_pass_downsampled_ = cloud_pass_;
      CloudPtr target_cloud;
      if (use_convex_hull_) {
        planeSegmentation(cloud_pass_downsampled_, *coefficients, *inliers);
        if (inliers->indices.size() > 3) {
          CloudPtr cloud_projected(new Cloud);
          cloud_hull_.reset(new Cloud);
          nonplane_cloud_.reset(new Cloud);

          planeProjection(cloud_pass_downsampled_, *cloud_projected, coefficients);
          convexHull(cloud_projected, *cloud_hull_, hull_vertices_);

          extractNonPlanePoints(cloud_pass_downsampled_, cloud_hull_, *nonplane_cloud_);
          target_cloud = nonplane_cloud_;
        }
        else {
          PCL_WARN("cannot segment plane\n");
        }
      }
      else {
        PCL_WARN("without plane segmentation\n");
        target_cloud = cloud_pass_downsampled_;
      }

      if (target_cloud != nullptr) {
        PCL_INFO("segmentation, please wait...\n");
        std::vector<pcl::PointIndices> cluster_indices;
        euclideanSegment(target_cloud, cluster_indices);
        if (!cluster_indices.empty()) {
          // select the cluster to track
          CloudPtr temp_cloud(new Cloud);
          extractSegmentCluster(target_cloud, cluster_indices, 0, *temp_cloud);
          Eigen::Vector4f c;
          pcl::compute3DCentroid<RefPointType>(*temp_cloud, c);
          int segment_index = 0;
          double segment_distance = c[0] * c[0] + c[1] * c[1];
          for (std::size_t i = 1; i < cluster_indices.size(); i++) {
            temp_cloud.reset(new Cloud);
            extractSegmentCluster(target_cloud, cluster_indices, int(i), *temp_cloud);
            pcl::compute3DCentroid<RefPointType>(*temp_cloud, c);
            double distance = c[0] * c[0] + c[1] * c[1];
            if (distance < segment_distance) {
              segment_index = int(i);
              segment_distance = distance;
            }
          }

          segmented_cloud_.reset(new Cloud);
          extractSegmentCluster(
              target_cloud, cluster_indices, segment_index, *segmented_cloud_);
          RefCloudPtr ref_cloud(new RefCloud);
          ref_cloud = segmented_cloud_;
          RefCloudPtr nonzero_ref(new RefCloud);
          removeZeroPoints(ref_cloud, *nonzero_ref);

          PCL_INFO("calculating cog\n");

          RefCloudPtr transed_ref(new RefCloud);
          pcl::compute3DCentroid<RefPointType>(*nonzero_ref, c);
          Eigen::Affine3f trans = Eigen::Affine3f::Identity();
          trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
          pcl::transformPointCloud<RefPointType>(
              *nonzero_ref, *transed_ref, trans.inverse());
          CloudPtr transed_ref_downsampled(new Cloud);
          gridSample(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
          tracker_->setReferenceCloud(transed_ref_downsampled);
          tracker_->setTrans(trans);
          reference_ = transed_ref;
          tracker_->setMinIndices(ref_cloud->size() / 2);
        }
        else {
          PCL_WARN("euclidean segmentation failed\n");
        }
      }
    }
    else {
      gridSampleApprox(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
      tracking(cloud_pass_downsampled_);
    }

    new_cloud_ = true;
    double end = pcl::getTime();
    computation_time_ = end - start;
    FPS_CALC_END("computation");
    counter_++;
  }

  void
  run()
  {
    pcl::OpenNIGrabber interface(device_id_);
    std::function<void(const CloudConstPtr&)> f = [this](const CloudConstPtr& cloud) {
      cloud_cb(cloud);
    };
    interface.registerCallback(f);

    viewer_.runOnVisualizationThread(
        [this](pcl::visualization::PCLVisualizer& viz) { viz_cb(viz); }, "viz_cb");

    interface.start();

    while (!viewer_.wasStopped())
      std::this_thread::sleep_for(1s);
    interface.stop();
  }

  pcl::visualization::CloudViewer viewer_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  CloudPtr cloud_pass_;
  CloudPtr cloud_pass_downsampled_;
  CloudPtr plane_cloud_;
  CloudPtr nonplane_cloud_;
  CloudPtr cloud_hull_;
  CloudPtr segmented_cloud_;
  CloudPtr reference_;
  std::vector<pcl::Vertices> hull_vertices_;

  std::string device_id_;
  std::mutex mtx_;
  bool new_cloud_;
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
  ParticleFilter::Ptr tracker_;
  int counter_;
  bool use_convex_hull_;
  bool visualize_non_downsample_;
  bool visualize_particles_;
  double tracking_time_;
  double computation_time_;
  double downsampling_time_;
  double downsampling_grid_size_;
};

void
usage(char** argv)
{
  // clang-format off
  std::cout << "usage: " << argv[0] << " <device_id> [-C] [-g]\n\n";
  std::cout << "  -C:  initialize the pointcloud to track without plane segmentation\n";
  std::cout << "  -D: visualizing with non-downsampled pointclouds.\n";
  std::cout << "  -P: not visualizing particle cloud.\n";
  std::cout << "  -fixed: use the fixed number of the particles.\n";
  std::cout << "  -d <value>: specify the grid size of downsampling (defaults to 0.01)." << std::endl;
  // clang-format on
}

int
main(int argc, char** argv)
{
  bool use_convex_hull = true;
  bool visualize_non_downsample = false;
  bool visualize_particles = true;
  bool use_fixed = false;

  double downsampling_grid_size = 0.01;

  if (pcl::console::find_argument(argc, argv, "-C") > 0)
    use_convex_hull = false;
  if (pcl::console::find_argument(argc, argv, "-D") > 0)
    visualize_non_downsample = true;
  if (pcl::console::find_argument(argc, argv, "-P") > 0)
    visualize_particles = false;
  if (pcl::console::find_argument(argc, argv, "-fixed") > 0)
    use_fixed = true;
  pcl::console::parse_argument(argc, argv, "-d", downsampling_grid_size);
  if (argc < 2) {
    usage(argv);
    exit(1);
  }

  std::string device_id = std::string(argv[1]);

  if (device_id == "--help" || device_id == "-h") {
    usage(argv);
    exit(1);
  }

  // open kinect
  OpenNISegmentTracking<pcl::PointXYZRGBA> v(device_id,
                                             8,
                                             downsampling_grid_size,
                                             use_convex_hull,
                                             visualize_non_downsample,
                                             visualize_particles,
                                             use_fixed);
  v.run();
}

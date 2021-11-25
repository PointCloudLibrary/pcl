#include <pcl/apps/organized_segmentation_demo.h>
#include <pcl/common/angles.h>
#include <pcl/io/openni_grabber.h> // for OpenNIGrabber
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/memory.h> // for pcl::dynamic_pointer_cast

#include <boost/signals2/connection.hpp> // for boost::signals2::connection

#include <QApplication>
#include <QEvent>
#include <QMutexLocker>
#include <QObject>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>

// #include <boost/filesystem.hpp>  // for boost::filesystem::directory_iterator
#include <boost/signals2/connection.hpp> // for boost::signals2::connection

void
displayPlanarRegions(
    std::vector<pcl::PlanarRegion<PointT>,
                Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>>& regions,
    const pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  char name[1024];
  unsigned char red[6] = {255, 0, 0, 255, 255, 0};
  unsigned char grn[6] = {0, 255, 0, 255, 0, 255};
  unsigned char blu[6] = {0, 0, 255, 0, 255, 255};

  pcl::PointCloud<PointT>::Ptr contour(new pcl::PointCloud<PointT>);

  for (std::size_t i = 0; i < regions.size(); i++) {
    Eigen::Vector3f centroid = regions[i].getCentroid();
    Eigen::Vector4f model = regions[i].getCoefficients();
    pcl::PointXYZ pt1 = pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ pt2 = pcl::PointXYZ(centroid[0] + (0.5f * model[0]),
                                      centroid[1] + (0.5f * model[1]),
                                      centroid[2] + (0.5f * model[2]));
    sprintf(name, "normal_%d", unsigned(i));
    viewer->addArrow(pt2, pt1, 1.0, 0, 0, false, name);

    contour->points = regions[i].getContour();
    sprintf(name, "plane_%02d", int(i));
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(
        contour, red[i % 6], grn[i % 6], blu[i % 6]);
    if (!viewer->updatePointCloud(contour, color, name))
      viewer->addPointCloud(contour, color, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }
}

void
displayEuclideanClusters(const pcl::PointCloud<PointT>::CloudVectorType& clusters,
                         const pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  char name[1024];
  unsigned char red[6] = {255, 0, 0, 255, 255, 0};
  unsigned char grn[6] = {0, 255, 0, 255, 0, 255};
  unsigned char blu[6] = {0, 0, 255, 0, 255, 255};

  for (std::size_t i = 0; i < clusters.size(); i++) {
    sprintf(name, "cluster_%d", int(i));
    pcl::PointCloud<PointT>::ConstPtr cluster_cloud(
        new pcl::PointCloud<PointT>(clusters[i]));
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(
        cluster_cloud, red[i % 6], grn[i % 6], blu[i % 6]);
    if (!viewer->updatePointCloud(cluster_cloud, color0, name))
      viewer->addPointCloud(cluster_cloud, color0, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
  }
}

void
displayCurvature(pcl::PointCloud<PointT>& cloud,
                 pcl::PointCloud<pcl::Normal>& normals,
                 const pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  pcl::PointCloud<pcl::PointXYZRGBA> curvature_cloud = cloud;
  for (std::size_t i = 0; i < cloud.size(); i++) {
    if (normals[i].curvature < 0.04) {
      curvature_cloud[i].r = 0;
      curvature_cloud[i].g = 255;
      curvature_cloud[i].b = 0;
    }
    else {
      curvature_cloud[i].r = 255;
      curvature_cloud[i].g = 0;
      curvature_cloud[i].b = 0;
    }
  }

  if (!viewer->updatePointCloud(curvature_cloud.makeShared(), "curvature"))
    viewer->addPointCloud(curvature_cloud.makeShared(), "curvature");
}

void
displayDistanceMap(pcl::PointCloud<PointT>& cloud,
                   float* distance_map,
                   const pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  pcl::PointCloud<pcl::PointXYZRGBA> distance_map_cloud = cloud;
  for (std::size_t i = 0; i < cloud.size(); i++) {
    if (distance_map[i] < 5.0) {
      distance_map_cloud[i].r = 255;
      distance_map_cloud[i].g = 0;
      distance_map_cloud[i].b = 0;
    }
    else {
      distance_map_cloud[i].r = 0;
      distance_map_cloud[i].g = 255;
      distance_map_cloud[i].b = 0;
    }
  }

  if (!viewer->updatePointCloud(distance_map_cloud.makeShared(), "distance_map"))
    viewer->addPointCloud(distance_map_cloud.makeShared(), "distance_map");
}

void
removePreviousDataFromScreen(std::size_t prev_models_size,
                             std::size_t prev_clusters_size,
                             const pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  char name[1024];
  for (std::size_t i = 0; i < prev_models_size; i++) {
    sprintf(name, "normal_%d", unsigned(i));
    viewer->removeShape(name);

    sprintf(name, "plane_%02d", int(i));
    viewer->removePointCloud(name);
  }

  for (std::size_t i = 0; i < prev_clusters_size; i++) {
    sprintf(name, "cluster_%d", int(i));
    viewer->removePointCloud(name);
  }
}

bool
compareClusterToRegion(pcl::PlanarRegion<PointT>& region,
                       pcl::PointCloud<PointT>& cluster)
{
  Eigen::Vector4f model = region.getCoefficients();
  pcl::PointCloud<PointT> poly;
  poly.points = region.getContour();

  for (const auto& point : cluster.points) {
    double ptp_dist = std::abs(model[0] * point.x + model[1] * point.y +
                               model[2] * point.z + model[3]);
    bool in_poly = pcl::isPointIn2DPolygon<PointT>(point, poly);
    if (in_poly && ptp_dist < 0.02)
      return true;
  }
  return false;
}

bool
comparePointToRegion(PointT& pt,
                     pcl::ModelCoefficients& model,
                     pcl::PointCloud<PointT>& poly)
{
  double ptp_dist = std::abs(model.values[0] * pt.x + model.values[1] * pt.y +
                             model.values[2] * pt.z + model.values[3]);
  if (ptp_dist >= 0.1)
    return false;

  // project the point onto the plane
  Eigen::Vector3f mc(model.values[0], model.values[1], model.values[2]);
  Eigen::Vector3f pt_vec;
  pt_vec[0] = pt.x;
  pt_vec[1] = pt.y;
  pt_vec[2] = pt.z;
  Eigen::Vector3f projected(pt_vec - mc * float(ptp_dist));
  PointT projected_pt;
  projected_pt.x = projected[0];
  projected_pt.y = projected[1];
  projected_pt.z = projected[2];

  PCL_INFO("pt: %lf %lf %lf\n", projected_pt.x, projected_pt.y, projected_pt.z);

  if (pcl::isPointIn2DPolygon(projected_pt, poly)) {
    PCL_INFO("inside!\n");
    return true;
  }
  PCL_INFO("not inside!\n");
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OrganizedSegmentationDemo::OrganizedSegmentationDemo(pcl::Grabber& grabber)
: grabber_(grabber)
{
  // Create a timer
  vis_timer_ = new QTimer(this);
  vis_timer_->start(5); // 5ms

  connect(vis_timer_, SIGNAL(timeout()), this, SLOT(timeoutSlot()));

  ui_ = new Ui::MainWindow;
  ui_->setupUi(this);

  this->setWindowTitle("PCL Organized Connected Component Segmentation Demo");

#if VTK_MAJOR_VERSION > 8
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vis_.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "", false));
#else
  vis_.reset(new pcl::visualization::PCLVisualizer("", false));
#endif // VTK_MAJOR_VERSION > 8
  setRenderWindowCompat(*(ui_->qvtk_widget), *(vis_->getRenderWindow()));
  vis_->setupInteractor(getInteractorCompat(*(ui_->qvtk_widget)),
                        getRenderWindowCompat(*(ui_->qvtk_widget)));

  refreshView();

  vis_->getInteractorStyle()->setKeyboardModifier(
      pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

  std::function<void(const CloudConstPtr&)> f = [this](const CloudConstPtr& cloud) {
    cloud_cb(cloud);
  };
  boost::signals2::connection c = grabber_.registerCallback(f);

  connect(ui_->captureButton, SIGNAL(clicked()), this, SLOT(toggleCapturePressed()));

  connect(ui_->euclideanComparatorButton,
          SIGNAL(clicked()),
          this,
          SLOT(useEuclideanComparatorPressed()));
  connect(ui_->planeComparatorButton,
          SIGNAL(clicked()),
          this,
          SLOT(usePlaneComparatorPressed()));
  connect(ui_->rgbComparatorButton,
          SIGNAL(clicked()),
          this,
          SLOT(useRGBComparatorPressed()));
  connect(ui_->edgeAwareComparatorButton,
          SIGNAL(clicked()),
          this,
          SLOT(useEdgeAwareComparatorPressed()));

  connect(ui_->displayCurvatureButton,
          SIGNAL(clicked()),
          this,
          SLOT(displayCurvaturePressed()));
  connect(ui_->displayDistanceMapButton,
          SIGNAL(clicked()),
          this,
          SLOT(displayDistanceMapPressed()));
  connect(ui_->displayNormalsButton,
          SIGNAL(clicked()),
          this,
          SLOT(displayNormalsPressed()));

  connect(ui_->disableRefinementButton,
          SIGNAL(clicked()),
          this,
          SLOT(disableRefinementPressed()));
  connect(ui_->planarRefinementButton,
          SIGNAL(clicked()),
          this,
          SLOT(usePlanarRefinementPressed()));

  connect(ui_->disableClusteringButton,
          SIGNAL(clicked()),
          this,
          SLOT(disableClusteringPressed()));
  connect(ui_->euclideanClusteringButton,
          SIGNAL(clicked()),
          this,
          SLOT(useEuclideanClusteringPressed()));

  capture_ = false;
  previous_data_size_ = 0;
  previous_clusters_size_ = 0;
  data_modified_ = true;

  display_normals_ = false;
  display_curvature_ = false;
  display_distance_map_ = false;

  use_planar_refinement_ = true;
  use_clustering_ = false;

  // Set up Normal Estimation
  // ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.02f); // set as default, well performing for tabletop
                                     // objects as imaged by a primesense sensor
  ne.setNormalSmoothingSize(20.0f);

  plane_comparator_.reset(new pcl::PlaneCoefficientComparator<PointT, pcl::Normal>());
  euclidean_comparator_.reset(
      new pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>());
  rgb_comparator_.reset(new pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal>());
  edge_aware_comparator_.reset(
      new pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>());
  euclidean_cluster_comparator_ =
      pcl::EuclideanClusterComparator<PointT, pcl::Label>::Ptr(
          new pcl::EuclideanClusterComparator<PointT, pcl::Label>());

  // Set up Organized Multi Plane Segmentation
  mps.setMinInliers(10000u);
  mps.setAngularThreshold(
      pcl::deg2rad(3.0)); // 3 degrees, set as default, well performing for tabletop
                          // objects as imaged by a primesense sensor
  mps.setDistanceThreshold(0.02); // 2cm, set as default, well performing for tabletop
                                  // objects as imaged by a primesense sensor

  PCL_INFO("starting grabber\n");
  grabber_.start();
}

void
OrganizedSegmentationDemo::refreshView()
{
#if VTK_MAJOR_VERSION > 8
  ui_->qvtk_widget->renderWindow()->Render();
#else
  ui_->qvtk_widget->update();
#endif // VTK_MAJOR_VERSION > 8
}

void
OrganizedSegmentationDemo::cloud_cb(const CloudConstPtr& cloud)
{
  if (!capture_)
    return;
  QMutexLocker locker(&mtx_);
  FPS_CALC("computation");

  // Estimate Normals
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(cloud);
  ne.compute(*normal_cloud);
  float* distance_map = ne.getDistanceMap();
  pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>::Ptr eapc =
      pcl::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>>(
          edge_aware_comparator_);
  eapc->setDistanceMap(distance_map);
  eapc->setDistanceThreshold(0.01f, false);

  // Segment Planes
  double mps_start = pcl::getTime();
  std::vector<pcl::PlanarRegion<PointT>,
              Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>>
      regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  mps.setInputNormals(normal_cloud);
  mps.setInputCloud(cloud);
  if (use_planar_refinement_) {
    mps.segmentAndRefine(regions,
                         model_coefficients,
                         inlier_indices,
                         labels,
                         label_indices,
                         boundary_indices);
  }
  else {
    mps.segment(regions);
  }
  double mps_end = pcl::getTime();
  std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;

  // Segment Objects
  pcl::PointCloud<PointT>::CloudVectorType clusters;

  if (use_clustering_ && !regions.empty()) {
    pcl::EuclideanClusterComparator<PointT, pcl::Label>::ExcludeLabelSetPtr
        plane_labels(
            new pcl::EuclideanClusterComparator<PointT, pcl::Label>::ExcludeLabelSet);
    for (std::size_t i = 0; i < label_indices.size(); ++i)
      if (label_indices[i].indices.size() > mps.getMinInliers())
        plane_labels->insert(i);

    euclidean_cluster_comparator_->setInputCloud(cloud);
    euclidean_cluster_comparator_->setLabels(labels);
    euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
    euclidean_cluster_comparator_->setDistanceThreshold(0.01f, false);

    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label>
        euclidean_segmentation(euclidean_cluster_comparator_);
    euclidean_segmentation.setInputCloud(cloud);
    euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);

    for (const auto& euclidean_label_index : euclidean_label_indices) {
      if (euclidean_label_index.indices.size() > 1000u) {
        pcl::PointCloud<PointT> cluster;
        pcl::copyPointCloud(*cloud, euclidean_label_index.indices, cluster);
        clusters.push_back(cluster);
      }
    }

    PCL_INFO("Got %d euclidean clusters!\n", clusters.size());
  }

  {
    QMutexLocker vis_locker(&vis_mtx_);
    prev_cloud_ = *cloud;
    prev_normals_ = *normal_cloud;
    prev_regions_ = regions;
    prev_distance_map_ = distance_map;
    prev_clusters_ = clusters;
    data_modified_ = true;
  }
}

void
OrganizedSegmentationDemo::timeoutSlot()
{
  {
    QMutexLocker vis_locker(&vis_mtx_);
    if (capture_ && data_modified_) {
      removePreviousDataFromScreen(previous_data_size_, previous_clusters_size_, vis_);
      if (!vis_->updatePointCloud(prev_cloud_.makeShared(), "cloud")) {
        vis_->addPointCloud(prev_cloud_.makeShared(), "cloud");
        vis_->resetCameraViewpoint("cloud");
      }

      displayPlanarRegions(prev_regions_, vis_);

      if (display_curvature_)
        displayCurvature(prev_cloud_, prev_normals_, vis_);
      else
        vis_->removePointCloud("curvature");

      if (display_distance_map_)
        displayDistanceMap(prev_cloud_, prev_distance_map_, vis_);
      else
        vis_->removePointCloud("distance_map");

      if (display_normals_) {
        vis_->removePointCloud("normals");
        vis_->addPointCloudNormals<PointT, pcl::Normal>(
            prev_cloud_.makeShared(), prev_normals_.makeShared(), 10, 0.05f, "normals");
        vis_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
      }
      else {
        vis_->removePointCloud("normals");
      }

      displayEuclideanClusters(prev_clusters_, vis_);

      previous_data_size_ = prev_regions_.size();
      previous_clusters_size_ = prev_clusters_.size();
      data_modified_ = false;
    }
  }
  refreshView();
}

void
OrganizedSegmentationDemo::useEuclideanComparatorPressed()
{
  QMutexLocker locker(&mtx_);
  PCL_INFO("Setting Comparator to Euclidean\n");
  mps.setComparator(euclidean_comparator_);
}

void
OrganizedSegmentationDemo::useRGBComparatorPressed()
{
  QMutexLocker locker(&mtx_);
  PCL_INFO("Setting Comparator to RGB\n");
  mps.setComparator(rgb_comparator_);
}

void
OrganizedSegmentationDemo::usePlaneComparatorPressed()
{
  QMutexLocker locker(&mtx_);
  PCL_INFO("Setting Comparator to Plane\n");
  mps.setComparator(plane_comparator_);
}

void
OrganizedSegmentationDemo::useEdgeAwareComparatorPressed()
{
  QMutexLocker locker(&mtx_);
  PCL_INFO("Setting Comparator to edge aware\n");
  mps.setComparator(edge_aware_comparator_);
}

void
OrganizedSegmentationDemo::displayCurvaturePressed()
{
  display_curvature_ = !display_curvature_;
}

void
OrganizedSegmentationDemo::displayDistanceMapPressed()
{
  display_distance_map_ = !display_distance_map_;
}

void
OrganizedSegmentationDemo::displayNormalsPressed()
{
  display_normals_ = !display_normals_;
}

int
main(int argc, char** argv)
{
  QApplication app(argc, argv);

  pcl::OpenNIGrabber grabber("#1");

  OrganizedSegmentationDemo seg_demo(grabber);
  seg_demo.show();
  return QApplication::exec();
}

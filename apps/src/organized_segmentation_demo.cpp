#include <pcl/common/angles.h>
#include <pcl/apps/organized_segmentation_demo.h>
//QT4
#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <vtkRenderWindow.h>

void
displayPlanarRegions (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, 
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

  pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);

  for (size_t i = 0; i < regions.size (); i++)
  {
    Eigen::Vector3f centroid = regions[i].getCentroid ();
    Eigen::Vector4f model = regions[i].getCoefficients ();
    pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                       centroid[1] + (0.5f * model[1]),
                                       centroid[2] + (0.5f * model[2]));
    sprintf (name, "normal_%d", unsigned (i));
    viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
    
    contour->points = regions[i].getContour ();
    sprintf (name, "plane_%02d", int (i));
    pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i%6], grn[i%6], blu[i%6]);
    if(!viewer->updatePointCloud(contour, color, name))
      viewer->addPointCloud (contour, color, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }
}

void
displayEuclideanClusters (const pcl::PointCloud<PointT>::CloudVectorType &clusters, 
                          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

  for (size_t i = 0; i < clusters.size (); i++)
  {
    sprintf (name, "cluster_%d" , int (i));
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),red[i%6],grn[i%6],blu[i%6]);
    if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name))
      viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
  }
}

void
displayCurvature (pcl::PointCloud<PointT>& cloud, pcl::PointCloud<pcl::Normal>& normals, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  pcl::PointCloud<pcl::PointXYZRGBA> curvature_cloud = cloud;
  for (size_t i  = 0; i < cloud.points.size (); i++)
  {
    if (normals.points[i].curvature < 0.04)
    {
      curvature_cloud.points[i].r = 0;
      curvature_cloud.points[i].g = 255;
      curvature_cloud.points[i].b = 0;
    }
    else
    {
      curvature_cloud.points[i].r = 255;
      curvature_cloud.points[i].g = 0;
      curvature_cloud.points[i].b = 0;
    }
  }
  
  if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(curvature_cloud), "curvature"))
    viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(curvature_cloud), "curvature");
  
}

void
displayDistanceMap (pcl::PointCloud<PointT>& cloud, float* distance_map, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  pcl::PointCloud<pcl::PointXYZRGBA> distance_map_cloud = cloud;
  for (size_t i  = 0; i < cloud.points.size (); i++)
  {
    if (distance_map[i] < 5.0)
    {
      distance_map_cloud.points[i].r = 255;
      distance_map_cloud.points[i].g = 0;
      distance_map_cloud.points[i].b = 0;
    }
    else
    {
      distance_map_cloud.points[i].r = 0;
      distance_map_cloud.points[i].g = 255;
      distance_map_cloud.points[i].b = 0;
    }
  }
  
  if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(distance_map_cloud), "distance_map"))
    viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(distance_map_cloud), "distance_map");
}

void
removePreviousDataFromScreen (size_t prev_models_size, size_t prev_clusters_size, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  for (size_t i = 0; i < prev_models_size; i++)
  {
    sprintf (name, "normal_%d", unsigned (i));
    viewer->removeShape (name);
    
    sprintf (name, "plane_%02d", int (i));
    viewer->removePointCloud (name);
  }
  
  for (size_t i = 0; i < prev_clusters_size; i++)
  {
    sprintf (name, "cluster_%d", int (i));
    viewer->removePointCloud (name);
  }
}

bool
compareClusterToRegion (pcl::PlanarRegion<PointT>& region, pcl::PointCloud<PointT>& cluster)
{
  Eigen::Vector4f model = region.getCoefficients ();
  pcl::PointCloud<PointT> poly;
  poly.points = region.getContour ();
  
  for (size_t i = 0; i < cluster.points.size (); i++)
  {
    double ptp_dist = fabs (model[0] * cluster.points[i].x +
                            model[1] * cluster.points[i].y +
                            model[2] * cluster.points[i].z +
                            model[3]);
    bool in_poly = pcl::isPointIn2DPolygon<PointT> (cluster.points[i], poly);
    if (in_poly && ptp_dist < 0.02)
      return true;
  }
  return false;
}

bool
comparePointToRegion (PointT& pt, pcl::ModelCoefficients& model, pcl::PointCloud<PointT>& poly)
{
  //bool dist_ok;
  
  double ptp_dist = fabs (model.values[0] * pt.x +
                          model.values[1] * pt.y +
                          model.values[2] * pt.z +
                          model.values[3]);
  if (ptp_dist >= 0.1)
    return (false);
//  else
//    dist_ok = true;

  //project the point onto the plane
  Eigen::Vector3f mc (model.values[0], model.values[1], model.values[2]);
  Eigen::Vector3f pt_vec;
  pt_vec[0] = pt.x;
  pt_vec[1] = pt.y;
  pt_vec[2] = pt.z;
  Eigen::Vector3f projected (pt_vec - mc * float (ptp_dist));
  PointT projected_pt;
  projected_pt.x = projected[0];
  projected_pt.y = projected[1];
  projected_pt.z = projected[2];  

  PCL_INFO ("pt: %lf %lf %lf\n", projected_pt.x, projected_pt.y, projected_pt.z);

  if (pcl::isPointIn2DPolygon (projected_pt, poly))
  {
    PCL_INFO ("inside!\n");
    return true;
  }
  else
  {
    PCL_INFO ("not inside!\n");
    return false;
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OrganizedSegmentationDemo::OrganizedSegmentationDemo (pcl::Grabber& grabber) : grabber_ (grabber)
{
  //Create a timer
  vis_timer_ = new QTimer (this);
  vis_timer_->start (5);//5ms

  connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot()));

  ui_ = new Ui::MainWindow;
  ui_->setupUi (this);
  
  this->setWindowTitle ("PCL Organized Connected Component Segmentation Demo");
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  ui_->qvtk_widget->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (ui_->qvtk_widget->GetInteractor (), ui_->qvtk_widget->GetRenderWindow ());
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  ui_->qvtk_widget->update ();

  boost::function<void (const CloudConstPtr&)> f = boost::bind (&OrganizedSegmentationDemo::cloud_cb, this, _1);
  boost::signals2::connection c = grabber_.registerCallback(f);
  
  connect (ui_->captureButton, SIGNAL(clicked()), this, SLOT(toggleCapturePressed()));

  connect (ui_->euclideanComparatorButton, SIGNAL (clicked ()), this, SLOT (useEuclideanComparatorPressed ()));
  connect (ui_->planeComparatorButton, SIGNAL (clicked ()), this, SLOT (usePlaneComparatorPressed ()));
  connect (ui_->rgbComparatorButton, SIGNAL (clicked ()), this, SLOT (useRGBComparatorPressed ()));
  connect (ui_->edgeAwareComparatorButton, SIGNAL (clicked ()), this, SLOT (useEdgeAwareComparatorPressed ()));

  connect (ui_->displayCurvatureButton, SIGNAL (clicked ()), this, SLOT (displayCurvaturePressed ()));
  connect (ui_->displayDistanceMapButton, SIGNAL (clicked ()), this, SLOT (displayDistanceMapPressed ()));
  connect (ui_->displayNormalsButton, SIGNAL (clicked ()), this, SLOT (displayNormalsPressed ()));

  connect (ui_->disableRefinementButton, SIGNAL (clicked ()), this, SLOT (disableRefinementPressed ()));
  connect (ui_->planarRefinementButton, SIGNAL (clicked ()), this, SLOT (usePlanarRefinementPressed ()));

  connect (ui_->disableClusteringButton, SIGNAL (clicked ()), this, SLOT (disableClusteringPressed ()));
  connect (ui_->euclideanClusteringButton, SIGNAL (clicked ()), this, SLOT (useEuclideanClusteringPressed ()));

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
  //ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor (0.02f);
  ne.setNormalSmoothingSize (20.0f);

  plane_comparator_.reset (new pcl::PlaneCoefficientComparator<PointT, pcl::Normal> ());
  euclidean_comparator_.reset (new pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal> ());
  rgb_comparator_.reset (new pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal> ());
  edge_aware_comparator_.reset (new pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> ());
  euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());

  // Set up Organized Multi Plane Segmentation
  mps.setMinInliers (10000);
  mps.setAngularThreshold (pcl::deg2rad (3.0)); //3 degrees
  mps.setDistanceThreshold (0.02); //2cm
  

  PCL_INFO ("starting grabber\n");
  grabber_.start ();
}

void 
OrganizedSegmentationDemo::cloud_cb (const CloudConstPtr& cloud)
{  
  if (!capture_)
    return;
  QMutexLocker locker (&mtx_);
  FPS_CALC ("computation");

  // Estimate Normals
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud (cloud);
  ne.compute (*normal_cloud);
  float* distance_map = ne.getDistanceMap ();
  boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> > eapc = boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> >(edge_aware_comparator_);
  eapc->setDistanceMap (distance_map);
  eapc->setDistanceThreshold (0.01f, false);

  // Segment Planes
  double mps_start = pcl::getTime ();
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;  
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  mps.setInputNormals (normal_cloud);
  mps.setInputCloud (cloud);
  if (use_planar_refinement_)
  {
    mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
  }
  else
  {
    mps.segment (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
  }
  double mps_end = pcl::getTime ();
  std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;

  //Segment Objects
  pcl::PointCloud<PointT>::CloudVectorType clusters;

  if (use_clustering_ && regions.size () > 0)
  {
    std::vector<bool> plane_labels;
    plane_labels.resize (label_indices.size (), false);
    for (size_t i = 0; i < label_indices.size (); i++)
    {
      if (label_indices[i].indices.size () > 10000)
      {
        plane_labels[i] = true;
      }
    }  
    
    euclidean_cluster_comparator_->setInputCloud (cloud);
    euclidean_cluster_comparator_->setLabels (labels);
    euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
    euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);

    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
    euclidean_segmentation.setInputCloud (cloud);
    euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
    
    for (size_t i = 0; i < euclidean_label_indices.size (); i++)
    {
      if (euclidean_label_indices[i].indices.size () > 1000)
      {
        pcl::PointCloud<PointT> cluster;
        pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,cluster);
        clusters.push_back (cluster);
      }    
    }
    
    PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
  }          

  {  
    QMutexLocker vis_locker (&vis_mtx_);
    prev_cloud_ = *cloud;
    prev_normals_ = *normal_cloud;
    prev_regions_ = regions;
    prev_distance_map_ = distance_map;
    prev_clusters_ = clusters;
    data_modified_ = true;
  }
}

void 
OrganizedSegmentationDemo::timeoutSlot ()
{
  {
    QMutexLocker vis_locker (&vis_mtx_);
    if (capture_ && data_modified_)
    {
      removePreviousDataFromScreen (previous_data_size_, previous_clusters_size_, vis_);
      if (!vis_->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(prev_cloud_), "cloud"))
      {
        vis_->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(prev_cloud_), "cloud");
        vis_->resetCameraViewpoint ("cloud");
      }

      displayPlanarRegions (prev_regions_, vis_);

      if (display_curvature_)
        displayCurvature (prev_cloud_, prev_normals_, vis_);
      else
        vis_->removePointCloud ("curvature");

      if (display_distance_map_)
        displayDistanceMap (prev_cloud_, prev_distance_map_, vis_);
      else
        vis_->removePointCloud ("distance_map");

      if (display_normals_)
      {
        vis_->removePointCloud ("normals");
        vis_->addPointCloudNormals<PointT,pcl::Normal>(boost::make_shared<pcl::PointCloud<PointT> >(prev_cloud_), boost::make_shared<pcl::PointCloud<pcl::Normal> >(prev_normals_), 10, 0.05f, "normals");
        vis_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
      }
      else
      {
        vis_->removePointCloud ("normals");
      }
      
      displayEuclideanClusters (prev_clusters_,vis_);

      previous_data_size_ = prev_regions_.size ();
      previous_clusters_size_ = prev_clusters_.size ();
      data_modified_ = false;
    }
  }
  
  ui_->qvtk_widget->update();
}

void
OrganizedSegmentationDemo::useEuclideanComparatorPressed ()
{
  QMutexLocker locker (&mtx_);
  PCL_INFO ("Setting Comparator to Euclidean\n");
  mps.setComparator (euclidean_comparator_);
}

void
OrganizedSegmentationDemo::useRGBComparatorPressed ()
{
  QMutexLocker locker (&mtx_);
  PCL_INFO ("Setting Comparator to RGB\n");
  mps.setComparator (rgb_comparator_);
}

void
OrganizedSegmentationDemo::usePlaneComparatorPressed ()
{
  QMutexLocker locker (&mtx_);
  PCL_INFO ("Setting Comparator to Plane\n");
  mps.setComparator (plane_comparator_);
}

void
OrganizedSegmentationDemo::useEdgeAwareComparatorPressed ()
{
  QMutexLocker locker (&mtx_);
  PCL_INFO ("Setting Comparator to edge aware\n");
  mps.setComparator (edge_aware_comparator_);
}

void
OrganizedSegmentationDemo::displayCurvaturePressed ()
{
  display_curvature_ = !display_curvature_;
}

void
OrganizedSegmentationDemo::displayDistanceMapPressed ()
{
  display_distance_map_ = !display_distance_map_;
}

void
OrganizedSegmentationDemo::displayNormalsPressed ()
{
  display_normals_ = !display_normals_;
}

int
main (int argc, char ** argv)
{
  QApplication app(argc, argv);
  
  //PCL_INFO ("Creating PCD Grabber\n");
  //std::vector<std::string> pcd_files;
  //boost::filesystem::directory_iterator end_itr;
  //for (boost::filesystem::directory_iterator itr("/u/atrevor/data/sushi_long_pcds_compressed"); itr != end_itr; ++itr)
  //{
  //  pcd_files.push_back (itr->path ().string ());
  //  std::cout << "added: " << itr->path ().string () << std::endl;
  //}
  //sort (pcd_files.begin (),pcd_files.end ());

  //pcl::PCDGrabber<pcl::PointXYZRGB> grabber(pcd_files,5.0,false);
  //PCL_INFO ("PCD Grabber created\n");

  pcl::OpenNIGrabber grabber ("#1");
  
  OrganizedSegmentationDemo seg_demo (grabber);
  seg_demo.show();
  return (app.exec ());
}

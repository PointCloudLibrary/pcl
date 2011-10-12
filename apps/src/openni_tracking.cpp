#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/auto.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/octree.h>
#include <pcl/search/organized_neighbor.h>

#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
  {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == 10)                          \
    {                                           \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
  }

using namespace pcl::tracking;

template <typename PointType>
class OpenNISegmentTracking
{
public:
  //typedef pcl::PointXYZRGBNormal RefPointType;
  typedef pcl::PointXYZRGB RefPointType;
  //typedef pcl::PointXYZ RefPointType;
  typedef ParticleXYZRPY ParticleT;
  
  typedef pcl::PointCloud<PointType> Cloud;
  typedef pcl::PointCloud<RefPointType> RefCloud;
  typedef typename RefCloud::Ptr RefCloudPtr;
  typedef typename RefCloud::ConstPtr RefCloudConstPtr;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef ParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
  //typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
  typedef typename ParticleFilter::CoherencePtr CoherencePtr;
  typedef typename pcl::KdTreeFLANN<PointType> KdTree;
  typedef typename KdTree::Ptr KdTreePtr;
  OpenNISegmentTracking (const std::string& device_id, int thread_nr, bool use_convex_hull, bool use_cog)
  : viewer_ ("PCL OpenNI Tracking Viewer")
  , device_id_ (device_id)
  , new_cloud_ (false)
  , ne_ (thread_nr)
  , firstp_ (0)
  , use_convex_hull_ (use_convex_hull)
  , use_cog_ (use_cog)
  {
    KdTreePtr tree (new KdTree (false));
    ne_.setSearchMethod (tree);
    ne_.setRadiusSearch (0.03);
    
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.02);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
    tracker_ = boost::shared_ptr<ParticleFilter> (new ParticleFilter (thread_nr));
    //tracker_ = boost::shared_ptr<ParticleFilter> (new ParticleFilter ());
    default_step_covariance[3] *= 50.0;
    default_step_covariance[4] *= 50.0;
    default_step_covariance[5] *= 50.0;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    //tracker_->setParticleNum (4000);
    tracker_->setParticleNum (400);
    //tracker_->setParticleNum (10);
    tracker_->setResampleLikelihoodThr(0.00);
    // setup coherences
    NearestPairPointCloudCoherence<RefPointType>::Ptr coherence = NearestPairPointCloudCoherence<RefPointType>::Ptr
      (new NearestPairPointCloudCoherence<RefPointType> ());
    
    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);
    
    // boost::shared_ptr<HSVColorCoherence<RefPointType> > color_coherence
    //   = boost::shared_ptr<HSVColorCoherence<RefPointType> > (new HSVColorCoherence<RefPointType> ());
    // color_coherence->setWeight (0.01);
    // coherence->addPointCoherence (color_coherence);
    
    // boost::shared_ptr<NormalCoherence<RefPointType> > normal_coherence
    //   = boost::shared_ptr<NormalCoherence<RefPointType> > (new NormalCoherence<RefPointType> ());
    // normal_coherence->setWeight (0.01);
    // coherence->addPointCoherence (normal_coherence);
    
    //boost::shared_ptr<pcl::search::KdTree<RefPointType> > search (new pcl::search::KdTree<RefPointType> (false));
    //coherence->setSearchMethodPolicy (pcl::tracking::NEAREST_NEIGHBOR);
    boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.02));
    //boost::shared_ptr<pcl::search::OrganizedNeighbor<RefPointType> > search (new pcl::search::OrganizedNeighbor<RefPointType>);
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);
    tracker_->setCloudCoherence (coherence);
  }

  bool
  drawParticles (pcl::visualization::PCLVisualizer& viz)
  {
    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
    if (particles)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      for (size_t i = 0; i < particles->points.size (); i++)
      {
        pcl::PointXYZ point;
        
        ParticleXYZRPY particle = particles->points[i];
        point.x = particles->points[i].x;
        point.y = particles->points[i].y;
        point.z = particles->points[i].z;
        particle_cloud->points.push_back (point);
      }
      
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color (particle_cloud, 0, 0, 255);
        if (!viz.updatePointCloud (particle_cloud, blue_color, "particle cloud"))
          viz.addPointCloud (particle_cloud, blue_color, "particle cloud");
      }
      return true;
    }
    else
    {
      PCL_WARN ("no particles\n");
      return false;
    }
  }
  
  void
  drawResult (pcl::visualization::PCLVisualizer& viz)
  {
    ParticleXYZRPY result = tracker_->getResult ();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
    RefCloudPtr result_cloud (new RefCloud ());
    
    pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud,
                                            transformation);
    
    {
      pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color (result_cloud, 255, 0, 0);
      if (!viz.updatePointCloud (result_cloud, red_color, "resultcloud"))
        viz.addPointCloud (result_cloud, red_color, "resultcloud");
    }
  }

  void
  viz_cb (pcl::visualization::PCLVisualizer& viz)
  {
    boost::mutex::scoped_lock lock (mtx_);
    viz.setBackgroundColor (0.8, 0.8, 0.8);
    
    if (!cloud_pass_downsampled_)
    {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
      return;
    }
    
    if (new_cloud_ && cloud_pass_downsampled_)
      if (!viz.updatePointCloud (cloud_pass_downsampled_, "cloudpass"))
      {
        viz.addPointCloud (cloud_pass_downsampled_, "cloudpass");
        viz.resetCameraViewpoint ("cloudpass");
      }
    
    if (new_cloud_)
    {
      bool ret = drawParticles (viz);
      if (ret)
      {
        drawResult (viz);
        viz.removeShape ("N");
        viz.addText ((boost::format ("number of Reference PointClouds: %d") % tracker_->getReferenceCloud ()->points.size ()).str (),
                     10, 20, 20, 0.0, 0.0, 0.0, "N");
        
        viz.removeShape ("M");
        viz.addText ((boost::format ("number of Measured PointClouds:  %d") % cloud_pass_downsampled_->points.size ()).str (),
                     10, 40, 20, 0.0, 0.0, 0.0, "M");
        
        viz.removeShape ("tracking");
        viz.addText ((boost::format ("tracking:        %f fps") % (1.0 / tracking_time_)).str (),
                     10, 60, 20, 0.0, 0.0, 0.0, "tracking");
        
        viz.removeShape ("downsampling");
        viz.addText ((boost::format ("downsampling:    %f fps") % (1.0 / downsampling_time_)).str (),
                     10, 80, 20, 0.0, 0.0, 0.0, "downsampling");
        
        viz.removeShape ("computation");
        viz.addText ((boost::format ("computation:     %f fps") % (1.0 / computation_time_)).str (),
                     10, 100, 20, 0.0, 0.0, 0.0, "computation");
      }
    }
    new_cloud_ = false;
  }

  void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
  {
    FPS_CALC_BEGIN;
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 10.0);
    //pass.setFilterLimits (0.0, 1.5);
    //pass.setFilterLimits (0.0, 0.6);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
    FPS_CALC_END("filterPassThrough");
  }

  void euclideanSegment (const CloudConstPtr &cloud,
                         std::vector<pcl::PointIndices> &cluster_indices)
  {
    FPS_CALC_BEGIN;
    pcl::EuclideanClusterExtraction<PointType> ec;
    KdTreePtr tree (new KdTree ());
    
    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    //ec.setMaxClusterSize (400);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    FPS_CALC_END("euclideanSegmentation");
  }
  
  void gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime ();
    pcl::VoxelGrid<PointType> grid;
    //pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    grid.setInputCloud (cloud);
    grid.filter (result);
    //result = *cloud;
    double end = pcl::getTime ();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }
  
  void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime ();
    //pcl::VoxelGrid<PointType> grid;
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    grid.setInputCloud (cloud);
    grid.filter (result);
    //result = *cloud;
    double end = pcl::getTime ();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }
  
  void planeSegmentation (const CloudConstPtr &cloud,
                          pcl::ModelCoefficients &coefficients,
                          pcl::PointIndices &inliers)
  {
    FPS_CALC_BEGIN;
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud);
    seg.segment (inliers, coefficients);
    FPS_CALC_END("planeSegmentation");
  }

  void planeProjection (const CloudConstPtr &cloud,
                        Cloud &result,
                        const pcl::ModelCoefficients::ConstPtr &coefficients)
  {
    FPS_CALC_BEGIN;
    pcl::ProjectInliers<PointType> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (result);
    FPS_CALC_END("planeProjection");
  }

  void convexHull (const CloudConstPtr &cloud,
                   Cloud &result,
                   std::vector<pcl::Vertices> &hull_vertices)
  {
    FPS_CALC_BEGIN;
    pcl::ConvexHull<PointType> chull;
    chull.setInputCloud (cloud);
    chull.reconstruct (*cloud_hull_, hull_vertices);
    FPS_CALC_END("convexHull");
  }

  void normalEstimation (const CloudConstPtr &cloud,
                         pcl::PointCloud<pcl::Normal> &result)
  {
    FPS_CALC_BEGIN;
    ne_.setInputCloud (cloud);
    ne_.compute (result);
    FPS_CALC_END("normalEstimation");
  }
  
  void tracking (const RefCloudConstPtr &cloud)
  {
    double start = pcl::getTime ();
    FPS_CALC_BEGIN;
    tracker_->setInputCloud (cloud);
    tracker_->compute ();
    double end = pcl::getTime ();
    FPS_CALC_END("tracking");
    tracking_time_ = end - start;
  }

  void addNormalToCloud (const CloudConstPtr &cloud,
                         const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
                         RefCloud &result)
  {
    result.width = cloud->width;
    result.height = cloud->height;
    result.is_dense = cloud->is_dense;
    for (size_t i = 0; i < cloud->points.size (); i++)
    {
      RefPointType point;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;
      point.rgb = cloud->points[i].rgb;
      // point.normal[0] = normals->points[i].normal[0];
      // point.normal[1] = normals->points[i].normal[1];
      // point.normal[2] = normals->points[i].normal[2];
      result.points.push_back (point);
    }
  }

  void extractNonPlanePoints (const CloudConstPtr &cloud,
                              const CloudConstPtr &cloud_hull,
                              Cloud &result)
  {
    pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
    pcl::PointIndices::Ptr inliers_polygon (new pcl::PointIndices ());
    polygon_extract.setHeightLimits (0.01, 10.0);
    polygon_extract.setInputPlanarHull (cloud_hull);
    polygon_extract.setInputCloud (cloud);
    polygon_extract.segment (*inliers_polygon);
    {
      pcl::ExtractIndices<PointType> extract_positive;
      extract_positive.setNegative (false);
      extract_positive.setInputCloud (cloud);
      extract_positive.setIndices (inliers_polygon);
      extract_positive.filter (result);
    }
  }

  void extractSegmentCluster (const CloudConstPtr &cloud,
                              const std::vector<pcl::PointIndices> cluster_indices,
                              const int segment_index,
                              Cloud &result)
  {
    pcl::PointIndices segmented_indices = cluster_indices[segment_index];
    for (size_t i = 0; i < segmented_indices.indices.size (); i++)
    {
      PointType point = cloud->points[segmented_indices.indices[i]];
      result.points.push_back (point);
    }
    result.width = result.points.size ();
    result.height = 1;
    result.is_dense = true;
  }
  
  void
  cloud_cb (const CloudConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock (mtx_);
    double start = pcl::getTime ();
    FPS_CALC_BEGIN;
    cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    filterPassThrough (cloud, *cloud_pass_);
    if (firstp_ < 3)
    {
      ++firstp_;
      gridSample (cloud_pass_, *cloud_pass_downsampled_, 0.01);
    }
    else if (firstp_ == 3)
    {
      gridSample (cloud_pass_, *cloud_pass_downsampled_, 0.01);
      ++firstp_;
      CloudPtr target_cloud;
      if (use_convex_hull_)
      {
        planeSegmentation (cloud_pass_downsampled_, *coefficients, *inliers);
        if (inliers->indices.size () > 3)
        {
          CloudPtr cloud_projected (new Cloud);
          cloud_hull_.reset (new Cloud);
          nonplane_cloud_.reset (new Cloud);
          
          planeProjection (cloud_pass_downsampled_, *cloud_projected, coefficients);
          convexHull (cloud_projected, *cloud_hull_, hull_vertices_);
          
          extractNonPlanePoints (cloud_pass_downsampled_, cloud_hull_, *nonplane_cloud_);
          target_cloud = nonplane_cloud_;
        }
        else
        {
          PCL_WARN ("cannot segment plane\n");
        }
      }
      else
      {
        PCL_WARN ("without plane segmentation\n");
        target_cloud = cloud_pass_downsampled_;
      }
      
      if (target_cloud != NULL)
      {
        std::vector<pcl::PointIndices> cluster_indices;
        euclideanSegment (target_cloud, cluster_indices);
        if (cluster_indices.size () > 0)
        {
          // select the cluster to track
          CloudPtr temp_cloud (new Cloud);
          extractSegmentCluster (target_cloud, cluster_indices, 0, *temp_cloud);
          Eigen::Vector4f c;
          pcl::compute3DCentroid<RefPointType> (*temp_cloud, c);
          int segment_index = 0;
          double segment_distance = c[0] * c[0] + c[1] * c[1];
          for (size_t i = 1; i < cluster_indices.size (); i++)
          {
            temp_cloud.reset (new Cloud);
            extractSegmentCluster (target_cloud, cluster_indices, i, *temp_cloud);
            pcl::compute3DCentroid<RefPointType> (*temp_cloud, c);
            double distance = c[0] * c[0] + c[1] * c[1];
            if (distance < segment_distance)
            {
              segment_index = i;
              segment_distance = distance;
            }
          }
          
          segmented_cloud_.reset (new Cloud);
          extractSegmentCluster (target_cloud, cluster_indices, segment_index, *segmented_cloud_);
          //pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
          //normalEstimation (segmented_cloud_, *normals);
          RefCloudPtr ref_cloud (new RefCloud);
          //addNormalToCloud (segmented_cloud_, normals, *ref_cloud);
          ref_cloud = segmented_cloud_;
          
          if (!use_cog_)
          {
            tracker_->setReferenceCloud (ref_cloud);
            reference_ = ref_cloud;
            std::cout << "N: " << ref_cloud->points.size () << std::endl;
          }
          else
          {
            PCL_INFO ("calculating cog\n");
            Eigen::Vector4f c;
            RefCloudPtr transed_ref (new RefCloud);
            pcl::compute3DCentroid<RefPointType> (*ref_cloud, c);
            Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
            trans.translation () = Eigen::Vector3f (c[0], c[1], c[2]);
            //pcl::transformPointCloudWithNormals<RefPointType> (*ref_cloud, *transed_ref, trans.inverse());
            pcl::transformPointCloud<RefPointType> (*ref_cloud, *transed_ref, trans.inverse());
            
            std::cout << "N: " << transed_ref->points.size () << std::endl;
            tracker_->setReferenceCloud (transed_ref);
            tracker_->setTrans (trans);
            reference_ = transed_ref;
          }
          tracker_->setMinIndices (ref_cloud->points.size () / 2);
        }
        else
        {
          PCL_WARN ("euclidean segmentation failed\n");
        }
      }
    }
    else
    {
      //normals_.reset (new pcl::PointCloud<pcl::Normal>);
      //normalEstimation (cloud_pass_downsampled_, *normals_);
      //RefCloudPtr tracking_cloud (new RefCloud ());
      //addNormalToCloud (cloud_pass_downsampled_, normals_, *tracking_cloud);
      //tracking_cloud = cloud_pass_downsampled_;
      
      //*cloud_pass_downsampled_ = *cloud_pass_;
      //cloud_pass_downsampled_ = cloud_pass_;
      gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_);
      tracking (cloud_pass_downsampled_);
      
      //cloud_pass_downsampled_ = cloud_pass_;
      //tracking (cloud_pass_);
      //tracking (reference_);
      //tracking (cloud_pass_);
    }
    
    new_cloud_ = true;
    double end = pcl::getTime ();
    computation_time_ = end - start;
    FPS_CALC_END("computation");
    
  }
      
  void
  run ()
  {
    pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);
    boost::function<void (const CloudConstPtr&)> f =
      boost::bind (&OpenNISegmentTracking::cloud_cb, this, _1);
    interface->registerCallback (f);
    
    viewer_.runOnVisualizationThread (boost::bind(&OpenNISegmentTracking::viz_cb, this, _1), "viz_cb");
    
    interface->start ();
      
    while (!viewer_.wasStopped ())
      boost::this_thread::sleep(boost::posix_time::seconds(1));
    interface->stop ();
  }
  //pcl::ApproximateVoxelGrid<PointType> grid_;
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
  boost::mutex mtx_;
  bool new_cloud_;
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
  boost::shared_ptr<ParticleFilter> tracker_;
  int firstp_;
  bool use_convex_hull_;
  bool use_cog_;
  double tracking_time_;
  double computation_time_;
  double downsampling_time_;
};

void
usage (char** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> [-C] [-g]\n\n";
  std::cout << "  -C:  initialize the pointcloud to track without plane segmentation"
            << std::endl;
  std::cout << "  -c: use COG of pointcloud as the origin of pointcloud to track."
            << std::endl;
}

int
main (int argc, char** argv)
{
  bool use_convex_hull = true;
  bool use_cog = false;
  
  if (pcl::console::find_argument (argc, argv, "-C") > 0)
    use_convex_hull = false;
  if (pcl::console::find_argument (argc, argv, "-c") > 0)
    use_cog = true;
  
  if (argc < 2)
  {
    usage (argv);
    exit (1);
  }
  
  std::string device_id = std::string (argv[1]);

  if (device_id == "--help" || device_id == "-h")
  {
    usage (argv);
    exit (1);
  }
  
  // open kinect
  pcl::OpenNIGrabber grabber ("");
  PCL_INFO ("PointXYZRGB mode enabled.\n");
  //OpenNISegmentTracking<pcl::PointXYZRGB> v (device_id, 8, use_convex_hull, use_cog);
  OpenNISegmentTracking<pcl::PointXYZRGB> v (device_id, 8, use_convex_hull, use_cog);
  //OpenNISegmentTracking<pcl::PointXYZ> v (device_id, 8, use_convex_hull, use_cog);
  v.run ();
}

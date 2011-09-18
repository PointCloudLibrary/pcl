#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
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

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/auto.h>
#include <pcl/search/octree.h>

#define FPS_CALC(_WHAT_)                                                \
  do                                                                    \
  {                                                                     \
    static unsigned count = 0;                                          \
    static double last = pcl::getTime ();                               \
    if (++count == 10)                                                  \
    {                                                                   \
      double now = pcl::getTime ();                                     \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      last = now;                                                       \
    }                                                                   \
  }while(false)

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
class OpenNITracking
{
public:
  typedef pcl::PointXYZRGBNormal RefPointType;
  typedef ParticleXYZRPY ParticleT;
  
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
  typedef typename ParticleFilter::CoherencePtr CoherencePtr;
  
  OpenNITracking (const std::string& device_id, const std::string& pcd_file)
  : viewer_ ("PCL OpenNI Tracking Viewer")
  , device_id_ (device_id)
  , pcd_file_ (pcd_file)
  , sensor_view (0)
  , reference_view (0)
  , new_cloud_ (false)
  , ne_ (8)                   // 8 threads
  {
    //pass_.setFilterFieldName ("z");
    //pass_.setFilterLimits (0.0, 2.0);
    //pass_.setKeepOrganized (true);
    grid_.setFilterFieldName ("z");
    grid_.setFilterLimits (0.0, 2.0);
    grid_.setLeafSize (0.01, 0.01, 0.01);

    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setMaxIterations (1000);
    seg_.setDistanceThreshold (0.03);
    
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
    ne_.setSearchMethod (tree);
    ne_.setRadiusSearch (0.03);
    //ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::COVARIANCE_MATRIX);
    //ne_.setRectSize (50, 50);
    
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.0002);
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.1);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.5);
    default_step_covariance[3] = 0.002 * M_PI * 2.0;
    default_step_covariance[4] = 0.002 * M_PI * 2.0;
    default_step_covariance[5] = 0.002 * M_PI * 2.0;
    default_initial_mean[0] = 0.0;
    default_initial_mean[1] = 1.0;
    default_initial_mean[2] = 0.0;
    default_initial_mean[3] = 0;
    default_initial_mean[4] = 0;
    default_initial_mean[5] = 0;
    initial_noise_covariance[2] = 0.0;
    initial_noise_covariance[3] = 0.0;
    initial_noise_covariance[4] = 0.0;
    initial_noise_covariance[5] = 0.0;
    tracker_ = boost::shared_ptr<ParticleFilterTracker<RefPointType, ParticleXYZRPY> >
      (new ParticleFilterTracker<RefPointType, ParticleXYZRPY> ());
      
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    tracker_->setParticleNum (200);
    // setup coherences
    NearestPairPointCloudCoherence<RefPointType>::Ptr coherence = NearestPairPointCloudCoherence<RefPointType>::Ptr
      (new NearestPairPointCloudCoherence<RefPointType> ());
    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    distance_coherence->setWeight (10.0);
    coherence->addPointCoherence (distance_coherence);
    
    boost::shared_ptr<HSVColorCoherence<RefPointType> > color_coherence
      = boost::shared_ptr<HSVColorCoherence<RefPointType> > (new HSVColorCoherence<RefPointType> ());
    color_coherence->setWeight (0.1);
    coherence->addPointCoherence (color_coherence);

    boost::shared_ptr<NormalCoherence<RefPointType> > normal_coherence
      = boost::shared_ptr<NormalCoherence<RefPointType> > (new NormalCoherence<RefPointType> ());
    normal_coherence->setWeight (0.1);
    coherence->addPointCoherence (normal_coherence);

    pcl::search::AutotunedSearch<RefPointType>::SearchPtr oct
      (new pcl::search::AutotunedSearch<RefPointType> (pcl::search::KDTREE));

    //pcl::search::Octree<RefPointType>::Ptr oct (new pcl::search::Octree<RefPointType> (0.01));
    coherence->setSearchMethod (oct);
    
    tracker_->setCloudCoherence (coherence);
    extract_positive_.setNegative (false);
  }

  void
  drawPlaneCoordinate (pcl::visualization::PCLVisualizer& viz)
  {
      pcl::PointXYZ O, X, Y, Z;
      O.x = plane_trans_ (0, 3);
      O.y = plane_trans_ (1, 3);
      O.z = plane_trans_ (2, 3);
      X.x = O.x + plane_trans_ (0, 0) * 0.1;
      X.y = O.y + plane_trans_ (1, 0) * 0.1;
      X.z = O.z + plane_trans_ (2, 0) * 0.1;
      Y.x = O.x + plane_trans_ (0, 1) * 0.1;
      Y.y = O.y + plane_trans_ (1, 1) * 0.1;
      Y.z = O.z + plane_trans_ (2, 1) * 0.1;
      Z.x = O.x + plane_trans_ (0, 2) * 0.15;
      Z.y = O.y + plane_trans_ (1, 2) * 0.15;
      Z.z = O.z + plane_trans_ (2, 2) * 0.15;
      drawLine (viz, O, X, "x");
      drawLine (viz, O, Y, "y");
      drawLine (viz, O, Z, "z");
  }

  void
  drawSearchArea (pcl::visualization::PCLVisualizer& viz)
  {
    const ParticleXYZRPY zero_particle;
    Eigen::Affine3f trans = tracker_->getTrans ();
    Eigen::Affine3f search_origin = trans * tracker_->toEigenMatrix (zero_particle);
    Eigen::Quaternionf q = Eigen::Quaternionf (search_origin.rotation ());
    
    pcl::ModelCoefficients coefficients;
    coefficients.values.push_back (search_origin.translation ()[0]);
    coefficients.values.push_back (search_origin.translation ()[1]);
    coefficients.values.push_back (search_origin.translation ()[2]);
    coefficients.values.push_back (q.x ());
    coefficients.values.push_back (q.y ());
    coefficients.values.push_back (q.z ());
    coefficients.values.push_back (q.w ());
    
    coefficients.values.push_back (1.0);
    coefficients.values.push_back (1.0);
    coefficients.values.push_back (1.0);
    
    viz.removeShape ("searcharea");
    viz.addCube (coefficients, "searcharea");
  }

  void drawLine (pcl::visualization::PCLVisualizer& viz, const pcl::PointXYZ& from, const pcl::PointXYZ& to, const std::string& name)
  {
    viz.removeShape (name);
    viz.addLine<pcl::PointXYZ> (from, to, name);
  }
  
  void
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
    }
    else
      PCL_WARN ("no particles\n");
  }
  
  void
  drawResult (pcl::visualization::PCLVisualizer& viz)
  {
    ParticleXYZRPY result = tracker_->getResult ();
    //std::cout << "result: " << result << std::endl; //debug
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
    
    pcl::transformPointCloud<pcl::PointXYZRGBNormal> (*(tracker_->getReferenceCloud ()), *result_cloud,
                                                      transformation);
    
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> red_color (result_cloud, 255, 0, 0);
      if (!viz.updatePointCloud (result_cloud, red_color, "resultcloud"))
        viz.addPointCloud (result_cloud, red_color, "resultcloud");
    }
  }

  void
  viz_cb (pcl::visualization::PCLVisualizer& viz)
  {
    boost::mutex::scoped_lock lock (mtx_);
    if (!cloud_pass_)
    {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
      return;
    }

    // if (!viz.updatePointCloud (cloud_pass_, "cloudpass"))
    //   {
    //     viz.addPointCloud (cloud_pass_, "cloudpass");
    //     viz.resetCameraViewpoint ("cloudpass");
    //   }
      
    if (new_cloud_ && cloud_hull_)
    {
      viz.removeShape ("hull");
      viz.addPolygonMesh<PointType> (cloud_hull_, hull_vertices_, "hull");
      drawPlaneCoordinate (viz);
      drawSearchArea (viz);
    }

      
    if (new_cloud_ && nonplane_cloud_)
      if (!viz.updatePointCloud (nonplane_cloud_, "nonplane cloud"))
      {
        viz.addPointCloud (nonplane_cloud_, "nonplane cloud");
        viz.resetCameraViewpoint ("nonplane cloud");
      }
      
    if (new_cloud_ && normals_)
    {
      viz.removePointCloud ("sensor normalcloud");
      viz.addPointCloudNormals<PointType, pcl::Normal> (nonplane_cloud_, normals_, 50, 0.05, "sensor normalcloud");
    }

    if (new_cloud_)
    {
      drawParticles (viz);
      drawResult (viz);
    }

    new_cloud_ = false;
  }

  void filterPassThrough (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, Cloud &result)
  {
    FPS_CALC_BEGIN;
    //ScopedTimeMeasure ("passthroughfilter");
    //pass_.setInputCloud (cloud);
    grid_.setInputCloud (cloud);
    //pass_.filter (result);
    grid_.filter (*cloud_pass_);
    FPS_CALC_END("planeSegmentation");
  }

  void planeSegmentation (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                          pcl::ModelCoefficients &coefficients,
                          pcl::PointIndices &inliers)
  {
    FPS_CALC_BEGIN;
    seg_.setInputCloud (cloud);
    seg_.segment (inliers, coefficients);
    FPS_CALC_END("planeSegmentation");
  }

  void planeProjection (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                        Cloud &result,
                        const pcl::ModelCoefficients::ConstPtr &coefficients)
  {
    FPS_CALC_BEGIN;
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (result);
    FPS_CALC_END("planeProjection");
  }

  void convexHull (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                   Cloud &result,
                   std::vector<pcl::Vertices> &hull_vertices)
  {
    FPS_CALC_BEGIN;
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud (cloud);
    chull.reconstruct (*cloud_hull_, hull_vertices);
    FPS_CALC_END("convexHull");
  }

  void tracking (const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &cloud)
  {
    FPS_CALC_BEGIN;
    tracker_->setInputCloud (cloud);
    tracker_->compute ();
    FPS_CALC_END("tracking");
  }
  
  void
  cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
      boost::mutex::scoped_lock lock (mtx_);
      FPS_CALC_BEGIN;
      cloud_pass_.reset (new Cloud);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      
      filterPassThrough (cloud, *cloud_pass_);
      
      planeSegmentation (cloud_pass_, *coefficients, *inliers);
      
      // enough pints...
      if (inliers->indices.size () > 3)
      {
        CloudPtr cloud_projected (new Cloud ());
        planeProjection (cloud_pass_, *cloud_projected, coefficients);

        cloud_hull_.reset (new Cloud);
        convexHull (cloud_projected, *cloud_hull_, hull_vertices_);
        
        plane_trans_ = estimatePlaneCoordinate(cloud_hull_);
        
        // setup offset to tracker_
        Eigen::Affine3f affine_plane = Eigen::Affine3f (plane_trans_);
        //std::cout << "trans: " << plane_trans_ << std::endl; //debug
        Eigen::Affine3f offset = Eigen::Affine3f::Identity ();
        offset = Eigen::Translation3f (0.0, 0.8, 0.0);
        tracker_->setTrans (affine_plane * offset);
        
        pcl::PointIndices::Ptr inliers_polygon (new pcl::PointIndices ());
        pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> polygon_extract;
        nonplane_cloud_.reset (new Cloud);
        
        polygon_extract.setHeightLimits (0.01, 10.0);
        polygon_extract.setInputPlanarHull (cloud_hull_);
        polygon_extract.setInputCloud (cloud_pass_);
        polygon_extract.segment (*inliers_polygon);
      
        extract_positive_.setInputCloud (cloud_pass_);
        extract_positive_.setIndices (inliers_polygon);
      
        extract_positive_.filter (*nonplane_cloud_);

        if (!nonplane_cloud_->points.empty ())
        {
          // calc normals
          normals_.reset (new pcl::PointCloud<pcl::Normal>);
          ne_.setInputCloud (nonplane_cloud_);
          ne_.compute (*normals_);

          //std::cout << "nonplane: " << nonplane_cloud_->points.size () << std::endl; //debug
          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tracking_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
          for (size_t i = 0; i < nonplane_cloud_->points.size (); i++)
          {
            pcl::PointXYZRGBNormal point;
            point.x = nonplane_cloud_->points[i].x;
            point.y = nonplane_cloud_->points[i].y;
            point.z = nonplane_cloud_->points[i].z;
            point.rgb = nonplane_cloud_->points[i].rgb;
            point.normal[0] = normals_->points[i].normal[0];
            point.normal[1] = normals_->points[i].normal[1];
            point.normal[2] = normals_->points[i].normal[2];
            tracking_cloud->points.push_back (point);
          }

          pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr tracking_const_ptr = tracking_cloud;
          tracking (tracking_cloud);
          //tracking (cloud_pass_);
          
        }
        new_cloud_ = true;
      }
      FPS_CALC_END("computation");
    }

  Eigen::Matrix4f 
  estimatePlaneCoordinate (CloudPtr cloud_hull)
  {
    if (cloud_hull->points.size() >= 3)
    {
      Eigen::Vector3f BA (cloud_hull->points[0].x - cloud_hull->points[1].x,
                          cloud_hull->points[0].y - cloud_hull->points[1].y,
                          cloud_hull->points[0].z - cloud_hull->points[1].z);
      Eigen::Vector3f BC (cloud_hull->points[2].x - cloud_hull->points[1].x,
                          cloud_hull->points[2].y - cloud_hull->points[1].y,
                          cloud_hull->points[2].z - cloud_hull->points[1].z);
      Eigen::Vector3f z = BC.cross (BA);
      z.normalize ();
      // check the direction of z
      
      Eigen::Vector3f B (cloud_hull->points[1].x, cloud_hull->points[1].y, cloud_hull->points[1].z);
      if (B.dot (z) > 0)
        z = - z;


      // calc x, y
      Eigen::Vector3f xx = BA;
      xx.normalize ();
      Eigen::Vector3f yy = z.cross (xx);
      yy.normalize ();
      Eigen::Vector3f ux (1.0, 0.0, 0.0);
      double tmp = ux.dot (yy) / ux.dot (xx);
      double beta2 = 1 / (1 + tmp * tmp);
      double beta = - sqrt (beta2);
      double alpha = - beta * tmp;
      Eigen::Vector3f y = alpha * xx + beta * yy;
      Eigen::Vector3f x = y.cross (z);
      x.normalize ();
      y.normalize ();
      
      Eigen::Matrix4f ret = Eigen::Matrix4f::Identity ();

      // fill rotation
      for (int i = 0; i < 3; i++)
      {
        ret(i, 0) = x[i];
        ret(i, 1) = y[i];
        ret(i, 2) = z[i];
      }
      
      Eigen::Vector3f OB (cloud_hull->points[1].x, cloud_hull->points[1].y, cloud_hull->points[1].z);
      double yscale = - OB.dot (y);
      double xscale = - OB.dot (x);
      Eigen::Vector3f position = OB + yscale * y + xscale * x;
        
      for (int i = 0; i < 3; i++)
        ret (i, 3) = position[i];
        
      return ret;
    }
    return Eigen::Matrix4f::Identity ();
  }
  
  void
  run ()
  {
    // first of all, read PCD data
    pcl::PointCloud<RefPointType>::Ptr ref_cloud (new pcl::PointCloud<RefPointType>);
    pcl::PointCloud<RefPointType>::Ptr ref_cloud_downsampled (new pcl::PointCloud<RefPointType>);
    if (pcl::io::loadPCDFile<RefPointType> (pcd_file_, *ref_cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file %s \n", pcd_file_.c_str ());
      return;
    }

    // downsample the model.
    {
      pcl::VoxelGrid<RefPointType> ref_grid;
      ref_grid.setLeafSize (0.01, 0.01, 0.01);
      ref_grid.setInputCloud (ref_cloud);
      ref_grid.filter (*ref_cloud_downsampled);
    }
      
    // send PCD to tracker_
    tracker_->setReferenceCloud (ref_cloud_downsampled);
    pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
      boost::bind (&OpenNITracking::cloud_cb, this, _1);
    interface->registerCallback (f);
    viewer_.runOnVisualizationThread (boost::bind(&OpenNITracking::viz_cb, this, _1), "viz_cb");
    
    interface->start ();
      
    while (!viewer_.wasStopped ())
      boost::this_thread::sleep(boost::posix_time::seconds(1));
    interface->stop ();
  }
  
  pcl::PassThrough<PointType> pass_;
  pcl::VoxelGrid<PointType> grid_;
  pcl::SACSegmentation<PointType> seg_;
  pcl::ExtractIndices<PointType> extract_positive_;
  
  pcl::visualization::CloudViewer viewer_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  CloudPtr cloud_pass_;
  CloudPtr plane_cloud_;
  CloudPtr nonplane_cloud_;
  CloudPtr cloud_hull_;
  std::vector<pcl::Vertices> hull_vertices_;
  Eigen::Matrix4f plane_trans_;
  
  std::string device_id_;
  std::string pcd_file_;
  boost::mutex mtx_;
  int sensor_view, reference_view;
  bool new_cloud_;
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_;
  //pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne_;
  boost::shared_ptr<ParticleFilterTracker<RefPointType, ParticleXYZRPY> > tracker_;
};

void
usage (char** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <pcd_file> <options>\n\n";
}

int
main (int argc, char** argv)
{
  
  if (argc < 3)
  {
    usage (argv);
    return 1;
  }
  
  std::string device_id = std::string (argv[1]);
  std::string pcd_file = std::string (argv[2]);
  
  if (device_id == "--help" || device_id == "-h" ||
      pcd_file == "--help" || pcd_file == "-h" )
  {
    usage (argv);
    return 1;
  }

  PCL_INFO ("using %s as reference.\n", pcd_file.c_str());
  
  // open kinect
  pcl::OpenNIGrabber grabber ("");
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  {
    PCL_INFO ("PointXYZRGB mode enabled.\n");
    OpenNITracking<pcl::PointXYZRGB> v (device_id, pcd_file);
    v.run ();
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled.\n");
    OpenNITracking<pcl::PointXYZRGB> v (device_id, pcd_file);
    v.run ();
  }
  return (0);

}


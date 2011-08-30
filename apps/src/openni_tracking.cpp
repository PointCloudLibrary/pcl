#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/apps/openni_tracking.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

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

template <typename PointType>
class OpenNITracking
{
public:
  typedef pcl::PointXYZRGBNormal RefPointType;
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  
  OpenNITracking (const std::string& device_id, const std::string& pcd_file)
    : viewer_ ("PCL OpenNI Tracking Viewer")
    , device_id_ (device_id)
    , pcd_file_ (pcd_file)
    , sensor_view (0)
    , reference_view (0)
    , new_cloud_ (false)
    {
      // pass_.setFilterFieldName ("z");
      // pass_.setFilterLimits (0.0, 1.0);
      // pass_.setKeepOrganized(true);

      grid_.setFilterFieldName ("z");
      grid_.setFilterLimits (0.0, 1.5);
      grid_.setLeafSize (0.01, 0.01, 0.01);

      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_PLANE);
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setMaxIterations (1000);
      seg_.setDistanceThreshold (0.01);
      
      //ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::COVARIANCE_MATRIX);
      //ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::AVERAGE_3D_GRADIENT);
      //ne_.setRectSize (50, 50);

      pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
      ne_.setSearchMethod (tree);
      ne_.setRadiusSearch (0.03);

      std::vector<double> default_step_covariance = std::vector<double> (6, 0.0001);
      std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.1);
      std::vector<double> default_initial_mean = std::vector<double> (6, 0.5);
      default_initial_mean[2] = 0.0;
      initial_noise_covariance[2] = 0.0;
      tracker_ = boost::shared_ptr<pcl::apps::PointCloudTracking>
        (new pcl::apps::PointCloudTracking (pcl::apps::PF::UPDATE_MEAN, 400, 1,
                                            10.0, 0.1, 0.01,
                                            default_step_covariance,
                                            initial_noise_covariance,
                                            default_initial_mean));
      
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
      pcl::apps::PointCloudTracking::ParticleFilterParameterPtr parameter = tracker_->getParticleFilterParameter ();
      Eigen::Matrix4f search_trans = parameter->getOffsetMatrix ();
      Eigen::Matrix4f search_origin = plane_trans_ * search_trans;
      drawCube (viz, search_origin, parameter->getXWidth (), parameter->getYWidth (), parameter->getZWidth (),
                "searcharea");
    }

  pcl::PointXYZ calcCubePoint (const Eigen::Matrix4f& origin, const Eigen::Vector4f& point)
    {
      Eigen::Vector4f position = origin * point;
      pcl::PointXYZ ret;
      ret.x = position[0];
      ret.y = position[1];
      ret.z = position[2];
      return ret;
    }

  void drawLine (pcl::visualization::PCLVisualizer& viz, const pcl::PointXYZ& from, const pcl::PointXYZ& to, const std::string& name)
    {
      viz.removeShape (name);
      viz.addLine<pcl::PointXYZ> (from, to, name);
    }
  
  void drawCube (pcl::visualization::PCLVisualizer& viz, const Eigen::Matrix4f& origin,
                 double width,  // x
                 double height, // y
                 double depth,  // z
                 const std::string& name = "cube")
    {
      pcl::PointXYZ A = calcCubePoint (origin, Eigen::Vector4f (+ width / 2.0, - height / 2.0, - depth / 2.0, 1.0));
      pcl::PointXYZ B = calcCubePoint (origin, Eigen::Vector4f (+ width / 2.0, + height / 2.0, - depth / 2.0, 1.0));
      pcl::PointXYZ C = calcCubePoint (origin, Eigen::Vector4f (- width / 2.0, + height / 2.0, - depth / 2.0, 1.0));
      pcl::PointXYZ D = calcCubePoint (origin, Eigen::Vector4f (- width / 2.0, - height / 2.0, - depth / 2.0, 1.0));
      pcl::PointXYZ E = calcCubePoint (origin, Eigen::Vector4f (+ width / 2.0, - height / 2.0, + depth / 2.0, 1.0));
      pcl::PointXYZ F = calcCubePoint (origin, Eigen::Vector4f (+ width / 2.0, + height / 2.0, + depth / 2.0, 1.0));
      pcl::PointXYZ G = calcCubePoint (origin, Eigen::Vector4f (- width / 2.0, + height / 2.0, + depth / 2.0, 1.0));
      pcl::PointXYZ H = calcCubePoint (origin, Eigen::Vector4f (- width / 2.0, - height / 2.0, + depth / 2.0, 1.0));
      
      drawLine (viz, A, B, name + "AB");
      drawLine (viz, B, C, name + "BC");
      drawLine (viz, C, D, name + "CD");
      drawLine (viz, D, A, name + "DA");
      drawLine (viz, E, F, name + "EF");
      drawLine (viz, F, G, name + "FG");
      drawLine (viz, G, H, name + "GH");
      drawLine (viz, H, E, name + "HE");
      drawLine (viz, A, E, name + "AE");
      drawLine (viz, B, F, name + "BF");
      drawLine (viz, C, G, name + "CG");
      drawLine (viz, D, H, name + "DH");
    }

  void
  drawParticles (pcl::visualization::PCLVisualizer& viz)
    {
      pcl::apps::PointCloudTracking::ParticleFilterPtr pfilter = tracker_->getParticleFilter ();
      pcl::apps::PointCloudTracking::ParticleFilterParameterPtr parameter = tracker_->getParticleFilterParameter ();
      std::vector<pcl::apps::PF::Particle> particles = pfilter->getParticles ();
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      for (size_t i = 0; i < particles.size (); i++)
      {
        pcl::PointXYZ point;
        
        pcl::apps::PF::Particle particle = particles[i];
        Eigen::Affine3f transformation = parameter->toEigenMatrix (particle.getState ());
        Eigen::Vector3f translation = transformation.translation ();
        point.x = translation[0];
        point.y = translation[1];
        point.z = translation[2];
        particle_cloud->points.push_back (point);
      }

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color (particle_cloud, 0, 255, 0);
      if (!viz.updatePointCloud (particle_cloud, green_color, "particle cloud"))
        viz.addPointCloud (particle_cloud, green_color, "particle cloud");
          
    }
  
  void
  viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      boost::mutex::scoped_lock lock (mtx_);
      viz.setBackgroundColor (0.8, 0.8, 0.8);
      if (!cloud_pass_)
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
        return;
      }
      
      // {
      //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color (cloud_pass_, 255, 0, 0);
      //   if (!viz.updatePointCloud (cloud_pass_, red_color, "sensor cloud"))
      //   {
      //     viz.addPointCloud (cloud_pass_, red_color, "sensor cloud");
      //     viz.resetCameraViewpoint ("sensor cloud");
      //   }
      // }
      
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

      drawParticles (viz);
      
      if (new_cloud_)
        new_cloud_ = false;
    }
  
  void
  cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
            const pcl::PointCloud<RefPointType>::ConstPtr &ref_cloud)
    {
      FPS_CALC ("computation");
      boost::mutex::scoped_lock lock (mtx_);
      //pass_.setInputCloud (cloud);
      grid_.setInputCloud (cloud);
      cloud_pass_.reset (new Cloud);
      //pass_.filter (*cloud_pass_);
      grid_.filter (*cloud_pass_);
      
      // calc plane
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      seg_.setInputCloud (cloud_pass_);
      seg_.segment (*inliers, *coefficients);

      CloudPtr cloud_projected (new Cloud ());
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setInputCloud (cloud_pass_);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cloud_projected);

      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::ConvexHull<pcl::PointXYZRGB> chull;
      cloud_hull_.reset (new Cloud);
      chull.setInputCloud (cloud_projected);
      chull.reconstruct (*cloud_hull_, hull_vertices_);
      
      plane_trans_ = estimatePlaneCoordinate(cloud_hull_);
      tracker_->getParticleFilterParameter ()->setTrans (Eigen::Affine3f (plane_trans_));
      pcl::PointIndices::Ptr inliers_polygon (new pcl::PointIndices ());
      pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> polygon_extract;
      nonplane_cloud_.reset (new Cloud);
      //polygon_extract.setHeightLimits (-10.0, 0.01);
      polygon_extract.setHeightLimits (0.01, 10.0);
      polygon_extract.setInputPlanarHull (cloud_hull_);
      polygon_extract.setInputCloud (cloud_pass_);
      polygon_extract.segment (*inliers_polygon);
      
      extract_positive_.setInputCloud (cloud_pass_);
      extract_positive_.setIndices (inliers_polygon);
      
      extract_positive_.filter (*nonplane_cloud_);

      // calc normals
      normals_.reset (new pcl::PointCloud<pcl::Normal>);
      ne_.setInputCloud (nonplane_cloud_);
      //ne_.setIndices (inliers_polygon);
      ne_.compute (*normals_);

      // std::cout << "nonplane_cloud: " << nonplane_cloud_->points.size () << std::endl;
      // std::cout << "normals: " << normals_->points.size () << std::endl;
      // creae PointXYZRGBNormal PointCloud
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
      tracker_->proc (tracking_const_ptr);
      
      new_cloud_ = true;
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
        Eigen::Vector3f A (cloud_hull->points[0].x, cloud_hull->points[0].y, cloud_hull->points[0].z);
        if (A.dot (z) > 0)
          z = - z;

        Eigen::Vector3f x (1.0, 0.0, 0.0);

        // x should not be parallel with z.
        if ( fabs (z.dot (x)) > 1.0 - 1.0e-4)
          x = Eigen::Vector3f (0.0, 1.0, 0.0);
        // calc y
        Eigen::Vector3f y = z.cross (x);
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
        double beta = - OB.dot (y);
        double gamma = - OB.dot (x);
        Eigen::Vector3f position = OB + beta * y + gamma * x;
        
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
      if (pcl::io::loadPCDFile<RefPointType> (pcd_file_, *ref_cloud) == -1)
      {
        PCL_ERROR ("Couldn't read file %s \n", pcd_file_.c_str ());
        return;
      }

      // sed PCD to tracker_
      tracker_->setReferencePointCloud (ref_cloud);
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
        boost::bind (&OpenNITracking::cloud_cb, this, _1, ref_cloud);
      interface->registerCallback (f);
      viewer_.runOnVisualizationThread (boost::bind(&OpenNITracking::viz_cb, this, _1), "viz_cb");
      
      interface->start ();
      
      while (!viewer_.wasStopped ())
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      interface->stop ();
    }
  
  //pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne_;
  pcl::NormalEstimation<PointType, pcl::Normal> ne_;
  //pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_;
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
  //pcl::apps::PointCloudTracking tracker_;
  boost::shared_ptr<pcl::apps::PointCloudTracking> tracker_;
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


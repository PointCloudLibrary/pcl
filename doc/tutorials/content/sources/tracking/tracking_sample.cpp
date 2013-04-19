#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include "segment_reference.h"

using namespace pcl::tracking;



//Draw the current particles
bool
drawParticles (pcl::visualization::PCLVisualizer& viz)
{
  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
  if (particles)
    {
      
      //Set pointCloud with particle's points
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      for (size_t i = 0; i < particles->points.size (); i++)
	{
	  pcl::PointXYZ point;
          
	  point.x = particles->points[i].x;
	  point.y = particles->points[i].y;
	  point.z = particles->points[i].z;
	  particle_cloud->points.push_back (point);
	}

      //Draw red particles 
      {
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);
//        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "particle cloud");

	if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
	  viz.addPointCloud (particle_cloud, red_color, "particle cloud");
      }
      return true;
    }
  else
    {
      return false;
    }
}

//Draw model reference point cloud
void
drawResult (pcl::visualization::PCLVisualizer& viz)
{
  ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

  // move a little bit for better visualization
  transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
  RefCloudPtr result_cloud (new RefCloud ());
  pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

  //Draw blue model reference point cloud
  {
    pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);
//  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "resultcloud");

    if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
      viz.addPointCloud (result_cloud, blue_color, "resultcloud");
  }
}
 

//visualization's callback function
void
viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  boost::mutex::scoped_lock lock (mtx_);
    
  if (!cloud_pass_)
    {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
      return;
   }

  //Draw downsampled point cloud from sensor    
  if (new_cloud_ && cloud_pass_downsampled_)
    {
      CloudPtr cloud_pass;
      cloud_pass = cloud_pass_downsampled_;
    
      if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
	{
	  viz.addPointCloud (cloud_pass, "cloudpass");
	  viz.resetCameraViewpoint ("cloudpass");
	}
    }

  //Draw point cloud related with tracking particles
  if (new_cloud_ && reference_)
    {
      bool ret = drawParticles (viz);
      if (ret)
      drawResult (viz);
    }
  new_cloud_ = false;
}


//OpenNI Grabber's cloud Callback function
void
cloud_cb (const CloudConstPtr &cloud)
{
  boost::mutex::scoped_lock lock (mtx_);
  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
  filterPassThrough (cloud, *cloud_pass_);

  //Point Cloud given at first few frames has some noise
  if (counter_ < 10)
    {
      gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
    }
  //Make model reference and set it to tracker_
  else if (counter_ == 10)
    {
      cloud_pass_downsampled_ = cloud_pass_;
      CloudPtr target_cloud;

      PCL_INFO ("segmentation, please wait...\n");
      //remove Plane's point cloud
      extractPlanes(target_cloud);

      if (target_cloud != NULL)
	     {
	       Eigen::Vector4f c;
	       Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
	       CloudPtr transed_ref_downsampled (new Cloud);
					
	       //Make model reference
	       initialize_segemnted_reference(target_cloud, transed_ref_downsampled, trans);
					
	       tracker_->setReferenceCloud (transed_ref_downsampled);
	       tracker_->setTrans (trans);
	     }
      PCL_INFO ("segmentation Complete!\n");
      PCL_INFO ("Start tracking\n");
    }
  //Track the object
  else
    {
      gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
      tracker_->setInputCloud (cloud_pass_downsampled_);
      tracker_->compute ();
    }
    
  new_cloud_ = true;
  counter_++;
}

 

int
main (int argc, char** argv)
{
  
  if (argc < 2)
    {
      PCL_WARN("Please set device_id (e.g. $ %s '#1')\n", argv[0]);
      exit (1);
    }

  std::string device_id = std::string (argv[1]);  
  device_id_ = device_id;

  viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");

  //Set parameters
  new_cloud_  = false;
  counter_    = 0;
  downsampling_grid_size_ =  0.002;

  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;

  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
    (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

  ParticleT bin_size;
  bin_size.x = 0.1f;
  bin_size.y = 0.1f;
  bin_size.z = 0.1f;
  bin_size.roll = 0.1f;
  bin_size.pitch = 0.1f;
  bin_size.yaw = 0.1f;


  //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
  tracker->setMaximumParticleNum (1000);
  tracker->setDelta (0.99);
  tracker->setEpsilon (0.2);
  tracker->setBinSize (bin_size);

  //Set all parameters for  ParticleFilter
  tracker_ = tracker;
  tracker_->setTrans (Eigen::Affine3f::Identity ());
  tracker_->setStepNoiseCovariance (default_step_covariance);
  tracker_->setInitialNoiseCovariance (initial_noise_covariance);
  tracker_->setInitialNoiseMean (default_initial_mean);
  tracker_->setIterationNum (1);
  tracker_->setParticleNum (600);
  tracker_->setResampleLikelihoodThr(0.00);
  tracker_->setUseNormal (false);


  //Setup coherence object for tracking
  ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
    (new ApproxNearestPairPointCloudCoherence<RefPointType> ());
    
  boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
    = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
  coherence->addPointCoherence (distance_coherence);

  boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
  coherence->setSearchMethod (search);
  coherence->setMaximumDistance (0.01);

  tracker_->setCloudCoherence (coherence);



  //Setup OpenNIGrabber and viewer
  pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id);
  boost::function<void (const CloudConstPtr&)> f =
    boost::bind (&cloud_cb, _1);
  interface->registerCallback (f);
    
  viewer_->runOnVisualizationThread (boost::bind(&viz_cb, _1), "viz_cb");

  //Start viewer and object tracking
  interface->start();
  while (!viewer_->wasStopped ())
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  interface->stop();

}

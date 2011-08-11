/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *  $Id$
 */

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>


#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)


const Eigen::Vector4f subsampling_leaf_size (0.01, 0.01, 0.01, 0.0);
const float normal_estimation_search_radius = 0.03;

template <typename PointType>
class OpenNIFeaturePersistence
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIFeaturePersistence (const std::string& device_id = "")
      : viewer ("PCL OpenNI Feature Persistence Viewer")
    , device_id_(device_id)
    {
      /// @TODO instanstiate multiscale feature stuff here
      subsampling_filter_.setLeafSize (subsampling_leaf_size);
      typename pcl::KdTreeFLANN<PointType>::Ptr normal_search_tree (new typename pcl::KdTreeFLANN<PointType>);
      normal_estimation_filter_.setSearchMethod (normal_search_tree);
      normal_estimation_filter_.setRadiusSearch (normal_estimation_search_radius);

      std::vector<float> scale_values;
      for (float x = 2.0; x < 3.6; x += 0.5)
        scale_values.push_back (x / 100.0);
      feature_persistence_.setScalesVector (scale_values);
      feature_persistence_.setAlpha (1.2);

      typename pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation (new typename pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> ());
      typename pcl::KdTreeFLANN<PointType>::Ptr fpfh_tree (new typename pcl::KdTreeFLANN<PointType> ());
      fpfh_estimation->setSearchMethod (fpfh_tree);
      feature_persistence_.setFeatureEstimator (fpfh_estimation);
      feature_persistence_.setDistanceMetric (pcl::CS);

      new_cloud_ = false;
    }


    void
    cloud_cb (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      //lock while we set our cloud;
      FPS_CALC ("computation");
      // Estimate surface normals
//      ne_.setInputCloud (cloud);


      cloud_ = cloud;

      new_cloud_ = true;
    }

    void
    viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      boost::mutex::scoped_lock lock (mtx_);
      if (!cloud_)
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        return;
      }

      CloudConstPtr temp_cloud;
      temp_cloud.swap (cloud_); //here we set cloud_ to null, so that

      if (!viz.updatePointCloud (temp_cloud, "OpenNICloud"))
      {
        viz.addPointCloud (temp_cloud, "OpenNICloud");
        viz.resetCameraViewpoint ("OpenNICloud");
      }
      // Render the data
      if (new_cloud_ && features_)
      {
        viz.removePointCloud ("featurecloud");
        viz.addPointCloud (temp_cloud, "featurecloud");
        new_cloud_ = false;
      }
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIFeaturePersistence::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);

      viewer.runOnVisualizationThread (boost::bind(&OpenNIFeaturePersistence::viz_cb, this, _1), "viz_cb");

      interface->start ();

      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }


    pcl::VoxelGrid<PointType> subsampling_filter_;
    pcl::NormalEstimation<PointType, pcl::Normal> normal_estimation_filter_;
    pcl::MultiscaleFeaturePersistence<PointType, pcl::FPFHSignature33> feature_persistence_;
    pcl::ExtractIndices<PointType> extract_indices_filter_;

    pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;
    // Data
    CloudPtr features_;
    CloudConstPtr cloud_;
    bool new_cloud_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << (int)driver.getBus (deviceIdx) << " @ " << (int)driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (wotks only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int
main (int argc, char ** argv)
{
  std::string arg;
  if (argc > 1)
    arg = std::string (argv[1]);

  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }

  pcl::OpenNIGrabber grabber ("");
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  {
    PCL_INFO ("PointXYZRGB mode enabled.\n");
    OpenNIFeaturePersistence<pcl::PointXYZRGB> v ("");
    v.run ();
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled.\n");
    OpenNIFeaturePersistence<pcl::PointXYZ> v ("");
    v.run ();
  }

  return (0);
}

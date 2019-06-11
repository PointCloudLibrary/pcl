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
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <mutex>


template <typename PointType>
class OpenNIPlanarSegmentation
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIPlanarSegmentation (const std::string& device_id = "", double threshold = 0.01)
      : viewer ("PCL OpenNI Planar Segmentation Viewer"),
        device_id_ (device_id)
    {
      grid_.setFilterFieldName ("z");
      grid_.setFilterLimits (0.0f, 3.0f);
      grid_.setLeafSize (0.01f, 0.01f, 0.01f);

      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_PLANE);
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setMaxIterations (1000);
      seg_.setDistanceThreshold (threshold);

      extract_.setNegative (false);
    }

    void 
    cloud_cb_ (const CloudConstPtr& cloud)
    {
      set (cloud);
    }

    void
    set (const CloudConstPtr& cloud)
    {
      //lock while we set our cloud;
      std::lock_guard<std::mutex> lock (mtx_);
      cloud_  = cloud;
    }

    CloudPtr
    get ()
    {
      //lock while we swap our cloud and reset it.
      std::lock_guard<std::mutex> lock (mtx_);
      CloudPtr temp_cloud (new Cloud);
      CloudPtr temp_cloud2 (new Cloud);

      grid_.setInputCloud (cloud_);
      grid_.filter (*temp_cloud);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

      seg_.setInputCloud (temp_cloud);
      seg_.segment (*inliers, *coefficients);

      extract_.setInputCloud (temp_cloud);
      extract_.setIndices (inliers);
      extract_.filter (*temp_cloud2);

      return (temp_cloud2);
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      std::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIPlanarSegmentation::cloud_cb_, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);
      
      interface->start ();
      
      while (!viewer.wasStopped ())
      {
        if (cloud_)
        {
          //the call to get() sets the cloud_ to null;
          viewer.showCloud (get ());
        }
      }

      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
    pcl::VoxelGrid<PointType> grid_;
    pcl::SACSegmentation<PointType> seg_;
    pcl::ExtractIndices<PointType> extract_;

    std::string device_id_;
    std::mutex mtx_;
    CloudConstPtr cloud_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n"
            << "where options are:\n         -thresh X        :: set the planar segmentation threshold (default: 0.5)\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int 
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    usage (argv);
    return 1;
  }

  std::string arg (argv[1]);
  
  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }

  double threshold = 0.05;
  pcl::console::parse_argument (argc, argv, "-thresh", threshold);

  pcl::OpenNIGrabber grabber (arg);
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    OpenNIPlanarSegmentation<pcl::PointXYZRGBA> v (arg, threshold);
    v.run ();
  }
  else
  {
    OpenNIPlanarSegmentation<pcl::PointXYZ> v (arg, threshold);
    v.run ();
  }

  return (0);
}

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
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>

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


class OpenNIUniformSampling
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

    OpenNIUniformSampling (const std::string& device_id = "", 
                       float leaf_size = 0.05)
    : viewer ("PCL OpenNI PassThrough Viewer")
    , device_id_(device_id)
    {
      pass_.setRadiusSearch (leaf_size);
    }

    void 
    cloud_cb_ (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      FPS_CALC ("computation");

      cloud_.reset (new Cloud);
      indices_.reset (new pcl::PointCloud<int>);
      keypoints_.reset (new pcl::PointCloud<pcl::PointXYZ>);
      // Computation goes here
      pass_.setInputCloud (cloud);
      pass_.compute (*indices_);
      *cloud_  = *cloud;
      
      pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZ> (*cloud, indices_->points, *keypoints_);
    }

    void
    viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      boost::mutex::scoped_lock lock (mtx_);
      if (!keypoints_ && !cloud_)
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        return;
      }

      FPS_CALC ("visualization");
      viz.removePointCloud ("raw");
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color_handler (cloud_);
      viz.addPointCloud<pcl::PointXYZRGBA> (cloud_, color_handler, "raw");

      if (!viz.updatePointCloud<pcl::PointXYZ> (keypoints_, "keypoints"))
      {
        viz.addPointCloud<pcl::PointXYZ> (keypoints_, "keypoints");
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5.0, "keypoints");
        viz.resetCameraViewpoint ("keypoints");
      }
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIUniformSampling::cloud_cb_, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);
      viewer.runOnVisualizationThread (boost::bind(&OpenNIUniformSampling::viz_cb, this, _1), "viz_cb");
      
      interface->start ();
      
      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }

    pcl::UniformSampling<pcl::PointXYZRGBA> pass_;
    pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;
    CloudPtr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_;
    pcl::PointCloud<int>::Ptr indices_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n"
            << "where options are:\n"
            << "                             -leaf X  :: set the UniformSampling leaf size (default: 0.01)\n";

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

  float leaf_res = 0.05f;
  pcl::console::parse_argument (argc, argv, "-leaf", leaf_res);
  PCL_INFO ("Using %f as a leaf size for UniformSampling.\n", leaf_res);

  pcl::OpenNIGrabber grabber (arg);
  OpenNIUniformSampling v (arg, leaf_res);
  v.run ();

  return (0);
}

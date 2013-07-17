/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/boundary.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

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

class OpenNIIntegralImageNormalEstimation
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZRGBNormal> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

    OpenNIIntegralImageNormalEstimation (const std::string& device_id = "")
      : viewer ("PCL OpenNI NormalEstimation Viewer") 
    , device_id_(device_id)
    {
      ne_.setNormalEstimationMethod (ne_.AVERAGE_3D_GRADIENT);
      //ne_.setNormalEstimationMethod (ne_.COVARIANCE_MATRIX);
      ne_.setRectSize (10, 10);
      new_cloud_ = false;

      pass_.setDownsampleAllData (true);
      pass_.setLeafSize (0.005f, 0.005f, 0.005f);

      pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>);
      be_.setRadiusSearch (0.02);
      be_.setSearchMethod (tree);
    }
    
    void 
    cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      //lock while we set our cloud;
      FPS_CALC ("computation");

      cloud_.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

      // Estimate surface normals
      ne_.setInputCloud (cloud);
      ne_.compute (*cloud_);
      copyPointCloud (*cloud, *cloud_);

//      cloud_pass_.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      // Passthrough
//      pass_.setInputCloud (cloud_);
//      pass_.filter (*cloud_pass_);

      be_.setInputCloud (cloud_);
      be_.setInputNormals (cloud_);
      boundaries_.reset (new pcl::PointCloud<pcl::Boundary>);
      be_.compute (*boundaries_);

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

      // Render the data 
      if (new_cloud_ && cloud_ && boundaries_)
      {
        CloudPtr temp_cloud;
        temp_cloud.swap (cloud_); //here we set cloud_ to null, so that

//        if (!viz.updatePointCloud<pcl::PointXYZRGBNormal> (temp_cloud, "OpenNICloud"))
//        {
//          viz.addPointCloud<pcl::PointXYZRGBNormal> (temp_cloud, "OpenNICloud");
//          viz.resetCameraViewpoint ("OpenNICloud");
//        }

        viz.removePointCloud ("normalcloud");
        pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2 (*boundaries_, *cloud2);
        ColorHandlerConstPtr color_handler (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (cloud2, "boundary_point"));
        viz.addPointCloud<pcl::PointXYZRGBNormal> (temp_cloud, color_handler, "normalcloud");
        viz.resetCameraViewpoint ("normalcloud");
        new_cloud_ = false;
      }
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &)> f = boost::bind (&OpenNIIntegralImageNormalEstimation::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);
     
      viewer.runOnVisualizationThread (boost::bind(&OpenNIIntegralImageNormalEstimation::viz_cb, this, _1), "viz_cb");

      interface->start ();
      
      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }

    pcl::ApproximateVoxelGrid<pcl::PointXYZRGBNormal> pass_;
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne_;
    pcl::BoundaryEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Boundary> be_;
    pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;
    // Data
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries_;
    CloudPtr cloud_, cloud_pass_;
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
    OpenNIIntegralImageNormalEstimation v ("");
    v.run ();
  }
  else
    PCL_ERROR ("The input device does not provide a PointXYZRGB mode.\n");

  return (0);
}

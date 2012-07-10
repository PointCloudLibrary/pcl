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
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;
using namespace pcl::visualization;
using namespace std;

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    if (++count == 100) \
    { \
      double now = pcl::getTime (); \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

template <typename PointType>
class OpenNI3DConvexHull
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNI3DConvexHull (const std::string& device_id = "")
      : viewer ("PCL OpenNI 3D Convex Hull Viewer") 
    , device_id_(device_id)
    {
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1.0);
    }
    
    void 
    cloud_cb (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      FPS_CALC ("computation");

      cloud_pass_.reset (new Cloud);
      // Computation goes here
      pass.setInputCloud (cloud);
      pass.filter (*cloud_pass_);

      // Estimate 3D convex hull
      pcl::ConvexHull<PointType> hr;
      hr.setInputCloud (cloud_pass_);
      cloud_hull_.reset (new Cloud);
      hr.reconstruct (*cloud_hull_, vertices_);

      cloud_ = cloud;
      new_cloud_ = true;
    }

    void
    viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      if (!cloud_ || !new_cloud_)
      {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        return;
      }

      {
        boost::mutex::scoped_lock lock (mtx_);
        FPS_CALC ("visualization");
        CloudPtr temp_cloud;
        temp_cloud.swap (cloud_pass_);

        if (!viz.updatePointCloud (temp_cloud, "OpenNICloud"))
        {
          viz.addPointCloud (temp_cloud, "OpenNICloud");
          viz.resetCameraViewpoint ("OpenNICloud");
        }
        // Render the data 
        if (new_cloud_ && cloud_hull_)
        {
          viz.removePointCloud ("hull");
          viz.addPolygonMesh<PointType> (cloud_hull_, vertices_, "hull");
        }
        new_cloud_ = false;
      }
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNI3DConvexHull::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);
     
      viewer.runOnVisualizationThread (boost::bind(&OpenNI3DConvexHull::viz_cb, this, _1), "viz_cb");

      interface->start ();
      
      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      }

      interface->stop ();
    }

    pcl::PassThrough<PointType> pass;
    pcl::visualization::CloudViewer viewer;

    std::string device_id_;
    boost::mutex mtx_;
    // Data
    CloudConstPtr cloud_;
    CloudPtr cloud_pass_, cloud_hull_;
    std::vector<pcl::Vertices> vertices_;
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
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    PCL_INFO ("PointXYZRGBA mode enabled.\n");
    OpenNI3DConvexHull<pcl::PointXYZRGBA> v ("");
    v.run ();
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled.\n");
    OpenNI3DConvexHull<pcl::PointXYZ> v ("");
    v.run ();
  }
  return (0);
}

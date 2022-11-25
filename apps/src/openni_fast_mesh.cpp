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

#include <pcl/common/time.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/visualization/pcl_visualizer.h> // for PCLVisualizer
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>
#include <thread>

using namespace pcl;
using namespace pcl::visualization;
using namespace std::chrono_literals;

// clang-format off
#define FPS_CALC(_WHAT_)                                                               \
  do {                                                                                 \
    static unsigned count = 0;                                                         \
    static double last = pcl::getTime();                                               \
    if (++count == 100) {                                                              \
      double now = pcl::getTime();                                                     \
      std::cout << "Average framerate(" << _WHAT_ << "): "                             \
                << double(count) / double(now - last) << " Hz" << std::endl;           \
      count = 0;                                                                       \
      last = now;                                                                      \
    }                                                                                  \
  } while (false)
// clang-format on

template <typename PointType>
class OpenNIFastMesh {
public:
  using Cloud = pcl::PointCloud<PointType>;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;

  OpenNIFastMesh(const std::string& device_id = "") : device_id_(device_id)
  {
    ofm.setTrianglePixelSize(3);
    ofm.setTriangulationType(pcl::OrganizedFastMesh<PointType>::QUAD_MESH);
  }

  void
  cloud_cb(const CloudConstPtr& cloud)
  {
    // Computation goes here
    FPS_CALC("computation");

    // Prepare input
    ofm.setInputCloud(cloud);

    // Store the results in a temporary object
    std::vector<pcl::Vertices> temp_verts;
    ofm.reconstruct(temp_verts);

    // Lock and copy
    {
      std::lock_guard<std::mutex> lock(mtx_);
      vertices_ = std::move(temp_verts);
      cloud_ = cloud; // reset (new Cloud (*cloud));
    }
  }

  void
  run(int argc, char** argv)
  {
    pcl::OpenNIGrabber interface(device_id_);

    std::function<void(const CloudConstPtr&)> f = [this](const CloudConstPtr& cloud) {
      cloud_cb(cloud);
    };
    boost::signals2::connection c = interface.registerCallback(f);

    view.reset(
        new pcl::visualization::PCLVisualizer(argc, argv, "PCL OpenNI Mesh Viewer"));

    interface.start();

    CloudConstPtr temp_cloud;
    std::vector<pcl::Vertices> temp_verts;

    while (!view->wasStopped()) {
      if (!cloud_ || !mtx_.try_lock()) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      temp_cloud = cloud_;
      temp_verts = std::move(vertices_);
      mtx_.unlock();

      if (!view->updatePolygonMesh<PointType>(temp_cloud, temp_verts, "surface")) {
        view->addPolygonMesh<PointType>(temp_cloud, temp_verts, "surface");
        view->resetCameraViewpoint("surface");
      }

      FPS_CALC("visualization");
      view->spinOnce(1);
    }

    interface.stop();
  }

  pcl::OrganizedFastMesh<PointType> ofm;
  std::string device_id_;
  std::mutex mtx_;
  // Data
  CloudConstPtr cloud_;
  std::vector<pcl::Vertices> vertices_;
  pcl::PolygonMesh::Ptr mesh_;

  pcl::visualization::PCLVisualizer::Ptr view;
};

void
usage(char** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
  if (driver.getNumberDevices() > 0) {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices(); ++deviceIdx) {
      std::cout << "Device: " << deviceIdx + 1
                << ", vendor: " << driver.getVendorName(deviceIdx)
                << ", product: " << driver.getProductName(deviceIdx)
                << ", connected: " << driver.getBus(deviceIdx) << " @ "
                << driver.getAddress(deviceIdx) << ", serial number: \'"
                << driver.getSerialNumber(deviceIdx) << "\'" << std::endl;
      std::cout << "device_id may be #1, #2, ... for the first second etc device in "
                   "the list or"
                << std::endl
                << "                 bus@address for the device connected to a "
                   "specific usb-bus / address combination (works only in Linux) or"
                << std::endl
                << "                 <serial-number> (only in Linux and for devices "
                   "which provide serial numbers)"
                << std::endl;
    }
  }
  else
    std::cout << "No devices connected." << std::endl;
}

int
main(int argc, char** argv)
{
  std::string arg;
  if (argc > 1)
    arg = std::string(argv[1]);

  if (arg == "--help" || arg == "-h") {
    usage(argv);
    return 1;
  }

  pcl::OpenNIGrabber grabber(arg);
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba>()) {
    PCL_INFO("PointXYZRGBA mode enabled.\n");
    OpenNIFastMesh<pcl::PointXYZRGBA> v(arg);
    v.run(argc, argv);
  }
  else {
    PCL_INFO("PointXYZ mode enabled.\n");
    OpenNIFastMesh<pcl::PointXYZ> v(arg);
    v.run(argc, argv);
  }
  return 0;
}

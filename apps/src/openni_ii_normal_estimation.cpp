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
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>
#include <thread>

using namespace std::chrono_literals;

// clang-format off
#define FPS_CALC(_WHAT_)                                                               \
  do {                                                                                 \
    static unsigned count = 0;                                                         \
    static double last = pcl::getTime();                                               \
    double now = pcl::getTime();                                                       \
    ++count;                                                                           \
    if (now - last >= 1.0) {                                                           \
      std::cout << "Average framerate(" << _WHAT_ << "): "                             \
                << double(count) / double(now - last) << " Hz" << std::endl;           \
      count = 0;                                                                       \
      last = now;                                                                      \
    }                                                                                  \
  } while (false)
// clang-format on

template <typename PointType>
class OpenNIIntegralImageNormalEstimation {
public:
  using Cloud = pcl::PointCloud<PointType>;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;

  OpenNIIntegralImageNormalEstimation(const std::string& device_id = "")
  : viewer("PCL OpenNI NormalEstimation Viewer"), device_id_(device_id)
  {
    ne_.setNormalEstimationMethod(
        pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::COVARIANCE_MATRIX);
    ne_.setNormalSmoothingSize(11.0);
    new_cloud_ = false;
    viewer.registerKeyboardCallback(
        &OpenNIIntegralImageNormalEstimation::keyboard_callback, *this);
  }

  void
  cloud_cb(const CloudConstPtr& cloud)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // lock while we set our cloud;

    normals_.reset(new pcl::PointCloud<pcl::Normal>);

    ne_.setInputCloud(cloud);
    ne_.compute(*normals_);
    cloud_ = cloud;

    new_cloud_ = true;
  }

  void
  viz_cb(pcl::visualization::PCLVisualizer& viz)
  {
    mtx_.lock();
    if (!cloud_ || !normals_) {
      mtx_.unlock();
      return;
    }

    CloudConstPtr temp_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr temp_normals;
    temp_cloud.swap(cloud_); // here we set cloud_ to null, so that
    temp_normals.swap(normals_);
    mtx_.unlock();

    if (!viz.updatePointCloud(temp_cloud, "OpenNICloud")) {
      viz.addPointCloud(temp_cloud, "OpenNICloud");
      viz.resetCameraViewpoint("OpenNICloud");
    }
    // Render the data
    if (new_cloud_) {
      viz.removePointCloud("normalcloud");
      viz.addPointCloudNormals<PointType, pcl::Normal>(
          temp_cloud, temp_normals, 100, 0.05f, "normalcloud");
      new_cloud_ = false;
    }
  }

  void
  keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    switch (event.getKeyCode()) {
    case '1':
      ne_.setNormalEstimationMethod(
          pcl::IntegralImageNormalEstimation<PointType,
                                             pcl::Normal>::COVARIANCE_MATRIX);
      std::cout << "switched to COVARIANCE_MATRIX method\n";
      break;
    case '2':
      ne_.setNormalEstimationMethod(
          pcl::IntegralImageNormalEstimation<PointType,
                                             pcl::Normal>::AVERAGE_3D_GRADIENT);
      std::cout << "switched to AVERAGE_3D_GRADIENT method\n";
      break;
    case '3':
      ne_.setNormalEstimationMethod(
          pcl::IntegralImageNormalEstimation<PointType,
                                             pcl::Normal>::AVERAGE_DEPTH_CHANGE);
      std::cout << "switched to AVERAGE_DEPTH_CHANGE method\n";
      break;
    case '4':
      ne_.setNormalEstimationMethod(
          pcl::IntegralImageNormalEstimation<PointType,
                                             pcl::Normal>::SIMPLE_3D_GRADIENT);
      std::cout << "switched to SIMPLE_3D_GRADIENT method\n";
      break;
    }
  }

  void
  run()
  {
    pcl::OpenNIGrabber interface(device_id_);

    std::function<void(const CloudConstPtr&)> f = [this](const CloudConstPtr& cloud) {
      cloud_cb(cloud);
    };
    boost::signals2::connection c = interface.registerCallback(f);

    viewer.runOnVisualizationThread(
        [this](pcl::visualization::PCLVisualizer& viz) { viz_cb(viz); }, "viz_cb");

    interface.start();

    while (!viewer.wasStopped()) {
      std::this_thread::sleep_for(1s);
    }

    interface.stop();
  }

  pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne_;
  pcl::visualization::CloudViewer viewer;
  std::string device_id_;
  std::mutex mtx_;
  // Data
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  CloudConstPtr cloud_;
  bool new_cloud_;
};

void
usage(char** argv)
{
  std::cout << "usage: " << argv[0] << " [<device_id>]\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
  if (driver.getNumberDevices() > 0) {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices(); ++deviceIdx) {
      // clang-format off
      std::cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << std::endl;
      std::cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << std::endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << std::endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << std::endl;
      // clang-format on
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

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
  if (arg == "--help" || arg == "-h" || driver.getNumberDevices() == 0) {
    usage(argv);
    return 1;
  }

  // clang-format off
  std::cout << "Press following keys to switch to the different integral image normal estimation methods:\n";
  std::cout << "<1> COVARIANCE_MATRIX method\n";
  std::cout << "<2> AVERAGE_3D_GRADIENT method\n";
  std::cout << "<3> AVERAGE_DEPTH_CHANGE method\n";
  std::cout << "<4> SIMPLE_3D_GRADIENT method\n";
  std::cout << "<Q,q> quit\n\n";
  // clang-format on

  pcl::OpenNIGrabber grabber(arg);
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba>()) {
    PCL_INFO("PointXYZRGBA mode enabled.\n");
    OpenNIIntegralImageNormalEstimation<pcl::PointXYZRGBA> v(arg);
    v.run();
  }
  else {
    PCL_INFO("PointXYZ mode enabled.\n");
    OpenNIIntegralImageNormalEstimation<pcl::PointXYZ> v(arg);
    v.run();
  }

  return 0;
}

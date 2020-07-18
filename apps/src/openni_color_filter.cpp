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
 *  Author: Suat Gedikli (gedikli@willowgarage.com)
 */

#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/filters/color.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>

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
class OpenNIPassthrough {
public:
  using Cloud = pcl::PointCloud<PointType>;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;

  OpenNIPassthrough(pcl::OpenNIGrabber& grabber,
                    unsigned char red,
                    unsigned char green,
                    unsigned char blue,
                    unsigned char radius)
  : viewer("PCL OpenNI ColorFilter Viewer"), grabber_(grabber)
  {
    std::function<void(const CloudConstPtr&)> f = [this](const CloudConstPtr& cloud) {
      cloud_cb_(cloud);
    };
    boost::signals2::connection c = grabber_.registerCallback(f);

    std::vector<bool> lookup(1 << 24, false);
    fillLookup(lookup, red, green, blue, radius);
    unsigned set = 0;
    for (unsigned i = 0; i < (1 << 24); ++i)
      if (lookup[i])
        ++set;

    std::cout << "used colors: " << set << std::endl;

    color_filter_.setLookUpTable(lookup);
  }

  void
  fillLookup(std::vector<bool>& lookup,
             unsigned char red,
             unsigned char green,
             unsigned char blue,
             unsigned radius)
  {
    unsigned radius_sqr = radius * radius;
    pcl::RGB color;
    for (color.rgba = 0; color.rgba < (1 << 24); ++color.rgba) {
      unsigned dist =
          (unsigned(color.r) - unsigned(red)) * (unsigned(color.r) - unsigned(red)) +
          (unsigned(color.g) - unsigned(green)) *
              (unsigned(color.g) - unsigned(green)) +
          (unsigned(color.b) - unsigned(blue)) * (unsigned(color.b) - unsigned(blue));
      if (dist < radius_sqr)
        lookup[color.rgba] = true;
      else
        lookup[color.rgba] = false;
    }
  }

  void
  cloud_cb_(const CloudConstPtr& cloud)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    FPS_CALC("computation");

    cloud_color_.reset(new Cloud);
    // Computation goes here
    color_filter_.setInputCloud(cloud);
    color_filter_.filter(*cloud_color_);
    cloud_ = cloud;
  }

  void
  run()
  {

    grabber_.start();

    while (!viewer.wasStopped()) {
      if (cloud_color_) {
        std::lock_guard<std::mutex> lock(mtx_);

        FPS_CALC("visualization");
        CloudPtr temp_cloud;
        temp_cloud.swap(cloud_color_); // here we set cloud_ to null, so that
        viewer.showCloud(temp_cloud);
      }
    }

    grabber_.stop();
  }

  pcl::ColorFilter<PointType> color_filter_;
  pcl::visualization::CloudViewer viewer;
  pcl::OpenNIGrabber& grabber_;
  std::string device_id_;
  std::mutex mtx_;
  CloudConstPtr cloud_;
  CloudPtr cloud_color_;
};

void
usage(char** argv)
{
  std::cout << "usage: " << argv[0]
            << " <device_id> [-rgb <red> <green> <blue> [-radius <radius>] ]\n\n"
            << std::endl;

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
  if (driver.getNumberDevices() > 0) {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices(); ++deviceIdx) {
      // clang-format off
      std::cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << (int)driver.getBus (deviceIdx) << " @ " << (int)driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << std::endl;
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
  if (argc < 2) {
    usage(argv);
    return 1;
  }

  std::string arg(argv[1]);

  if (arg == "--help" || arg == "-h") {
    usage(argv);
    return 1;
  }

  unsigned char red = 0, green = 0, blue = 0;
  int rr, gg, bb;
  unsigned char radius = 442; // all colors!

  if (pcl::console::parse_3x_arguments(argc, argv, "-rgb", rr, gg, bb, true) != -1) {
    std::cout << "-rgb present" << std::endl;
    int rad;
    int idx = pcl::console::parse_argument(argc, argv, "-radius", rad);
    if (idx != -1) {
      if (rad > 0)
        radius = rad;
    }
    if (rr >= 0 && rr < 256)
      red = (unsigned char)rr;
    if (gg >= 0 && gg < 256)
      green = (unsigned char)gg;
    if (bb >= 0 && bb < 256)
      blue = (unsigned char)bb;
  }

  pcl::OpenNIGrabber grabber(arg);

  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba>()) {
    OpenNIPassthrough<pcl::PointXYZRGBA> v(grabber, red, green, blue, radius);
    v.run();
  }
  else {
    std::cout << "device does not provide rgb stream" << std::endl;
  }

  return 0;
}

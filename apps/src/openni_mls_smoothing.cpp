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
#include <pcl/console/parse.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/surface/mls.h>
#include <pcl/visualization/keyboard_event.h> // for KeyboardEvent
#include <pcl/visualization/pcl_visualizer.h> // for PCLVisualizer
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
      if (*stop_computing_)                                                            \
        std::cout << "Press 's' to start computing!\n";                                \
    }                                                                                  \
  } while (false)
// clang-format on

int default_polynomial_order = 0;
double default_search_radius = 0.0, default_sqr_gauss_param = 0.0;

template <typename PointType>
class OpenNISmoothing;

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* stop_void)
{
  std::shared_ptr<bool> stop = *static_cast<std::shared_ptr<bool>*>(stop_void);
  if (event.getKeySym() == "s" && event.keyDown()) {
    *stop = !*stop;
    if (*stop)
      std::cout << "Computing is now stopped!\n";
    else
      std::cout << "Computing commencing!\n";
  }
}

template <typename PointType>
class OpenNISmoothing {
public:
  using Cloud = pcl::PointCloud<PointType>;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;

  OpenNISmoothing(double search_radius,
                  bool sqr_gauss_param_set,
                  double sqr_gauss_param,
                  int polynomial_order,
                  const std::string& device_id = "")
  : viewer("PCL OpenNI MLS Smoothing"), device_id_(device_id)
  {
    // Start 4 threads
    smoother_.setSearchRadius(search_radius);
    if (sqr_gauss_param_set)
      smoother_.setSqrGaussParam(sqr_gauss_param);
    smoother_.setPolynomialOrder(polynomial_order);

    typename pcl::search::KdTree<PointType>::Ptr tree(
        new typename pcl::search::KdTree<PointType>());
    smoother_.setSearchMethod(tree);

    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, viewport_input_);
    viewer.setBackgroundColor(0, 0, 0, viewport_input_);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, viewport_smoothed_);
    viewer.setBackgroundColor(0, 0, 0, viewport_smoothed_);

    stop_computing_.reset(new bool(true));
    cloud_.reset();
    cloud_smoothed_.reset(new Cloud);
  }

  void
  cloud_cb_(const CloudConstPtr& cloud)
  {
    FPS_CALC("computation");

    mtx_.lock();
    if (!*stop_computing_) {
      smoother_.setInputCloud(cloud);
      smoother_.process(*cloud_smoothed_);
    }
    cloud_ = cloud;
    mtx_.unlock();
  }

  void
  run()
  {
    pcl::OpenNIGrabber interface(device_id_);

    std::function<void(const CloudConstPtr&)> f = [this](const CloudConstPtr& cloud) {
      cloud_cb_(cloud);
    };
    boost::signals2::connection c = interface.registerCallback(f);

    viewer.registerKeyboardCallback(keyboardEventOccurred,
                                    reinterpret_cast<void*>(&stop_computing_));

    interface.start();

    while (!viewer.wasStopped()) {
      FPS_CALC("visualization");
      viewer.spinOnce();

      if (cloud_ && mtx_.try_lock()) {
        if (!viewer.updatePointCloud(cloud_, "input_cloud"))
          viewer.addPointCloud(cloud_, "input_cloud", viewport_input_);
        if (!*stop_computing_ &&
            !viewer.updatePointCloud(cloud_smoothed_, "smoothed_cloud"))
          viewer.addPointCloud(cloud_smoothed_, "smoothed_cloud", viewport_smoothed_);
        mtx_.unlock();
      }
    }

    interface.stop();
  }

  pcl::MovingLeastSquares<PointType, PointType> smoother_;
  pcl::visualization::PCLVisualizer viewer;
  std::string device_id_;
  std::mutex mtx_;
  CloudConstPtr cloud_;
  CloudPtr cloud_smoothed_;
  int viewport_input_, viewport_smoothed_;
  std::shared_ptr<bool> stop_computing_;
};

void
usage(char** argv)
{
  // clang-format off
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n"
            << "where options are:\n"
            << "                     -search_radius X = sphere radius to be used for finding the k-nearest neighbors used for fitting (default: " << default_search_radius << ")\n"
            << "                     -sqr_gauss_param X = parameter used for the distance based weighting of neighbors (recommended = search_radius^2) (default: " << default_sqr_gauss_param << ")\n"
            << "                     -polynomial_order X = order of the polynomial to be fit (0 means tangent estimation) (default: " << default_polynomial_order << ")\n";
  // clang-format on

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
  if (argc < 2) {
    usage(argv);
    return 1;
  }

  std::string arg(argv[1]);

  if (arg == "--help" || arg == "-h") {
    usage(argv);
    return 1;
  }

  // Command line parsing
  double search_radius = default_search_radius;
  double sqr_gauss_param = default_sqr_gauss_param;
  bool sqr_gauss_param_set = true;
  int polynomial_order = default_polynomial_order;

  pcl::console::parse_argument(argc, argv, "-search_radius", search_radius);
  if (pcl::console::parse_argument(argc, argv, "-sqr_gauss_param", sqr_gauss_param) ==
      -1)
    sqr_gauss_param_set = false;
  pcl::console::parse_argument(argc, argv, "-polynomial_order", polynomial_order);

  pcl::OpenNIGrabber grabber(arg);
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba>()) {
    OpenNISmoothing<pcl::PointXYZRGBA> v(
        search_radius, sqr_gauss_param_set, sqr_gauss_param, polynomial_order, arg);
    v.run();
  }
  else {
    OpenNISmoothing<pcl::PointXYZ> v(
        search_radius, sqr_gauss_param_set, sqr_gauss_param, polynomial_order, arg);
    v.run();
  }

  return 0;
}

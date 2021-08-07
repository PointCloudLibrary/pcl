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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/asio.hpp>

#include <iostream>
#include <thread>
#include <vector>

using boost::asio::ip::tcp;

using namespace pcl;
using namespace pcl::io;

using namespace std::chrono_literals;

class SimpleOpenNIViewer {
public:
  SimpleOpenNIViewer()
  : viewer_("Input Point Cloud - Shift-to-depth conversion viewer")
  , grabber_("",
             pcl::OpenNIGrabber::OpenNI_Default_Mode,
             pcl::OpenNIGrabber::OpenNI_Default_Mode)
  {}

  void
  image_callback(const openni_wrapper::Image::Ptr& image,
                 const openni_wrapper::DepthImage::Ptr& depth_image,
                 float)
  {

    std::vector<std::uint16_t> raw_shift_data;
    std::vector<std::uint16_t> raw_depth_data;

    std::vector<std::uint8_t> rgb_data;

    std::uint32_t width = depth_image->getWidth();
    std::uint32_t height = depth_image->getHeight();

    // copy raw shift data from depth_image
    raw_shift_data.resize(width * height);
    depth_image->fillDepthImageRaw(
        width,
        height,
        &raw_shift_data[0],
        static_cast<unsigned int>(width * sizeof(std::uint16_t)));

    // convert raw shift data to raw depth data
    raw_depth_data.resize(width * height);
    grabber_.convertShiftToDepth(
        &raw_shift_data[0], &raw_depth_data[0], raw_shift_data.size());

    // check for color data
    if (image->getEncoding() != openni_wrapper::Image::RGB) {
      // copy raw rgb data from image
      rgb_data.resize(width * height * 3);
      image->fillRGB(width,
                     height,
                     &rgb_data[0],
                     static_cast<unsigned int>(width * sizeof(std::uint8_t) * 3));
    }

    // empty pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    // convert raw depth and rgb data to pointcloud
    convert(
        raw_depth_data, rgb_data, width, height, depth_image->getFocalLength(), *cloud);

    // display pointcloud
    viewer_.showCloud(cloud);
  }

  void
  run()
  {
    // initialize OpenNIDevice to shift-value mode
    int depthformat = openni_wrapper::OpenNIDevice::OpenNI_shift_values;

    // Set the depth output format
    grabber_.getDevice()->setDepthOutputFormat(
        static_cast<openni_wrapper::OpenNIDevice::DepthMode>(depthformat));

    // define image callback
    std::function<void(const openni_wrapper::Image::Ptr&,
                       const openni_wrapper::DepthImage::Ptr&,
                       float)>
        image_cb = [this](const openni_wrapper::Image::Ptr& img,
                          const openni_wrapper::DepthImage::Ptr& depth,
                          float f) { image_callback(img, depth, f); };
    grabber_.registerCallback(image_cb);

    // start grabber thread
    grabber_.start();
    while (true) {
      std::this_thread::sleep_for(1s);
    }
    grabber_.stop();
  }

protected:
  /* helper method to convert depth&rgb data to pointcloud*/
  void
  convert(std::vector<std::uint16_t>& depthData_arg,
          std::vector<std::uint8_t>& rgbData_arg,
          std::size_t width_arg,
          std::size_t height_arg,
          float focalLength_arg,
          pcl::PointCloud<PointXYZRGB>& cloud_arg) const
  {
    std::size_t cloud_size = width_arg * height_arg;

    // Reset point cloud
    cloud_arg.clear();
    cloud_arg.reserve(cloud_size);

    // Define point cloud parameters
    cloud_arg.width = static_cast<std::uint32_t>(width_arg);
    cloud_arg.height = static_cast<std::uint32_t>(height_arg);
    cloud_arg.is_dense = false;

    // Calculate center of disparity image
    int centerX = static_cast<int>(width_arg / 2);
    int centerY = static_cast<int>(height_arg / 2);

    const float fl_const = 1.0f / focalLength_arg;
    static const float bad_point = std::numeric_limits<float>::quiet_NaN();

    std::size_t i = 0;
    for (int y = -centerY; y < +centerY; ++y)
      for (int x = -centerX; x < +centerX; ++x) {
        PointXYZRGB newPoint;

        const std::uint16_t& pixel_depth = depthData_arg[i];

        if (pixel_depth) {
          float depth = pixel_depth / 1000.0f; // raw mm -> m

          // Define point location
          newPoint.z = depth;
          newPoint.x = static_cast<float>(x) * depth * fl_const;
          newPoint.y = static_cast<float>(y) * depth * fl_const;

          // Define point color
          newPoint.r = rgbData_arg[i * 3 + 0];
          newPoint.g = rgbData_arg[i * 3 + 1];
          newPoint.b = rgbData_arg[i * 3 + 2];
        }
        else {
          // Define bad point
          newPoint.x = newPoint.y = newPoint.z = bad_point;
          newPoint.rgba = 0;
        }

        // Add point to cloud
        cloud_arg.push_back(newPoint);
        // Increment point iterator
        ++i;
      }
  }

  pcl::visualization::CloudViewer viewer_;
  pcl::OpenNIGrabber grabber_;
};

int
main(int, char**)
{
  SimpleOpenNIViewer v;
  v.run();

  return 0;
}

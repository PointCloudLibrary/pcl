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
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/asio.hpp>

#include <mutex>
#include <thread>

using boost::asio::ip::tcp;
using namespace std::chrono_literals;

struct PointCloudBuffers {
  using Ptr = std::shared_ptr<PointCloudBuffers>;
  std::vector<short> points;
  std::vector<unsigned char> rgb;
};

void
CopyPointCloudToBuffers(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud,
                        PointCloudBuffers& cloud_buffers)
{
  const std::size_t nr_points = cloud->points.size();

  cloud_buffers.points.resize(nr_points * 3);
  cloud_buffers.rgb.resize(nr_points * 3);

  const pcl::PointXYZ bounds_min(-0.9f, -0.8f, 1.0f);
  const pcl::PointXYZ bounds_max(0.9f, 3.0f, 3.3f);

  std::size_t j = 0;
  for (std::size_t i = 0; i < nr_points; ++i) {

    const pcl::PointXYZRGBA& point = (*cloud)[i];

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      continue;

    if (point.x < bounds_min.x || point.y < bounds_min.y || point.z < bounds_min.z ||
        point.x > bounds_max.x || point.y > bounds_max.y || point.z > bounds_max.z)
      continue;

    const int conversion_factor = 500;

    cloud_buffers.points[j * 3 + 0] = static_cast<short>(point.x * conversion_factor);
    cloud_buffers.points[j * 3 + 1] = static_cast<short>(point.y * conversion_factor);
    cloud_buffers.points[j * 3 + 2] = static_cast<short>(point.z * conversion_factor);

    cloud_buffers.rgb[j * 3 + 0] = point.r;
    cloud_buffers.rgb[j * 3 + 1] = point.g;
    cloud_buffers.rgb[j * 3 + 2] = point.b;

    j++;
  }

  cloud_buffers.points.resize(j * 3);
  cloud_buffers.rgb.resize(j * 3);
}

template <typename PointType>
class PCLMobileServer {
public:
  using Cloud = pcl::PointCloud<PointType>;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;

  PCLMobileServer(const std::string& device_id = "",
                  int port = 11111,
                  float leaf_size_x = 0.01,
                  float leaf_size_y = 0.01,
                  float leaf_size_z = 0.01)
  : port_(port), device_id_(device_id), viewer_("PCL OpenNI Mobile Server")
  {
    voxel_grid_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
  }

  void
  handleIncomingCloud(const CloudConstPtr& new_cloud)
  {
    CloudPtr temp_cloud(new Cloud);
    voxel_grid_filter_.setInputCloud(new_cloud);
    voxel_grid_filter_.filter(*temp_cloud);

    PointCloudBuffers::Ptr new_buffers = PointCloudBuffers::Ptr(new PointCloudBuffers);
    CopyPointCloudToBuffers(temp_cloud, *new_buffers);

    std::lock_guard<std::mutex> lock(mutex_);
    filtered_cloud_ = temp_cloud;
    buffers_ = new_buffers;
  }

  PointCloudBuffers::Ptr
  getLatestBuffers()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffers_;
  }

  CloudPtr
  getLatestPointCloud()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return filtered_cloud_;
  }

  void
  run()
  {
    pcl::OpenNIGrabber grabber(device_id_);
    std::function<void(const CloudConstPtr&)> handler_function =
        [this](const CloudConstPtr& cloud) { handleIncomingCloud(cloud); };
    grabber.registerCallback(handler_function);
    grabber.start();

    // wait for first cloud
    while (!getLatestPointCloud())
      std::this_thread::sleep_for(10ms);

    viewer_.showCloud(getLatestPointCloud());

    boost::asio::io_service io_service;
    tcp::endpoint endpoint(tcp::v4(), static_cast<unsigned short>(port_));
    tcp::acceptor acceptor(io_service, endpoint);
    tcp::socket socket(io_service);

    std::cout << "Listening on port " << port_ << "..." << std::endl;
    acceptor.accept(socket);

    std::cout << "Client connected." << std::endl;

    double start_time = pcl::getTime();
    int counter = 0;

    while (!viewer_.wasStopped()) {

      // wait for client
      unsigned int nr_points = 0;
      boost::asio::read(socket, boost::asio::buffer(&nr_points, sizeof(nr_points)));

      PointCloudBuffers::Ptr buffers_to_send = getLatestBuffers();

      nr_points = static_cast<unsigned int>(buffers_to_send->points.size() / 3);
      boost::asio::write(socket, boost::asio::buffer(&nr_points, sizeof(nr_points)));

      if (nr_points) {
        boost::asio::write(socket,
                           boost::asio::buffer(&buffers_to_send->points.front(),
                                               nr_points * 3 * sizeof(short)));
        boost::asio::write(socket,
                           boost::asio::buffer(&buffers_to_send->rgb.front(),
                                               nr_points * 3 * sizeof(unsigned char)));
      }

      counter++;

      double new_time = pcl::getTime();
      double elapsed_time = new_time - start_time;
      if (elapsed_time > 1.0) {
        double frames_per_second = counter / elapsed_time;
        start_time = new_time;
        counter = 0;
        std::cout << "fps: " << frames_per_second << std::endl;
      }

      viewer_.showCloud(getLatestPointCloud());
    }

    grabber.stop();
  }

  int port_;
  std::string device_id_;
  std::mutex mutex_;

  pcl::VoxelGrid<PointType> voxel_grid_filter_;
  pcl::visualization::CloudViewer viewer_;

  CloudPtr filtered_cloud_;
  PointCloudBuffers::Ptr buffers_;
};

void
usage(char** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n"
            << "where options are:\n"
            << "  -port p :: set the server port (default: 11111)\n"
            << "  -leaf x, y, z  :: set the voxel grid leaf size (default: 0.01)\n";
}

int
main(int argc, char** argv)
{
  std::string device_id = "";
  if ((argc > 1) && (argv[1][0] != '-'))
    device_id = std::string(argv[1]);

  if (pcl::console::find_argument(argc, argv, "-h") != -1) {
    usage(argv);
    return 0;
  }

  int port = 11111;
  float leaf_x = 0.01f, leaf_y = 0.01f, leaf_z = 0.01f;

  pcl::console::parse_argument(argc, argv, "-port", port);
  pcl::console::parse_3x_arguments(argc, argv, "-leaf", leaf_x, leaf_y, leaf_z, false);

  pcl::OpenNIGrabber grabber(device_id);
  if (!grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba>()) {
    std::cout << "OpenNI grabber does not provide the rgba cloud format." << std::endl;
    return 1;
  }

  PCL_INFO("Using %f, %f, %f as a leaf size for VoxelGrid.\n", leaf_x, leaf_y, leaf_z);

  PCLMobileServer<pcl::PointXYZRGBA> server(device_id, port, leaf_x, leaf_y, leaf_z);
  server.run();

  return 0;
}

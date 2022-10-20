/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
#include <pcl/features/organized_edge_detection.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <mutex>

using PointT = pcl::PointXYZRGBA;

class OpenNIOrganizedEdgeDetection {
private:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::PointCloud<PointT> cloud_;
  std::mutex cloud_mutex;

public:
  OpenNIOrganizedEdgeDetection()
  : viewer(new pcl::visualization::PCLVisualizer("PCL Organized Edge Detection"))
  {}
  ~OpenNIOrganizedEdgeDetection() = default;

  pcl::visualization::PCLVisualizer::Ptr
  initCloudViewer(const pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    viewer->setSize(640, 480);
    viewer->addPointCloud<PointT>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem(0.2f, "global");
    viewer->initCameraParameters();
    viewer->registerKeyboardCallback(&OpenNIOrganizedEdgeDetection::keyboard_callback,
                                     *this);
    viewer->resetCameraViewpoint("cloud");

    const int point_size = 2;
    viewer->addPointCloud<PointT>(cloud, "nan boundary edges");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        point_size,
        "nan boundary edges");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                             0.0f,
                                             0.0f,
                                             1.0f,
                                             "nan boundary edges");

    viewer->addPointCloud<PointT>(cloud, "occluding edges");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluding edges");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "occluding edges");

    viewer->addPointCloud<PointT>(cloud, "occluded edges");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluded edges");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "occluded edges");

    viewer->addPointCloud<PointT>(cloud, "high curvature edges");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        point_size,
        "high curvature edges");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                             1.0f,
                                             1.0f,
                                             0.0f,
                                             "high curvature edges");

    viewer->addPointCloud<PointT>(cloud, "rgb edges");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "rgb edges");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "rgb edges");

    return viewer;
  }

  void
  keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
  {
    if (event.keyUp()) {
      double opacity;
      switch (event.getKeyCode()) {
      case '1':
        viewer->getPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "nan boundary edges");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY,
            1.0 - opacity,
            "nan boundary edges");
        break;
      case '2':
        viewer->getPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "occluding edges");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY,
            1.0 - opacity,
            "occluding edges");
        break;
      case '3':
        viewer->getPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "occluded edges");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY,
            1.0 - opacity,
            "occluded edges");
        break;
      case '4':
        viewer->getPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY,
            opacity,
            "high curvature edges");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY,
            1.0 - opacity,
            "high curvature edges");
        break;
      case '5':
        viewer->getPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "rgb edges");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0 - opacity, "rgb edges");
        break;
      }
    }
  }

  void
  cloud_cb_(const pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    if (!viewer->wasStopped()) {
      cloud_mutex.lock();
      cloud_ = *cloud;
      cloud_mutex.unlock();
    }
  }

  void
  run()
  {
    pcl::OpenNIGrabber interface;

    std::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f =
        [this](const pcl::PointCloud<PointT>::ConstPtr& cloud) { cloud_cb_(cloud); };

    // Make and initialize a cloud viewer
    pcl::PointCloud<PointT>::Ptr init_cloud_ptr(new pcl::PointCloud<PointT>);
    viewer = initCloudViewer(init_cloud_ptr);
    boost::signals2::connection c = interface.registerCallback(f);

    interface.start();

    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setNormalSmoothingSize(10.0f);
    ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

    float th_dd = 0.04f;
    int max_search = 100;
    pcl::OrganizedEdgeFromRGBNormals<PointT, pcl::Normal, pcl::Label> oed;
    oed.setDepthDisconThreshold(th_dd);
    oed.setMaxSearchNeighbors(max_search);

    oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING |
                    oed.EDGELABEL_OCCLUDED);
    // oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING |
    // oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);

    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> label_indices;

    while (!viewer->wasStopped()) {
      viewer->spinOnce();

      if (cloud_mutex.try_lock()) {
        labels.clear();
        label_indices.clear();

        double normal_start = pcl::getTime();

        if (oed.getEdgeType() & oed.EDGELABEL_HIGH_CURVATURE) {
          pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(
              new pcl::PointCloud<pcl::Normal>);
          ne.setInputCloud(cloud_.makeShared());
          ne.compute(*normal_cloud);
          double normal_end = pcl::getTime();
          std::cout << "Normal Estimation took " << double(normal_end - normal_start)
                    << std::endl;

          oed.setInputNormals(normal_cloud);
        }

        double oed_start = pcl::getTime();

        oed.setInputCloud(cloud_.makeShared());
        oed.compute(labels, label_indices);

        double oed_end = pcl::getTime();
        std::cout << "Edge Detection took " << double(oed_end - oed_start) << std::endl;
        std::cout << "Frame took " << double(oed_end - normal_start) << std::endl;

        // Make gray point cloud
        for (auto& point : cloud_.points) {
          std::uint8_t gray = std::uint8_t((point.r + point.g + point.b) / 3);
          point.r = point.g = point.b = gray;
        }

        // Show the gray point cloud
        if (!viewer->updatePointCloud(cloud_.makeShared(), "cloud"))
          viewer->addPointCloud(cloud_.makeShared(), "cloud");

        // Show edges
        pcl::PointCloud<PointT>::Ptr occluding_edges(new pcl::PointCloud<PointT>),
            occluded_edges(new pcl::PointCloud<PointT>),
            nan_boundary_edges(new pcl::PointCloud<PointT>),
            high_curvature_edges(new pcl::PointCloud<PointT>),
            rgb_edges(new pcl::PointCloud<PointT>);

        pcl::copyPointCloud(cloud_, label_indices[0].indices, *nan_boundary_edges);
        pcl::copyPointCloud(cloud_, label_indices[1].indices, *occluding_edges);
        pcl::copyPointCloud(cloud_, label_indices[2].indices, *occluded_edges);
        pcl::copyPointCloud(cloud_, label_indices[3].indices, *high_curvature_edges);
        pcl::copyPointCloud(cloud_, label_indices[4].indices, *rgb_edges);

        if (!viewer->updatePointCloud<PointT>(nan_boundary_edges, "nan boundary edges"))
          viewer->addPointCloud<PointT>(nan_boundary_edges, "nan boundary edges");

        if (!viewer->updatePointCloud<PointT>(occluding_edges, "occluding edges"))
          viewer->addPointCloud<PointT>(occluding_edges, "occluding edges");

        if (!viewer->updatePointCloud<PointT>(occluded_edges, "occluded edges"))
          viewer->addPointCloud<PointT>(occluded_edges, "occluded edges");

        if (!viewer->updatePointCloud<PointT>(high_curvature_edges,
                                              "high curvature edges"))
          viewer->addPointCloud<PointT>(high_curvature_edges, "high curvature edges");

        if (!viewer->updatePointCloud<PointT>(rgb_edges, "rgb edges"))
          viewer->addPointCloud<PointT>(rgb_edges, "rgb edges");

        cloud_mutex.unlock();
      }
    }

    interface.stop();
  }
};

void
usage(char** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n";

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

  if (arg == "--help" || arg == "-h") {
    usage(argv);
    return 1;
  }

  // clang-format off
  std::cout << "Press following keys to enable/disable the different edge types:" << std::endl;
  std::cout << "<1> EDGELABEL_NAN_BOUNDARY edge" << std::endl;
  std::cout << "<2> EDGELABEL_OCCLUDING edge" << std::endl;
  std::cout << "<3> EDGELABEL_OCCLUDED edge" << std::endl;
  //std::cout << "<4> EDGELABEL_HIGH_CURVATURE edge" << std::endl;
  //std::cout << "<5> EDGELABEL_RGB_CANNY edge" << std::endl;
  std::cout << "<Q,q> quit" << std::endl;
  // clang-format on
  pcl::OpenNIGrabber grabber(arg);
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb>()) {
    OpenNIOrganizedEdgeDetection app;
    app.run();
  }
  else
    PCL_ERROR("The input device does not provide a PointXYZRGBA mode.\n");

  return 0;
}

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
#include <pcl/io/openni_grabber.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <mutex>

using PointT = pcl::PointXYZRGBA;

class OpenNIOrganizedMultiPlaneSegmentation {
private:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::PointCloud<PointT>::ConstPtr prev_cloud;
  std::mutex cloud_mutex;

public:
  OpenNIOrganizedMultiPlaneSegmentation() = default;
  ~OpenNIOrganizedMultiPlaneSegmentation() = default;

  pcl::visualization::PCLVisualizer::Ptr
  cloudViewer(const pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(
        cloud, 0, 255, 0);
    viewer->addPointCloud<PointT>(cloud, single_color, "cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
    viewer->addCoordinateSystem(1.0, "global");
    viewer->initCameraParameters();
    return viewer;
  }

  void
  cloud_cb_(const pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    if (!viewer->wasStopped()) {
      cloud_mutex.lock();
      prev_cloud = cloud;
      cloud_mutex.unlock();
    }
  }

  void
  removePreviousDataFromScreen(std::size_t prev_models_size)
  {
    char name[1024];
    for (std::size_t i = 0; i < prev_models_size; i++) {
      sprintf(name, "normal_%lu", i);
      viewer->removeShape(name);

      sprintf(name, "plane_%02zu", i);
      viewer->removePointCloud(name);
    }
  }

  void
  run()
  {
    pcl::OpenNIGrabber interface;

    std::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f =
        [this](const pcl::PointCloud<PointT>::ConstPtr& cloud) { cloud_cb_(cloud); };

    // make a viewer
    pcl::PointCloud<PointT>::Ptr init_cloud_ptr(new pcl::PointCloud<PointT>);
    viewer = cloudViewer(init_cloud_ptr);
    boost::signals2::connection c = interface.registerCallback(f);

    interface.start();

    unsigned char red[6] = {255, 0, 0, 255, 255, 0};
    unsigned char grn[6] = {0, 255, 0, 255, 0, 255};
    unsigned char blu[6] = {0, 0, 255, 0, 255, 255};

    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.03f);
    ne.setNormalSmoothingSize(20.0f);

    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(10000);
    mps.setAngularThreshold(0.017453 * 2.0); // 3 degrees
    mps.setDistanceThreshold(0.02);          // 2cm

    std::vector<pcl::PlanarRegion<PointT>,
                Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>>
        regions;
    pcl::PointCloud<PointT>::Ptr contour(new pcl::PointCloud<PointT>);
    std::size_t prev_models_size = 0;
    char name[1024];

    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);

      if (prev_cloud && cloud_mutex.try_lock()) {
        regions.clear();
        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(
            new pcl::PointCloud<pcl::Normal>);
        double normal_start = pcl::getTime();
        ne.setInputCloud(prev_cloud);
        ne.compute(*normal_cloud);
        double normal_end = pcl::getTime();
        std::cout << "Normal Estimation took " << double(normal_end - normal_start)
                  << std::endl;

        double plane_extract_start = pcl::getTime();
        mps.setInputNormals(normal_cloud);
        mps.setInputCloud(prev_cloud);
        mps.segmentAndRefine(regions);
        double plane_extract_end = pcl::getTime();
        std::cout << "Plane extraction took "
                  << double(plane_extract_end - plane_extract_start) << std::endl;
        std::cout << "Frame took " << double(plane_extract_end - normal_start)
                  << std::endl;

        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

        if (!viewer->updatePointCloud<PointT>(prev_cloud, "cloud"))
          viewer->addPointCloud<PointT>(prev_cloud, "cloud");

        removePreviousDataFromScreen(prev_models_size);
        // Draw Visualization
        for (std::size_t i = 0; i < regions.size(); i++) {
          Eigen::Vector3f centroid = regions[i].getCentroid();
          Eigen::Vector4f model = regions[i].getCoefficients();
          pcl::PointXYZ pt1 = pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
          pcl::PointXYZ pt2 = pcl::PointXYZ(centroid[0] + (0.5f * model[0]),
                                            centroid[1] + (0.5f * model[1]),
                                            centroid[2] + (0.5f * model[2]));
          sprintf(name, "normal_%lu", i);
          viewer->addArrow(pt2, pt1, 1.0, 0, 0, false, name);

          contour->points = regions[i].getContour();
          sprintf(name, "plane_%02zu", i);
          pcl::visualization::PointCloudColorHandlerCustom<PointT> color(
              contour, red[i], grn[i], blu[i]);
          viewer->addPointCloud(contour, color, name);
          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
        }
        prev_models_size = regions.size();
        cloud_mutex.unlock();
      }
    }

    interface.stop();
  }
};

int
main()
{
  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;
  multi_plane_app.run();
  return 0;
}

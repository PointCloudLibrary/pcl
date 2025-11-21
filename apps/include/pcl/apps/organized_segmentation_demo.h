/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <pcl/apps/organized_segmentation_demo_qt.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/grabber.h> // for Grabber
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZRGBA;

// Useful macros
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

namespace Ui {
class MainWindow;
}

class OrganizedSegmentationDemo : public QMainWindow {
  Q_OBJECT
public:
  using Cloud = pcl::PointCloud<PointT>;
  using CloudPtr = Cloud::Ptr;
  using CloudConstPtr = Cloud::ConstPtr;

  OrganizedSegmentationDemo(pcl::Grabber& grabber);

  ~OrganizedSegmentationDemo() override
  {
    if (grabber_.isRunning())
      grabber_.stop();
  }

  void
  cloud_cb(const CloudConstPtr& cloud);

protected:
  pcl::visualization::PCLVisualizer::Ptr vis_;
  pcl::Grabber& grabber_;

  QMutex mtx_;
  QMutex vis_mtx_;
  Ui::MainWindow* ui_;
  QTimer* vis_timer_;
  pcl::PointCloud<PointT> prev_cloud_;
  pcl::PointCloud<pcl::Normal> prev_normals_;
  std::vector<pcl::PlanarRegion<PointT>,
              Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>>
      prev_regions_;
  float* prev_distance_map_;

  pcl::PointCloud<PointT>::CloudVectorType prev_clusters_;

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;

  bool capture_;
  bool data_modified_;
  std::size_t previous_data_size_;
  std::size_t previous_clusters_size_;

  bool display_normals_;
  bool display_curvature_;
  bool display_distance_map_;

  bool use_planar_refinement_;
  bool use_clustering_;

  pcl::PlaneCoefficientComparator<PointT, pcl::Normal>::Ptr plane_comparator_;
  pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr
      euclidean_comparator_;
  pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr rgb_comparator_;
  pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal> rgb_comp_;
  pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>::Ptr edge_aware_comparator_;
  pcl::EuclideanClusterComparator<PointT, pcl::Label>::Ptr
      euclidean_cluster_comparator_;

public Q_SLOTS:
  void
  toggleCapturePressed()
  {
    capture_ = !capture_;
  }

  void
  usePlaneComparatorPressed();
  void
  useEuclideanComparatorPressed();
  void
  useRGBComparatorPressed();
  void
  useEdgeAwareComparatorPressed();

  void
  displayCurvaturePressed();
  void
  displayDistanceMapPressed();
  void
  displayNormalsPressed();

  void
  disableRefinementPressed()
  {
    use_planar_refinement_ = false;
  }

  void
  usePlanarRefinementPressed()
  {
    use_planar_refinement_ = true;
  }

  void
  disableClusteringPressed()
  {
    use_clustering_ = false;
  }

  void
  useEuclideanClusteringPressed()
  {
    use_clustering_ = true;
  }

private Q_SLOTS:
  void
  timeoutSlot();

private:
  void
  refreshView();
};

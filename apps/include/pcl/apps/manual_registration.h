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

#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QMainWindow>
#include <QMutex>
#include <QTimer>

using PointT = pcl::PointXYZ;

namespace Ui {
class MainWindow;
}

class ManualRegistration : public QMainWindow {
  Q_OBJECT
public:
  using Cloud = pcl::PointCloud<PointT>;
  using CloudPtr = Cloud::Ptr;
  using CloudConstPtr = Cloud::ConstPtr;

  PCL_MAKE_ALIGNED_OPERATOR_NEW

  ManualRegistration(float voxel_size);

  ~ManualRegistration() override = default;

  void
  setSrcCloud(pcl::PointCloud<PointT>::Ptr cloud_src)
  {
    cloud_src_ = std::move(cloud_src);
    vis_src_->addPointCloud(cloud_src_, "cloud_src_");
  }
  void
  setDstCloud(pcl::PointCloud<PointT>::Ptr cloud_dst)
  {
    cloud_dst_ = std::move(cloud_dst);
    vis_dst_->addPointCloud(cloud_dst_, "cloud_dst_");
  }

  void
  SrcPointPickCallback(const pcl::visualization::PointPickingEvent& event, void*);
  void
  DstPointPickCallback(const pcl::visualization::PointPickingEvent& event, void*);

protected:
  void
  refreshView();

  pcl::visualization::PCLVisualizer::Ptr vis_src_;
  pcl::visualization::PCLVisualizer::Ptr vis_dst_;

  pcl::PointCloud<PointT>::Ptr cloud_src_;
  pcl::PointCloud<PointT>::Ptr cloud_dst_;

  QMutex mtx_;
  QMutex vis_mtx_;
  Ui::MainWindow* ui_;

  bool src_point_selected_{false};
  bool dst_point_selected_{false};

  pcl::PointXYZ src_point_;
  pcl::PointXYZ dst_point_;

  pcl::PointCloud<pcl::PointXYZ> src_pc_;
  pcl::PointCloud<pcl::PointXYZ> dst_pc_;

  Eigen::Matrix4f transform_ = Eigen::Affine3f::Identity().matrix();

  std::set<std::string> annotations_src_;
  std::set<std::string> annotations_dst_;

  const float voxel_size_;

public Q_SLOTS:
  void
  confirmSrcPointPressed();
  void
  confirmDstPointPressed();
  void
  calculatePressed();
  void
  clearPressed();
  void
  orthoChanged(int state);
  void
  applyTransformPressed();
  void
  refinePressed();
};

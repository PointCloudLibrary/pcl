/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#include <pcl/apps/modeler/downsample_worker.h>
#include <pcl/apps/modeler/parameter_dialog.h>
#include <pcl/apps/modeler/parameter.h>
#include <pcl/apps/modeler/cloud_item.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::DownSampleWorker::DownSampleWorker(const std::vector<CloudItem*>& polymeshs, QWidget* parent) :
  AbstractWorker(polymeshs, parent),
  x_min_(std::numeric_limits<double>::max()), x_max_(std::numeric_limits<double>::min()),
  y_min_(std::numeric_limits<double>::max()), y_max_(std::numeric_limits<double>::min()),
  z_min_(std::numeric_limits<double>::max()), z_max_(std::numeric_limits<double>::min()),
  leaf_size_x_(NULL), leaf_size_y_(NULL), leaf_size_z_(NULL)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::DownSampleWorker::~DownSampleWorker(void)
{
  delete leaf_size_x_;
  delete leaf_size_y_;
  delete leaf_size_z_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::DownSampleWorker::initParameters(PointCloud2Ptr input_cloud)
{
  Eigen::Vector4f min_pt, max_pt;
  int x_idx = pcl::getFieldIndex(*input_cloud, "x");
  int y_idx = pcl::getFieldIndex(*input_cloud, "y");
  int z_idx = pcl::getFieldIndex(*input_cloud, "z");
  pcl::getMinMax3D(input_cloud, x_idx, y_idx, z_idx, min_pt, max_pt);

  x_min_ = std::min(double(min_pt.x()), x_min_);
  x_max_ = std::max(double(max_pt.x()), x_max_);

  y_min_ = std::min(double(min_pt.y()), y_min_);
  y_max_ = std::max(double(max_pt.y()), y_max_);

  z_min_ = std::min(double(min_pt.z()), z_min_);
  z_max_ = std::max(double(max_pt.z()), z_max_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::DownSampleWorker::setupParameters()
{
  double x_range = x_max_ - x_min_;
  double y_range = y_max_ - y_min_;
  double z_range = z_max_ - z_min_;

  double range_max = std::max(x_range, std::max(y_range, z_range));
  double size = range_max/1000;
  double step = range_max/1000;

  leaf_size_x_ = new DoubleParameter("Leaf Size X", "The X size of the voxel grid", size, 0, x_max_-x_min_, step);
  leaf_size_y_ = new DoubleParameter("Leaf Size Y", "The Y size of the voxel grid", size, 0, y_max_-y_min_, step);
  leaf_size_z_ = new DoubleParameter("Leaf Size Z", "The Z size of the voxel grid", size, 0, z_max_-z_min_, step);

  parameter_dialog_->addParameter(leaf_size_x_);
  parameter_dialog_->addParameter(leaf_size_y_);
  parameter_dialog_->addParameter(leaf_size_z_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::DownSampleWorker::processImpl(CloudItem* polymesh) const
{
  sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);

  pcl::VoxelGrid<sensor_msgs::PointCloud2> voxel_grid;
  voxel_grid.setInputCloud (polymesh->getCloud());
  voxel_grid.setLeafSize (float (*leaf_size_x_), float (*leaf_size_y_), float (*leaf_size_z_));
  voxel_grid.filter (*output_cloud);

  *(polymesh->getCloud()) = *output_cloud;

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::DownSampleWorker::postProcessImpl(CloudItem* polymesh) const
{
  polymesh->updateGeometryItems();

  return;
}

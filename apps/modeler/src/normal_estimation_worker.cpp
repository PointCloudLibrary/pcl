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

#include <pcl/apps/modeler/normal_estimation_worker.h>
#include <pcl/apps/modeler/parameter_dialog.h>
#include <pcl/apps/modeler/parameter.h>
#include <pcl/apps/modeler/polymesh_item.h>
#include <pcl/apps/modeler/normal_item.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::NormalEstimationWorker::NormalEstimationWorker(const std::vector<PolymeshItem*>& polymeshs, QWidget* parent) :
  x_min_(std::numeric_limits<double>::max()), x_max_(std::numeric_limits<double>::min()),
  y_min_(std::numeric_limits<double>::max()), y_max_(std::numeric_limits<double>::min()),
  z_min_(std::numeric_limits<double>::max()), z_max_(std::numeric_limits<double>::min()),
  search_radius_(NULL),
  AbstractWorker(polymeshs, parent)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::NormalEstimationWorker::~NormalEstimationWorker(void)
{
  delete search_radius_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalEstimationWorker::initParameters(PointCloud2Ptr input_cloud)
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
pcl::modeler::NormalEstimationWorker::setupParameters()
{
  double x_range = x_max_ - x_min_;
  double y_range = y_max_ - y_min_;
  double z_range = z_max_ - z_min_;

  double range_max = std::max(x_range, std::max(y_range, z_range));
  double radius = range_max/100;
  double step = range_max/1000;

  search_radius_ = new DoubleParameter("Search Radius",
    "The sphere radius that is to be used for determining the nearest neighbors", radius, 0, x_max_-x_min_, step);

  parameter_dialog_->addParameter(search_radius_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalEstimationWorker::processImpl(PolymeshItem* polymesh) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*(polymesh->getCloud()), *cloud);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (*search_radius_);

  // Compute the features
  ne.compute (*cloud_normals);


  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalEstimationWorker::postProcessImpl(PolymeshItem* polymesh) const
{
  polymesh->attachNormalItem();

  return;
}

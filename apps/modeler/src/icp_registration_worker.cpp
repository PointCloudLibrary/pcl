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

#include <pcl/apps/modeler/icp_registration_worker.h>
#include <pcl/apps/modeler/parameter_dialog.h>
#include <pcl/apps/modeler/parameter.h>
#include <pcl/apps/modeler/cloud_mesh.h>
#include <pcl/apps/modeler/cloud_mesh_item.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ICPRegistrationWorker::ICPRegistrationWorker(CloudMesh::PointCloudPtr cloud, const QList<CloudMeshItem*>& cloud_mesh_items, QWidget* parent)
  : AbstractWorker(cloud_mesh_items, parent),
  cloud_(cloud),
  x_min_(std::numeric_limits<double>::max()), x_max_(std::numeric_limits<double>::min()),
  y_min_(std::numeric_limits<double>::max()), y_max_(std::numeric_limits<double>::min()),
  z_min_(std::numeric_limits<double>::max()), z_max_(std::numeric_limits<double>::min()),
  max_correspondence_distance_(NULL),
  max_iterations_(NULL),
  transformation_epsilon_(NULL),
  euclidean_fitness_epsilon_(NULL)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ICPRegistrationWorker::~ICPRegistrationWorker(void)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ICPRegistrationWorker::initParameters(CloudMeshItem* cloud_mesh_item)
{
  cloud_->clear();

  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*(cloud_mesh_item->getCloudMesh()->getCloud()), min_pt, max_pt);

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
pcl::modeler::ICPRegistrationWorker::setupParameters()
{
  double x_range = x_max_ - x_min_;
  double y_range = y_max_ - y_min_;
  double z_range = z_max_ - z_min_;

  double range_max = std::max(x_range, std::max(y_range, z_range));
  double max_correspondence_distance = range_max/2;
  double step = range_max/1000;

  max_correspondence_distance_ = new DoubleParameter("Max Correspondence Distance",
    "If the distance is larger than this threshold, the points will be ignored in the alignment process.", max_correspondence_distance, 0, x_max_-x_min_, step);

  max_iterations_ = new IntParameter("Max Iterations",
    "Set the maximum number of iterations the internal optimization should run for.", 10, 0, 256);

  double transformation_epsilon = range_max/2;
  transformation_epsilon_ = new DoubleParameter("Transformation Epsilon",
    "Maximum allowable difference between two consecutive transformations.", 0.0, 0, transformation_epsilon, step);

  double euclidean_fitness_epsilon = range_max/2;
  euclidean_fitness_epsilon_ = new DoubleParameter("Euclidean Fitness Epsilon",
    "Maximum allowed Euclidean error between two consecutive steps in the ICP loop.", 0.0, 0, euclidean_fitness_epsilon, step);

  parameter_dialog_->addParameter(max_correspondence_distance_);
  parameter_dialog_->addParameter(max_iterations_);
  parameter_dialog_->addParameter(transformation_epsilon_);
  parameter_dialog_->addParameter(euclidean_fitness_epsilon_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ICPRegistrationWorker::processImpl(CloudMeshItem* cloud_mesh_item)
{
  if (cloud_->empty())
  {
    *cloud_ = *(cloud_mesh_item->getCloudMesh()->getCloud());
    return;
  }

  pcl::IterativeClosestPoint<CloudMesh::PointT, CloudMesh::PointT> icp;

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (*max_correspondence_distance_);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (*max_iterations_);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (*transformation_epsilon_);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (*euclidean_fitness_epsilon_);

  icp.setInputSource(cloud_mesh_item->getCloudMesh()->getCloud());
  icp.setInputTarget(cloud_);
  pcl::PointCloud<CloudMesh::PointT> result;
  icp.align(result);

  result.sensor_origin_ = cloud_mesh_item->getCloudMesh()->getCloud()->sensor_origin_;
  result.sensor_orientation_ = cloud_mesh_item->getCloudMesh()->getCloud()->sensor_orientation_;

  *(cloud_mesh_item->getCloudMesh()->getCloud()) = result;
  *cloud_ += result;

  return;
}

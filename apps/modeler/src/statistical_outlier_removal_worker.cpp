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

#include <pcl/apps/modeler/statistical_outlier_removal_worker.h>
#include <pcl/apps/modeler/parameter.h>
#include <pcl/apps/modeler/parameter_dialog.h>
#include <pcl/apps/modeler/cloud_mesh.h>
#include <pcl/apps/modeler/cloud_mesh_item.h>
#include <pcl/filters/statistical_outlier_removal.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::StatisticalOutlierRemovalWorker::StatisticalOutlierRemovalWorker(const QList<CloudMeshItem*>& cloud_mesh_items, QWidget* parent) :
  AbstractWorker(cloud_mesh_items, parent),
  mean_k_(NULL), stddev_mul_thresh_(NULL)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::StatisticalOutlierRemovalWorker::~StatisticalOutlierRemovalWorker(void)
{
  delete mean_k_;
  delete stddev_mul_thresh_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::StatisticalOutlierRemovalWorker::initParameters(CloudMeshItem *)
{
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::StatisticalOutlierRemovalWorker::setupParameters()
{
  mean_k_ = new IntParameter("Mean K", "The number of nearest neighbors to use for mean distance estimation", 8, 1, 1024, 1);
  stddev_mul_thresh_ = new DoubleParameter("Standard Deviation Multiplier", "The distance threshold will be equal to: mean + stddev_mult * stddev. Points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.", 1.0, 0.1, 10, 0.1);

  parameter_dialog_->addParameter(mean_k_);
  parameter_dialog_->addParameter(stddev_mul_thresh_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::StatisticalOutlierRemovalWorker::processImpl(CloudMeshItem* cloud_mesh_item)
{
  pcl::StatisticalOutlierRemoval<pcl::PointSurfel> sor;
  sor.setInputCloud(cloud_mesh_item->getCloudMesh()->getCloud());
  sor.setMeanK(*mean_k_);
  sor.setStddevMulThresh(*stddev_mul_thresh_);

  CloudMesh::PointCloudPtr cloud(new CloudMesh::PointCloud());
  sor.filter(*cloud);

  cloud_mesh_item->getCloudMesh()->getCloud() = cloud;

  emitDataUpdated(cloud_mesh_item);

  return;
}

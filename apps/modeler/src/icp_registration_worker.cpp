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

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ICPRegistrationWorker::ICPRegistrationWorker(CloudMesh::PointCloudPtr cloud, const QList<CloudMeshItem*>& cloud_mesh_items, QWidget* parent)
  : AbstractWorker(cloud_mesh_items, parent),
  cloud_(cloud)
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

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ICPRegistrationWorker::setupParameters()
{
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
  icp.setInputCloud(cloud_);
  icp.setInputTarget(cloud_mesh_item->getCloudMesh()->getCloud());
  pcl::PointCloud<CloudMesh::PointT> result;
  icp.align(result);

  result.sensor_origin_ = cloud_mesh_item->getCloudMesh()->getCloud()->sensor_origin_;
  result.sensor_orientation_ = cloud_mesh_item->getCloudMesh()->getCloud()->sensor_orientation_;

  *cloud_ = result;

  return;
}

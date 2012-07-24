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

#include <pcl/apps/modeler/channel_actor_item.h>

#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkMatrix4x4.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <pcl/point_cloud.h>
#include <pcl/apps/modeler/cloud_mesh.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ChannelActorItem::ChannelActorItem(QTreeWidgetItem* parent,
  const boost::shared_ptr<CloudMesh>& cloud_mesh,
  const vtkSmartPointer<vtkRenderWindow>& render_window,
  const vtkSmartPointer<vtkActor>& actor,
  const std::string& channel_name)
  :QTreeWidgetItem(parent),
  AbstractItem(),
  cloud_mesh_(cloud_mesh),
  render_window_(render_window),
  actor_(actor),
  viewpoint_transformation_(vtkSmartPointer<vtkMatrix4x4>::New())
{
  setText(0, QString(channel_name.c_str()));

  CloudMesh::PointCloudConstPtr cloud = cloud_mesh->getCloud();
  const Eigen::Vector4f& origin = cloud->sensor_origin_;
  const Eigen::Quaternionf& orientation = cloud->sensor_orientation_;
  convertToVtkMatrix (origin, orientation, viewpoint_transformation_);

  Eigen::Matrix3f rotation;
  rotation = orientation;

  vtkSmartPointer<vtkCamera> camera = render_window->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
  camera->SetPosition(origin[0], origin[1], origin[2]);
  camera->SetFocalPoint(origin [0] + rotation (0, 2), origin [1] + rotation (1, 2), origin [2] + rotation (2, 2));
  camera->SetViewUp(rotation (0, 1), rotation (1, 1), rotation (2, 1));

  attachActor();
  render_window_->Render();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ChannelActorItem::~ChannelActorItem ()
{
  detachActor();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ChannelActorItem::convertToVtkMatrix(const Eigen::Vector4f &origin,
  const Eigen::Quaternion<float> &orientation,
  vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
{
  // set rotation
  Eigen::Matrix3f rot = orientation.toRotationMatrix ();
  for (int i = 0; i < 3; i++)
    for (int k = 0; k < 3; k++)
      vtk_matrix->SetElement (i, k, rot (i, k));

  // set translation
  vtk_matrix->SetElement (0, 3, origin (0));
  vtk_matrix->SetElement (1, 3, origin (1));
  vtk_matrix->SetElement (2, 3, origin (2));
  vtk_matrix->SetElement (3, 3, 1.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ChannelActorItem::createActor()
{
  createActorImpl();
  render_window_->Render();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ChannelActorItem::attachActor()
{
  render_window_->GetRenderers()->GetFirstRenderer()->AddActor(actor_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ChannelActorItem::detachActor()
{
  render_window_->GetRenderers()->GetFirstRenderer()->RemoveActor(actor_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ChannelActorItem::prepareContextMenu(QMenu* menu) const
{

}
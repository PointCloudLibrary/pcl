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
#include <pcl/apps/modeler/cloud_mesh.h>
#include <pcl/point_cloud.h>

#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ChannelActorItem::ChannelActorItem(
    QTreeWidgetItem* parent,
    const CloudMesh::Ptr& cloud_mesh,
    const vtkSmartPointer<vtkRenderWindow>& render_window,
    const vtkSmartPointer<vtkActor>& actor,
    const std::string& channel_name)
: QTreeWidgetItem(parent)
, cloud_mesh_(cloud_mesh)
, poly_data_(vtkSmartPointer<vtkPolyData>::New())
, render_window_(render_window)
, color_scheme_("rgb")
, actor_(actor)
{
  setFlags(flags() & (~Qt::ItemIsDragEnabled));
  setFlags(flags() & (~Qt::ItemIsDropEnabled));

  setText(0, QString(channel_name.c_str()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ChannelActorItem::~ChannelActorItem() { detachActor(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ChannelActorItem::init()
{
  initImpl();

  attachActor();
  render_window_->Render();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ChannelActorItem::update()
{
  updateImpl();

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
pcl::modeler::ChannelActorItem::prepareContextMenu(QMenu*) const
{}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ChannelActorItem::switchRenderWindow(vtkRenderWindow* render_window)
{
  detachActor();
  render_window_ = vtkSmartPointer<vtkRenderWindow>(render_window);
  attachActor();
}

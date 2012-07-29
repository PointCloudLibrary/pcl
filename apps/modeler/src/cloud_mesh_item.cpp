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

#include <pcl/apps/modeler/cloud_mesh_item.h>
#include <pcl/apps/modeler/render_window.h>
#include <pcl/apps/modeler/render_window_item.h>
#include <pcl/apps/modeler/points_actor_item.h>
#include <pcl/apps/modeler/normals_actor_item.h>
#include <pcl/apps/modeler/surface_actor_item.h>
#include <pcl/apps/modeler/cloud_mesh.h>
#include <pcl/apps/modeler/main_window.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudMeshItem::CloudMeshItem (QTreeWidgetItem* parent, const std::string& filename)
  :QTreeWidgetItem(parent),
  AbstractItem(),
  filename_(filename),
  cloud_mesh_(boost::shared_ptr<CloudMesh>(new CloudMesh()))
{
  setText(0, QString(filename.c_str()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudMeshItem::~CloudMeshItem ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::CloudMeshItem::savePointCloud(const QList<CloudMeshItem*>& items, const QString& filename)
{
  if (items.size() == 1)
    return (CloudMesh::save(*items.first()->getCloudMesh()->getCloud(), filename.toStdString()));
  
  QList<CloudMeshItem*>::const_iterator items_it = items.begin();
  pcl::PointCloud<pcl::PointSurfel> cloud = *(*items_it)->getCloudMesh()->getCloud();
  ++ items_it;

  while (items_it != items.end())
  {
    cloud += *(*items_it)->getCloudMesh()->getCloud();
    ++ items_it;
  }

  return (CloudMesh::save(cloud, filename.toStdString()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::CloudMeshItem::open()
{
  if(!cloud_mesh_->open(filename_))
    return (false);

  createChannels();

  treeWidget()->expandItem(this);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::close()
{
  parent()->removeChild(this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::prepareContextMenu(QMenu* menu) const
{
  menu->addAction(ui()->actionDownSamplePoints);
  menu->addAction(ui()->actionEstimateNormals);
  menu->addAction(ui()->actionPoissonReconstruction);
  menu->addAction(ui()->actionSavePointCloud);
  menu->addAction(ui()->actionClosePointCloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::createChannels()
{
  RenderWindowItem* render_window_item = dynamic_cast<RenderWindowItem*>(parent());
  addChild(new PointsActorItem(this, cloud_mesh_, render_window_item->getRenderWindow()->GetRenderWindow()));
  addChild(new NormalsActorItem(this, cloud_mesh_, render_window_item->getRenderWindow()->GetRenderWindow()));
  addChild(new SurfaceActorItem(this, cloud_mesh_, render_window_item->getRenderWindow()->GetRenderWindow()));
  for (int i = 0, i_end = childCount(); i < i_end; ++ i)
  {
    ChannelActorItem* child_item = dynamic_cast<ChannelActorItem*>(child(i));
    child_item->init();
  }

  return;
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::updateChannels()
{
  cloud_mesh_->updateVtkPoints();
  cloud_mesh_->updateVtkPolygons();

  for (int i = 0, i_end = childCount(); i < i_end; ++ i)
  {
    ChannelActorItem* child_item = dynamic_cast<ChannelActorItem*>(child(i));
    child_item->update();
  }

  return;
}
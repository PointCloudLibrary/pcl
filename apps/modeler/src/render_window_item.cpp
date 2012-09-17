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

#include <pcl/apps/modeler/render_window_item.h>
#include <pcl/apps/modeler/render_window.h>
#include <pcl/apps/modeler/cloud_mesh_item.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/apps/modeler/parameter.h>
#include <pcl/apps/modeler/parameter_dialog.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWindowItem::RenderWindowItem(QTreeWidget * parent)
  : QTreeWidgetItem(parent),
  AbstractItem(),
  render_window_(new RenderWindow(this)),
  background_color_(new ColorParameter("Background Color", "The background color of the render window", QColor(0, 0, 0))),
  show_axes_(new BoolParameter("Show Axes", "Show Axes", true))
{
  setFlags(flags()&(~Qt::ItemIsDragEnabled));
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWindowItem::~RenderWindowItem()
{
  render_window_->deleteLater();
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::RenderWindowItem::openPointCloud(const QString& filename)
{
  CloudMeshItem* cloud_mesh_item = new CloudMeshItem(this, filename.toStdString());
  addChild(cloud_mesh_item);

  if (!cloud_mesh_item->open())
  {
    removeChild(cloud_mesh_item);
    delete cloud_mesh_item;
    return (false);
  }

  treeWidget()->setCurrentItem(cloud_mesh_item);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudMeshItem*
pcl::modeler::RenderWindowItem::addPointCloud(CloudMesh::PointCloudPtr cloud)
{
  CloudMeshItem* cloud_mesh_item = new CloudMeshItem(this, cloud);
  addChild(cloud_mesh_item);

  treeWidget()->setCurrentItem(cloud_mesh_item);

  return (cloud_mesh_item);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindowItem::prepareContextMenu(QMenu* menu) const
{
  menu->addAction(ui()->actionOpenPointCloud);
  menu->addAction(ui()->actionImportPointCloud);
  if (treeWidget()->topLevelItem(0) != this && childCount() == 0)
    menu->addAction(ui()->actionCloseRenderWindow);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindowItem::prepareProperties(ParameterDialog* parameter_dialog)
{
  double r, g, b;
  render_window_->getBackground(r, g, b);
  QColor color(static_cast<int> (r*255), static_cast<int> (g*255), static_cast<int> (b*255));
  background_color_->setDefaultValue(color);
  parameter_dialog->addParameter(background_color_);
  parameter_dialog->addParameter(show_axes_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::RenderWindowItem::setProperties()
{
  QColor color = *background_color_;
  render_window_->setBackground(color.red()/255.0, color.green()/255.0, color.blue()/255.0);

  render_window_->setShowAxes(*show_axes_);

  return;
}

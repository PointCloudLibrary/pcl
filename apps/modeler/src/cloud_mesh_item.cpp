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

#include <pcl/apps/modeler/cloud_mesh.h>
#include <pcl/apps/modeler/cloud_mesh_item.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/apps/modeler/normals_actor_item.h>
#include <pcl/apps/modeler/parameter.h>
#include <pcl/apps/modeler/parameter_dialog.h>
#include <pcl/apps/modeler/points_actor_item.h>
#include <pcl/apps/modeler/render_window.h>
#include <pcl/apps/modeler/render_window_item.h>
#include <pcl/apps/modeler/surface_actor_item.h>
#include <pcl/common/common.h>

#include <vtkRenderWindow.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudMeshItem::CloudMeshItem(QTreeWidgetItem* parent,
                                           const std::string& filename)
: QTreeWidgetItem(parent)
, filename_(filename)
, cloud_mesh_(new CloudMesh)
, translation_x_(new DoubleParameter("Translation X", "Translation X", 0.0, -1.0, 1.0))
, translation_y_(new DoubleParameter("Translation Y", "Translation Y", 0.0, -1.0, 1.0))
, translation_z_(new DoubleParameter("Translation Z", "Translation Z", 0.0, -1.0, 1.0))
, rotation_x_(new DoubleParameter("Rotation X", "Rotation X", 0.0, -180.0, 180.0))
, rotation_y_(new DoubleParameter("Rotation Y", "Rotation Y", 0.0, -180.0, 180.0))
, rotation_z_(new DoubleParameter("Rotation Z", "Rotation Z", 0.0, -180.0, 180.0))
{
  setFlags(flags() & (~Qt::ItemIsDropEnabled));
  setText(0, QString(filename.c_str()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudMeshItem::CloudMeshItem(QTreeWidgetItem* parent,
                                           CloudMesh::PointCloudPtr cloud)
: QTreeWidgetItem(parent)
, filename_("unnamed point cloud")
, cloud_mesh_(new CloudMesh(std::move(cloud)))
, translation_x_(new DoubleParameter("Translation X", "Translation X", 0.0, -1.0, 1.0))
, translation_y_(new DoubleParameter("Translation Y", "Translation Y", 0.0, -1.0, 1.0))
, translation_z_(new DoubleParameter("Translation Z", "Translation Z", 0.0, -1.0, 1.0))
, rotation_x_(new DoubleParameter("Rotation X", "Rotation X", 0.0, -180.0, 180.0))
, rotation_y_(new DoubleParameter("Rotation Y", "Rotation Y", 0.0, -180.0, 180.0))
, rotation_z_(new DoubleParameter("Rotation Z", "Rotation Z", 0.0, -180.0, 180.0))
{
  setFlags(flags() & (~Qt::ItemIsDropEnabled));
  setText(0, QString(filename_.c_str()));

  createChannels();

  treeWidget()->expandItem(this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudMeshItem::CloudMeshItem(QTreeWidgetItem* parent,
                                           const CloudMeshItem& cloud_mesh_item)
: QTreeWidgetItem(parent)
, filename_(cloud_mesh_item.filename_)
, cloud_mesh_(cloud_mesh_item.cloud_mesh_)
, translation_x_(new DoubleParameter("Translation X", "Translation X", 0.0, -1.0, 1.0))
, translation_y_(new DoubleParameter("Translation Y", "Translation Y", 0.0, -1.0, 1.0))
, translation_z_(new DoubleParameter("Translation Z", "Translation Z", 0.0, -1.0, 1.0))
, rotation_x_(new DoubleParameter("Rotation X", "Rotation X", 0.0, -180.0, 180.0))
, rotation_y_(new DoubleParameter("Rotation Y", "Rotation Y", 0.0, -180.0, 180.0))
, rotation_z_(new DoubleParameter("Rotation Z", "Rotation Z", 0.0, -180.0, 180.0))
{
  setFlags(flags() & (~Qt::ItemIsDropEnabled));
  setText(0, QString(filename_.c_str()));

  createChannels();

  treeWidget()->expandItem(this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::CloudMeshItem::savePointCloud(const QList<CloudMeshItem*>& items,
                                            const QString& filename)
{
  if (items.size() == 1)
    return items.first()->getCloudMesh()->save(filename.toStdString());

  std::vector<const CloudMesh*> cloud_meshes;
  for (const auto& item : items) {
    cloud_meshes.push_back(item->getCloudMesh().get());
  }

  return CloudMesh::save(cloud_meshes, filename.toStdString());
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::CloudMeshItem::open()
{
  if (!cloud_mesh_->open(filename_))
    return false;

  createChannels();

  treeWidget()->expandItem(this);

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::prepareContextMenu(QMenu* menu) const
{
  menu->addMenu(ui()->menuFilters);
  menu->addMenu(ui()->menuRegistration);
  menu->addMenu(ui()->menuSurfaceReconstruction);
  menu->addAction(ui()->actionSavePointCloud);
  menu->addAction(ui()->actionClosePointCloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::createChannels()
{
  RenderWindowItem* render_window_item = dynamic_cast<RenderWindowItem*>(parent());
  vtkRenderWindow* win =
      getRenderWindowCompat(*(render_window_item->getRenderWindow()));

  addChild(new PointsActorItem(this, cloud_mesh_, win));
  addChild(new NormalsActorItem(this, cloud_mesh_, win));
  addChild(new SurfaceActorItem(this, cloud_mesh_, win));

  for (int i = 0, i_end = childCount(); i < i_end; ++i) {
    ChannelActorItem* child_item = dynamic_cast<ChannelActorItem*>(child(i));
    child_item->init();
  }

  render_window_item->getRenderWindow()->updateAxes();
  render_window_item->getRenderWindow()->resetCamera();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::updateChannels()
{
  cloud_mesh_->updateVtkPoints();
  cloud_mesh_->updateVtkPolygons();

  for (int i = 0, i_end = childCount(); i < i_end; ++i) {
    ChannelActorItem* child_item = dynamic_cast<ChannelActorItem*>(child(i));
    child_item->update();
  }

  RenderWindowItem* render_window_item = dynamic_cast<RenderWindowItem*>(parent());
  render_window_item->getRenderWindow()->updateAxes();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::prepareProperties(ParameterDialog* parameter_dialog)
{
  translation_x_->reset();
  translation_y_->reset();
  translation_z_->reset();
  rotation_x_->reset();
  rotation_y_->reset();
  rotation_z_->reset();
  parameter_dialog->addParameter(translation_x_);
  parameter_dialog->addParameter(translation_y_);
  parameter_dialog->addParameter(translation_z_);
  parameter_dialog->addParameter(rotation_x_);
  parameter_dialog->addParameter(rotation_y_);
  parameter_dialog->addParameter(rotation_z_);

  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*(cloud_mesh_->getCloud()), min_pt, max_pt);
  double x_range = max_pt.x() - min_pt.x();
  double y_range = max_pt.y() - min_pt.y();
  double z_range = max_pt.z() - min_pt.z();
  translation_x_->setLow(-x_range / 2);
  translation_x_->setHigh(x_range / 2);
  translation_x_->setStep(x_range / 1000);
  translation_y_->setLow(-y_range / 2);
  translation_y_->setHigh(y_range / 2);
  translation_y_->setStep(y_range / 1000);
  translation_z_->setLow(-z_range / 2);
  translation_z_->setHigh(z_range / 2);
  translation_z_->setStep(z_range / 1000);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::setProperties()
{
  cloud_mesh_->transform(*translation_x_,
                         *translation_y_,
                         *translation_z_,
                         *rotation_x_,
                         *rotation_y_,
                         *rotation_z_);

  updateChannels();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMeshItem::updateRenderWindow()
{
  RenderWindowItem* render_window_item = dynamic_cast<RenderWindowItem*>(parent());
  for (int i = 0, i_end = childCount(); i < i_end; ++i) {
    ChannelActorItem* child_item = dynamic_cast<ChannelActorItem*>(child(i));
    child_item->switchRenderWindow(
        getRenderWindowCompat(*render_window_item->getRenderWindow()));
  }

  render_window_item->getRenderWindow()->updateAxes();
  render_window_item->getRenderWindow()->resetCamera();
}

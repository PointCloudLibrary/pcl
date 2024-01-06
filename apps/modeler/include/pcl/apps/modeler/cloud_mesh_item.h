/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */

#pragma once

#include <pcl/apps/modeler/abstract_item.h>
#include <pcl/apps/modeler/cloud_mesh.h>

#include <QTreeWidgetItem>

namespace pcl {
namespace modeler {

class CloudMesh;
class DoubleParameter;

class CloudMeshItem : public QTreeWidgetItem, public AbstractItem {
public:
  CloudMeshItem(QTreeWidgetItem* parent, const std::string& filename);
  CloudMeshItem(QTreeWidgetItem* parent, CloudMesh::PointCloudPtr cloud);
  CloudMeshItem(QTreeWidgetItem* parent, const CloudMeshItem& cloud_mesh_item);

  inline CloudMesh::Ptr&
  getCloudMesh()
  {
    return cloud_mesh_;
  }

  inline const CloudMesh::Ptr&
  getCloudMesh() const
  {
    return cloud_mesh_;
  }

  static bool
  savePointCloud(const QList<CloudMeshItem*>& items, const QString& filename);

  bool
  open();

  void
  createChannels();

  void
  updateChannels();

  std::string
  getItemName() const override
  {
    return "Cloud Mesh Item";
  }

  void
  updateRenderWindow();

protected:
  void
  prepareContextMenu(QMenu* menu) const override;

  void
  prepareProperties(ParameterDialog* parameter_dialog) override;

  void
  setProperties() override;

private:
  std::string filename_;
  CloudMesh::Ptr cloud_mesh_;

  DoubleParameter* translation_x_;
  DoubleParameter* translation_y_;
  DoubleParameter* translation_z_;
  DoubleParameter* rotation_x_;
  DoubleParameter* rotation_y_;
  DoubleParameter* rotation_z_;
};

} // namespace modeler
} // namespace pcl

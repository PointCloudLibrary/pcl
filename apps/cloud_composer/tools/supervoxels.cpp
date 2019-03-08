/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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


#include <pcl/apps/cloud_composer/tools/supervoxels.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/apps/cloud_composer/tools/impl/supervoxels.hpp>

Q_PLUGIN_METADATA(IID "cloud_composer.ToolFactory/1.0")

pcl::cloud_composer::SupervoxelsTool::SupervoxelsTool (PropertiesModel* parameter_model, QObject* parent)
: SplitItemTool (parameter_model, parent)
{
  
}

pcl::cloud_composer::SupervoxelsTool::~SupervoxelsTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::SupervoxelsTool::performAction (ConstItemList input_data, PointTypeFlags::PointType type)
{
  if (type != PointTypeFlags::NONE)
  {
    switch ((uint8_t) type)
    {
      case (PointTypeFlags::XYZ | PointTypeFlags::RGB):
        return this->performTemplatedAction<pcl::PointXYZRGB> (input_data);
      case (PointTypeFlags::XYZ | PointTypeFlags::RGBA):
        return this->performTemplatedAction<pcl::PointXYZRGBA> (input_data);
    }
  }
  
  QList <CloudComposerItem*> output;
  
  qCritical () << "supervoxels requires templated types!";
  
  return output;
} 

template QList <pcl::cloud_composer::CloudComposerItem*> pcl::cloud_composer::SupervoxelsTool::performTemplatedAction <pcl::PointXYZRGB> (QList <const CloudComposerItem*>);
//template QList <pcl::cloud_composer::CloudComposerItem*> pcl::cloud_composer::SupervoxelsTool::performTemplatedAction <pcl::PointXYZRGBA> (QList <const CloudComposerItem*>);


/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::SupervoxelsToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);
  
  parameter_model->addProperty ("Resolution", 0.008,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Seed Resolution", 0.08,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("RGB Weight", 0.2,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Normals Weight", 0.8,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Spatial Weight", 0.4,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  
  return parameter_model;
}

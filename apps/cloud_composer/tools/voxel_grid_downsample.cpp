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


#include <pcl/apps/cloud_composer/tools/voxel_grid_downsample.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>


#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  Q_EXPORT_PLUGIN2(cloud_composer_voxel_grid_downsample_tool, pcl::cloud_composer::VoxelGridDownsampleToolFactory)
#else
  Q_PLUGIN_METADATA(IID "cloud_composer.ToolFactory/1.0")
#endif

pcl::cloud_composer::VoxelGridDownsampleTool::VoxelGridDownsampleTool (PropertiesModel* parameter_model, QObject* parent)
  : ModifyItemTool (parameter_model, parent)
{

  
}

pcl::cloud_composer::VoxelGridDownsampleTool::~VoxelGridDownsampleTool ()
{
  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::VoxelGridDownsampleTool::performAction (ConstItemList input_data, PointTypeFlags::PointType)
{
  QList <CloudComposerItem*> output;
  const CloudComposerItem* input_item;
  // Check input data length
  if ( input_data.size () == 0)
  {
    qCritical () << "Empty input in VoxelGridDownsampleTool!";
    return output;
  }
  else if ( input_data.size () > 1)
  {
    qWarning () << "Input vector has more than one item in VoxelGridDownsampleTool";
  }
  input_item = input_data.value (0);
    
  if (input_item->type () == CloudComposerItem::CLOUD_ITEM)
  {
    double leaf_x = parameter_model_->getProperty("Leaf Size x").toDouble ();
    double leaf_y = parameter_model_->getProperty("Leaf Size y").toDouble ();
    double leaf_z = parameter_model_->getProperty("Leaf Size z").toDouble ();
    
    pcl::PCLPointCloud2::ConstPtr input_cloud = input_item->data (ItemDataRole::CLOUD_BLOB).value <pcl::PCLPointCloud2::ConstPtr> ();
    
    //////////////// THE WORK - FILTERING OUTLIERS ///////////////////
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox_grid;
    vox_grid.setInputCloud (input_cloud);
    vox_grid.setLeafSize (float (leaf_x), float (leaf_y), float (leaf_z));
    
    
    //Create output cloud
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
    //Filter!  
    vox_grid.filter (*cloud_filtered);

    //////////////////////////////////////////////////////////////////
    //Get copies of the original origin and orientation
    Eigen::Vector4f source_origin = input_item->data (ItemDataRole::ORIGIN).value<Eigen::Vector4f> ();
    Eigen::Quaternionf source_orientation =  input_item->data (ItemDataRole::ORIENTATION).value<Eigen::Quaternionf> ();
    //Put the modified cloud into an item, stick in output
    CloudItem* cloud_item = new CloudItem (input_item->text () + tr (" vox ds")
                                           , cloud_filtered
                                           , source_origin
                                           , source_orientation);

    
    output.append (cloud_item);
  }
  else
  {
    qDebug () << "Input item in VoxelGridDownsampleTool is not a cloud!!!";
  }
  
  
  return output;
}

/////////////////// PARAMETER MODEL /////////////////////////////////
pcl::cloud_composer::PropertiesModel*
pcl::cloud_composer::VoxelGridDownsampleToolFactory::createToolParameterModel (QObject* parent)
{
  PropertiesModel* parameter_model = new PropertiesModel(parent);
  
  parameter_model->addProperty ("Leaf Size x", 0.01,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Leaf Size y", 0.01,  Qt::ItemIsEditable | Qt::ItemIsEnabled);
  parameter_model->addProperty ("Leaf Size z", 0.01,  Qt::ItemIsEditable | Qt::ItemIsEnabled);

  
  return parameter_model;
}

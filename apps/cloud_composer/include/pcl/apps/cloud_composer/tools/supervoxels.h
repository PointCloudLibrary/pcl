 /*
  * Software License Agreement  (BSD License)
  *
  *  Point Cloud Library  (PCL) - www.pointclouds.org
  *  Copyright  (c) 2012, Jeremie Papon.
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
  *   * Neither the name of Willow Garage, Inc. nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *
  */
 
#pragma once
 
 #include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
 #include <pcl/apps/cloud_composer/tool_interface/tool_factory.h>
 
 
 namespace pcl
 {
   namespace cloud_composer
   {
     class SupervoxelsTool : public SplitItemTool
     {
       Q_OBJECT
     public:
       SupervoxelsTool (PropertiesModel* parameter_model, QObject* parent);
       
       QList <CloudComposerItem*>
       performAction (QList <const CloudComposerItem*> input_data, PointTypeFlags::PointType type = PointTypeFlags::NONE) override;
       
       template <typename PointT> QList <CloudComposerItem*>
       performTemplatedAction (const QList <const CloudComposerItem*>& input_data);
       
       inline QString
       getToolName () const override { return "Voxel Superpixels Tool";}
     
       
    };
     
     
     class SupervoxelsToolFactory : public QObject, public ToolFactory
     {
       Q_OBJECT
       Q_INTERFACES (pcl::cloud_composer::ToolFactory)
       Q_PLUGIN_METADATA(IID "cloud_composer.ToolFactory/1.0")
     public:
       SplitItemTool*
       createTool (PropertiesModel* parameter_model, QObject* parent = nullptr) override 
       {
         return new SupervoxelsTool(parameter_model, parent);
       }
       
       PropertiesModel*
       createToolParameterModel (QObject* parent) override;
       
       inline QString 
       getPluginName () const override { return "Supervoxels";}
       
       inline QString 
       getToolGroupName () const override { return "Segmentation";}
       
       inline QString
       getIconName () const override { return ":/supervoxels.png"; }
       
       inline CloudComposerItem::ItemType
       getInputItemType () const override
       {
         return CloudComposerItem::CLOUD_ITEM;
       }
       
       inline QList <CloudComposerItem::ItemType>
       getRequiredInputChildrenTypes () const override 
       {
         QList <CloudComposerItem::ItemType> input_types;
         return input_types;
       }
     };
     
     
     
   }
 }

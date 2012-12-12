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

#ifndef EUCLIDEAN_CLUSTERING_H_
#define EUCLIDEAN_CLUSTERING_H_

#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>
#include <pcl/apps/cloud_composer/tool_interface/tool_factory.h>


namespace pcl
{
  namespace cloud_composer
  {
    class EuclideanClusteringTool : public SplitItemTool
    {
      Q_OBJECT
      public:
        EuclideanClusteringTool (PropertiesModel* parameter_model, QObject* parent);
        virtual ~EuclideanClusteringTool ();
        
        virtual QList <CloudComposerItem*>
        performAction (QList <const CloudComposerItem*> input_data, PointTypeFlags::PointType type = PointTypeFlags::NONE);
            
        inline virtual QString
        getToolName () const { return "Euclidean Clustering Tool";}
    };

    
    class EuclideanClusteringToolFactory : public QObject, public ToolFactory
    {
      Q_OBJECT
      Q_INTERFACES (pcl::cloud_composer::ToolFactory)
      public:
        SplitItemTool*
        createTool (PropertiesModel* parameter_model, QObject* parent = 0) 
        {
            return new EuclideanClusteringTool(parameter_model, parent);
        }
        
        PropertiesModel*
        createToolParameterModel (QObject* parent);
        
        inline virtual QString 
        getPluginName () const { return "Euclidean Clustering";}
        
        inline virtual QString 
        getToolGroupName () const { return "Segmentation";}
        
        inline virtual QString
        getIconName () const { return ":/euclidean_clustering.png"; }
        
        inline virtual CloudComposerItem::ItemType
        getInputItemType () const
        {
          return CloudComposerItem::CLOUD_ITEM;
        }
        
        inline virtual QList <CloudComposerItem::ItemType>
        getRequiredInputChildrenTypes () const 
        {
          return QList <CloudComposerItem::ItemType> ();
        }
    };



  }
}






#endif //EUCLIDEAN_CLUSTERING_H_
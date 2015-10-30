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

#ifndef TOOL_FACTORY_H_
#define TOOL_FACTORY_H_

#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>

class QAction;

namespace pcl
{
  namespace cloud_composer
  {
    
    
    class AbstractTool;
    class AbstractCommand;
    class PropertiesModel;
    
    class ToolFactory
    {
      public:
        virtual AbstractTool*
        createTool (PropertiesModel* parameter_model = 0, QObject* parent = 0) = 0;
            
        virtual PropertiesModel*
        createToolParameterModel (QObject* parent) = 0;
        
        virtual QString
        getPluginName () const = 0;
        
        virtual QString
        getToolGroupName () const = 0; 
        
        virtual QString 
        getIconName () const = 0;
        
        /** \brief Reimpliment this function to return the proper number if tool requires more than one input item */
        inline virtual int
        getNumInputItems () const 
        { 
          return 1;
        }
        
        /** \brief Returns a list of allowed input item types. Implement in tools so GUI can prevent impossible actions */
        virtual CloudComposerItem::ItemType
        getInputItemType () const = 0;
        
        /** \brief Returns a list of required input children. Implement in tools so GUI can prevent impossible actions */
        virtual QList <CloudComposerItem::ItemType>
        getRequiredInputChildrenTypes () const = 0;

    };

  }
}

Q_DECLARE_METATYPE (pcl::cloud_composer::ToolFactory*);

Q_DECLARE_INTERFACE(pcl::cloud_composer::ToolFactory,
                    "cloud_composer.ToolFactory/1.0")

#endif //TOOL_FACTORY_H_

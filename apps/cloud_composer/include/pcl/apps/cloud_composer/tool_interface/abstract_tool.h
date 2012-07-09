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

#ifndef ABSTRACT_TOOL_H_
#define ABSTRACT_TOOL_H_

#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

namespace pcl
{
  namespace cloud_composer
  {
    class AbstractTool : public QObject
    {
      Q_OBJECT
      public:
        typedef boost::shared_ptr<pcl::cloud_composer::AbstractTool> Ptr;
        typedef boost::shared_ptr<pcl::cloud_composer::AbstractTool> ConstPtr;
        
        AbstractTool (QStandardItemModel* parameter_model = 0, QObject* parent = 0) 
                      : QObject (parent) 
                      , parameter_model_ (parameter_model)
                      {}
        virtual ~AbstractTool () { qDebug() << "Tool Destructed"; }
        
        /**  \brief Function called which does work in plugin 
         *  \param data input_data from the model - const for good reason
         *  Returned list will become the output, replacing input_data in the model - you must deep copy
         *  the input_data, since undo works by switching back and forth
         */ 
        virtual QList <CloudComposerItem*>
        performAction (QList <const CloudComposerItem*> input_data) = 0;
        
        virtual CloudCommand*
        createCommand (QList <const CloudComposerItem*> input_data) = 0;
        
        QString 
        getActionText () const {return action_text_;}
        
        void
        setActionText (const QString text) { action_text_ = text; }
              
        virtual QString
        getToolName () const = 0;
        
     protected:
        QStandardItemModel* parameter_model_;   
       
      private:
        QString action_text_;
        
    };
    
    class ModifyTool : public AbstractTool
    {
      Q_OBJECT
      public:
        ModifyTool (QStandardItemModel* parameter_model = 0, QObject* parent = 0) 
                      : AbstractTool (parameter_model, parent) 
                      {}
        
        virtual ~ModifyTool () { }
        
        virtual QList <CloudComposerItem*>
        performAction (QList <const CloudComposerItem*> input_data) = 0;
        
        inline virtual CloudCommand* 
        createCommand (QList <const CloudComposerItem*> input_data) 
        {
          return new ModifyCloudCommand (input_data);
        }
        
        inline virtual QString
        getToolName () const { return "ModifyTool";}
        
    };
    
    class NewItemTool : public AbstractTool
    {
      Q_OBJECT
      public:
        NewItemTool (QStandardItemModel* parameter_model = 0, QObject* parent = 0) 
                      : AbstractTool (parameter_model, parent)
                      {}
        
        virtual ~NewItemTool () { }
        
        virtual QList <CloudComposerItem*>
        performAction (QList <const CloudComposerItem*> input_data) = 0;
        
        inline virtual CloudCommand*
        createCommand (QList <const CloudComposerItem*> input_data) 
        {
          return new NewItemCloudCommand (input_data);
        }
        
        inline virtual QString
        getToolName () const { return "NewItemTool";}
      
    };
    
    
  }
}


#endif //ABSTRACT_TOOL_H_

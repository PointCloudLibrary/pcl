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

#ifndef TOOLBOX_MODEL_H_
#define TOOLBOX_MODEL_H_

#include <pcl/apps/cloud_composer/qt.h>

enum TOOLBOX_ROLES 
{ 
  FACTORY = Qt::UserRole,
  PARAMETER_MODEL, 
  ALLOWED_INPUT_ITEM_TYPES,
  REQUIRED_INPUT_CHILDREN_TYPES
};

class QTreeView;
namespace pcl
{
  namespace cloud_composer
  {
    
    class CloudCommand;
    class AbstractTool;
    class ToolFactory;
    class ProjectModel;
    
    class ToolBoxModel : public QStandardItemModel
    {
      Q_OBJECT
      
    public:
      ToolBoxModel (QTreeView* tool_view = 0, QTreeView* parameter_view = 0, QObject *parent = 0);
      ToolBoxModel (const ToolBoxModel& to_copy);
      virtual ~ToolBoxModel ();
      
      void
      addTool (ToolFactory* tool_factory);
      
      void
      setSelectionModel (QItemSelectionModel* selection_model);
      
      /** \brief Enables/Disables Tools based on currently selected items from model */
      void
      updateEnabledTools (const QItemSelection current_selection);
      
      void
      enableAllTools ();
      
    public Q_SLOTS:
      void
      activeProjectChanged (ProjectModel* new_model, ProjectModel* previous_model);
      
      void 
      selectedToolChanged (const QModelIndex & current, const QModelIndex & previous);
      
      void
      toolAction ();
      
      /** \brief This slot is called when the selection in cloud browser changes. Updates enabling of tools */
      void 
      selectedItemChanged ( const QItemSelection & selected, const QItemSelection & deselected );
      
      /** \brief This slot is called whenever the current project model emits layoutChanged, and calls updateEnabledTools */
      void
      modelChanged ();
    Q_SIGNALS:
      void
      enqueueToolAction (AbstractTool* tool);
      
    private:
      QStandardItem* 
      addToolGroup (QString tool_group_name);
      
      QTreeView* tool_view_;
      QTreeView* parameter_view_;
      QItemSelectionModel* selection_model_;
      QSet <QStandardItem*> tool_items;
      
      ProjectModel* project_model_;
      
    };
  }
}

Q_DECLARE_METATYPE (pcl::cloud_composer::ToolBoxModel);
Q_DECLARE_METATYPE (QStandardItemModel*);

#endif //TOOLBOX_MODEL_H_


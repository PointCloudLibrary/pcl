/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Jeremie Papon.
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ITEM_INSPECTOR_H_
#define ITEM_INSPECTOR_H_

#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/properties_model.h>
class QItemSelectionModel;

namespace pcl
{
  namespace cloud_composer
  {
    /** \brief View class for displaying properties of an item
     * \author Jeremie Papon
     * \ingroup cloud_composer
     */
    class ItemInspector : public QTabWidget
    {
      Q_OBJECT
      public:
        ItemInspector (QWidget* parent = 0);
        virtual ~ItemInspector();
      
      public Q_SLOTS:
        void 
        setModel (ProjectModel* new_model);
        void 
        selectionChanged (const QModelIndex &current, const QModelIndex &previous);
        void 
        itemChanged (QStandardItem* item);
        
        
        
      private:
        void 
        createItemWidgets ();
        /** \brief Stores the state of the current tree view in item_treestate_map_  */
        void 
        storeTreeState ();
        /** \brief Retores the state of \param model 's view from item_treestate_map_  */
        void
        restoreTreeState ();
        /** \brief Removes the extra tabs the item might have */
        void
        removeTabs ();
        /** \brief Refreshes the data shown in the current displayed view widget */
        void
        updateView ();
        
        //! The tree object used to display/edit parameters 
        QTreeView* parameter_view_;
        
        
        ProjectModel* current_project_model_;
        PropertiesModel* current_item_properties_model_;
        const QItemSelectionModel *current_selection_model_;
        QMap <QString, QWidget*> itemtype_widget_map;
        QMap <QStandardItemModel*, QList <QPersistentModelIndex> > item_treestate_map_;
    };
    
    
  }
}

















#endif

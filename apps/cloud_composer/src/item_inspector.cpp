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


#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/item_inspector.h>
#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>


pcl::cloud_composer::ItemInspector::ItemInspector (QWidget* parent)
  : QTabWidget(parent)
{
  current_item_properties_model_ = 0;
  current_project_model_ = 0;
  current_selection_model_ = 0;
  
  parameter_view_ = new QTreeView ();
  addTab (parameter_view_, "Parameters");

  
}

pcl::cloud_composer::ItemInspector::~ItemInspector ()
{
  
}

void
pcl::cloud_composer::ItemInspector::setModel (ProjectModel* new_model)
{
  // DISABLED: JUST KEEP TREES ALWAYS EXPANDED
   //If we have a model loaded, save its tree state 
  //if (current_item_properties_model_)
  //  storeTreeState ();
  QItemSelectionModel* new_selection_model = new_model->getSelectionModel ();
  current_project_model_ = new_model;
  
  if (current_selection_model_)
  {
    disconnect (current_selection_model_, SIGNAL (currentChanged (const QModelIndex, const QModelIndex)),
                this, SLOT (selectionChanged (const QModelIndex, const QModelIndex)));
    removeTabs ();
    
  }
  current_selection_model_ = new_selection_model;
  connect (current_selection_model_, SIGNAL (currentChanged (const QModelIndex, const QModelIndex)),
           this, SLOT (selectionChanged (const QModelIndex,const QModelIndex)));
  
  updateView ();
  
}

void
pcl::cloud_composer::ItemInspector::selectionChanged (const QModelIndex &, const QModelIndex &)
{
  //If we have a model loaded, save its tree state 
 // if (current_item_properties_model_)
 //   storeTreeState ();
  if (current_project_model_)
    updateView ();
  
}

void
pcl::cloud_composer::ItemInspector::storeTreeState ()
{
  QList <QPersistentModelIndex> expanded_list;
  int row_count = current_item_properties_model_->rowCount ();
  for (int i = 0; i < row_count ; ++i)
  {
    QModelIndex index = current_item_properties_model_->index (i, 0);
    if (parameter_view_->isExpanded (index))
    {
      expanded_list <<  QPersistentModelIndex (index);
    }
  }
  // save list
  item_treestate_map_.insert (current_item_properties_model_, expanded_list);
}

void
pcl::cloud_composer::ItemInspector::restoreTreeState ()
{
  if (item_treestate_map_.contains (current_item_properties_model_))
  {
    parameter_view_->setUpdatesEnabled (false);
    
    foreach (QPersistentModelIndex item_index, item_treestate_map_.value (current_item_properties_model_)) 
    {
      if (item_index.isValid ())
       parameter_view_->setExpanded (item_index, true);
    }
    parameter_view_->setUpdatesEnabled (true);
  }
  
}

void 
pcl::cloud_composer::ItemInspector::itemChanged (QStandardItem *)
{

  
}

void
pcl::cloud_composer::ItemInspector::removeTabs ()
{
  clear ();
  addTab (parameter_view_, "Parameters");
}

void
pcl::cloud_composer::ItemInspector::updateView ()
{

  current_item_properties_model_ = 0;
  QModelIndex current_item = current_selection_model_->currentIndex ();
  const QStandardItemModel* model = 0;
  CloudComposerItem* cloud_item = 0;
  if (current_item.isValid ())
    model = dynamic_cast<const QStandardItemModel*> (current_item.model ());
        
  if (model)
  {
    cloud_item = dynamic_cast<CloudComposerItem*> (model->itemFromIndex (current_item));
    if (cloud_item)
    {
      current_item_properties_model_ = cloud_item->data (ItemDataRole::PROPERTIES).value <PropertiesModel*> ();
      //Display any additional graphical data if this item has any
      QMap <QString, QWidget*> tabs = cloud_item->getInspectorTabs ();
      foreach (QString tab_name, tabs.keys ())
      {
        addTab (tabs.value (tab_name), tab_name);
        tabs.value (tab_name)->show ();
      }
    }
  }
      
  
  parameter_view_->setModel (current_item_properties_model_);
  parameter_view_->resizeColumnToContents (0);
  // restoreTreeState ();
  parameter_view_->expandAll ();
}

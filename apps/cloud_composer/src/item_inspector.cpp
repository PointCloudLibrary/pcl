#include <pcl/apps/cloud_composer/item_inspector.h>
#include <pcl/apps/cloud_composer/cloud_item.h>

#include <QItemSelectionModel>


pcl::cloud_composer::ItemInspector::ItemInspector (QWidget* parent)
  : QTreeView(parent)
{
  current_project_model_ = 0;
  current_selection_model_ = 0;
}

pcl::cloud_composer::ItemInspector::~ItemInspector ()
{
  
}

void
pcl::cloud_composer::ItemInspector::setProjectAndSelectionModels (ProjectModel* new_model, const QItemSelectionModel* new_selection_model)
{
    //If we have a model loaded, save its tree state 
  if (current_item_model_)
    storeTreeState ();
  
  current_project_model_ = new_model;
  
  if (current_selection_model_)
  {
    disconnect (current_selection_model_, SIGNAL (currentChanged (const QModelIndex, const QModelIndex)),
                this, SLOT (selectionChanged (const QModelIndex, const QModelIndex)));
                
  }
  current_selection_model_ = new_selection_model;
  connect (current_selection_model_, SIGNAL (currentChanged (const QModelIndex, const QModelIndex)),
           this, SLOT (selectionChanged (const QModelIndex,const QModelIndex)));
  
  QModelIndex current_item = current_selection_model_->currentIndex ();
  current_item_model_ = VPtr<QStandardItemModel>::asPtr (current_item.data (PROPERTIES));
  if (current_item_model_)
  {
    this->setModel (current_item_model_);
    restoreTreeState ();
  }
  else
    this->setModel (0);
  
}

void
pcl::cloud_composer::ItemInspector::selectionChanged (const QModelIndex &current, const QModelIndex &previous)
{
  //If we have a model loaded, save its tree state 
  if (current_item_model_)
    storeTreeState ();
  if (current_project_model_)
  {
    QStandardItem* selected_item = current_project_model_->itemFromIndex (current);
    current_item_model_ = VPtr<QStandardItemModel>::asPtr (selected_item->data (PROPERTIES));
    if (current_item_model_)
    {
      this->setModel (current_item_model_);
      restoreTreeState ();
    }
  }
  
}

void
pcl::cloud_composer::ItemInspector::storeTreeState ()
{
  QList <QStandardItem*> expanded_list;
  int row_count = current_item_model_->rowCount ();
  for (int i = 0; i < row_count ; ++i)
  {
    QModelIndex index = current_item_model_->index (i, 0);
    if (this->isExpanded (index))
    {
      expanded_list << current_item_model_->itemFromIndex (index);
    }
  }
  // save list
  item_treestate_map_.insert (current_item_model_, expanded_list);
}

void
pcl::cloud_composer::ItemInspector::restoreTreeState ()
{
  if (item_treestate_map_.contains (current_item_model_))
  {
    this->setUpdatesEnabled (false);
    QList <QStandardItem*> expanded_list = item_treestate_map_.value (current_item_model_);
    foreach (QStandardItem* item, expanded_list) 
    {
      QModelIndex item_index = current_item_model_->indexFromItem (item);
      if (item_index.isValid ())
       this->setExpanded (item_index, true);
    }
    this->setUpdatesEnabled (true);
  }
  
}

void 
pcl::cloud_composer::ItemInspector::itemChanged (QStandardItem *item)
{

  
}

void
pcl::cloud_composer::ItemInspector::updateView ()
{
  
}
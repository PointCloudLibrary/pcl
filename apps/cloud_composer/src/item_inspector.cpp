#include <pcl/apps/cloud_composer/item_inspector.h>

#include <QItemSelectionModel>


pcl::cloud_composer::ItemInspector::ItemInspector (QWidget* parent)
  : QTableView(parent)
{
  current_model_ = 0;
  current_selection_model_ = 0;
}

pcl::cloud_composer::ItemInspector::~ItemInspector ()
{
  
}
/*
void
pcl::cloud_composer::ItemInspector::setModel (ProjectModel* new_model)
{
  if (current_model_)
  {
    disconnect (current_model_, SIGNAL (itemChanged(QStandardItem*)),
    this, SLOT (itemChanged (QStandardItem*)));
  }
  current_model_ = new_model;
  connect (current_model_, SIGNAL (itemChanged(QStandardItem*)),
           this, SLOT (itemChanged (QStandardItem*)));
}

void
pcl::cloud_composer::ItemInspector::setSelectionModel (const QItemSelectionModel* new_selection_model)
{
  if (current_selection_model_)
  {
    disconnect (current_selection_model_, SIGNAL (currentChanged (const QModelIndex, const QModelIndex)),
                this, SLOT (selectionChanged (const QModelIndex, const QModelIndex)));
                
  }
  current_selection_model_ = new_selection_model;
  connect (current_selection_model_, SIGNAL (currentChanged (const QModelIndex, const QModelIndex)),
           this, SLOT (selectionChanged (const QModelIndex,const QModelIndex)));
  
}

void
pcl::cloud_composer::ItemInspector::selectionChanged (const QModelIndex &current, const QModelIndex &previous)
{
  
  
}

void 
pcl::cloud_composer::ItemInspector::itemChanged (QStandardItem *item)
{
  QStandardItem* current_item = current_model_->itemFromIndex (current_selection_model_->currentIndex ());
  if (current_item == item)
    updateView ();
  
}
*/
void
pcl::cloud_composer::ItemInspector::updateView ()
{
  
}
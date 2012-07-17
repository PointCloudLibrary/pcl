#include <pcl/apps/cloud_composer/items/fpfh_item.h>
#include <QDebug>


pcl::cloud_composer::FPFHItem::FPFHItem (QString name, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_ptr, double radius)
  : CloudComposerItem (name)
  , fpfh_ptr_ (fpfh_ptr)
  , radius_ (radius)

{
  
  this->setData (QVariant::fromValue (fpfh_ptr_), FPFH_CLOUD);
  
  addProperty ("Radius", QVariant (radius_), Qt::ItemIsEnabled);
  
}

pcl::cloud_composer::FPFHItem*
pcl::cloud_composer::FPFHItem::clone () const
{
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_copy (new pcl::PointCloud<pcl::FPFHSignature33> (*fpfh_ptr_));
  FPFHItem* new_item = new FPFHItem (this->text (), fpfh_copy, radius_);
  QStandardItemModel* new_item_properties = new_item->getProperties ();
  
  for (int i=0; i < properties_->rowCount (); ++i){
    QList <QStandardItem*> new_row;
    QStandardItem* parent = properties_->item(i,0);
    QModelIndex parent_index = properties_->index(i,0);
    new_row.append (parent->clone ());
    for (int j=0; j < properties_->columnCount (parent_index); ++j)
    {
      if (properties_->item (i,j))      
        new_row.append (properties_->item(i,j)->clone ());
    }
    new_item_properties->appendRow (new_row);
  }
  new_item->setProperties (new_item_properties);
  
  return new_item;  
}

pcl::cloud_composer::FPFHItem::~FPFHItem ()
{
  
}


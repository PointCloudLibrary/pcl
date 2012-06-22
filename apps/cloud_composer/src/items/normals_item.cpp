#include <pcl/apps/cloud_composer/items/normals_item.h>
#include <QDebug>


pcl::cloud_composer::NormalsItem::NormalsItem (QString name, pcl::PointCloud<pcl::Normal>::Ptr normals_ptr, double radius)
  : CloudComposerItem (name)
  , normals_ptr_ (normals_ptr)
  , radius_ (radius)

{
  
  this->setData (QVariant::fromValue (normals_ptr_), NORMALS_CLOUD);
  
  addProperty ("Radius", QVariant (radius_), Qt::ItemIsEnabled);

  
  
}

pcl::cloud_composer::NormalsItem*
pcl::cloud_composer::NormalsItem::clone () const
{
  pcl::PointCloud<pcl::Normal>::Ptr normals_copy (new pcl::PointCloud<pcl::Normal> (*normals_ptr_));
  //Vector4f and Quaternionf do deep copies using copy constructor
  NormalsItem* new_item = new NormalsItem (this->text (), normals_copy, radius_);
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

pcl::cloud_composer::NormalsItem::~NormalsItem ()
{
  
}


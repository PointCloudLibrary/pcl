#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <QDebug>


pcl::cloud_composer::CloudItem::CloudItem (QString name,
                                           sensor_msgs::PointCloud2::Ptr cloud_ptr, 
                                           Eigen::Vector4f origin, 
                                           Eigen::Quaternionf orientation)
  : CloudComposerItem (name)
  , cloud_ptr_ (cloud_ptr)
  , origin_ (origin)
  , orientation_ (orientation)
{
  
  this->setData (QVariant::fromValue (cloud_ptr_), CLOUD);
  this->setData (QVariant::fromValue (origin_), ORIGIN);
  this->setData (QVariant::fromValue (orientation_), ORIENTATION);
  
  //Create a color and geometry handler for this cloud
  color_handler_.reset (new pcl::visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_ptr));
  this->setData (QVariant::fromValue (color_handler_), COLOR);
  geometry_handler_.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2> (cloud_ptr));
  this->setData (QVariant::fromValue (geometry_handler_), GEOMETRY);
  
  
  QStandardItem* core = new QStandardItem ("Core Properties");
  properties_->appendRow (core);
  addProperty ("Name", QVariant (this->text ()), Qt::NoItemFlags, core);
  addProperty ("Height", QVariant (cloud_ptr_->height), Qt::NoItemFlags, core);
  addProperty ("Width", QVariant (cloud_ptr_->width), Qt::NoItemFlags,core);
  
  
}

pcl::cloud_composer::CloudItem*
pcl::cloud_composer::CloudItem::clone () const
{
  sensor_msgs::PointCloud2::Ptr cloud_copy (new sensor_msgs::PointCloud2 (*cloud_ptr_));
  //Vector4f and Quaternionf do deep copies using copy constructor
  CloudItem* new_item = new CloudItem (this->text (), cloud_copy, origin_,orientation_);
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

pcl::cloud_composer::CloudItem::~CloudItem ()
{
  
}


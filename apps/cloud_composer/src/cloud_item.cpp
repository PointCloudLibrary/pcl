#include <pcl/apps/cloud_composer/cloud_item.h>



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

pcl::cloud_composer::CloudItem::~CloudItem ()
{
  
}


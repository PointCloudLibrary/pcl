#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>

pcl::cloud_composer::CloudItem::CloudItem (QString name,
                                           sensor_msgs::PointCloud2::Ptr cloud_ptr, 
                                           const Eigen::Vector4f& origin, 
                                           const Eigen::Quaternionf& orientation)
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
  properties_->addProperty ("Name", QVariant (this->text ()), Qt::NoItemFlags, core);
  properties_->addProperty ("Height", QVariant (cloud_ptr_->height), Qt::NoItemFlags, core);
  properties_->addProperty ("Width", QVariant (cloud_ptr_->width), Qt::NoItemFlags,core);

}

pcl::cloud_composer::CloudItem*
pcl::cloud_composer::CloudItem::clone () const
{
  sensor_msgs::PointCloud2::Ptr cloud_copy (new sensor_msgs::PointCloud2 (*cloud_ptr_));
  //Vector4f and Quaternionf do deep copies using copy constructor
  CloudItem* new_item = new CloudItem (this->text (), cloud_copy, origin_,orientation_);
  
  PropertiesModel* new_item_properties = new_item->getProperties ();
  new_item_properties->copyProperties (properties_);
  
  return new_item;  
}

pcl::cloud_composer::CloudItem::~CloudItem ()
{
  
}


void
pcl::cloud_composer::CloudItem::paintView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const
{
   vis->addPointCloud (cloud_ptr_, geometry_handler_, color_handler_, origin_, orientation_, item_id_.toStdString ());
}

void
pcl::cloud_composer::CloudItem::removeFromView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const
{  
  vis->removePointCloud (item_id_.toStdString ());
}
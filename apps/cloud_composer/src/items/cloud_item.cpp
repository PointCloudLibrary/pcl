#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/filters/passthrough.h>

pcl::cloud_composer::CloudItem::CloudItem (QString name,
                                           sensor_msgs::PointCloud2::Ptr cloud_ptr, 
                                           const Eigen::Vector4f& origin, 
                                           const Eigen::Quaternionf& orientation)
  : CloudComposerItem (name)
  , cloud_ptr_ (cloud_ptr)
  , origin_ (origin)
  , orientation_ (orientation)
{
  
  //Sanitize the cloud data using passthrough
 // qDebug () << "Cloud size before passthrough : "<<cloud_ptr->width<<"x"<<cloud_ptr->height;
  sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2);
  pcl::PassThrough<sensor_msgs::PointCloud2> pass_filter;
  pass_filter.setInputCloud (cloud_ptr);
  pass_filter.setKeepOrganized (false);
  pass_filter.filter (*cloud_filtered);
//  qDebug () << "Cloud size after passthrough : "<<cloud_filtered->width<<"x"<<cloud_filtered->height;
  cloud_ptr_ = cloud_filtered;
    
  this->setData (QVariant::fromValue (cloud_ptr_), CLOUD);
  this->setData (QVariant::fromValue (origin_), ORIGIN);
  this->setData (QVariant::fromValue (orientation_), ORIENTATION);
  
  
  //Create a color and geometry handler for this cloud
  color_handler_.reset (new pcl::visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud_ptr));
  this->setData (QVariant::fromValue (color_handler_), COLOR);
  geometry_handler_.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2> (cloud_ptr));
  this->setData (QVariant::fromValue (geometry_handler_), GEOMETRY);
  
  properties_->addCategory ("Core Properties");
  properties_->addProperty ("Name", QVariant (this->text ()), Qt::NoItemFlags, "Core Properties");
  properties_->addProperty ("Height", QVariant (cloud_ptr_->height), Qt::NoItemFlags, "Core Properties");
  properties_->addProperty ("Width", QVariant (cloud_ptr_->width), Qt::NoItemFlags,"Core Properties");
  properties_->addCategory ("Display Properties");
  properties_->addProperty ("Point Size", QVariant (1.0), Qt::ItemIsEditable | Qt::ItemIsEnabled, "Display Properties");
  properties_->addProperty ("Opacity", QVariant (1.0), Qt::ItemIsEditable | Qt::ItemIsEnabled, "Display Properties");
 
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
  vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, properties_->getProperty ("Point Size").toDouble (), item_id_.toStdString ());
  vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, properties_->getProperty ("Opacity").toDouble (), item_id_.toStdString ());
 
}

void
pcl::cloud_composer::CloudItem::removeFromView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const
{  
  vis->removePointCloud (item_id_.toStdString ());
}
#include <pcl/apps/cloud_composer/items/fpfh_item.h>
#include <QDebug>


pcl::cloud_composer::FPFHItem::FPFHItem (QString name, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_ptr, double radius)
  : CloudComposerItem (name)
  , fpfh_ptr_ (fpfh_ptr)
  , radius_ (radius)

{
  
  this->setData (QVariant::fromValue (fpfh_ptr_), FPFH_CLOUD);
  
  properties_->addProperty ("Radius", QVariant (radius_), Qt::ItemIsEnabled);
  
}

pcl::cloud_composer::FPFHItem*
pcl::cloud_composer::FPFHItem::clone () const
{
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_copy (new pcl::PointCloud<pcl::FPFHSignature33> (*fpfh_ptr_));
  FPFHItem* new_item = new FPFHItem (this->text (), fpfh_copy, radius_);
  
  PropertiesModel* new_item_properties = new_item->getProperties ();
  new_item_properties->copyProperties (properties_);
  
  return new_item;  
}

pcl::cloud_composer::FPFHItem::~FPFHItem ()
{
  
}


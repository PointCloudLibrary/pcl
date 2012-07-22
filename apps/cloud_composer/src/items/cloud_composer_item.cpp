#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>
//Needed for the helper function which gets a cloud ptr... this is a bad dependency
#include <pcl/apps/cloud_composer/items/cloud_item.h>



pcl::cloud_composer::CloudComposerItem::CloudComposerItem (QString name)
  : QStandardItem(name)
{
 //Set up the properties 
  properties_ = new PropertiesModel (this);

  //Set the pointer to a data role so item inspector can get it
  this->setData ( qVariantFromValue (static_cast<void*> (properties_)), PROPERTIES); 
  
  item_id_ = name + QString ("%1").arg ((long)this);  
  
}

pcl::cloud_composer::CloudComposerItem::~CloudComposerItem ()
{
  properties_->deleteLater ();
}





pcl::cloud_composer::CloudComposerItem*
pcl::cloud_composer::CloudComposerItem::clone () const
{
  CloudComposerItem* new_item = new CloudComposerItem (this->text ());
  
  PropertiesModel* new_item_properties = new_item->getProperties ();
  new_item_properties->copyProperties (properties_);
  
  return new_item;  
}

bool
pcl::cloud_composer::CloudComposerItem::getCloudConstPtr (sensor_msgs::PointCloud2::ConstPtr& const_ptr) const
{
  if (this->type () != CLOUD_ITEM)
  {
    qWarning () << "Attempted to get cloud from non-cloud item!";
    return false;
  }
  QVariant cloud_variant = this->data (CLOUD);
  const_ptr = cloud_variant.value <sensor_msgs::PointCloud2::Ptr> ();
  if (const_ptr)
    return true;
  else
  {
    qWarning () << "Fetched cloud, but pointer is NULL!!!";
    return false;
  }
}

void
pcl::cloud_composer::CloudComposerItem::paintView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const
{
  qDebug () << "Paint View in Cloud Composer Item - doing nothing";
}

void
pcl::cloud_composer::CloudComposerItem::removeFromView (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis) const
{
  qDebug () << "Remove from View in Cloud Composer Item - doing nothing";
}

QMap <QString, QWidget*>
pcl::cloud_composer::CloudComposerItem::getInspectorTabs ()
{
  return QMap <QString, QWidget*> ();
}
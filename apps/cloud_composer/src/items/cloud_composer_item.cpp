#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>




pcl::cloud_composer::CloudComposerItem::CloudComposerItem (QString name)
  : QStandardItem(name)
{
  //Set up the properties, store pointer locally for convenience
  properties_ = new PropertiesModel (this);
  
  QString item_id = name + QString ("%1").arg ((long)this);  
  
  
  this->setData (QVariant::fromValue (properties_), ItemDataRole::PROPERTIES); 
  this->setData (QVariant (item_id), ItemDataRole::ITEM_ID);
  
  this->setForeground (QBrush (Qt::black));
  
}


pcl::cloud_composer::CloudComposerItem::~CloudComposerItem ()
{
  properties_->deleteLater ();
  qDebug () << "Cloud Composer Item Destructor";
}

pcl::cloud_composer::CloudComposerItem*
pcl::cloud_composer::CloudComposerItem::clone () const
{
  CloudComposerItem* new_item = new CloudComposerItem (this->text ());
  
  PropertiesModel* new_item_properties = new_item->getPropertiesModel ();
  new_item_properties->copyProperties (properties_);
  
  return new_item;  
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::CloudComposerItem::getChildren (CloudComposerItem::ItemType type) const
{
  QList <CloudComposerItem*> items;
  for (int i = 0; i < this->rowCount (); ++i)
  {
    if ( this->child (i)->type () == type )
    {
        items.append (dynamic_cast <CloudComposerItem*> (this->child (i)));
    }
  }
  
  return items;
}

void 
pcl::cloud_composer::CloudComposerItem::addChild (CloudComposerItem *item_arg)
{
  this->appendRow (item_arg);
}

void
pcl::cloud_composer::CloudComposerItem::paintView (boost::shared_ptr<pcl::visualization::PCLVisualizer>) const
{
  qDebug () << "Paint View in Cloud Composer Item - doing nothing";
}

void
pcl::cloud_composer::CloudComposerItem::removeFromView (boost::shared_ptr<pcl::visualization::PCLVisualizer>) const
{
  qDebug () << "Remove from View in Cloud Composer Item - doing nothing";
}

QMap <QString, QWidget*>
pcl::cloud_composer::CloudComposerItem::getInspectorTabs ()
{
  return QMap <QString, QWidget*> ();
}

/*
template <typename CloudPtrT>
CloudPtrT
pcl::cloud_composer::CloudComposerItem::getCloudPtr () const
{
  QVariant cloud_variant = this->data (CLOUD);
  // Get Extract the pointer from the cloud contained in this item, if the type can't be converted, default-constructed value is returned
  CloudPtrT ptr;
  if (cloud_variant.canConvert <CloudPtrT> ())
    ptr =  cloud_variant.value <CloudPtrT> ();
  else
    qCritical () << "Requested Cloud of incorrect type from "<<this->text ()<<" correct type is "<<cloud_variant.typeName();
    
  return ptr;
}
*/



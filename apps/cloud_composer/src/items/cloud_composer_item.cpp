#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>
//Needed for the helper function which gets a cloud ptr... this is a bad dependency
#include <pcl/apps/cloud_composer/items/cloud_item.h>

pcl::cloud_composer::CloudComposerItem::CloudComposerItem (QString name)
  : QStandardItem(name)
{
 //Set up the properties 
  properties_ = new QStandardItemModel (0,2);
  properties_->setHorizontalHeaderItem (0, new QStandardItem ("Name"));
  properties_->setHorizontalHeaderItem (1, new QStandardItem ("Value"));
  //Set the pointer to a data role so item inspector can get it
  this->setData ( qVariantFromValue (static_cast<void*> (properties_)), PROPERTIES); 
  
}

pcl::cloud_composer::CloudComposerItem::~CloudComposerItem ()
{
  properties_->deleteLater ();
}

void
pcl::cloud_composer::CloudComposerItem::addProperty (const QString prop_name, QVariant value,  Qt::ItemFlags flags, QStandardItem* parent)
{
  QStandardItem* parent_item = parent;
  if (!parent_item)
    parent_item = properties_->invisibleRootItem ();
  QList <QStandardItem*> new_row;
  
  QStandardItem* new_property = new QStandardItem (prop_name);
  new_property->setFlags (Qt::ItemIsSelectable);
  new_row.append (new_property);
  
  QStandardItem* new_value = new QStandardItem ();
  new_value->setFlags (flags);
  new_value->setData (value, Qt::EditRole);
  new_row.append (new_value);
 
  parent_item->appendRow (new_row);
  
}

QVariant 
pcl::cloud_composer::CloudComposerItem::getProperty (const QString prop_name) const
{
  QList<QStandardItem*> items = properties_->findItems (prop_name);
  if (items.size () == 0)
  {
    qWarning () << "No property named "<<prop_name<<" found in "<<this->text ();
    return QVariant ();
  }
  else if (items.size () > 1)
  {
    qWarning () << "Multiple properties found with name "<<prop_name<<" in "<<this->text ();
  }
  
  return items.value (0)->data (Qt::EditRole);
}

pcl::cloud_composer::CloudComposerItem*
pcl::cloud_composer::CloudComposerItem::clone () const
{
  CloudComposerItem* new_item = new CloudComposerItem (this->text ());
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

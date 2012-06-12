#include <pcl/apps/cloud_composer/cloud_composer_item.h>


pcl::cloud_composer::CloudComposerItem::CloudComposerItem (QString name)
  : QStandardItem(name)
{
 //Set up the properties 
  properties_ = new QStandardItemModel (0,2);
  properties_->setHorizontalHeaderItem (0, new QStandardItem ("Name"));
  properties_->setHorizontalHeaderItem (1, new QStandardItem ("Value"));
  //Set the pointer to a data role so item inspector can get it
  this->setData ( qVariantFromValue ( (void*)properties_), PROPERTIES); 
}

pcl::cloud_composer::CloudComposerItem::~CloudComposerItem ()
{
 
}

void
pcl::cloud_composer::CloudComposerItem::addProperty (const QString prop_name, QVariant value, QStandardItem* parent)
{
  QStandardItem* parent_item = parent;
  if (!parent_item)
    parent_item = properties_->invisibleRootItem ();
  QList <QStandardItem*> new_row;
  QStandardItem* new_property = new QStandardItem (prop_name);
  new_row.append (new_property);
  QStandardItem* new_value = new QStandardItem ();
  new_value->setData (value, Qt::EditRole);
  new_row.append (new_value);
  parent_item->appendRow (new_row);
  
  
}
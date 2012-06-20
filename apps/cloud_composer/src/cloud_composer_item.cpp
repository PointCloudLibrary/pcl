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

pcl::cloud_composer::CloudComposerItem*
pcl::cloud_composer::CloudComposerItem::clone () const
{
  CloudComposerItem* new_item = new CloudComposerItem (this->text ());
  QStandardItemModel* new_item_properties = new_item->getProperties ();
  
  for (int i=0; i < properties_->rowCount (); ++i){
    QList <QStandardItem*> new_row;
    new_row.append (properties_->item(i,0)->clone ());
    new_row.append (properties_->item(i,1)->clone ());
    new_item_properties->appendRow (new_row);
  }
  new_item->setProperties (new_item_properties);
  
  return new_item;  
}
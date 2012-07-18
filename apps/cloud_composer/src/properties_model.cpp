#include <pcl/apps/cloud_composer/properties_model.h>
#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>

pcl::cloud_composer::PropertiesModel::PropertiesModel (CloudComposerItem* parent_item, QObject* parent)
  : QStandardItemModel (parent)
  , parent_item_ (parent_item)
{
  setHorizontalHeaderItem (0, new QStandardItem ("Name"));
  setHorizontalHeaderItem (1, new QStandardItem ("Value"));
  
  connect (this, SIGNAL (itemChanged (QStandardItem*)),
           this, SLOT (propertyChanged (QStandardItem*)));
  
}

pcl::cloud_composer::PropertiesModel::~PropertiesModel ()
{
  
}

void
pcl::cloud_composer::PropertiesModel::addProperty (const QString prop_name, QVariant value,  Qt::ItemFlags flags, QStandardItem* parent)
{
  QStandardItem* parent_item = parent;
  if (!parent_item)
    parent_item = invisibleRootItem ();
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
pcl::cloud_composer::PropertiesModel::getProperty (const QString prop_name) const
{
  QList<QStandardItem*> items = findItems (prop_name);
  if (items.size () == 0)
  {
    qWarning () << "No property named "<<prop_name<<" found in "<<parent_item_->text ();
    return QVariant ();
  }
  else if (items.size () > 1)
  {
    qWarning () << "Multiple properties found with name "<<prop_name<<" in "<<parent_item_->text ();
  }
  QStandardItem* property = items.value (0);
  int row = property->row ();
  return item (row,1)->data (Qt::EditRole);
}

void
pcl::cloud_composer::PropertiesModel::copyProperties (const PropertiesModel* to_copy)
{
  for (int i=0; i < to_copy->rowCount (); ++i){
    QList <QStandardItem*> new_row;
    QStandardItem* parent = to_copy->item(i,0);
    QModelIndex parent_index = to_copy->index(i,0);
    new_row.append (parent->clone ());
    for (int j=0; j < to_copy->columnCount (parent_index); ++j)
    {
      if (to_copy->item (i,j))      
        new_row.append (to_copy->item(i,j)->clone ());
    }
    appendRow (new_row);
  }
}


void
pcl::cloud_composer::PropertiesModel::propertyChanged (QStandardItem* property_item)
{
  parent_item_->propertyChanged ();
  
}
#include <pcl/apps/cloud_composer/properties_model.h>
#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>

#include <QDebug>

pcl::cloud_composer::PropertiesModel::PropertiesModel (QObject* parent)
  : QStandardItemModel (parent)
{
  setHorizontalHeaderItem (0, new QStandardItem ("Name"));
  setHorizontalHeaderItem (1, new QStandardItem ("Value"));  
}

pcl::cloud_composer::PropertiesModel::PropertiesModel (CloudComposerItem* parent_item, QObject* parent)
  : QStandardItemModel (parent)
  , parent_item_ (parent_item)
{
  setHorizontalHeaderItem (0, new QStandardItem ("Name"));
  setHorizontalHeaderItem (1, new QStandardItem ("Value"));
  
  connect (this, SIGNAL (itemChanged (QStandardItem*)),
           this, SLOT (propertyChanged (QStandardItem*)));  
}

pcl::cloud_composer::PropertiesModel::PropertiesModel (const PropertiesModel& to_copy)
: QStandardItemModel ()
{
  for (int i=0; i < to_copy.rowCount (); ++i){
    QList <QStandardItem*> new_row;
    QStandardItem* parent = to_copy.item(i,0);
    QModelIndex parent_index = to_copy.index(i,0);
    new_row.append (parent->clone ());
    for (int j=0; j < to_copy.columnCount (parent_index); ++j)
    {
      if (to_copy.item (i,j))      
        new_row.append (to_copy.item(i,j)->clone ());
    }
    appendRow (new_row);
  }
}

void
pcl::cloud_composer::PropertiesModel::addProperty (const QString& prop_name, const QVariant& value,  Qt::ItemFlags flags, const QString& category)
{
  QStandardItem* parent_item = invisibleRootItem ();
  if (category.size () > 0)
  {
    QList<QStandardItem*> items = findItems (category);
    if (items.empty ())
      qWarning () << "No category named "<<prop_name<<" found in "<<parent_item_->text ()<<" adding to root";
    else if (items.size () > 1)
      qCritical () << "Multiple categories with same name found!! This is not good...";
    else
      parent_item = items.at (0);
  }

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

void
pcl::cloud_composer::PropertiesModel::addCategory (const QString& category_name)
{
  QStandardItem* new_category = new QStandardItem (category_name);
  appendRow (new_category);
}

QVariant 
pcl::cloud_composer::PropertiesModel::getProperty (const QString& prop_name) const
{
  //qDebug () << "Searching for property " << prop_name;
  QList<QStandardItem*> items = findItems (prop_name, Qt::MatchExactly | Qt::MatchRecursive, 0);
  if (items.empty ())
  {
    qWarning () << "No property named "<<prop_name<<" found in "<<parent_item_->text ();
    return QVariant ();
  }
  if (items.size () > 1)
  {
    qWarning () << "Multiple properties found with name "<<prop_name<<" in "<<parent_item_->text ();
  }
 // qDebug () << "Found properties size ="<<items.size ();
  
  QStandardItem* property = items.value (0);
 // qDebug () << "Prop name="<<prop_name<<" row="<<property->row ()<<" col="<<property->column();
  int row = property->row ();
  QStandardItem* parent_item = property->parent ();
  if (parent_item == nullptr)
    parent_item = invisibleRootItem ();
  return parent_item->child (row,1)->data (Qt::EditRole);
}

void
pcl::cloud_composer::PropertiesModel::copyProperties (const PropertiesModel* to_copy)
{
  for (int i=0; i < to_copy->rowCount (); ++i){
    QList <QStandardItem*> new_row;
    QStandardItem* parent = to_copy->item(i,0);
    qDebug () << "Copying "<<parent->text()<< " cols ="<<to_copy->columnCount ();
    new_row.append (parent->clone ());
    for (int j=1; j < to_copy->columnCount (); ++j)
    {
      if (to_copy->item (i,j))      
      {
        new_row.append (to_copy->item(i,j)->clone ());
      }
    }
    appendRow (new_row);    
  }
}


void
pcl::cloud_composer::PropertiesModel::propertyChanged (QStandardItem*)
{
  //qDebug () << "Property Changed in properties model";
  parent_item_->propertyChanged ();  
}

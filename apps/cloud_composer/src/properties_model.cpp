/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <pcl/apps/cloud_composer/properties_model.h>
#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>


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

pcl::cloud_composer::PropertiesModel::~PropertiesModel ()
{
  
}

void
pcl::cloud_composer::PropertiesModel::addProperty (const QString prop_name, QVariant value,  Qt::ItemFlags flags, QString category)
{
  QStandardItem* parent_item = invisibleRootItem ();
  if (category.size () > 0)
  {
    QList<QStandardItem*> items = findItems (category);
    if (items.size () == 0)
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
pcl::cloud_composer::PropertiesModel::addCategory (const QString category_name)
{
  QStandardItem* new_category = new QStandardItem (category_name);
  appendRow (new_category);
}

QVariant 
pcl::cloud_composer::PropertiesModel::getProperty (const QString prop_name) const
{
  //qDebug () << "Searching for property " << prop_name;
  QList<QStandardItem*> items = findItems (prop_name, Qt::MatchExactly | Qt::MatchRecursive, 0);
  if (items.size () == 0)
  {
    qWarning () << "No property named "<<prop_name<<" found in "<<parent_item_->text ();
    return QVariant ();
  }
  else if (items.size () > 1)
  {
    qWarning () << "Multiple properties found with name "<<prop_name<<" in "<<parent_item_->text ();
  }
 // qDebug () << "Found properties size ="<<items.size ();
  
  QStandardItem* property = items.value (0);
 // qDebug () << "Prop name="<<prop_name<<" row="<<property->row ()<<" col="<<property->column();
  int row = property->row ();
  QStandardItem* parent_item = property->parent ();
  if (parent_item == 0)
    parent_item = invisibleRootItem ();
  return parent_item->child (row,1)->data (Qt::EditRole);
}

void
pcl::cloud_composer::PropertiesModel::copyProperties (const PropertiesModel* to_copy)
{
  for (int i=0; i < to_copy->rowCount (); ++i){
    QList <QStandardItem*> new_row;
    QStandardItem* parent = to_copy->item(i,0);
    QModelIndex parent_index = to_copy->index(i,0);
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


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

#include <pcl/apps/cloud_composer/items/cloud_composer_item.h>

#include <QDebug>

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
pcl::cloud_composer::CloudComposerItem::paintView (pcl::visualization::PCLVisualizer::Ptr) const
{
  qDebug () << "Paint View in Cloud Composer Item - doing nothing";
}

void
pcl::cloud_composer::CloudComposerItem::removeFromView (pcl::visualization::PCLVisualizer::Ptr) const
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



/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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


#include <pcl/apps/modeler/tree_model.h>
#include <pcl/apps/modeler/dock_widget.h>
#include <pcl/apps/modeler/render_widget.h>
#include <pcl/apps/modeler/points_item.h>
#include <pcl/apps/modeler/points_item.h>
#include <pcl/apps/modeler/normals_item.h>
#include <pcl/apps/modeler/surface_item.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/common/io.h>

#include <QStandardItem>


/////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeModel::TreeModel(QObject* parent) : 
  QStandardItemModel(parent)
{
  connect(this, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(slotOnDataChanged(QStandardItem*)));
  connect(this, SIGNAL(rowsAboutToBeInserted(QModelIndex, int, int)), this, SLOT(slotOnAboutToBeInserted(QModelIndex, int, int)));
  connect(this, SIGNAL(rowsAboutToBeRemoved(QModelIndex, int, int)), this, SLOT(slotOnAboutToBeRemoved(QModelIndex, int, int)));
  connect(this, SIGNAL(rowsInserted(QModelIndex, int, int)), this, SLOT(slotOnInserted(QModelIndex, int, int)));
  connect(this, SIGNAL(rowsRemoved(QModelIndex, int, int)), this, SLOT(slotOnRemoved(QModelIndex, int, int)));

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeModel::~TreeModel()
{
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::MainWindow*
pcl::modeler::TreeModel::parent()
{
  return (dynamic_cast<MainWindow*>(QStandardItemModel::parent()));
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeModel::emitItemChanged(QStandardItem* item)
{
  emit itemChanged(item);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeModel::slotOnDataChanged(QStandardItem* item)
{
  dynamic_cast<TreeItem*>(item)->updateOnDataChanged();
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeModel::slotOnAboutToBeInserted(const QModelIndex& parent, int start, int end)
{
  for (int i = start; i <= end; ++ i)
  {
    TreeItem* item = dynamic_cast<TreeItem*>(itemFromIndex(index(i, 0, parent)));
    if (item != NULL)
      item->updateOnAboutToBeInserted();
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeModel::slotOnAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
  for (int i = start; i <= end; ++ i)
  {
    TreeItem* item = dynamic_cast<TreeItem*>(itemFromIndex(index(i, 0, parent)));
    item->updateOnAboutToBeRemoved();
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeModel::slotOnInserted(const QModelIndex& parent, int start, int end)
{
  for (int i = start; i <= end; ++ i)
  {
    TreeItem* item = dynamic_cast<TreeItem*>(itemFromIndex(index(i, 0, parent)));
    item->updateOnInserted();
  }

  this->parent()->ui()->treeViewSceneExplorer->expand(parent);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeModel::slotOnRemoved(const QModelIndex& parent, int start, int end)
{
  for (int i = start; i <= end; ++ i)
  {
    TreeItem* item = dynamic_cast<TreeItem*>(itemFromIndex(index(i, 0, parent)));
    if (item != NULL)
      item->updateOnRemoved();
  }

  return;
}

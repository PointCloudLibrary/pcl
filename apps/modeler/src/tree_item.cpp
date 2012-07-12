/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <pcl/apps/modeler/tree_item.h>
#include <pcl/apps/modeler/main_window.h>
#include <QMenu>
#include <QPoint>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeItem::TreeItem(MainWindow* main_window) : 
  main_window_(main_window),
  QStandardItem()
{
  setEditable(false);
  old_state_ = clone();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeItem::TreeItem(MainWindow* main_window, const QString & text) : 
  main_window_(main_window),
  QStandardItem(text)
{
  setEditable(false);
  old_state_ = clone();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeItem::TreeItem(MainWindow* main_window, const QIcon & icon, const QString & text) : 
  main_window_(main_window),
  QStandardItem(icon, text)
{
  setEditable(false);
  old_state_ = clone();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeItem::~TreeItem()
{
  updateOnAboutToBeRemoved();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::TreeItem*
pcl::modeler::TreeItem::parent()
{
  return (dynamic_cast<TreeItem*>(QStandardItem::parent()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::updateOnDataChanged()
{
  handleDataChange();

  delete old_state_;
  old_state_ = clone();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::handleDataChange()
{
  if (checkState() == old_state_->checkState())
    return;

  updateParentCheckState();
  forceChildCheckState(checkState());

  return;
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::updateOnAboutToBeInserted()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::updateOnAboutToBeRemoved()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::updateOnInserted()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::updateOnRemoved()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::updateOnSelectionChange(bool selected)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::showContextMenu(const QPoint* position)
{
  QMenu menu(main_window_);
  prepareContextMenu(&menu);
  menu.exec(*position);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::prepareContextMenu(QMenu* menu) const
{

}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::forceChildCheckState(Qt::CheckState check_state)
{
  for (int i = 0, i_end = rowCount(); i < i_end; ++ i)
      child(i)->setCheckState(check_state);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::TreeItem::updateParentCheckState()
{
  QStandardItem* parent = this->parent();
  if(parent == NULL || !parent->isCheckable())
    return;

  // Do not update parent if the check state is consistent
  if (checkState() == Qt::Checked 
    && (parent->checkState() == Qt::Checked || parent->checkState() == Qt::PartiallyChecked))
      return;
  if (checkState() == Qt::Unchecked && parent->checkState() == Qt::Unchecked)
    return;

  bool checked_found = false;
  bool unchecked_found = false;
  for (int i = 0, i_end = parent->rowCount(); i < i_end; ++ i)
  {
    if (parent->child(i)->checkState() == Qt::Checked
      || parent->child(i)->checkState() == Qt::PartiallyChecked)
      checked_found = true;
    else
      unchecked_found = true;

    // set to partial checked state
    if (checked_found && unchecked_found)
    {
      parent->setCheckState(Qt::PartiallyChecked);
      return;
    }
  }

  // set to unchecked state
  if (checked_found == false)
      parent->setCheckState(Qt::Unchecked);

  // set to checked state
  if (unchecked_found == false)
      parent->setCheckState(Qt::Checked);

  return;
}

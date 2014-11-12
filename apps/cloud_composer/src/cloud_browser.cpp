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


#include <pcl/apps/cloud_composer/cloud_browser.h>
#include <pcl/apps/cloud_composer/project_model.h>


pcl::cloud_composer::CloudBrowser::CloudBrowser (QWidget* parent)
  : QTreeView (parent)
{
   this->setItemDelegate (new BackgroundDelegate (this));
}


void
pcl::cloud_composer::CloudBrowser::setModel (QAbstractItemModel* new_model)
{
  QTreeView::setModel (new_model);
  
  current_project_model_ = dynamic_cast <ProjectModel*>(new_model);
}



void
pcl::cloud_composer::BackgroundDelegate::paint (QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  // Fill the background before calling the base class paint
  // otherwise selected cells would have a white background
 // QVariant background = index.data (Qt::BackgroundRole);
 // if (background.canConvert<QBrush> ())
 //   painter->fillRect (option.rect, background.value<QBrush> ());

  QVariant text_color_variant = index.data (Qt::TextColorRole);
  if (text_color_variant.canConvert<QColor> ())
  {
    QColor text_color = text_color_variant.value<QColor> ();
    QPalette palette = option.palette;
    QStyleOptionViewItem option_copy = option;
    option_copy.palette.setColor (QPalette::HighlightedText, text_color);
    QStyledItemDelegate::paint (painter, option_copy, index);
  }
  else
    QStyledItemDelegate::paint (painter, option, index);
  
  
}
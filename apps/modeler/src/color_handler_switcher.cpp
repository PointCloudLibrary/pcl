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

#include <set>
#include <algorithm>

#include <pcl/apps/modeler/qt.h>

#include <ui_color_handler_switcher.h>
#include <pcl/apps/modeler/color_handler_switcher.h>
#include <pcl/apps/modeler/cloud_item.h>
#include <pcl/apps/modeler/geometry_item.h>
#include <pcl/apps/modeler/main_window.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ColorHandlerSwitcher::ColorHandlerSwitcher(QWidget * parent, Qt::WindowFlags f) :
  color_picker_(new QColorDialog(this)),
  ui_(new Ui::ColorHandlerSwitcher),
  QDialog(parent, f)

{
  ui_->setupUi(this);

  color_picker_->setWindowFlags(Qt::SubWindow);
  color_picker_->setOption(QColorDialog::NoButtons, true);
  ui_->verticalLayout->addWidget(color_picker_);

  connect(ui_->buttonBox->button(QDialogButtonBox::Apply), SIGNAL(clicked()),this, SLOT(slotSwitchColorHandler()));
  connect(ui_->buttonBox, SIGNAL(accepted()), this, SLOT(slotSwitchColorHandler()));

  connect(ui_->comboBoxColorHandlers, SIGNAL(currentIndexChanged(const QString &)),
    this, SLOT(slotToggleColorPicker(const QString &)));

  setupAvaiableFieldNames();
  ui_->comboBoxColorHandlers->setCurrentIndex(0);

  exec();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ColorHandlerSwitcher::~ColorHandlerSwitcher()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ColorHandlerSwitcher::setupAvaiableFieldNames()
{
  TreeView* tree_view = dynamic_cast<MainWindow*>(parent())->ui()->treeViewSceneExplorer;
  std::vector<GeometryItem*> geometry_items = tree_view->selectedItems<GeometryItem>();
  if (geometry_items.empty())
    return;

  std::vector<CloudItem*> cloud_items;
  for (size_t i = 0, i_end = geometry_items.size(); i < i_end; ++ i)
  {
    cloud_items.push_back(dynamic_cast<CloudItem*>(geometry_items[i]->parent()));
  }

  std::vector<std::string> intersection = cloud_items[0]->getAvaiableFieldNames();
  std::sort(intersection.begin(), intersection.end());

  for (size_t i = 1, i_end = cloud_items.size(); i < i_end; ++ i)
  {
    CloudItem* polymesh = cloud_items[i];
    std::vector<std::string> field_names = polymesh->getAvaiableFieldNames();
    std::sort(field_names.begin(), field_names.end());

    std::vector<std::string> temp;
    std::set_union(intersection.begin(), intersection.end(),
      field_names.begin(), field_names.end(), temp.begin());

    intersection = temp;
  }

  for (size_t i = 0, i_end = intersection.size(); i < i_end; ++ i)
    ui_->comboBoxColorHandlers->addItem(intersection[i].c_str());

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ColorHandlerSwitcher::slotSwitchColorHandler()
{
  TreeView* tree_view = dynamic_cast<MainWindow*>(parent())->ui()->treeViewSceneExplorer;
  std::vector<GeometryItem*> geometry_items = tree_view->selectedItems<GeometryItem>();
  if (geometry_items.empty())
    return;

  std::string color_handler = ui_->comboBoxColorHandlers->currentText().toStdString();
  if (color_handler == "custom")
  {
    QColor color = color_picker_->currentColor();
    for (size_t i = 0, i_end = geometry_items.size(); i < i_end; ++ i)
      geometry_items[i]->setColorHandler(color.red(), color.green(), color.blue());
  }
  else
  {
    for (size_t i = 0, i_end = geometry_items.size(); i < i_end; ++ i)
      geometry_items[i]->setColorHandler(color_handler);
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ColorHandlerSwitcher::slotToggleColorPicker(const QString &text)
{
  if (text.toStdString() == "custom")
    color_picker_->show();
  else
  {
    color_picker_->hide();
    adjustSize();
  }

  return;
}

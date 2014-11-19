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

#include <pcl/apps/modeler/parameter_dialog.h>
#include <pcl/apps/modeler/parameter.h>

#include <cassert>
#include <fstream>

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ParameterDialog::addParameter(pcl::modeler::Parameter* parameter)
{
  if (name_parameter_map_.find(parameter->getName()) == name_parameter_map_.end())
  {
    name_parameter_map_.insert(std::make_pair(parameter->getName(), parameter));
  }
  else
  {
    assert(false);
  }
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ParameterDialog::ParameterDialog(const std::string& title, QWidget* parent) : QDialog(parent), parameter_model_(NULL)
{
  setModal(false);
  setWindowTitle(QString(title.c_str())+" Parameters");
}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::modeler::ParameterDialog::exec()
{
  pcl::modeler::ParameterModel parameterModel (int (name_parameter_map_.size ()), 2, this);
  parameter_model_ = &parameterModel;

  QStringList headerLabels;
  headerLabels.push_back("Variable Name");
  headerLabels.push_back("Variable Value");
  parameterModel.setHorizontalHeaderLabels(headerLabels);

  QTableView tableView(this);
  tableView.setModel(&parameterModel);

  size_t currentRow = 0;
  for(std::map<std::string, Parameter*>::iterator it = name_parameter_map_.begin();
    it != name_parameter_map_.end();
    ++ it)
  {
    QModelIndex name = parameterModel.index(int (currentRow), 0, QModelIndex());
    parameterModel.setData(name, QVariant(it->first.c_str()));

    QModelIndex value = parameterModel.index(int (currentRow), 1, QModelIndex());
    std::pair<QVariant, int> model_data = it->second->toModelData();
    parameterModel.setData(value, model_data.first, model_data.second);

    currentRow ++;
  }

  ParameterDelegate parameterDelegate(name_parameter_map_);
  tableView.setItemDelegate(&parameterDelegate);

  tableView.horizontalHeader()->setStretchLastSection(true);
#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
  tableView.horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
#else
  tableView.horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
#endif
  tableView.setShowGrid(true);
  tableView.verticalHeader()->hide();
  tableView.setSelectionBehavior(QAbstractItemView::SelectRows);
  tableView.resizeColumnsToContents();

  int totlen = tableView.columnWidth(0) + tableView.columnWidth(1) + frameSize().width();
  setMinimumWidth(totlen);

  QPushButton* pushButtonReset = new QPushButton("Reset", this);
  QPushButton* pushButtonApply = new QPushButton("Apply", this);
  QPushButton* pushButtonCancel = new QPushButton("Cancel", this);

  connect(pushButtonReset, SIGNAL(clicked()), this, SLOT(reset()));
  connect(pushButtonApply, SIGNAL(clicked()), this, SLOT(accept()));
  connect(pushButtonCancel, SIGNAL(clicked()), this, SLOT(reject()));

  QGridLayout gridLayout(this);
  gridLayout.addWidget(&tableView, 0, 0, 1, 3);
  gridLayout.addWidget(pushButtonReset, 1, 0);
  gridLayout.addWidget(pushButtonApply, 1, 1);
  gridLayout.addWidget(pushButtonCancel, 1, 2);
  setLayout(&gridLayout);

  int result = QDialog::exec();

  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ParameterDialog::reset()
{
  size_t currentRow = 0;
  for (std::map<std::string, Parameter*>::iterator it = name_parameter_map_.begin();
    it != name_parameter_map_.end();
    ++ it)
  {
    it->second->reset();

    QModelIndex value = parameter_model_->index(int (currentRow), 1, QModelIndex());
    std::pair<QVariant, int> model_data = it->second->toModelData();
    parameter_model_->setData(value, model_data.first, model_data.second);

    currentRow ++;
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::Parameter* pcl::modeler::ParameterDelegate::getCurrentParameter(const QModelIndex &index) const
{
  std::map<std::string, Parameter*>::iterator currentParameter = parameter_map_.begin();

  size_t currentRow = 0;
  while(currentRow < index.row() && currentParameter != parameter_map_.end()) {
    ++ currentParameter;
    ++ currentRow;
  }

  assert(currentParameter != parameter_map_.end());

  return currentParameter->second;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::ParameterDelegate::ParameterDelegate(std::map<std::string, Parameter*>& parameterMap, QObject *parent):
  QStyledItemDelegate(parent),
  parameter_map_(parameterMap)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
QWidget *
pcl::modeler::ParameterDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &, const QModelIndex &index) const
{
  return getCurrentParameter(index)->createEditor(parent);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ParameterDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
  getCurrentParameter(index)->setEditorData(editor);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ParameterDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
  getCurrentParameter(index)->setModelData(editor, model, index);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ParameterDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &) const
{
  editor->setGeometry(option.rect);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ParameterDelegate::initStyleOption(QStyleOptionViewItem *option, const QModelIndex &index) const
{
  option->displayAlignment |= Qt::AlignHCenter;
  QStyledItemDelegate::initStyleOption(option, index);
}

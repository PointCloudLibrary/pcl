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

#include <pcl/apps/modeler/parameter.h>

#include <QAbstractItemModel>
#include <QCheckBox>
#include <QColorDialog>
#include <QSpinBox>

#include <cassert>
#include <fstream>
#include <iomanip>

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::Parameter::setModelData(QWidget* editor,
                                      QAbstractItemModel* model,
                                      const QModelIndex& index)
{
  getEditorData(editor);
  std::pair<QVariant, int> model_data = toModelData();
  model->setData(index, model_data.first, model_data.second);
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::modeler::IntParameter::valueTip()
{
  return QString("value range: [%1, %3]").arg(low_).arg(high_).toStdString();
}

//////////////////////////////////////////////////////////////////////////////////////////////
QWidget*
pcl::modeler::IntParameter::createEditor(QWidget* parent)
{
  QSpinBox* editor = new QSpinBox(parent);
  editor->setMinimum(low_);
  editor->setMaximum(high_);
  editor->setSingleStep(step_);

  return editor;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::IntParameter::setEditorData(QWidget* editor)
{
  QSpinBox* spinBox = static_cast<QSpinBox*>(editor);
  spinBox->setAlignment(Qt::AlignHCenter);

  int value = int(*this);
  spinBox->setValue(value);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::IntParameter::getEditorData(QWidget* editor)
{
  QSpinBox* spinBox = static_cast<QSpinBox*>(editor);
  int value = spinBox->text().toInt();
  current_value_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::pair<QVariant, int>
pcl::modeler::IntParameter::toModelData()
{
  std::pair<QVariant, int> model_data;
  model_data.first = int(*this);
  model_data.second = Qt::EditRole;
  return model_data;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::modeler::BoolParameter::valueTip()
{
  return QString("bool value").toStdString();
}

//////////////////////////////////////////////////////////////////////////////////////////////
QWidget*
pcl::modeler::BoolParameter::createEditor(QWidget* parent)
{
  QCheckBox* editor = new QCheckBox(parent);

  return editor;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::BoolParameter::setEditorData(QWidget* editor)
{
  QCheckBox* checkBox = static_cast<QCheckBox*>(editor);

  bool value = bool(*this);
  checkBox->setCheckState(value ? (Qt::Checked) : (Qt::Unchecked));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::BoolParameter::getEditorData(QWidget* editor)
{
  QCheckBox* checkBox = static_cast<QCheckBox*>(editor);
  bool value = (checkBox->checkState() == Qt::Checked);
  current_value_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::pair<QVariant, int>
pcl::modeler::BoolParameter::toModelData()
{
  std::pair<QVariant, int> model_data;
  model_data.first = bool(*this);
  model_data.second = Qt::EditRole;
  return model_data;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::modeler::DoubleParameter::valueTip()
{
  return QString("value range: [%1, %3]").arg(low_).arg(high_).toStdString();
}

//////////////////////////////////////////////////////////////////////////////////////////////
QWidget*
pcl::modeler::DoubleParameter::createEditor(QWidget* parent)
{
  QDoubleSpinBox* editor = new QDoubleSpinBox(parent);
  editor->setMinimum(low_);
  editor->setMaximum(high_);
  editor->setSingleStep(step_);
  editor->setDecimals(6);

  return editor;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::DoubleParameter::setEditorData(QWidget* editor)
{
  QDoubleSpinBox* spinBox = static_cast<QDoubleSpinBox*>(editor);
  spinBox->setAlignment(Qt::AlignHCenter);

  double value = double(*this);
  spinBox->setValue(value);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::DoubleParameter::getEditorData(QWidget* editor)
{
  QDoubleSpinBox* spinBox = static_cast<QDoubleSpinBox*>(editor);
  double value = spinBox->text().toDouble();
  current_value_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::pair<QVariant, int>
pcl::modeler::DoubleParameter::toModelData()
{
  std::pair<QVariant, int> model_data;
  model_data.first = double(*this);
  model_data.second = Qt::EditRole;
  return model_data;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::modeler::ColorParameter::valueTip()
{
  return "Color";
}

//////////////////////////////////////////////////////////////////////////////////////////////
QWidget*
pcl::modeler::ColorParameter::createEditor(QWidget* parent)
{
  QColorDialog* editor = new QColorDialog(parent);

  return editor;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ColorParameter::setEditorData(QWidget* editor)
{
  QColorDialog* color_dialog = static_cast<QColorDialog*>(editor);

  QColor value = QColor(*this);
  color_dialog->setCurrentColor(value);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::ColorParameter::getEditorData(QWidget* editor)
{
  QColorDialog* color_dialog = static_cast<QColorDialog*>(editor);

  QColor value = color_dialog->currentColor();
  current_value_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::pair<QVariant, int>
pcl::modeler::ColorParameter::toModelData()
{
  std::pair<QVariant, int> model_data;
  model_data.first = QBrush(QColor(*this));
  model_data.second = Qt::BackgroundRole;
  return model_data;
}

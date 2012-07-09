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

#include <pcl/apps/modeler/qt.h>
#include <pcl/apps/modeler/parameter.h>

#include <cassert>
#include <fstream>
#include <iomanip>

//////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::modeler::IntParameter::valueTip() 
{
  return QString("value range: [%1, %3]").arg(low_).arg(high_).toStdString();
}

//////////////////////////////////////////////////////////////////////////////////////////////
QWidget *
pcl::modeler::IntParameter::createEditor(QWidget *parent)
{
  QSpinBox *editor = new QSpinBox(parent);
  editor->setMinimum(low_);
  editor->setMaximum(high_);
  editor->setSingleStep(step_);

  return editor;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::IntParameter::setEditorData(QWidget *editor)
{
  QSpinBox *spinBox = static_cast<QSpinBox*>(editor);
  spinBox->setAlignment(Qt::AlignHCenter);

  int value = int (*this);
  spinBox->setValue(value);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::IntParameter::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index)
{
  QSpinBox *spinBox = static_cast<QSpinBox*>(editor);
  int value = spinBox->text().toInt();
  current_value_ = value;
  model->setData(index, value, Qt::EditRole);
}

//////////////////////////////////////////////////////////////////////////////////////////////
QString
pcl::modeler::IntParameter::toString()
{
  int value = int (*this);
  return QString("%1").arg(value);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::IntParameter::loadValue(const std::string& valueString)
{
  int value = QString(valueString.c_str()).toInt();
  current_value_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::modeler::DoubleParameter::valueTip() 
{
  return QString("value range: [%1, %3]").arg(low_).arg(high_).toStdString();
}

//////////////////////////////////////////////////////////////////////////////////////////////
QWidget *
pcl::modeler::DoubleParameter::createEditor(QWidget *parent)
{
  QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
  editor->setMinimum(low_);
  editor->setMaximum(high_);
  editor->setSingleStep(step_);
  editor->setDecimals(6);

  return editor;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::DoubleParameter::setEditorData(QWidget *editor)
{
  QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
  spinBox->setAlignment(Qt::AlignHCenter);

  double value = double (*this);
  spinBox->setValue(value);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::DoubleParameter::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index)
{
  QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
  double value = spinBox->text().toDouble();
  current_value_ = value;
  model->setData(index, value, Qt::EditRole);
}

//////////////////////////////////////////////////////////////////////////////////////////////
QString
pcl::modeler::DoubleParameter::toString()
{
  double value = double (*this);
  return QString("%1").arg(value);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::DoubleParameter::loadValue(const std::string& valueString)
{
  double value = QString(valueString.c_str()).toDouble();
  current_value_ = value;
}

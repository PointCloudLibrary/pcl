/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/apps/in_hand_scanner/main_window.h>
#include "ui_main_window.h"

#include <QHBoxLayout>
#include <QTimer>

#include <pcl/apps/in_hand_scanner/in_hand_scanner.h>
#include <pcl/apps/in_hand_scanner/input_data_processing.h>
#include <pcl/apps/in_hand_scanner/icp.h>
#include <pcl/apps/in_hand_scanner/integration.h>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::MainWindow::MainWindow (QWidget* parent)
  : QMainWindow (parent),
    ui_ (new Ui::MainWindow ()),
    ihs_ (new InHandScanner ())
{
  ui_->setupUi (this);

  // In hand scanner
  QHBoxLayout* layout = new QHBoxLayout ();
  layout->addWidget (ihs_);
  ui_->centralWidget->setLayout (layout);

  QTimer::singleShot (0, ihs_, SLOT (startGrabber ()));

  // Input data processing
  const pcl::ihs::InputDataProcessing& idp = ihs_->getInputDataProcessing ();
  ui_->spinBox_x_min->setValue (static_cast <int> (idp.getXMin ()));
  ui_->spinBox_x_max->setValue (static_cast <int> (idp.getXMax ()));
  ui_->spinBox_y_min->setValue (static_cast <int> (idp.getYMin ()));
  ui_->spinBox_y_max->setValue (static_cast <int> (idp.getYMax ()));
  ui_->spinBox_z_min->setValue (static_cast <int> (idp.getZMin ()));
  ui_->spinBox_z_max->setValue (static_cast <int> (idp.getZMax ()));

  ui_->spinBox_h_min->setValue (static_cast <int> (idp.getHMin ()));
  ui_->spinBox_h_max->setValue (static_cast <int> (idp.getHMax ()));
  ui_->spinBox_s_min->setValue (static_cast <int> (idp.getSMin () * 100.f));
  ui_->spinBox_s_max->setValue (static_cast <int> (idp.getSMax () * 100.f));
  ui_->spinBox_v_min->setValue (static_cast <int> (idp.getVMin () * 100.f));
  ui_->spinBox_v_max->setValue (static_cast <int> (idp.getVMax () * 100.f));

  ui_->checkBox_inverted->setChecked (idp.getColorSegmentationInverted ());
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::MainWindow::~MainWindow()
{
  delete ui_;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::setXMin (const int x_min)
{
  ihs_->getInputDataProcessing ().setXMin (static_cast <float> (x_min));
}

void
pcl::ihs::MainWindow::setXMax (const int x_max)
{
  ihs_->getInputDataProcessing ().setXMax (static_cast <float> (x_max));
}

void
pcl::ihs::MainWindow::setYMin (const int y_min)
{
  ihs_->getInputDataProcessing ().setYMin (static_cast <float> (y_min));
}

void
pcl::ihs::MainWindow::setYMax (const int y_max)
{
  ihs_->getInputDataProcessing ().setYMax (static_cast <float> (y_max));
}

void
pcl::ihs::MainWindow::setZMin (const int z_min)
{
  ihs_->getInputDataProcessing ().setZMin (static_cast <float> (z_min));
}

void
pcl::ihs::MainWindow::setZMax (const int z_max)
{
  ihs_->getInputDataProcessing ().setZMax (static_cast <float> (z_max));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::setHMin (const int h_min)
{
  ihs_->getInputDataProcessing ().setHMin (static_cast <float> (h_min));
}

void
pcl::ihs::MainWindow::setHMax (const int h_max)
{
  ihs_->getInputDataProcessing ().setHMax (static_cast <float> (h_max));
}

void
pcl::ihs::MainWindow::setSMin (const int s_min)
{
  ihs_->getInputDataProcessing ().setSMin (static_cast <float> (s_min) / 100.f);
}

void
pcl::ihs::MainWindow::setSMax (const int s_max)
{
  ihs_->getInputDataProcessing ().setSMax (static_cast <float> (s_max) / 100.f);
}

void
pcl::ihs::MainWindow::setVMin (const int v_min)
{
  ihs_->getInputDataProcessing ().setVMin (static_cast <float> (v_min) / 100.f);
}

void
pcl::ihs::MainWindow::setVMax (const int v_max)
{
  ihs_->getInputDataProcessing ().setVMax (static_cast <float> (v_max) / 100.f);
}

void
pcl::ihs::MainWindow::setColorSegmentationInverted (const bool is_inverted)
{
  ihs_->getInputDataProcessing ().setColorSegmentationInverted (is_inverted);
}

////////////////////////////////////////////////////////////////////////////////

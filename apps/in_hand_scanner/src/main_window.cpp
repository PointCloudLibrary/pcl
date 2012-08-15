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

#include <ostream>
#include <sstream>

#include <QTimer>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/apps/in_hand_scanner/in_hand_scanner.h>

////////////////////////////////////////////////////////////////////////////////

pcl::InHandScannerMainWindow::InHandScannerMainWindow (QWidget* p_parent)
  : QMainWindow        (p_parent),

    p_ui_              (new Ui::MainWindow),
    p_timer_           (new QTimer (this)),

    p_visualizer_      (new PCLVisualizer ("", false)),
    p_in_hand_scanner_ (new InHandScanner ())
{
  p_ui_->setupUi (this);
  this->setWindowTitle ("PCL in-hand scanner");

  // Timer
  p_timer_->start (5); // ms
  connect (p_timer_.get (), SIGNAL (timeout ()), this, SLOT (visualizationSlot ()));

  // Visualization
  p_ui_->qvtkWidget->SetRenderWindow (p_visualizer_->getRenderWindow ());
  p_visualizer_->setupInteractor (p_ui_->qvtkWidget->GetInteractor (), p_ui_->qvtkWidget->GetRenderWindow ());
  p_visualizer_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  p_ui_->qvtkWidget->update ();

  // Scanner
  p_in_hand_scanner_->setVisualizer (p_visualizer_);
  p_in_hand_scanner_->start ();
}

////////////////////////////////////////////////////////////////////////////////

pcl::InHandScannerMainWindow::~InHandScannerMainWindow ()
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::InHandScannerMainWindow::visualizationSlot ()
{
  p_in_hand_scanner_->draw ();
  p_ui_->qvtkWidget->update ();
}

////////////////////////////////////////////////////////////////////////////////

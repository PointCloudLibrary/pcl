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

#include <limits>

#include <QDoubleValidator>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QString>
#include <QTimer>

#include <pcl/apps/in_hand_scanner/help_window.h>
#include <pcl/apps/in_hand_scanner/in_hand_scanner.h>
#include <pcl/apps/in_hand_scanner/input_data_processing.h>
#include <pcl/apps/in_hand_scanner/icp.h>
#include <pcl/apps/in_hand_scanner/integration.h>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::MainWindow::MainWindow (QWidget* parent)
  : QMainWindow  (parent),
    ui_          (new Ui::MainWindow ()),
    help_window_ (new HelpWindow (this)),
    ihs_         (new InHandScanner ())
{
  ui_->setupUi (this);

  QWidget* spacer = new QWidget ();
  spacer->setSizePolicy (QSizePolicy::Expanding, QSizePolicy::Expanding);
  ui_->toolBar->insertWidget (ui_->actionHelp, spacer);

  const double max = std::numeric_limits <double>::max ();

  // In hand scanner
  QHBoxLayout* layout = new QHBoxLayout (ui_->placeholder_in_hand_scanner);
  layout->addWidget (ihs_);
  // ui_->centralWidget->setLayout (layout);

  QTimer::singleShot (0, ihs_, SLOT (startGrabber ()));

  connect (ui_->toolButton_1, SIGNAL (clicked ()), ihs_, SLOT (showUnprocessedData ()));
  connect (ui_->toolButton_2, SIGNAL (clicked ()), ihs_, SLOT (showProcessedData ()));
  connect (ui_->toolButton_3, SIGNAL (clicked ()), ihs_, SLOT (registerContinuously ()));
  connect (ui_->toolButton_4, SIGNAL (clicked ()), ihs_, SLOT (registerOnce ()));
  connect (ui_->toolButton_5, SIGNAL (clicked ()), ihs_, SLOT (showModel ()));
  connect (ui_->toolButton_6, SIGNAL (clicked ()), ihs_, SLOT (removeUnfitVertices ()));
  connect (ui_->toolButton_0, SIGNAL (clicked ()), ihs_, SLOT (reset ()));

  connect (ui_->actionReset_camera,        SIGNAL (triggered ()), ihs_, SLOT (resetCamera ()));
  connect (ui_->actionToggle_coloring,     SIGNAL (triggered ()), ihs_, SLOT (toggleColoring ()));
  connect (ui_->actionMesh_representation, SIGNAL (triggered ()), ihs_, SLOT (toggleMeshRepresentation ()));

  connect (ui_->actionSaveAs, SIGNAL (triggered ()), this, SLOT (saveAs ()));

  connect (ihs_, SIGNAL (runningModeChanged (RunningMode)), this, SLOT (runningModeChanged (RunningMode)));

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

  ui_->checkBox_color_segmentation_inverted->setChecked (idp.getColorSegmentationInverted ());
  ui_->checkBox_color_segmentation_enabled->setChecked (idp.getColorSegmentationEnabled ());

  ui_->spinBox_xyz_erode_size->setValue  (idp.getXYZErodeSize ());
  ui_->spinBox_hsv_dilate_size->setValue (idp.getHSVDilateSize ());

  // Registration
  ui_->lineEdit_epsilon->setValidator (new QDoubleValidator (0., max, 2));
  ui_->lineEdit_max_fitness->setValidator (new QDoubleValidator (0., max, 2));

  ui_->lineEdit_epsilon->setText (QString ().setNum (ihs_->getICP ().getEpsilon ()));
  ui_->spinBox_max_iterations->setValue (static_cast <int> (ihs_->getICP ().getMaxIterations ()));
  ui_->spinBox_min_overlap->setValue (static_cast <int> (100.f * ihs_->getICP ().getMinOverlap ()));
  ui_->lineEdit_max_fitness->setText (QString ().setNum (ihs_->getICP ().getMaxFitness ()));

  ui_->doubleSpinBox_correspondence_rejection_factor->setValue (ihs_->getICP ().getCorrespondenceRejectionFactor ());
  ui_->spinBox_correspondence_rejection_max_angle->setValue (static_cast <int> (ihs_->getICP ().getMaxAngle ()));

  // Integration
  ui_->lineEdit_max_squared_distance->setValidator (new QDoubleValidator (0., max, 2));

  ui_->lineEdit_max_squared_distance->setText (QString ().setNum (ihs_->getIntegration ().getMaxSquaredDistance ()));
  ui_->spinBox_averaging_max_angle->setValue (static_cast <int> (ihs_->getIntegration ().getMaxAngle ()));
  ui_->spinBox_max_age->setValue (static_cast <int> (ihs_->getIntegration ().getMaxAge ()));
  ui_->spinBox_min_directions->setValue (static_cast <int> (ihs_->getIntegration ().getMinDirections ()));

  // Help
  connect (ui_->actionHelp, SIGNAL (triggered ()), this, SLOT (showHelp ()));
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::MainWindow::~MainWindow ()
{
  delete ui_;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::showHelp ()
{
  help_window_->show ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::saveAs ()
{
  QString filename = QFileDialog::getSaveFileName (this, "Save the model mesh.", "", "Polygon File Format (*.ply);;VTK File Format (*.vtk)");

  if (filename.isEmpty ()) return;

  if      (filename.endsWith ("ply", Qt::CaseInsensitive))
    ihs_->saveAs (filename.toStdString (), pcl::ihs::InHandScanner::FT_PLY);
  else if (filename.endsWith ("vtk", Qt::CaseInsensitive))
    ihs_->saveAs (filename.toStdString (), pcl::ihs::InHandScanner::FT_VTK);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::runningModeChanged (const RunningMode mode)
{
  switch (mode)
  {
    case InHandScanner::RM_UNPROCESSED:         ui_->toolButton_1->setChecked (true); break;
    case InHandScanner::RM_PROCESSED:           ui_->toolButton_2->setChecked (true); break;
    case InHandScanner::RM_REGISTRATION_CONT:   ui_->toolButton_3->setChecked (true); break;
    case InHandScanner::RM_REGISTRATION_SINGLE:                                       break;
    case InHandScanner::RM_SHOW_MODEL:          ui_->toolButton_5->setChecked (true); break;
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::keyPressEvent (QKeyEvent* event)
{
  ihs_->keyPressEvent (event);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::setXMin (const int x_min)
{
  ihs_->getInputDataProcessing ().setXMin (static_cast <float> (x_min));
  ui_->spinBox_x_min->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getXMin ()));
}

void
pcl::ihs::MainWindow::setXMax (const int x_max)
{
  ihs_->getInputDataProcessing ().setXMax (static_cast <float> (x_max));
  ui_->spinBox_x_max->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getXMax ()));
}

void
pcl::ihs::MainWindow::setYMin (const int y_min)
{
  ihs_->getInputDataProcessing ().setYMin (static_cast <float> (y_min));
  ui_->spinBox_y_min->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getYMin ()));
}

void
pcl::ihs::MainWindow::setYMax (const int y_max)
{
  ihs_->getInputDataProcessing ().setYMax (static_cast <float> (y_max));
  ui_->spinBox_y_max->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getYMax ()));
}

void
pcl::ihs::MainWindow::setZMin (const int z_min)
{
  ihs_->getInputDataProcessing ().setZMin (static_cast <float> (z_min));
  ui_->spinBox_z_min->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getZMin ()));
}

void
pcl::ihs::MainWindow::setZMax (const int z_max)
{
  ihs_->getInputDataProcessing ().setZMax (static_cast <float> (z_max));
  ui_->spinBox_z_max->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getZMax ()));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::setHMin (const int h_min)
{
  ihs_->getInputDataProcessing ().setHMin (static_cast <float> (h_min));
  ui_->spinBox_h_min->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getHMin ()));
}

void
pcl::ihs::MainWindow::setHMax (const int h_max)
{
  ihs_->getInputDataProcessing ().setHMax (static_cast <float> (h_max));
  ui_->spinBox_h_max->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getHMax ()));
}

void
pcl::ihs::MainWindow::setSMin (const int s_min)
{
  ihs_->getInputDataProcessing ().setSMin (.01f * static_cast <float> (s_min));
  ui_->spinBox_s_min->setValue (static_cast <int> (100.f * ihs_->getInputDataProcessing ().getSMin () + 0.5f));
}

void
pcl::ihs::MainWindow::setSMax (const int s_max)
{
  ihs_->getInputDataProcessing ().setSMax (.01f * static_cast <float> (s_max));
  ui_->spinBox_s_max->setValue (static_cast <int> (100.f * ihs_->getInputDataProcessing ().getSMax () + 0.5f));
}

void
pcl::ihs::MainWindow::setVMin (const int v_min)
{
  ihs_->getInputDataProcessing ().setVMin (.01f * static_cast <float> (v_min));
  ui_->spinBox_v_min->setValue (static_cast <int> (100.f * ihs_->getInputDataProcessing ().getVMin () + 0.5f));
}

void
pcl::ihs::MainWindow::setVMax (const int v_max)
{
  ihs_->getInputDataProcessing ().setVMax (.01f * static_cast <float> (v_max));
  ui_->spinBox_v_max->setValue (static_cast <int> (100.f * ihs_->getInputDataProcessing ().getVMax () + 0.5f));
}

void
pcl::ihs::MainWindow::setColorSegmentationInverted (const bool is_inverted)
{
  ihs_->getInputDataProcessing ().setColorSegmentationInverted (is_inverted);
  ui_->checkBox_color_segmentation_inverted->setChecked (ihs_->getInputDataProcessing ().getColorSegmentationInverted ());
}

void
pcl::ihs::MainWindow::setColorSegmentationEnabled (const bool is_enabled)
{
  ihs_->getInputDataProcessing ().setColorSegmentationEnabled (is_enabled);
  ui_->checkBox_color_segmentation_enabled->setChecked (ihs_->getInputDataProcessing ().getColorSegmentationEnabled ());
}

void
pcl::ihs::MainWindow::setXYZErodeSize (const int size)
{
  ihs_->getInputDataProcessing ().setXYZErodeSize (static_cast <unsigned int> (size));
  ui_->spinBox_xyz_erode_size->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getXYZErodeSize ()));
}

void
pcl::ihs::MainWindow::setHSVDilateSize (const int size)
{
  ihs_->getInputDataProcessing ().setHSVDilateSize (static_cast <unsigned int> (size));
  ui_->spinBox_hsv_dilate_size->setValue (static_cast <int> (ihs_->getInputDataProcessing ().getHSVDilateSize ()));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::setEpsilon ()
{
  ihs_->getICP ().setEpsilon (ui_->lineEdit_epsilon->text ().toFloat ());
  ui_->lineEdit_epsilon->setText (QString ().setNum (ihs_->getICP ().getEpsilon ()));
}

void
pcl::ihs::MainWindow::setMaxIterations (const int iterations)
{
  ihs_->getICP ().setMaxIterations (static_cast <unsigned int> (iterations));
  ui_->spinBox_max_iterations->setValue (static_cast <int> (ihs_->getICP ().getMaxIterations ()));
}

void
pcl::ihs::MainWindow::setMinOverlap (const int overlap)
{
  ihs_->getICP ().setMinOverlap (.01f * static_cast <float> (overlap));
  ui_->spinBox_min_overlap->setValue (static_cast <int> (100.f * ihs_->getICP ().getMinOverlap () + 0.5f));
}

void
pcl::ihs::MainWindow::setMaxFitness ()
{
  ihs_->getICP ().setMaxFitness (ui_->lineEdit_max_fitness->text ().toFloat ());
  ui_->lineEdit_max_fitness->setText (QString ().setNum (ihs_->getICP ().getMaxFitness ()));
}

void
pcl::ihs::MainWindow::setCorrespondenceRejectionFactor (const double factor)
{
  ihs_->getICP ().setCorrespondenceRejectionFactor (static_cast <float> (factor));
  ui_->doubleSpinBox_correspondence_rejection_factor->setValue (static_cast <double> (ihs_->getICP ().getCorrespondenceRejectionFactor ()));
}

void
pcl::ihs::MainWindow::setCorrespondenceRejectionMaxAngle (const int angle)
{
  ihs_->getICP ().setMaxAngle (static_cast <float> (angle));
  ui_->spinBox_correspondence_rejection_max_angle->setValue (static_cast <int> (ihs_->getICP ().getMaxAngle ()));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MainWindow::setMaxSquaredDistance ()
{
  ihs_->getIntegration ().setMaxSquaredDistance (ui_->lineEdit_max_squared_distance->text ().toFloat ());
  ui_->lineEdit_max_squared_distance->setText (QString ().setNum (ihs_->getIntegration ().getMaxSquaredDistance ()));
}

void
pcl::ihs::MainWindow::setAveragingMaxAngle (const int angle)
{
  ihs_->getIntegration ().setMaxAngle (static_cast <float> (angle));
  ui_->spinBox_averaging_max_angle->setValue (static_cast <int> (ihs_->getIntegration ().getMaxAngle ()));
}

void
pcl::ihs::MainWindow::setMaxAge (const int age)
{
  ihs_->getIntegration ().setMaxAge (static_cast <unsigned int> (age));
  ui_->spinBox_max_age->setValue (static_cast <int> (ihs_->getIntegration ().getMaxAge ()));
}

void
pcl::ihs::MainWindow::setMinDirections (const int directions)
{
  ihs_->getIntegration ().setMinDirections (static_cast <unsigned int> (directions));
  ui_->spinBox_min_directions->setValue (static_cast <int> (ihs_->getIntegration ().getMinDirections ()));
}

////////////////////////////////////////////////////////////////////////////////

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

#ifndef PCL_APPS_IN_HAND_SCANNER_MAIN_WINDOW_H
#define PCL_APPS_IN_HAND_SCANNER_MAIN_WINDOW_H

#include <QMainWindow>

#include <pcl/apps/in_hand_scanner/in_hand_scanner.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace Ui
{
  class MainWindow;
}

namespace pcl
{
  namespace ihs
  {
    class HelpWindow;
  } // End namespace ihs
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// MainWindow
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {
    class MainWindow : public QMainWindow
    {
      Q_OBJECT

      public:

        typedef pcl::ihs::InHandScanner    InHandScanner;
        typedef pcl::ihs::HelpWindow       HelpWindow;
        typedef InHandScanner::RunningMode RunningMode;

        explicit MainWindow (QWidget* parent = 0);
        ~MainWindow ();

      public Q_SLOTS:

        void showHelp ();
        void saveAs ();

        // In hand scanner
        void runningModeChanged (const RunningMode mode);
        void keyPressEvent (QKeyEvent* event);

        // Input data processing.
        void setXMin (const int x_min);
        void setXMax (const int x_max);
        void setYMin (const int y_min);
        void setYMax (const int y_max);
        void setZMin (const int z_min);
        void setZMax (const int z_max);

        void setHMin (const int h_min);
        void setHMax (const int h_max);
        void setSMin (const int s_min);
        void setSMax (const int s_max);
        void setVMin (const int v_min);
        void setVMax (const int v_max);

        void setColorSegmentationInverted (const bool is_inverted);
        void setColorSegmentationEnabled (const bool is_enabled);

        void setXYZErodeSize (const int size);
        void setHSVDilateSize (const int size);

        // Registration
        void setEpsilon ();
        void setMaxIterations (const int iterations);
        void setMinOverlap (const int overlap);
        void setMaxFitness ();

        void setCorrespondenceRejectionFactor (const double factor);
        void setCorrespondenceRejectionMaxAngle (const int angle);

        // Integration
        void setMaxSquaredDistance ();
        void setAveragingMaxAngle (const int angle);
        void setMaxAge (const int age);
        void setMinDirections (const int directions);

      private:

        Ui::MainWindow* ui_;
        HelpWindow*     help_window_;
        InHandScanner*  ihs_;
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_MAIN_WINDOW_H

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

#ifndef PCL_IN_HAND_SCANNER_MAIN_WINDOW_H
#define PCL_IN_HAND_SCANNER_MAIN_WINDOW_H


#include <boost/shared_ptr.hpp>

#include <QMainWindow>
#include <ui_main_window.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

class QTimer;

namespace pcl
{
  class InHandScanner;

  namespace visualization
  {
    class PCLVisualizer;
  } // End namespace visualization

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// InHandScannerMainWindow
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  class InHandScannerMainWindow : public QMainWindow
  {
      Q_OBJECT

    private:

      typedef boost::shared_ptr<Ui::MainWindow> MainWindowPtr;
      typedef boost::shared_ptr<QTimer>         QTimerPtr;

      typedef pcl::InHandScanner               InHandScanner;
      typedef boost::shared_ptr<InHandScanner> InHandScannerPtr;

      typedef pcl::visualization::PCLVisualizer PCLVisualizer;
      typedef boost::shared_ptr<PCLVisualizer>  PCLVisualizerPtr;

    public:

      explicit InHandScannerMainWindow (QWidget* p_parent = 0);
      ~InHandScannerMainWindow ();

    private slots:

      void
      visualizationSlot ();

    private:

      MainWindowPtr    p_ui_;
      QTimerPtr        p_timer_;

      PCLVisualizerPtr p_visualizer_;
      InHandScannerPtr p_in_hand_scanner_;
  };

} // End namespace pcl

#endif // PCL_IN_HAND_SCANNER_MAIN_WINDOW_H


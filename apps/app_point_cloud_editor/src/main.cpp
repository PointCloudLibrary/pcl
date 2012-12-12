///
/// Copyright (c) 2012, Texas A&M University
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
///  * Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above
///    copyright notice, this list of conditions and the following
///    disclaimer in the documentation and/or other materials provided
///    with the distribution.
///  * Neither the name of Texas A&M University nor the names of its
///    contributors may be used to endorse or promote products derived
///    from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
/// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
/// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
/// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
/// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// The following software was written as part of a collaboration with the
/// University of South Carolina, Interdisciplinary Mathematics Institute.
///

/// @file main.cpp
/// @details the driver which keeps the point cloud editor widget in loop.
/// @author Yue Li and Matthew Hielsberg

///
/// @mainpage Point Cloud Editor Documentation
///
/// @section intro Introduction
/// The point cloud editor provides the functionalities for
/// visualizing point cloud stored using PCL's cloud data structure.
/// Users are able to rotate the point cloud, zoom in/out, as well as adjust
/// the point render size. On the editing aspect, this tool supports several
/// cloud editing features including points selection (by clicking or with a
/// rubberband), copy, delete, paste, move. Users are also able to cancel the
/// selections as well as undo the point move.
///

#include <QApplication>
#include <pcl/apps/point_cloud_editor/mainWindow.h>

int
main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  //QApplication::setWindowIcon(QIcon(":/pceditor.icns"));
  MainWindow main_window(argc, argv);
  main_window.show();
  return (app.exec());
}

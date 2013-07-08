/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 *  Author: Qinghua Li, Yan Zhuang, Xuedong Wang
 *
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
 *   * Neither the name of Intelligent Robotics Lab, DLUT. nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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
 */

#include <QApplication>
#include "../include/qt_gui.h"

/* Better separation of function, communication and gui */

// Connect Signals and Slots for the graphical interface
void
guiConnections (Graphical_UI* gui, GraphManager* graph_mgr)
{
  QObject::connect (gui, SIGNAL (loadData()), graph_mgr, SLOT (loadData ()));
  QObject::connect (gui, SIGNAL (showBAImage()), graph_mgr, SLOT (showBAImage ()));
  QObject::connect (gui, SIGNAL (extractFeatures()), graph_mgr, SLOT (extractFeatures ()));
  QObject::connect (gui, SIGNAL (placeRecognition()), graph_mgr, SLOT (placeRecognition ()));
  QObject::connect (graph_mgr, SIGNAL (reset()), gui, SLOT (reset ()));
  QObject::connect (graph_mgr, SIGNAL (newLibBAImage(QImage)), gui, SLOT (setLibBAImage (QImage)));
  QObject::connect (graph_mgr, SIGNAL (newQueryBAImage(QImage)), gui, SLOT (setQueryBAImage (QImage)));
  QObject::connect (graph_mgr, SIGNAL (newFeaturesImage(QImage)), gui, SLOT (setFeaturesImage (QImage)));
  QObject::connect (graph_mgr, SIGNAL (setGUIStatus(QString)), gui, SLOT (setStatus (QString)));
}

/* On program startup:
 * Create
 * - a Qt Application
 * - a GraphManager, managing the whole graph
 * - a Graphical_UI pointer(gui), then show the Qt interface
 * - let the above communicate internally via QT Signals
 */

int
main (int argc, char** argv)
{
  QApplication app(argc, argv);

  GraphManager graph_mgr;

  Graphical_UI* gui = new Graphical_UI ();
  gui->show ();
  guiConnections (gui, &graph_mgr);

  app.exec ();

  delete gui;
}


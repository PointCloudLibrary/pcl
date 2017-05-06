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

/*
 * This is the main widget of the application.
 * It sets up some not yet useful menus and three qlabels in the layout
 * of the central widget that can be used to show qimages via the slots
 * setQueryBAImage, setFeaturesImage and setLibBAImage.
 *
 *  Created on: 2012.07.07
 *      Author: Qinghua Li
 */

#ifndef QT_GUI_H_
#define QT_GUI_H_

#include <QMainWindow>
#include <QGridLayout>
#include "../include/glviewer.h"
#include "../include/graph_manager.h"

class QSplitter;

//TODO:
// Buttons for extract features and place recognition
// GUI for switching on/off the individual visualizations

/* 
 * Small GUI Class to visualize and control place recognition
 * See Help->About for a short description
 */

class Graphical_UI : public QMainWindow
{
  Q_OBJECT
  public:
    Graphical_UI ();
    ~Graphical_UI ();

  Q_SIGNALS:
    // User wants to load 3D laser scanning data
    void loadData();
    // User wants to show bearing-angle(BA)image
    void showBAImage();
    // User wants to extract global spatial features and local SURF features of
    // the current scene and save to database
    void extractFeatures();
    // User wants to recognize the current scene
    void placeRecognition();

  public Q_SLOTS:
    void
    setLibBAImage (QImage);

    void
    setQueryBAImage (QImage);

    void
    setFeaturesImage (QImage);

  private Q_SLOTS:
    void
    reset ();   /* Start over with new graph */

    void
    loadDataCmd ();

    void
    showBAImageCmd ();

    void
    showFlowsheet ();

    void
    extractFeaturesCmd ();

    void
    placeRecognitionCmd ();

    void
    set2DImage (bool is_on);

    void
    about ();

    void
    help ();

    void
    setStatus (QString);

  private:
    // Menus and Menu elements are defined here
    void
    createMenus ();

    QString *infoText;
    QString *licenseText;
    QString *helpText;

    QLabel *statusLabel;
    QLabel *infoLabel;
    QLabel *infoLabel2;
    QLabel *lib_BAimage_label;
    QLabel *query_BAimage_label;
    QLabel *features_image_label;

    QSplitter* vsplitter;
    GLViewer* lib_glviewer;
    GLViewer* query_glviewer;
};

#endif // QT_GUI_H_
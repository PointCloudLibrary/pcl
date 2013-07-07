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
 * graph_manager.h
 *
 *  Created on: 2012.07.07
 *      Author: Qinghua Li
 */

#ifndef GRAPH_MANAGER_H_
#define GRAPH_MANAGER_H_

#include <QObject>
#include <QPainter>
#include <QTextStream>
#include "../include/scene_recognition.h"

//!Manage the whole graph and point cloud data
class GraphManager : public QObject
{
  Q_OBJECT
  public:
    GraphManager ();
    ~GraphManager ();

  Q_SIGNALS:
    // User selected to reset the graph
    void reset();

    // Connect to these signals to show Bearing-Angle or features images
    void newLibBAImage(QImage);
    void newQueryBAImage(QImage);
    void newFeaturesImage(QImage);

    void setGUIStatus(QString message);

  public Q_SLOTS:
    // Load query or database point cloud 
    void
    loadData ();
    // Show the Bearing-Angle image
    void
    showBAImage ();
    // Extract the global spatial features of point cloud and SURF features of Bearing-Angle image,
    // then save to database
    void
    extractFeatures ();
    // Place recognition(the target of application)
    void
    placeRecognition ();

  public:
    // Draw the text on the Bearing-Angle or features image
    void
    drawQImageText (QImage &image, const QString &str);

    // Store the point cloud data
    static std::vector< std::vector<pcl::PointXYZ> > lib_cloud;
    static std::vector< std::vector<pcl::PointXYZ> > query_cloud;

  private:
    IplImage* BA_iplimage;
    int cloud_width, cloud_height;
};

#endif // GRAPH_MANAGER_H_
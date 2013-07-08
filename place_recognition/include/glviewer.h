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
 * glviewer.h
 *
 *  Created on: 2012.07.07
 *      Author: Qinghua Li
 */

#ifndef GLVIEWER_H_
#define GLVIEWER_H_

#include <QGLWidget>
#include <pcl/point_types.h>

// OpenGL based display of the 3D point cloud
class GLViewer : public QGLWidget
{
  public:
    GLViewer (std::vector< std::vector<pcl::PointXYZ> > &points, int flag, QWidget *parent = 0);
    ~GLViewer ();

    QSize
    minimumSizeHint () const;
    QSize
    sizeHint () const;

    void
    reset ();

  protected:
    void
    setXRotation (int angle);
    void
    setYRotation (int angle);
    void
    setZRotation (int angle);

    void
    initializeGL ();
    void
    paintGL ();
    void
    resizeGL (int width, int height);

    // Draw colored axis, scale long
    void
    drawAxis (float scale);
    // Draw the scene
    void
    drawPointCloud ();

    void
    wheelEvent (QWheelEvent *event);
    void
    mousePressEvent (QMouseEvent *event);
    void
    mouseMoveEvent (QMouseEvent *event);

    void
    keyPressEvent (QKeyEvent *e);

  private:
    int label;
    float bg_color[4];
    int xRot, yRot, zRot;
    float xTra, yTra, zTra;

    int width_, height_;
    double fov_;   /* Field of view */
    double rotation_step_, translation_step_;

    QPoint lastPos;
    bool show_axis_;
    std::vector< std::vector<pcl::PointXYZ> > &point_cloud;
};

#endif // GLVIEWER_H_


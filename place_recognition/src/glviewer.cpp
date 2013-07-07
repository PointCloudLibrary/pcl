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

#include <QtGui>
#include <QtOpenGL>
#include <QKeyEvent>
#include "../include/glviewer.h"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif

GLViewer::GLViewer (std::vector < std::vector < pcl::PointXYZ > >&points, int flag, QWidget *parent)
  : QGLWidget(QGLFormat(QGL::SampleBuffers|QGL::StereoBuffers), parent),
    xRot(0),
    yRot(0),
    zRot(0),
    xTra(-1),
    yTra(-2),
    zTra(-10),
    width_(0),
    height_(0),
    fov_(45.0),
    show_axis_(true),
    point_cloud(points),
    rotation_step_(1.0),
    translation_step_(0.2)
{
  label = flag;
  bg_color[0] = bg_color[1] = bg_color[2] = bg_color[3] = 0.0; // Black background
  setSizePolicy (QSizePolicy::Expanding, QSizePolicy::Expanding); // Can make good use of more space
  setFocusPolicy (Qt::StrongFocus);
}

GLViewer::~GLViewer ()
{
}


QSize
GLViewer::minimumSizeHint () const
{
  return QSize (400, 400);
}

QSize
GLViewer::sizeHint () const
{
  return QSize (640, 480);
}

static void qNormalizeAngle (int &angle)
{
  while (angle < 0)
    angle += 360 * 16;
  while (angle > 360 * 16)
    angle -= 360 * 16;
}

void
GLViewer::setXRotation (int angle)
{
  qNormalizeAngle (angle);
  if (angle != xRot)
  {
    xRot = angle;
    updateGL ();
  }
}

void
GLViewer::setYRotation (int angle)
{
  qNormalizeAngle (angle);
  if (angle != yRot)
  {
    yRot = angle;
    updateGL ();
  }
}

void
GLViewer::setZRotation (int angle)
{
  qNormalizeAngle (angle);
  if (angle != zRot)
  {
    zRot = angle;
    updateGL ();
  }
}

void
GLViewer::initializeGL ()
{
  glShadeModel (GL_SMOOTH);
  glClearColor (bg_color[0], bg_color[1], bg_color[2], bg_color[3]);

  glClearDepth (1.0);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LEQUAL);
  glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void
GLViewer::paintGL ()
{
  if (!this->isVisible ())
    return;

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity ();

  drawPointCloud ();
}

void
GLViewer::resizeGL (int width, int height)
{
  if (height == 0)
  {
    height = 1;
  }

  width_ = width;
  height_ = height;

  glViewport (0, 0, width, height);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();

  float ratio = (float) width / (float) height;
  gluPerspective (fov_, ratio, 0.01, 1e3);
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();
}

void
GLViewer::drawAxis (float scale)
{
  glBegin (GL_LINES);
  glLineWidth (4);
  glColor4f (0.9, 0, 0, 1.0);
  glVertex3f (0, 0, 0);
  glColor4f (0.9, 0, 0, 0.0);
  glVertex3f (scale, 0, 0);
  glColor4f (0, 0.9, 0, 1.0);
  glVertex3f (0, 0, 0);
  glColor4f (0, 0.9, 0, 0.0);
  glVertex3f (0, scale, 0);
  glColor4f (0, 0, 0.9, 1.0);
  glVertex3f (0, 0, 0);
  glColor4f (0, 0, 0.9, 0.0);
  glVertex3f (0, 0, scale);
  glEnd ();
}

void
GLViewer::drawPointCloud ()
{
  if (point_cloud.size () > 0)
  {
    glColor4f (1 - bg_color[0], 1 - bg_color[1], 1 - bg_color[2], 1.0); /* Inverse of bg color */

    if (label == 1)
    {
      this->renderText (10, 25, tr ("Query Scene"), QFont ("Times", 16, QFont::Normal, true));
    }
    if (label == 2)
    {
      this->renderText (10, 25, tr ("Corresponding Scene in Database"), QFont ("Times", 16, QFont::Normal, true));
    }

    glTranslatef (xTra, yTra, zTra);
    int x_steps = (xRot / 16.0) / rotation_step_;
    int y_steps = (yRot / 16.0) / rotation_step_;
    int z_steps = (zRot / 16.0) / rotation_step_;
    glRotatef (x_steps * rotation_step_, 1.0, 0.0, 0.0);
    glRotatef (y_steps * rotation_step_, 0.0, 1.0, 0.0);
    glRotatef (z_steps * rotation_step_, 0.0, 0.0, 1.0);

    if (show_axis_)
    {
      drawAxis (0.5);
    }

    glColor3f (0.0, 1.0, 0.0);
    glPointSize (1.0);
    glBegin (GL_POINTS);
    for (int i = 0; i < (int) point_cloud.size (); ++i)
    {
      for (int j = 0; j < (int) point_cloud[i].size (); ++j)
      {
        glVertex3f (-point_cloud[i][j].y, point_cloud[i][j].z, -point_cloud[i][j].x);
      }
    }
    glEnd ();
  }
  else
  {
    glColor4f (1 - bg_color[0], 1 - bg_color[1], 1 - bg_color[2], 1.0); /* Inverse of bg color */
    this->renderText (width_ / 5, height_ / 2, tr ("Waiting for point cloud to display..."),
                      QFont ("Times", 16, QFont::Normal, true));
  }
}

// Reset the initial view of scene
void
GLViewer::reset ()
{
  xRot = 0;
  yRot = 0;
  zRot = 0;
  xTra = -1;
  yTra = -2;
  zTra = -10;
  updateGL ();
}

void
GLViewer::wheelEvent (QWheelEvent *event)
{
  zTra += ((float) event->delta ()) / 25.0;
  updateGL ();
}

void
GLViewer::mousePressEvent (QMouseEvent *event)
{
  lastPos = event->pos ();
}

void
GLViewer::mouseMoveEvent (QMouseEvent *event)
{
  /* Consolidate setRotation methods */
  int dx = event->x () - lastPos.x ();
  int dy = event->y () - lastPos.y ();

  if (event->buttons () & Qt::LeftButton)
  {
    setXRotation (xRot - 8 * dy);
    setYRotation (yRot + 8 * dx);
  }
  else if (event->buttons () & Qt::RightButton)
  {
    setXRotation (xRot - 8 * dy);
    setZRotation (zRot + 8 * dx);
  }
  else if (event->buttons () & Qt::MidButton)
  {
    xTra += dx / 200.0;
    yTra -= dy / 200.0;
    updateGL ();
  }
  lastPos = event->pos ();
}

void
GLViewer::keyPressEvent (QKeyEvent *e)
{
  switch (e->key ())
  {
    // Reset
    case Qt::Key_R:
      reset ();
      break;

    // Front
    case Qt::Key_W:
      zTra += translation_step_;
      updateGL ();
      break;

    // Back
    case Qt::Key_S:
      zTra -= translation_step_;
      updateGL ();
      break;

    // Left
    case Qt::Key_A:
      xTra -= translation_step_;
      updateGL ();
      break;

    // Right
    case Qt::Key_D:
      xTra += translation_step_;
      updateGL ();
      break;

    // Up
    case Qt::Key_Q:
      yTra += translation_step_;
      updateGL ();
      break;

    // Down
    case Qt::Key_E:
      yTra -= translation_step_;
      updateGL ();
      break;

    // Switch drawing the coordinate axis
    case Qt::Key_X:
      show_axis_ = !show_axis_;
      updateGL ();
      break;

    // Close the widget
    case Qt::Key_Escape:
      close ();
  }
}
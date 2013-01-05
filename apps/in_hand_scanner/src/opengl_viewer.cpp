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

#include <pcl/apps/in_hand_scanner/opengl_viewer.h>

#include <cmath>

#ifdef __APPLE__
#  include <OpenGL/gl.h>
#  include <OpenGL/glu.h>
#else
#  include <GL/gl.h>
#  include <GL/glu.h>
#endif

#include <QtOpenGL>

#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp> // TODO: PointIHS is not registered

////////////////////////////////////////////////////////////////////////////////
// Internal classes
////////////////////////////////////////////////////////////////////////////////

pcl::ihs::OpenGLViewer::FaceVertexMesh::FaceVertexMesh ()
  : vertices  (),
    triangles ()
{
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::OpenGLViewer::FaceVertexMesh::FaceVertexMesh (const Mesh& mesh)
  : vertices (mesh.getVertexDataCloud ()),
    triangles ()
{
  for (CloudIHS::iterator it=vertices.begin (); it!=vertices.end (); ++it)
  {
    std::swap (it->r, it->b);
  }

  triangles.reserve (mesh.sizeFaces ());
  pcl::ihs::OpenGLViewer::FaceVertexMesh::Triangle triangle;

  for (unsigned int i=0; i<mesh.sizeFaces (); ++i)
  {
    Mesh::VertexAroundFaceCirculator circ = mesh.getVertexAroundFaceCirculator (Mesh::FaceIndex (i));
    triangle.first  = (circ++).getTargetIndex ().get ();
    triangle.second = (circ++).getTargetIndex ().get ();
    triangle.third  = (circ  ).getTargetIndex ().get ();

    triangles.push_back (triangle);
  }
}

////////////////////////////////////////////////////////////////////////////////
// OpenGLViewer
////////////////////////////////////////////////////////////////////////////////

pcl::ihs::OpenGLViewer::OpenGLViewer (QWidget* parent)
  : QGLWidget            (parent),
    mutex_vis_           (),
    timer_               (new QTimer (this)),
    T_object_            (Eigen::Matrix4f::Identity ()),
    drawn_clouds_        (),
    drawn_meshes_        (),
    display_mode_        (DM_POINTS),
    draw_cube_           (false),
    cube_coefficients_   (),
    cam_R_               (1., 0., 0., 0.),
    cam_t_               (0., 0., 0.),
    cam_pivot_           (0., 0., 0.),
    cam_pivot_id_        (""),
    mouse_pressed_begin_ (false),
    x_prev_              (0),
    y_prev_              (0)
{
  // Timer: Defines the update rate for the visualization
  connect (timer_.get (), SIGNAL (timeout ()), this, SLOT (timerCallback ()));
  timer_->start (33);

  QWidget::setAutoFillBackground (false);
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::OpenGLViewer::~OpenGLViewer ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  timer_->stop ();
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::addCloud (const CloudXYZRGBNormalConstPtr& cloud, const std::string& id)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  if (drawn_meshes_.find (id) != drawn_meshes_.end ())
  {
    std::cerr << "Trying to add a cloud with the id '" << id << "' although a mesh with the same id already exists!\n";
    return (false);
  }

  CloudXYZRGBNormalPtr c (new CloudXYZRGBNormal ());
  c->reserve (cloud->size ());

  for (CloudXYZRGBNormal::const_iterator it=cloud->begin (); it!=cloud->end (); ++it)
  {
    if (!boost::math::isnan (it->x))
    {
      c->push_back (*it);
      std::swap (c->back ().r, c->back ().b);
    }
  }

  if (drawn_clouds_.find (id) == drawn_clouds_.end ())
  {
    drawn_clouds_.insert (std::make_pair (id, c));
  }
  else
  {
    drawn_clouds_ [id] = c;
  }

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::removeCloud (const std::string& id)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  if (drawn_clouds_.find (id) == drawn_clouds_.end ())
  {
    return (false);
  }
  else
  {
    drawn_clouds_.erase (id);
    return (true);
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::removeAllClouds ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  drawn_clouds_.clear ();
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::addMesh (const MeshConstPtr& mesh, const std::string& id)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  if (drawn_clouds_.find (id) != drawn_clouds_.end ())
  {
    std::cerr << "Trying to add a cloud with the id '" << id << "' although a mesh with the same id already exists!\n";
    return (false);
  }

  if (drawn_meshes_.find (id) == drawn_meshes_.end ())
  {
    drawn_meshes_.insert (std::make_pair (id, FaceVertexMeshPtr (new FaceVertexMesh (*mesh))));
  }
  else
  {
    drawn_meshes_ [id] = FaceVertexMeshPtr (new FaceVertexMesh (*mesh));
  }

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::addMesh (const CloudXYZRGBNormalConstPtr& cloud, const std::string& id)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  if (!cloud->isOrganized ())
  {
    return (false);
  }

  if (drawn_clouds_.find (id) != drawn_clouds_.end ())
  {
    drawn_clouds_.erase (id);
  }
  lock.unlock ();

  // Convert the cloud to a mesh using the following pattern
  // 2 - 1 //
  // | / | //
  // 3 - 0 //
  const int w        = cloud->width;
  const int h        = cloud->height;
  const int offset_1 = -w;
  const int offset_2 = -w - 1;
  const int offset_3 =    - 1;

  FaceVertexMeshPtr mesh (new FaceVertexMesh ());
  std::vector <int> indices (w * h, -1); // Map the original indices to the vertex indices.
  CloudIHS& vertices = mesh->vertices;
  std::vector <FaceVertexMesh::Triangle>& triangles = mesh->triangles;
  vertices.reserve (cloud->size ());
  triangles.reserve (2 * (w-1) * (h-1));

  // Helper functor
  struct AddVertex
  {
    inline int operator () (const PointXYZRGBNormal& pt, CloudIHS& vertices, int& ind_o) const
    {
      if (ind_o == -1)
      {
        ind_o = vertices.size ();
        vertices.push_back (PointIHS (pt, -pt.normal_z));
        std::swap (vertices.back ().r, vertices.back ().b);
      }
      return (ind_o);
    }
  };
  AddVertex addVertex;

  int ind_o_0, ind_o_1, ind_o_2, ind_o_3; // Index into the organized cloud.
  int ind_v_0, ind_v_1, ind_v_2, ind_v_3; // Index to the new vertices.

  for (int r=1; r<h; ++r)
  {
    for (int c=1; c<w; ++c)
    {
      ind_o_0 = r*w + c;
      ind_o_1 = ind_o_0 + offset_1;
      ind_o_2 = ind_o_0 + offset_2;
      ind_o_3 = ind_o_0 + offset_3;

      const PointXYZRGBNormal& pt_0 = cloud->operator [] (ind_o_0);
      const PointXYZRGBNormal& pt_1 = cloud->operator [] (ind_o_1);
      const PointXYZRGBNormal& pt_2 = cloud->operator [] (ind_o_2);
      const PointXYZRGBNormal& pt_3 = cloud->operator [] (ind_o_3);

      if (!boost::math::isnan (pt_1.x) && !boost::math::isnan (pt_3.x))
      {
        if (!boost::math::isnan (pt_2.x)) // 1-2-3 is valid
        {
          if (std::abs (pt_1.z - pt_2.z) < 1 &&
              std::abs (pt_1.z - pt_3.z) < 1 &&
              std::abs (pt_2.z - pt_3.z) < 1) // distance threshold
          {
            ind_v_1 = addVertex (pt_1, vertices, indices [ind_o_1]);
            ind_v_2 = addVertex (pt_2, vertices, indices [ind_o_2]);
            ind_v_3 = addVertex (pt_3, vertices, indices [ind_o_3]);

            triangles.push_back (FaceVertexMesh::Triangle (ind_v_1, ind_v_2, ind_v_3));
          }
        }
        if (!boost::math::isnan (pt_0.x)) // 0-1-3 is valid
        {
          if (std::abs (pt_0.z - pt_1.z) < 1 &&
              std::abs (pt_0.z - pt_3.z) < 1 &&
              std::abs (pt_1.z - pt_3.z) < 1) // distance threshold
          {
            ind_v_1 = addVertex (pt_1, vertices, indices [ind_o_1]);
            ind_v_3 = addVertex (pt_3, vertices, indices [ind_o_3]);
            ind_v_0 = addVertex (pt_0, vertices, indices [ind_o_0]);

            triangles.push_back (FaceVertexMesh::Triangle (ind_v_1, ind_v_3, ind_v_0));
          }
        }
      }
    }
  }

  // Finally add the mesh.
  lock.lock ();
  if (drawn_meshes_.find (id) == drawn_meshes_.end ())
  {
    drawn_meshes_.insert (std::make_pair (id, mesh));
  }
  else
  {
    drawn_meshes_ [id] = mesh;
  }

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::removeMesh (const std::string& id)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  if (drawn_meshes_.find (id) == drawn_meshes_.end ())
  {
    return (false);
  }
  else
  {
    drawn_meshes_.erase (id);
    return (true);
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::removeAllMeshes ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  drawn_meshes_.clear ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setCubeCoefficients (const CubeCoefficients& coeffs)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  cube_coefficients_ = coeffs;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::enableDrawCube ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  draw_cube_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::disableDrawCube ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  draw_cube_ = false;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setPivot (const qreal x, const qreal y, const qreal z)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  cam_pivot_.setX (x);
  cam_pivot_.setY (y);
  cam_pivot_.setZ (z);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setPivot (const std::string& id)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  cam_pivot_id_ = id;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::stopTimer ()
{
  if (timer_)
  {
    timer_->stop ();
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::toggleDisplayMode ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  switch (display_mode_)
  {
    case DM_POINTS: display_mode_ = DM_FACES;  std::cerr << "Drawing the faces.\n";  break;
    case DM_FACES:  display_mode_ = DM_POINTS; std::cerr << "Drawing the points.\n"; break;
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::resetCamera ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  cam_R_ = QQuaternion (1., 0., 0., 0.);
  cam_t_ = QVector3D   (0., 0., 0.);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setObjectTransformation (const Eigen::Matrix4f& T)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  T_object_ = T;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::timerCallback ()
{
  QWidget::update ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::paintEvent (QPaintEvent* /*event*/)
{
  this->calcPivot ();
  this->makeCurrent ();

  // Clear information from the last draw
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor (0.f, 0.f, 0.f, 0.f);

  this->setupViewport (this->width (), this->height ());

  // Move light with camera (see example 5-7)
  // http://www.glprogramming.com/red/chapter05.html
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // light 0 (directional)
  glLightfv (GL_LIGHT0, GL_AMBIENT , Eigen::Vector4f (0.1f, 0.1f, 0.1f, 1.0f).eval ().data ());
  glLightfv (GL_LIGHT0, GL_DIFFUSE , Eigen::Vector4f (0.6f, 0.6f, 0.6f, 1.0f).eval ().data () );
  glLightfv (GL_LIGHT0, GL_SPECULAR, Eigen::Vector4f (0.2f, 0.2f, 0.2f, 1.0f).eval ().data ());
  glLightfv (GL_LIGHT0, GL_POSITION, Eigen::Vector4f (0.3f, 0.5f, 0.8f, 0.0f).normalized ().eval ().data ());

  // light 1 (directional)
  glLightfv (GL_LIGHT1, GL_AMBIENT , Eigen::Vector4f ( 0.0f, 0.0f, 0.0f, 1.0f).eval ().data ());
  glLightfv (GL_LIGHT1, GL_DIFFUSE , Eigen::Vector4f ( 0.3f, 0.3f, 0.3f, 1.0f).eval ().data () );
  glLightfv (GL_LIGHT1, GL_SPECULAR, Eigen::Vector4f ( 0.1f, 0.1f, 0.1f, 1.0f).eval ().data ());
  glLightfv (GL_LIGHT1, GL_POSITION, Eigen::Vector4f (-0.3f, 0.5f, 0.8f, 0.0f).normalized ().eval ().data ());

  // Material
  glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glMaterialfv    (GL_FRONT, GL_SPECULAR , Eigen::Vector4f (0.1f, 0.1f, 0.1f, 1.0f).eval ().data ());
  glMaterialf     (GL_FRONT, GL_SHININESS, 25.f);

  glEnable (GL_DEPTH_TEST);
  glEnable (GL_NORMALIZE);
  glEnable (GL_COLOR_MATERIAL);
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  glEnable (GL_LIGHT1);

  // Projection matrix
  glMatrixMode   (GL_PROJECTION);
  glLoadIdentity ();
  gluPerspective (43., 4./3., 1., 5000.);
  glMatrixMode   (GL_MODELVIEW);

  // ModelView matrix
  QQuaternion cam_R;
  QVector3D   cam_t;
  {
    boost::mutex::scoped_lock lock (mutex_vis_);
    cam_R = cam_R_;
    cam_t = cam_t_;
  }

  const QVector3D o  (0., 0., 0.);
  const QVector3D ey (0., 1., 0.);
  const QVector3D ez (0., 0., 1.);

  const QVector3D eye    = cam_R.rotatedVector ( o ) + cam_t;
  const QVector3D center = cam_R.rotatedVector ( ez) + cam_t;
  const QVector3D up     = cam_R.rotatedVector (-ey).normalized ();

  glMatrixMode (GL_MODELVIEW);
  gluLookAt (eye.   x (), eye.   y (), eye.   z (),
             center.x (), center.y (), center.z (),
             up.    x (), up.    y (), up.    z ());

  // Draw everything
  glPushMatrix ();
  {
    glMultMatrixf (T_object_.data ());
    this->drawClouds ();
    this->drawMeshes ();
  }
  glPopMatrix ();

  glDisable (GL_LIGHTING); // This is needed so the color is right.
  this->drawCube ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::calcPivot ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  Eigen::Vector4f pivot;

  if (drawn_clouds_.find (cam_pivot_id_) != drawn_clouds_.end ())
  {
    if (pcl::compute3DCentroid (*(drawn_clouds_ [cam_pivot_id_]), pivot))
    {
      lock.unlock ();
      this->setPivot (pivot.x (), pivot.y (), pivot.z ());
    }
  }
  else if (drawn_meshes_.find (cam_pivot_id_) != drawn_meshes_.end ())
  {
    if (pcl::compute3DCentroid (drawn_meshes_ [cam_pivot_id_]->vertices, pivot))
    {
      lock.unlock ();
      this->setPivot (pivot.x (), pivot.y (), pivot.z ());
    }
  }
  cam_pivot_id_.clear ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::drawClouds ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  glEnableClientState (GL_VERTEX_ARRAY);
  glEnableClientState (GL_COLOR_ARRAY);
  glEnableClientState (GL_NORMAL_ARRAY);

  for (CloudXYZRGBNormalMap::const_iterator it=drawn_clouds_.begin (); it!=drawn_clouds_.end (); ++it)
  {
    if (it->second && !it->second->empty ())
    {
      const CloudXYZRGBNormal& cloud = *it->second;

      glVertexPointer (3, GL_FLOAT        , sizeof (PointXYZRGBNormal), &(cloud [0].x       ));
      glColorPointer  (3, GL_UNSIGNED_BYTE, sizeof (PointXYZRGBNormal), &(cloud [0].b       ));
      glNormalPointer (   GL_FLOAT        , sizeof (PointXYZRGBNormal), &(cloud [0].normal_x));

      glDrawArrays (GL_POINTS, 0, cloud.size ());
    }
  }

  glDisableClientState (GL_VERTEX_ARRAY);
  glDisableClientState (GL_COLOR_ARRAY);
  glDisableClientState (GL_NORMAL_ARRAY);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::drawMeshes ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  glEnableClientState (GL_VERTEX_ARRAY);
  glEnableClientState (GL_COLOR_ARRAY);
  glEnableClientState (GL_NORMAL_ARRAY);

  for (FaceVertexMeshMap::const_iterator it=drawn_meshes_.begin (); it!=drawn_meshes_.end (); ++it)
  {
    if (it->second && !it->second->vertices.empty ())
    {
      const FaceVertexMesh& mesh = *it->second;

      glVertexPointer (3, GL_FLOAT        , sizeof (PointIHS), &(mesh.vertices [0].x       ));
      glColorPointer  (3, GL_UNSIGNED_BYTE, sizeof (PointIHS), &(mesh.vertices [0].b       ));
      glNormalPointer (   GL_FLOAT        , sizeof (PointIHS), &(mesh.vertices [0].normal_x));

      switch (display_mode_)
      {
        case DM_POINTS:
        {
          glDrawArrays (GL_POINTS, 0, mesh.vertices.size ());
          break;
        }
        case DM_FACES:
        {
          glDrawElements (GL_TRIANGLES, 3*mesh.triangles.size (), GL_UNSIGNED_INT, &mesh.triangles [0]);
          break;
        }
      }
    }
  }

  glDisableClientState (GL_VERTEX_ARRAY);
  glDisableClientState (GL_COLOR_ARRAY);
  glDisableClientState (GL_NORMAL_ARRAY);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::drawCube ()
{
  CubeCoefficients coeffs;
  {
    boost::mutex::scoped_lock lock (mutex_vis_);

    if (draw_cube_) coeffs = cube_coefficients_;
    else            return;
  }

  glColor3f (1.f, 1.f, 1.f);

  // Front
  glBegin(GL_LINE_STRIP);
  {
    glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_min);
    glVertex3f (coeffs.x_max, coeffs.y_min, coeffs.z_min);
    glVertex3f (coeffs.x_max, coeffs.y_max, coeffs.z_min);
    glVertex3f (coeffs.x_min, coeffs.y_max, coeffs.z_min);
    glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_min);
  }
  glEnd();

  // Back
  glBegin (GL_LINE_STRIP);
  {
    glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_max);
    glVertex3f (coeffs.x_max, coeffs.y_min, coeffs.z_max);
    glVertex3f (coeffs.x_max, coeffs.y_max, coeffs.z_max);
    glVertex3f (coeffs.x_min, coeffs.y_max, coeffs.z_max);
    glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_max);
  }
  glEnd();

  // Sides
  glBegin (GL_LINES);
  {
    glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_min);
    glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_max);

    glVertex3f (coeffs.x_max, coeffs.y_min, coeffs.z_min);
    glVertex3f (coeffs.x_max, coeffs.y_min, coeffs.z_max);

    glVertex3f (coeffs.x_max, coeffs.y_max, coeffs.z_min);
    glVertex3f (coeffs.x_max, coeffs.y_max, coeffs.z_max);

    glVertex3f (coeffs.x_min, coeffs.y_max, coeffs.z_min);
    glVertex3f (coeffs.x_min, coeffs.y_max, coeffs.z_max);
  }
  glEnd();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::initializeGL ()
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setupViewport (const int w, const int h)
{
  const float aspect_ratio = 4./3.;

  // Use the biggest possible area of the window to draw to
  //    case 1 (w < w_scaled):        case 2 (w >= w_scaled):
  //      w
  //    |---|         ^               |-------------|  ^
  //    |---| ^       |               |    |   |    |  | h
  //    |   | | h_sc  | h             |-------------|  v
  //    |---| v       |                    <---> w_sc
  //    |---|         v               <----- w ----->
  const float w_scaled = h * aspect_ratio;
  const float h_scaled = w / aspect_ratio;

  if (w < w_scaled)
  {
    glViewport (0, static_cast <GLint> ((h - h_scaled) / 2.f), w, static_cast <GLsizei> (h_scaled));
  }
  else
  {
    glViewport (static_cast <GLint> ((w - w_scaled) / 2.f), 0, static_cast <GLsizei> (w_scaled), h);
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::resizeGL (int w, int h)
{
  this->setupViewport (w, h);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::mousePressEvent (QMouseEvent* /*event*/)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  mouse_pressed_begin_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::mouseMoveEvent (QMouseEvent* event)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  if (mouse_pressed_begin_)
  {
    x_prev_ = event->pos ().x ();
    y_prev_ = event->pos ().y ();
    mouse_pressed_begin_ = false;
    return;
  }
  if (event->pos ().x () == x_prev_ && event->pos ().y () == y_prev_) return;
  if (this->width () == 0 || this->height () == 0)                    return;

  const qreal dx = static_cast <qreal> (event->pos ().x ()) - static_cast <qreal> (x_prev_);
  const qreal dy = static_cast <qreal> (event->pos ().y ()) - static_cast <qreal> (y_prev_);
  const qreal w  = static_cast <qreal> (this->width ());
  const qreal h  = static_cast <qreal> (this->height ());
  const qreal d  = std::sqrt (w*w + h*h);

  const QVector3D o  (0., 0., 0.);
  const QVector3D ex (1., 0., 0.);
  const QVector3D ey (0., 1., 0.);
  const QVector3D ez (0., 0., 1.);

  // Scale with the distance between the pivot and camera eye.
  const qreal scale = std::max ((cam_pivot_ - cam_R_.rotatedVector (o) - cam_t_).length (), 10.) / d;

  if (QApplication::mouseButtons () == Qt::LeftButton)
  {
    const QVector3D rot_axis  = (cam_R_.rotatedVector (ex) * dy - cam_R_.rotatedVector (ey) * dx).normalized ();
    const qreal     rot_angle = -400. * std::atan (std::sqrt ((dx*dx + dy*dy)) / d);

    const QQuaternion dR = QQuaternion::fromAxisAndAngle (rot_axis, rot_angle);
    cam_t_ = dR.rotatedVector (cam_t_ - cam_pivot_) + cam_pivot_;
    cam_R_ = (dR * cam_R_).normalized ();
  }
  else if (QApplication::mouseButtons () == Qt::MiddleButton)
  {
    cam_t_ += 1.3 * scale * cam_R_.rotatedVector (ey * -dy + ex * -dx);
  }
  else if (QApplication::mouseButtons () == Qt::RightButton)
  {
    cam_t_ += 2.6 * scale * cam_R_.rotatedVector (ez * -dy);
  }

  x_prev_ = event->pos ().x ();
  y_prev_ = event->pos ().y ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::wheelEvent (QWheelEvent* event)
{
  if (QApplication::mouseButtons () == Qt::NoButton)
  {
    boost::mutex::scoped_lock lock (mutex_vis_);

    // Scale with the distance between the pivot and camera eye.
    const QVector3D o  (0., 0., 0.);
    const QVector3D ez (0., 0., 1.);
    const qreal w     = static_cast <qreal> (this->width ());
    const qreal h     = static_cast <qreal> (this->height ());
    const qreal d     = std::sqrt (w*w + h*h);
    const qreal scale = std::max ((cam_pivot_ - cam_R_.rotatedVector (o) - cam_t_).length (), 10.) / d;

    // http://doc.qt.digia.com/qt/qwheelevent.html#delta
    cam_t_ += scale * cam_R_.rotatedVector (ez * event->delta ());
  }
}

////////////////////////////////////////////////////////////////////////////////

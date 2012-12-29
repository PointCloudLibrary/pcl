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

#include <pcl/apps/in_hand_scanner/in_hand_scanner.h>

#include <cmath>

#ifdef __APPLE__
#  include <OpenGL/gl.h>
#  include <OpenGL/glu.h>
#else
#  include <GL/gl.h>
#  include <GL/glu.h>
#endif

#include <QtOpenGL>
#include <QPainter>

#include <pcl/common/transforms.h>
//#include <pcl/common/centroid.h>
//#include <pcl/common/impl/centroid.hpp> // TODO: PointIHS is not registered
#include <pcl/exceptions.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/apps/in_hand_scanner/icp.h>
#include <pcl/apps/in_hand_scanner/input_data_processing.h>
#include <pcl/apps/in_hand_scanner/integration.h>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InHandScanner::InHandScanner (QWidget* parent)
  : QGLWidget              (parent),
    mutex_comp_vis_        (),
    mutex_comp_events_     (),
    mutex_vis_events_      (),
    computation_fps_       (),
    visualization_fps_     (),
    running_mode_          (RM_UNPROCESSED),
    iteration_             (0),
    grabber_               (),
    starting_grabber_      (false),
    new_data_connection_   (),
    cloud_draw_tmp_        (),
    cloud_draw_            (),
    input_data_processing_ (new InputDataProcessing ()),
    draw_crop_box_         (false),
//    icp_                   (new ICP ()),
//    transformation_        (Eigen::Matrix4f::Identity ()),
//    integration_           (new Integration ()),
//    mesh_model_            (new Mesh ()),
//    mesh_model_draw        (new Mesh ()),
    display_mode_          (DM_POINTS),
    cam_R_                 (1., 0., 0., 0.),
    cam_t_                 (0., 0., 0.),
    cam_pivot_             (0., 0., 0.),
    mouse_pressed_begin_   (false),
    x_prev_                (0),
    y_prev_                (0),
    destructor_called_     (false)
{
  // Initialize the pivot
  float x_min, x_max, y_min, y_max, z_min, z_max;
  input_data_processing_->getCropBox (x_min, x_max, y_min, y_max, z_min, z_max);
  cam_pivot_.setX ((x_min + x_max) / 2.);
  cam_pivot_.setY ((y_min + y_max) / 2.);
  cam_pivot_.setZ ((z_min + z_max) / 2.);

  // Timer: Defines the update rate for the visualization
  QTimer* timer = new QTimer (this);
  connect (timer, SIGNAL (timeout ()), this, SLOT (timerCallback ()));
  timer->start (33);

  setAutoFillBackground (false);
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InHandScanner::~InHandScanner ()
{
  destructor_called_ = true;
  if (grabber_ && grabber_->isRunning ()) grabber_->stop ();
  if (new_data_connection_.connected ())  new_data_connection_.disconnect ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::startGrabber ()
{
  QtConcurrent::run (boost::bind (&pcl::ihs::InHandScanner::startGrabberImpl, this));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::timerCallback ()
{
  this->update ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::setRunningMode (const RunningMode& mode)
{
  boost::mutex::scoped_lock lock_ve (mutex_vis_events_);
  boost::mutex::scoped_lock lock_cv (mutex_comp_events_);

  switch (mode)
  {
    case RM_SHOW_MODEL:
    {
      draw_crop_box_ = false;
      std::cerr << "Show the model\n";
      break;
    }
    case RM_UNPROCESSED:
    {
      draw_crop_box_ = false;
      std::cerr << "Showing the unprocessed input data.\n";
      break;
    }
    case RM_PROCESSED:
    {
      draw_crop_box_ = true;
      std::cerr << "Showing the processed input data.\n";
      break;
    }
    case RM_REGISTRATION_CONT:
    {
      draw_crop_box_ = true;
      std::cerr << "Continuous registration.\n";
      break;
    }
    case RM_REGISTRATION_SINGLE:
    {
      draw_crop_box_ = true;
      std::cerr << "Single registration.\n";
      break;
    }
    default:
    {
      std::cerr << "ERROR in in_hand_scanner.cpp: Unknown command!\n";
      return;
    }
  }

  running_mode_ = mode;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::setDisplayMode (const DisplayMode& mode)
{
  boost::mutex::scoped_lock lock_ve (mutex_vis_events_);

  switch (mode)
  {
    case DM_POINTS: std::cerr << "Drawing the points.\n";                            break;
    case DM_EDGES:  std::cerr << "Drawing the edges of the faces.\n";                break;
    case DM_FACES:  std::cerr << "Drawing faces of the mesh without edges.\n";       break;
    default:        std::cerr << "ERROR in in_hand_scanner.cpp: Unknown command!\n"; return;
  }

  display_mode_ = mode;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::reset ()
{
  boost::mutex::scoped_lock lock_ce (mutex_comp_events_);
  std::cerr << "Reset the scanning pipeline.\n";
  running_mode_ = RM_UNPROCESSED;
  iteration_    = 0;
  lock_ce.unlock ();

  this->resetCamera ();
//  transformation_  = Eigen::Matrix4f::Identity ();
//  mesh_vec_draw_.clear ();
//  mesh_model_->clear ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::resetCamera ()
{
  boost::mutex::scoped_lock lock_ve (mutex_vis_events_);

  cam_R_ = QQuaternion (1., 0., 0., 0.);
  cam_t_ = QVector3D   (0., 0., 0.);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::startGrabberImpl ()
{
  starting_grabber_ = true;
  try
  {
    grabber_ = GrabberPtr (new Grabber ());
  }
  catch (const pcl::PCLException& e)
  {
    std::cerr << "ERROR in in_hand_scanner.cpp: " << e.what () << std::endl;
    exit (EXIT_FAILURE);
  }

  if (destructor_called_) return;

  boost::function <void (const CloudXYZRGBAConstPtr&)> new_data_cb = boost::bind (&pcl::ihs::InHandScanner::newDataCallback, this, _1);

  if (destructor_called_) return;

  new_data_connection_ = grabber_->registerCallback (new_data_cb);

  if (destructor_called_) return;

  grabber_->start ();

  if (destructor_called_) return;

  starting_grabber_ = false;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::newDataCallback (const CloudXYZRGBAConstPtr& cloud_in)
{
  RunningMode running_mode;
  unsigned int iteration;
  {
    boost::mutex::scoped_lock lock_ce (mutex_comp_events_);
    running_mode = running_mode_;
    iteration    = iteration_;

    this->calcFPS (computation_fps_);
  }

  // Input data processing
  CloudXYZRGBNormalPtr cloud_data;
  if      (running_mode == RM_SHOW_MODEL)  cloud_data = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());
  else if (running_mode == RM_UNPROCESSED) cloud_data = input_data_processing_->calculateNormals (cloud_in);
  else if (running_mode >= RM_PROCESSED)   cloud_data = input_data_processing_->process (cloud_in);

  // Registration & integration
  if (running_mode >= RM_REGISTRATION_CONT)
  {
    if (iteration == 0)
    {
//      transformation_ = Eigen::Matrix4f::Identity ();
//      integration_->reconstructMesh (cloud_data, mesh_model_);
//      cloud_data = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());
//      ++iteration;
    }
    else
    {
//      Eigen::Matrix4f T = Eigen::Matrix4f::Identity ();
//      if (icp_->findTransformation (mesh_model_, cloud_data, transformation_, T))
//      {
//        transformation_ = T;
//        integration_->merge (cloud_data, mesh_model_, transformation_);
//        integration_->age (mesh_model_);
//        cloud_data = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());

//        // Does not work here because multiple threads!
//        // interactor_style_->transformCamera (InteractorStyle::Quaternion (T.topLeftCorner <3, 3> ().cast <double> ()), T.topRightCorner <3, 1> ().cast <double> ());

//        ++iteration_;
//      }
    }
  }

  {
    boost::mutex::scoped_lock lock_cv (mutex_comp_vis_);
    boost::mutex::scoped_lock lock_ce (mutex_comp_events_);
    cloud_draw_tmp_ = cloud_data;
    iteration_      = iteration;
    if (running_mode == RM_REGISTRATION_SINGLE)
    {
      running_mode_ = RM_PROCESSED;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

//vtkSmartPointer <vtkPolyData>
//pcl::ihs::InHandScanner::convertToPolyData (const CloudXYZRGBNormalConstPtr& cloud) const
//{
//  // test
//  std::cerr << "Converting a cloud to polydata in ";
//  pcl::StopWatch sw;
//  // end test
//  vtkSmartPointer <vtkPolyData>          poly_data = vtkPolyData::New ();
//  vtkSmartPointer <vtkPoints>            coords    = vtkPoints::New ();
//  vtkSmartPointer <vtkFloatArray>        normals   = vtkFloatArray::New ();
//  vtkSmartPointer <vtkUnsignedCharArray> colors    = vtkUnsignedCharArray::New ();

//  coords->SetNumberOfPoints (cloud->size ());

//  normals->SetNumberOfComponents (3);
//  normals->SetNumberOfTuples (cloud->size ());

//  colors->SetNumberOfComponents (3);
//  colors->SetNumberOfTuples (cloud->size ());
//  colors->SetName ("Colors");

//  for (unsigned int i=0; i<cloud->size (); ++i)
//  {
//    const PointXYZRGBNormal& p = (*cloud) [i];

//    coords->SetPoint   (i, p.x, p.y, p.z);
//    normals->SetTuple3 (i, p.normal_x, p.normal_y, p.normal_z);
//    colors->SetTuple3  (i, p.r, p.g,  p.b);
//  }

//  poly_data->SetPoints (coords);
//  poly_data->GetPointData ()->SetNormals (normals);
//  poly_data->GetPointData ()->SetScalars (colors);

//  // test
//  std::cerr << sw.getTimeSeconds ()*1000 << "ms" << std::endl;
//  // end test

//  return (poly_data);
//}

////////////////////////////////////////////////////////////////////////////////

//vtkSmartPointer <vtkPolyData>
//pcl::ihs::InHandScanner::convertToPolyData (const MeshConstPtr& mesh) const
//{
//  // test
//  std::cerr << "Converting a cloud to polydata in ";
//  pcl::StopWatch sw;
//  // end test

//  vtkSmartPointer <vtkPolyData>          poly_data = vtkPolyData::New ();
//  vtkSmartPointer <vtkPoints>            coords    = vtkPoints::New ();
//  vtkSmartPointer <vtkFloatArray>        normals   = vtkFloatArray::New ();
//  vtkSmartPointer <vtkUnsignedCharArray> colors    = vtkUnsignedCharArray::New ();
//  vtkSmartPointer <vtkIdList>            face      = vtkIdList::New ();
//  vtkSmartPointer <vtkCellArray>         faces     = vtkCellArray::New ();

//  // Set the points
//  const CloudIHS& cloud = mesh->getVertexDataCloud ();

//  coords->SetNumberOfPoints (cloud.size ());

//  normals->SetNumberOfComponents (3);
//  normals->SetNumberOfTuples (cloud.size ());

//  colors->SetNumberOfComponents (3);
//  colors->SetNumberOfTuples (cloud.size ());
//  colors->SetName ("Colors");

//  for (unsigned int i=0; i<cloud.size (); ++i)
//  {
//    const PointIHS& p = cloud [i];

//    coords->SetPoint   (i, p.x, p.y, p.z);
//    normals->SetTuple3 (i, p.normal_x, p.normal_y, p.normal_z);
//    colors->SetTuple3  (i, p.r, p.g,  p.b);
//  }

//  // Set the faces
//  // faces->SetNumberOfCells (mesh->sizeFaces ());
//  for (unsigned int i=0; i<mesh->sizeFaces (); ++i)
//  {
//    Mesh::VertexAroundFaceCirculator       circ     = mesh->getVertexAroundFaceCirculator (Mesh::FaceIndex (i));
//    const Mesh::VertexAroundFaceCirculator circ_end = circ;
//    face->Reset ();
//    do
//    {
//      face->InsertNextId (circ.getTargetIndex ().get ());
//    } while (++circ != circ_end);
//    faces->InsertNextCell (face); // Isn't there a faces->setCell (i, face) method?
//  }

//  poly_data->SetPoints (coords);
//  poly_data->SetPolys (faces);
//  poly_data->GetPointData ()->SetNormals (normals);
//  poly_data->GetPointData ()->SetScalars (colors);

//  // test
//  std::cerr << sw.getTimeSeconds ()*1000 << "ms" << std::endl;
//  // end test

//  return (poly_data);
//}

////////////////////////////////////////////////////////////////////////////////

QSize
pcl::ihs::InHandScanner::minimumSizeHint () const
{
  return (QSize (160, 120));
}

////////////////////////////////////////////////////////////////////////////////

QSize
pcl::ihs::InHandScanner::sizeHint () const
{
  return (QSize (640, 480));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::initializeGL ()
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::setupLighting ()
{
  // light 0 (directional)
  const GLfloat light0_ambient  [] = {0.2f, 0.2f, 0.2f, 1.0f};
  const GLfloat light0_diffuse  [] = {0.5f, 0.5f, 0.5f, 1.0f};
  const GLfloat light0_specular [] = {0.3f, 0.3f, 0.3f, 1.0f};
  GLfloat       light0_position [] = {0.5f, 0.3f, 0.1f, 0.0f};
  const GLfloat norm = light0_position [0] * light0_position [0] +
                       light0_position [1] * light0_position [1] +
                       light0_position [2] * light0_position [2];
  light0_position [0] /= norm;
  light0_position [1] /= norm;
  light0_position [2] /= norm;

  glLightfv (GL_LIGHT0, GL_AMBIENT , light0_ambient );
  glLightfv (GL_LIGHT0, GL_DIFFUSE , light0_diffuse );
  glLightfv (GL_LIGHT0, GL_SPECULAR, light0_specular);
  glLightfv (GL_LIGHT0, GL_POSITION, light0_position);

  // light 1 (positioned)
  const GLfloat light1_ambient  [] = {0.0f, 0.0f, 0.0f, 1.0f};
  const GLfloat light1_diffuse  [] = {0.8f, 0.8f, 0.8f, 1.0f};
  const GLfloat light1_specular [] = {0.0f, 0.0f, 0.0f, 1.0f};
  const GLfloat light1_position [] = {0.0f, 0.0f, 0.0f, 1.0f};

  glLightfv (GL_LIGHT1, GL_AMBIENT , light1_ambient );
  glLightfv (GL_LIGHT1, GL_DIFFUSE , light1_diffuse );
  glLightfv (GL_LIGHT1, GL_SPECULAR, light1_specular);
  glLightfv (GL_LIGHT1, GL_POSITION, light1_position);

  // Material
  const GLfloat material_specular [] = {0.5f, 0.5f, 0.5f, 1.0f};
  const GLfloat material_shininess   = 25.0f;

  glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glMaterialfv    (GL_FRONT, GL_SPECULAR , material_specular );
  glMaterialf     (GL_FRONT, GL_SHININESS, material_shininess);

  // Enable depth test, color and lighting
  glEnable (GL_COLOR_MATERIAL);
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  glEnable (GL_LIGHT1);
  glEnable (GL_NORMALIZE);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::setupViewport (const int w, const int h)
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
pcl::ihs::InHandScanner::resizeGL (int w, int h)
{
  this->setupViewport (w, h);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::paintEvent (QPaintEvent* /*event*/)
{
  {
    boost::mutex::scoped_lock lock_ve (mutex_vis_events_);
    this->calcFPS (visualization_fps_);
  }
  this->makeCurrent ();

  // Clear information from the last draw
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor (0.f, 0.f, 0.f, 0.f);

  this->setupLighting ();
  this->setupViewport (this->width (), this->height ());

  glEnable (GL_DEPTH_TEST);

  // Projection matrix
  glMatrixMode   (GL_PROJECTION);
  glLoadIdentity ();
  gluPerspective (43., 4./3., 1., 5000.);
  glMatrixMode   (GL_MODELVIEW);

  // Draw everything
  this->drawCloud ();
  this->drawCropBox ();
  this->drawText (); // NOTE: Must come AFTER the opengl calls
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::drawCloud ()
{
  boost::mutex::scoped_lock lock_cv (mutex_comp_vis_);

  // Copy from the other thread.
  if (cloud_draw_tmp_)
  {
    cloud_draw_.clear ();
    cloud_draw_.reserve (cloud_draw_tmp_->size ());

    Eigen::Vector4f centroid (0.f, 0.f, 0.f, 1.f);
    for (CloudXYZRGBNormal::iterator it=cloud_draw_tmp_->begin(); it!=cloud_draw_tmp_->end (); ++it)
    {
      if (pcl::isFinite (*it))
      {
        // Change from bgr to rgb format. A quick google search only showed how to set up OpenGL with BGR for textures but I didn't find anything for glColorPointer.
        // TODO: Maybe use shaders / colormap / lookup table instead.
        std::swap (it->r, it->b);

        cloud_draw_.push_back (*it);

        centroid += it->getVector4fMap ();
      }
    }
    cloud_draw_tmp_ = CloudXYZRGBNormalPtr ();

    if (cloud_draw_.size ())
    {
      boost::mutex::scoped_lock lock_ve (mutex_vis_events_);
      cam_pivot_.setX (centroid.x () / static_cast <float> (cloud_draw_.size ()));
      cam_pivot_.setY (centroid.y () / static_cast <float> (cloud_draw_.size ()));
      cam_pivot_.setZ (centroid.z () / static_cast <float> (cloud_draw_.size ()));
    }
  }
  lock_cv.unlock ();

  if (cloud_draw_.size ())
  {
    std::vector <float> vec; vec.reserve (3 * cloud_draw_.size ());
    std::vector <float> nor; nor.reserve (vec.size ());
    std::vector <float> rgb; rgb.reserve (vec.size ());
    for (unsigned int i=0; i<cloud_draw_.size (); ++i)
    {
      vec.push_back (cloud_draw_ [i].x);
      vec.push_back (cloud_draw_ [i].y);
      vec.push_back (cloud_draw_ [i].z);

      nor.push_back (cloud_draw_ [i].normal_x);
      nor.push_back (cloud_draw_ [i].normal_y);
      nor.push_back (cloud_draw_ [i].normal_z);

      rgb.push_back (static_cast <float> (cloud_draw_ [i].b) / 255.);
      rgb.push_back (static_cast <float> (cloud_draw_ [i].g) / 255.);
      rgb.push_back (static_cast <float> (cloud_draw_ [i].r) / 255.);
    }

    // View matrix
    QQuaternion cam_R;
    QVector3D   cam_t;
    {
      boost::mutex::scoped_lock lock_ve (mutex_vis_events_);
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
    glLoadIdentity();
    gluLookAt (eye.   x (), eye.   y (), eye.   z (),
               center.x (), center.y (), center.z (),
               up.    x (), up.    y (), up.    z ());

    // Draw
    // test
    glBegin (GL_POINTS);
    for (unsigned int i=0; i<vec.size (); i+=3)
    {
      glColor3f  (rgb [i], rgb [i+1], rgb [i+2]);
      glNormal3f (nor [i], nor [i+1], nor [i+2]);
      glVertex3f (vec [i], vec [i+1], vec [i+2]);
    }
    glEnd ();
    // end test

//    glEnableClientState (GL_VERTEX_ARRAY);
//    glEnableClientState (GL_COLOR_ARRAY);
//    glEnableClientState (GL_NORMAL_ARRAY);

//    glVertexPointer (3, GL_FLOAT        , sizeof (PointXYZRGBNormal), &(cloud_draw_ [0].x       ));
//    glColorPointer  (3, GL_UNSIGNED_BYTE, sizeof (PointXYZRGBNormal), &(cloud_draw_ [0].b       ));
//    glNormalPointer (   GL_FLOAT        , sizeof (PointXYZRGBNormal), &(cloud_draw_ [0].normal_x));

//    glDrawArrays (GL_POINTS, 0, cloud_draw_.size ());
//    glDrawElements (GL_TRIANGLES, 3 * t.size (), GL_UNSIGNED_INT, &t [0]);
  }
}

//{
//  // Copy from the other thread.
//  typedef std::vector <vtkSmartPointer <vtkPolyData> > PolyDataVec;

//  CloudXYZRGBNormalPtr cloud_temp;
//  MeshPtrVec           mesh_vec_temp;
//  if (mutex_comp_vis_.try_lock ())
//  {
//    cloud_temp.swap (cloud_draw_);

//    mesh_vec_temp.reserve (mesh_vec_draw_.size ());
//    for (MeshPtrVec::iterator it=mesh_vec_draw_.begin (); it<mesh_vec_draw_.end (); ++it)
//    {
//      MeshPtr mesh_temp;
//      mesh_temp.swap (*it);
//      mesh_vec_temp.push_back (mesh_temp);
//    }
//    mesh_vec_draw_.clear ();

//    mutex_comp_vis_.unlock ();
//  }

//  // Convert the cloud to poly data.
//  PolyDataVec poly_data_vec;
//  if (cloud_temp && !cloud_temp->empty ())
//  {
//    if (display_mode_ == DM_POINTS)
//    {
//      poly_data_vec.push_back (this->convertToPolyData (cloud_temp));
//    }
//    else
//    {
//      MeshPtr mesh_temp (new Mesh ());
//      integration_->reconstructMesh (cloud_temp, mesh_temp);
//      poly_data_vec.push_back (this->convertToPolyData (mesh_temp));
//    }
//  }

//  // Convert the meshes to poly data.
//  for (MeshPtrVec::const_iterator it_m=mesh_vec_temp.begin (); it_m!=mesh_vec_temp.end (); ++it_m)
//  {
//    if (*it_m && !(*it_m)->empty ())
//    {
//      poly_data_vec.push_back (this->convertToPolyData (*it_m));
//    }
//  }

//  // Remove the old data and draw the new data.
//  if (poly_data_vec.empty ())
//  {
//    return;
//  }

//  vtkSmartPointer <vtkRenderWindow> rw = visualizer_->getRenderWindow ();
//  if (!rw.GetPointer ())
//  {
//    std::cerr << "Bug in InHandScanner::draw: Could not get the render window!\n";
//    exit (EXIT_FAILURE);
//  }
//  vtkSmartPointer <vtkRenderer> renderer = rw->GetRenderers ()->GetFirstRenderer ();
//  if (!renderer.GetPointer ())
//  {
//    std::cerr << "Bug in InHandScanner::draw: Could not get the renderer!\n";
//    exit (EXIT_FAILURE);
//  }
//  for (Actors::const_iterator it=actors_.begin (); it!=actors_.end (); ++it)
//  {
//    renderer->RemoveActor (*it);
//  }
//  actors_.clear ();

//  for (PolyDataVec::const_iterator it=poly_data_vec.begin (); it!=poly_data_vec.end (); ++it)
//  {
//    vtkSmartPointer <vtkPolyDataMapper> mapper = vtkPolyDataMapper::New ();
//    mapper->SetInput (*it);

//    vtkSmartPointer <vtkActor> actor = vtkActor::New ();
//    actor->SetMapper (mapper);

//    switch (display_mode_)
//    {
//      case DM_POINTS: actor->GetProperty ()->SetRepresentationToPoints ();    break;
//      case DM_EDGES:  actor->GetProperty ()->SetRepresentationToWireframe (); break;
//      case DM_FACES:  actor->GetProperty ()->SetRepresentationToSurface ();   break;
//    }

//    actors_.push_back (actor);
//    renderer->AddActor (actor);
//  }
//}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::drawCropBox ()
{

}

//{
//  static bool crop_box_added = false;
//  if (draw_crop_box_ && !crop_box_added)
//  {
//    float x_min, x_max, y_min, y_max, z_min, z_max;
//    input_data_processing_->getCropBox (x_min, x_max, y_min, y_max, z_min, z_max);
//    visualizer_->addCube (x_min, x_max, y_min, y_max, z_min, z_max, 1., 1., 1., "crop_box");
//    crop_box_added = true;
//  }
//  else if (!draw_crop_box_ && crop_box_added)
//  {
//    visualizer_->removeShape ("crop_box");
//    crop_box_added = false;
//  }
//}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::drawText ()
{
  QPainter painter (this);
  painter.setPen (Qt::white);
  QFont font;

  if (starting_grabber_)
  {
    font.setPointSize (this->width () / 20);
    painter.setFont (font);
    painter.drawText (0, 0, this->width (), this->height (), Qt::AlignHCenter | Qt::AlignVCenter, "Starting the grabber.\n Please wait.");
  }
  else
  {
    std::string vis_fps ("Visualization: "), comp_fps ("Computation: ");
    {
      boost::mutex::scoped_lock lock_ve (mutex_vis_events_);
      boost::mutex::scoped_lock lock_ce (mutex_comp_events_);

      vis_fps.append  (visualization_fps_.str ()).append (" fps");
      comp_fps.append (computation_fps_.str   ()).append (" fps");
    }

    const std::string str = std::string (comp_fps).append ("\n").append (vis_fps);

    font.setPointSize (this->width () / 50);

    painter.setFont (font);
    painter.drawText (0, 0, this->width (), this->height (), Qt::AlignBottom | Qt::AlignLeft, str.c_str ());
  }
}
//{
//  std::string vis_fps ("Visualization: "), comp_fps ("Computation: ");
//  bool draw = false;

//  if (mutex_comp_vis_.try_lock ())
//  {
//    vis_fps.append (visualization_fps_.str ()).append (" fps");
//    comp_fps.append (computation_fps_.str ()).append (" fps");
//    draw = true;
//    mutex_comp_vis_.unlock ();
//  }

//  if (!draw) return;

//  if (!visualizer_->updateText (vis_fps, 1, 15, "visualization_fps"))
//  {
//    visualizer_->addText (vis_fps, 1, 15, "visualization_fps");
//  }

//  if (!visualizer_->updateText (comp_fps, 1, 30, "computation_fps"))
//  {
//    visualizer_->addText (comp_fps, 1, 30, "computation_fps");
//  }
//}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::mousePressEvent (QMouseEvent* /*event*/)
{
  boost::mutex::scoped_lock lock_ve (mutex_vis_events_);
  mouse_pressed_begin_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::mouseMoveEvent (QMouseEvent* event)
{
  boost::mutex::scoped_lock lock_ve (mutex_vis_events_);

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
pcl::ihs::InHandScanner::wheelEvent (QWheelEvent* event)
{
  if (QApplication::mouseButtons () == Qt::NoButton)
  {
    boost::mutex::scoped_lock lock_ve (mutex_vis_events_);

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

void
pcl::ihs::InHandScanner::keyPressEvent (QKeyEvent* event)
{
  // Don't allow keyboard callbacks when the grabber is starting up.
  if (starting_grabber_) return;

  DisplayMode mode;
  {
    boost::mutex::scoped_lock lock_ve (mutex_vis_events_);
    mode = display_mode_;
  }

  switch (event->key ())
  {
    case Qt::Key_H:
    {
      std::cerr << "======================================================================\n"
                << "Help:\n"
                << "----------------------------------------------------------------------\n"
                << "ESC: Quit the application.\n"
                << "----------------------------------------------------------------------\n"
                << "1  : Shows the unprocessed input data.\n"
                << "2  : Shows the processed input data.\n"
                << "3  : Registers new data to the first acquired data continuously.\n"
                << "4  : Registers new data once and returns to '2'.\n"
                << "5  : Shows the model shape.\n"
                << "0  : Reset the scanning pipeline.\n"
                << "----------------------------------------------------------------------\n"
                << "c  : Reset the camera.\n"
                << "s  : Switch representation between points, edges and a faces.\n"
                << "======================================================================\n";
      break;
    }
    case Qt::Key_Escape: QApplication::quit ();                    break;
    case Qt::Key_1: this->setRunningMode (RM_UNPROCESSED);         break;
    case Qt::Key_2: this->setRunningMode (RM_PROCESSED);           break;
    case Qt::Key_3: this->setRunningMode (RM_REGISTRATION_CONT);   break;
    case Qt::Key_4: this->setRunningMode (RM_REGISTRATION_SINGLE); break;
    case Qt::Key_5: this->setRunningMode (RM_SHOW_MODEL);          break;
    case Qt::Key_0: this->reset ();                                break;
    case Qt::Key_C: this->resetCamera ();                          break;
    case Qt::Key_S:
    {
      switch (mode)
      {
        case DM_POINTS: this->setDisplayMode (DM_EDGES);  break;
        case DM_EDGES:  this->setDisplayMode (DM_FACES);  break;
        case DM_FACES:  this->setDisplayMode (DM_POINTS); break;
      }                                                   break;
    }
    default: break;
  }
}

////////////////////////////////////////////////////////////////////////////////

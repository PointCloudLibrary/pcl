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

#include <QApplication>
#include <QtCore>
#include <QKeyEvent>
#include <QPainter>
#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
#include <QtConcurrent/QtConcurrent>
#endif

#include <pcl/exceptions.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/geometry/get_boundary.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/apps/in_hand_scanner/icp.h>
#include <pcl/apps/in_hand_scanner/input_data_processing.h>
#include <pcl/apps/in_hand_scanner/integration.h>
#include <pcl/apps/in_hand_scanner/mesh_processing.h>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InHandScanner::InHandScanner (Base* parent)
  : Base                   (parent),
    mutex_                 (),
    computation_fps_       (),
    visualization_fps_     (),
    running_mode_          (RM_UNPROCESSED),
    iteration_             (0),
    grabber_               (),
    starting_grabber_      (false),
    new_data_connection_   (),
    input_data_processing_ (new InputDataProcessing ()),
    icp_                   (new ICP ()),
    transformation_        (Eigen::Matrix4f::Identity ()),
    integration_           (new Integration ()),
    mesh_processing_       (new MeshProcessing ()),
    mesh_model_            (new Mesh ()),
    destructor_called_     (false)
{
  // http://doc.qt.digia.com/qt/qmetatype.html#qRegisterMetaType
  qRegisterMetaType <pcl::ihs::InHandScanner::RunningMode> ("RunningMode");

  Base::setScalingFactor (0.01);

  // Initialize the pivot
  const float x_min = input_data_processing_->getXMin ();
  const float x_max = input_data_processing_->getXMax ();
  const float y_min = input_data_processing_->getYMin ();
  const float y_max = input_data_processing_->getYMax ();
  const float z_min = input_data_processing_->getZMin ();
  const float z_max = input_data_processing_->getZMax ();

  Base::setPivot (Eigen::Vector3d ((x_min + x_max) / 2., (y_min + y_max) / 2., (z_min + z_max) / 2.));
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InHandScanner::~InHandScanner ()
{
  boost::mutex::scoped_lock lock (mutex_);
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
pcl::ihs::InHandScanner::showUnprocessedData ()
{
  boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  std::cerr << "Showing the unprocessed input data.\n";
  Base::setDrawBox (false);
  Base::setColoring (Base::COL_RGB);

  running_mode_ = RM_UNPROCESSED;
  emit runningModeChanged (running_mode_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::showProcessedData ()
{
  boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  std::cerr << "Showing the processed input data.\n";
  Base::setDrawBox (true);
  Base::setColoring (Base::COL_RGB);

  running_mode_ = RM_PROCESSED;
  emit runningModeChanged (running_mode_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::registerContinuously ()
{
  boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  std::cerr << "Continuous registration.\n";
  Base::setDrawBox (true);
  Base::setColoring (Base::COL_VISCONF);

  running_mode_ = RM_REGISTRATION_CONT;
  emit runningModeChanged (running_mode_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::registerOnce ()
{
  boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  std::cerr << "Single registration.\n";
  Base::setDrawBox (true);

  running_mode_ = RM_REGISTRATION_SINGLE;
  emit runningModeChanged (running_mode_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::showModel ()
{
  boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  std::cerr << "Show the model\n";
  Base::setDrawBox (false);

  running_mode_ = RM_SHOW_MODEL;
  emit runningModeChanged (running_mode_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::removeUnfitVertices ()
{
  boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  std::cerr << "Removing unfit vertices ...\n";

  integration_->removeUnfitVertices (mesh_model_);
  if (mesh_model_->emptyVertices ())
  {
    std::cerr << "Mesh got empty -> Reset\n";
    lock.unlock ();
    this->reset ();
  }
  else
  {
    lock.unlock ();
    this->showModel ();
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::reset ()
{
  boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  std::cerr << "Reset the scanning pipeline.\n";

  mesh_model_->clear ();
  Base::removeAllMeshes ();

  iteration_      = 0;
  transformation_ = Eigen::Matrix4f::Identity ();

  lock.unlock ();
  this->showUnprocessedData ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::saveAs (const std::string& filename, const FileType& filetype)
{
  boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  pcl::PolygonMesh pm;
  pcl::geometry::toFaceVertexMesh (*mesh_model_, pm);

  switch (filetype)
  {
    case FT_PLY: pcl::io::savePLYFile (filename, pm); break;
    case FT_VTK: pcl::io::saveVTKFile (filename, pm); break;
    default:                                          break;
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::keyPressEvent (QKeyEvent* event)
{
  // Don't allow keyboard callbacks while the grabber is starting up.
  if (starting_grabber_)  return;
  if (destructor_called_) return;

  switch (event->key ())
  {
    case Qt::Key_1: this->showUnprocessedData ();      break;
    case Qt::Key_2: this->showProcessedData ();        break;
    case Qt::Key_3: this->registerContinuously ();     break;
    case Qt::Key_4: this->registerOnce ();             break;
    case Qt::Key_5: this->showModel ();                break;
    case Qt::Key_6: this->removeUnfitVertices ();      break;
    case Qt::Key_0: this->reset ();                    break;
    case Qt::Key_C: Base::resetCamera ();              break;
    case Qt::Key_K: Base::toggleColoring ();           break;
    case Qt::Key_S: Base::toggleMeshRepresentation (); break;
    default:                                           break;
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::newDataCallback (const CloudXYZRGBAConstPtr& cloud_in)
{
  Base::calcFPS (computation_fps_); // Must come before the lock!

  boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  pcl::StopWatch sw;

  // Input data processing
  CloudXYZRGBNormalPtr cloud_data;
  CloudXYZRGBNormalPtr cloud_discarded;
  if (running_mode_ == RM_SHOW_MODEL)
  {
    cloud_data = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());
  }
  else if (running_mode_ == RM_UNPROCESSED)
  {
    if (!input_data_processing_->calculateNormals (cloud_in, cloud_data))
      return;
  }
  else if (running_mode_ >= RM_PROCESSED)
  {
    if (!input_data_processing_->segment (cloud_in, cloud_data, cloud_discarded))
      return;
  }

  double time_input_data_processing = sw.getTime ();

  // Registration & integration
  if (running_mode_ >= RM_REGISTRATION_CONT)
  {
    std::cerr << "\nGlobal iteration " << iteration_ << "\n";
    std::cerr << "Input data processing:\n"
              << "  - time                           : "
              << std::setw (8) << std::right << time_input_data_processing << " ms\n";

    if (iteration_ == 0)
    {
      transformation_ = Eigen::Matrix4f::Identity ();

      sw.reset ();
      integration_->reconstructMesh (cloud_data, mesh_model_);
      std::cerr << "Integration:\n"
                << "  - time reconstruct mesh          : "
                << std::setw (8) << std::right << sw.getTime () << " ms\n";

      cloud_data = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());
      ++iteration_;
    }
    else
    {
      Eigen::Matrix4f T_final = Eigen::Matrix4f::Identity ();
      if (icp_->findTransformation (mesh_model_, cloud_data, transformation_, T_final))
      {
        transformation_ = T_final;

        sw.reset ();
        integration_->merge (cloud_data, mesh_model_, transformation_);
        std::cerr << "Integration:\n"
                  << "  - time merge                     : "
                  << std::setw (8) << std::right << sw.getTime () << " ms\n";

        sw.reset ();
        integration_->age (mesh_model_);
        std::cerr << "  - time age                       : "
                  << std::setw (8) << std::right << sw.getTime () << " ms\n";

        sw.reset ();
        std::vector <Mesh::HalfEdgeIndices> boundary_collection;
        pcl::geometry::getBoundBoundaryHalfEdges (*mesh_model_, boundary_collection, 1000);
        std::cerr << "  - time compute boundary          : "
                  << std::setw (8) << std::right << sw.getTime () << " ms\n";

        sw.reset ();
        mesh_processing_->processBoundary (*mesh_model_, boundary_collection);
        std::cerr << "  - time mesh processing           : "
                  << std::setw (8) << std::right << sw.getTime () << " ms\n";

        cloud_data = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());
        ++iteration_;
      }
    }
  }

  // Visualization & copy back some variables
  double time_model = 0;
  double time_data  = 0;

  if (mesh_model_->empty ()) Base::setPivot ("data");
  else                       Base::setPivot ("model");

  sw.reset ();
  Base::addMesh (mesh_model_, "model", Eigen::Isometry3d (transformation_.inverse ().cast <double> ()));
  time_model = sw.getTime ();

  sw.reset ();
  Base::addMesh (cloud_data , "data"); // Converts to a mesh for visualization

  if (running_mode_ < RM_REGISTRATION_CONT && cloud_discarded)
  {
    Base::addMesh (cloud_discarded, "cloud_discarded");
  }
  else
  {
    Base::removeMesh ("cloud_discarded");
  }
  time_data = sw.getTime ();

  if (running_mode_ >= RM_REGISTRATION_CONT)
  {
    std::cerr << "Copy to visualization thread:\n"
              << "  - time model                     : "
              << std::setw (8) << std::right << time_model << " ms\n"
              << "  - time data                      : "
              << std::setw (8) << std::right << time_data << " ms\n";
  }

  if (running_mode_ == RM_REGISTRATION_SINGLE)
  {
    lock.unlock ();
    this->showProcessedData ();
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::paintEvent (QPaintEvent* event)
{
  // boost::mutex::scoped_lock lock (mutex_);
  if (destructor_called_) return;

  Base::calcFPS (visualization_fps_);
  Base::BoxCoefficients coeffs (input_data_processing_->getXMin (),
                                input_data_processing_->getXMax (),
                                input_data_processing_->getYMin (),
                                input_data_processing_->getYMax (),
                                input_data_processing_->getZMin (),
                                input_data_processing_->getZMax (),
                                Eigen::Isometry3d::Identity ());
  Base::setBoxCoefficients (coeffs);

  Base::setVisibilityConfidenceNormalization (static_cast <float> (integration_->getMinDirections ()));
  // lock.unlock ();

  Base::paintEvent (event);
  this->drawText (); // NOTE: Must come AFTER the opengl calls
}

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

    vis_fps.append  (visualization_fps_.str ()).append (" fps");
    comp_fps.append (computation_fps_.str   ()).append (" fps");

    const std::string str = std::string (comp_fps).append ("\n").append (vis_fps);

    font.setPointSize (this->width () / 50);

    painter.setFont (font);
    painter.drawText (0, 0, this->width (), this->height (), Qt::AlignBottom | Qt::AlignLeft, str.c_str ());
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::startGrabberImpl ()
{
  boost::mutex::scoped_lock lock (mutex_);
  starting_grabber_ = true;
  lock.unlock ();

  try
  {
    grabber_ = GrabberPtr (new Grabber ());
  }
  catch (const pcl::PCLException& e)
  {
    std::cerr << "ERROR in in_hand_scanner.cpp: " << e.what () << std::endl;
    exit (EXIT_FAILURE);
  }

  lock.lock ();
  if (destructor_called_) return;

  boost::function <void (const CloudXYZRGBAConstPtr&)> new_data_cb = boost::bind (&pcl::ihs::InHandScanner::newDataCallback, this, _1);
  new_data_connection_ = grabber_->registerCallback (new_data_cb);
  grabber_->start ();

  starting_grabber_ = false;
}

////////////////////////////////////////////////////////////////////////////////

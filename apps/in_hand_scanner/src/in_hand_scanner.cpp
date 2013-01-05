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
#include <QKeyEvent>
#include <QPainter>
#include <QtCore>

#include <pcl/common/transforms.h>
#include <pcl/exceptions.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/apps/in_hand_scanner/icp.h>
#include <pcl/apps/in_hand_scanner/input_data_processing.h>
#include <pcl/apps/in_hand_scanner/integration.h>

////////////////////////////////////////////////////////////////////////////////
// InHandScanner
////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InHandScanner::InHandScanner (Base* parent)
  : Base                   (parent),
    mutex_ihs_             (),
    computation_fps_       (),
    visualization_fps_     (),
    reset_                 (false),
    running_mode_          (RM_UNPROCESSED),
    iteration_             (0),
    grabber_               (),
    starting_grabber_      (false),
    new_data_connection_   (),
    input_data_processing_ (new InputDataProcessing ()),
    icp_                   (new ICP ()),
    transformation_        (Eigen::Matrix4f::Identity ()),
    integration_           (new Integration ()),
    mesh_model_            (new Mesh ()),
    destructor_called_     (false)
{
  // Initialize the pivot and crop box
  float x_min, x_max, y_min, y_max, z_min, z_max;
  input_data_processing_->getCropBox (x_min, x_max, y_min, y_max, z_min, z_max);

  Base::setPivot ((x_min + x_max) / 2., (y_min + y_max) / 2., (z_min + z_max) / 2.);
  Base::setCubeCoefficients (Base::CubeCoefficients (x_min, x_max, y_min, y_max, z_min, z_max));
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::InHandScanner::~InHandScanner ()
{
  boost::mutex::scoped_lock lock (mutex_ihs_);
  destructor_called_ = true;

  if (grabber_ && grabber_->isRunning ()) grabber_->stop ();
  if (new_data_connection_.connected ())  new_data_connection_.disconnect ();
}

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
pcl::ihs::InHandScanner::startGrabber ()
{
  QtConcurrent::run (boost::bind (&pcl::ihs::InHandScanner::startGrabberImpl, this));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::setRunningMode (const RunningMode& mode)
{
  switch (mode)
  {
    case RM_SHOW_MODEL:
    {
      Base::disableDrawCube ();
      std::cerr << "Show the model\n";
      break;
    }
    case RM_UNPROCESSED:
    {
      Base::disableDrawCube ();
      std::cerr << "Showing the unprocessed input data.\n";
      break;
    }
    case RM_PROCESSED:
    {
      Base::enableDrawCube ();
      std::cerr << "Showing the processed input data.\n";
      break;
    }
    case RM_REGISTRATION_CONT:
    {
      Base::enableDrawCube ();
      std::cerr << "Continuous registration.\n";
      break;
    }
    case RM_REGISTRATION_SINGLE:
    {
      Base::enableDrawCube ();
      std::cerr << "Single registration.\n";
      break;
    }
    default:
    {
      std::cerr << "ERROR in in_hand_scanner.cpp: Unknown command!\n";
      return;
    }
  }

  boost::mutex::scoped_lock lock (mutex_ihs_);
  running_mode_ = mode;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::reset ()
{
  boost::mutex::scoped_lock lock (mutex_ihs_);

  // The actual reset is done in newDataCallback
  reset_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::newDataCallback (const CloudXYZRGBAConstPtr& cloud_in)
{
  RunningMode running_mode;
  unsigned int iteration;
  {
    boost::mutex::scoped_lock lock (mutex_ihs_);

    if (destructor_called_)
    {
      return;
    }

    if (reset_)
    {
      std::cerr << "Reset the scanning pipeline.\n";

      mesh_model_->clear ();
      running_mode_    = RM_UNPROCESSED;
      iteration_       = 0;
      transformation_  = Eigen::Matrix4f::Identity ();

      Base::resetCamera ();
      Base::removeAllClouds ();
      Base::removeAllMeshes ();

      reset_ = false;
    }

    running_mode   = running_mode_;
    iteration      = iteration_;
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
    std::cerr << "\nGlobal iteration " << iteration_ << "\n";
    if (iteration == 0)
    {
      transformation_ = Eigen::Matrix4f::Identity ();
      integration_->reconstructMesh (cloud_data, mesh_model_);
      cloud_data = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());
      ++iteration;
    }
    else
    {
      Eigen::Matrix4f T_final = Eigen::Matrix4f::Identity ();
      if (icp_->findTransformation (mesh_model_, cloud_data, transformation_, T_final))
      {
        transformation_ = T_final;
        integration_->merge (cloud_data, mesh_model_, transformation_);
        integration_->age (mesh_model_);
        cloud_data = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());

        Base::setObjectTransformation (transformation_.inverse ());

        ++iteration;
      }
    }
  }

  // Visualization & copy back some variables
  {
    boost::mutex::scoped_lock lock (mutex_ihs_);

    if (destructor_called_)
    {
      return;
    }

    if (mesh_model_->empty ()) Base::setPivot ("data");
    else                       Base::setPivot ("model");

    Base::addMesh (cloud_data , "data"); // Converts to a mesh for visualization
    Base::addMesh (mesh_model_, "model");

    iteration_ = iteration;

    if (running_mode == RM_REGISTRATION_SINGLE)
    {
      running_mode_ = RM_PROCESSED;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::InHandScanner::paintEvent (QPaintEvent* event)
{
  {
    boost::mutex::scoped_lock lock (mutex_ihs_);
    this->calcFPS (visualization_fps_);
  }

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
    {
      boost::mutex::scoped_lock lock (mutex_ihs_);

      vis_fps.append  (visualization_fps_.str ()).append (" fps");
      comp_fps.append (computation_fps_.str   ()).append (" fps");
    }

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
  boost::mutex::scoped_lock lock (mutex_ihs_);
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

void
pcl::ihs::InHandScanner::keyPressEvent (QKeyEvent* event)
{
  // Don't allow keyboard callbacks when the grabber is starting up.
  if (starting_grabber_) return;

  if (event->key () == Qt::Key_Escape)
  {
    boost::mutex::scoped_lock lock (mutex_ihs_);
    QApplication::quit ();
    return;
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
                << "s  : Toggle the mesh representation between points and faces.\n"
                << "======================================================================\n";
      break;
    }
    case Qt::Key_1: this->setRunningMode (RM_UNPROCESSED);         break;
    case Qt::Key_2: this->setRunningMode (RM_PROCESSED);           break;
    case Qt::Key_3: this->setRunningMode (RM_REGISTRATION_CONT);   break;
    case Qt::Key_4: this->setRunningMode (RM_REGISTRATION_SINGLE); break;
    case Qt::Key_5: this->setRunningMode (RM_SHOW_MODEL);          break;
    case Qt::Key_0: this->reset ();                                break;
    case Qt::Key_C: Base::resetCamera ();                          break;
    case Qt::Key_S: Base::toggleDisplayMode ();                    break;
    default:                                                       break;
  }
}

////////////////////////////////////////////////////////////////////////////////

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

#include <pcl/apps/in_hand_scanner/offline_integration.h>

#include <iomanip>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <QApplication>
#include <QFileDialog>
#include <QtCore>
#include <QKeyEvent>
#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
#include <QtConcurrent/QtConcurrent>
#endif

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/apps/in_hand_scanner/integration.h>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::OfflineIntegration::OfflineIntegration (Base* parent)
  : Base               (parent),
    mutex_             (),
    mutex_quit_        (),
    computation_fps_   (),
    visualization_fps_ (),
    path_dir_          (),
    mesh_model_        (new Mesh ()),
    normal_estimation_ (new NormalEstimation ()),
    integration_       (new Integration ()),
    destructor_called_ (false)
{
  normal_estimation_->setNormalEstimationMethod (NormalEstimation::AVERAGE_3D_GRADIENT);
  normal_estimation_->setMaxDepthChangeFactor (0.02f); // in meters
  normal_estimation_->setNormalSmoothingSize (10.0f);

  integration_->setMaxSquaredDistance (1e-4); // in m^2
  integration_->setMinDirections (2);


  Base::setVisibilityConfidenceNormalization (static_cast <float> (integration_->getMinDirections ()));
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::OfflineIntegration::~OfflineIntegration ()
{
  boost::mutex::scoped_lock lock (mutex_);
  boost::mutex::scoped_lock lock_quit (mutex_quit_);
  destructor_called_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OfflineIntegration::start ()
{
  QString dir = QFileDialog::getExistingDirectory (0, "Please select a directory containing .pcd and .transform files.");

  if (dir.isEmpty ())
  {
    return (QApplication::quit ());
  }

  path_dir_ = dir.toStdString ();
  QtConcurrent::run (boost::bind (&pcl::ihs::OfflineIntegration::computationThread, this));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OfflineIntegration::computationThread ()
{
  std::vector <std::string> filenames;

  if (!this->getFilesFromDirectory (path_dir_, ".pcd", filenames))
  {
    std::cerr << "ERROR in offline_integration.cpp: Could not get the files from the directory\n";
    return;
  }

  // First cloud is reference model
  std::cerr << "Processing file " << std::setw (5) << 1 << " / " << filenames.size () << std::endl;
  CloudXYZRGBNormalPtr cloud_model (new CloudXYZRGBNormal ());
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity ();
  if (!this->load (filenames [0], cloud_model, T))
  {
    std::cerr << "ERROR in offline_integration.cpp: Could not load the model cloud.\n";
    return;
  }

  pcl::transformPointCloudWithNormals (*cloud_model, *cloud_model, T);

  if (!integration_->reconstructMesh (cloud_model, mesh_model_))
  {
    std::cerr << "ERROR in offline_integration.cpp: Could not reconstruct the model mesh.\n";
    return;
  }

  Base::setPivot ("model");
  Base::addMesh (mesh_model_, "model");

  if (filenames.size () < 1)
  {
    return;
  }

  for (unsigned int i=1; i<filenames.size (); ++i)
  {
    std::cerr << "Processing file " << std::setw (5) << i+1 << " / " << filenames.size () << std::endl;

    {
      boost::mutex::scoped_lock lock (mutex_);
      if (destructor_called_) return;
    }
    boost::mutex::scoped_lock lock_quit (mutex_quit_);

    CloudXYZRGBNormalPtr cloud_data (new CloudXYZRGBNormal ());
    if (!this->load (filenames [i], cloud_data, T))
    {
      std::cerr << "ERROR in offline_integration.cpp: Could not load the data cloud.\n";
      return;
    }

    if (!integration_->merge (cloud_data, mesh_model_, T))
    {
      std::cerr << "ERROR in offline_integration.cpp: merge failed.\n";
      return;
    }

    integration_->age (mesh_model_);

    {
      lock_quit.unlock ();
      boost::mutex::scoped_lock lock (mutex_);
      if (destructor_called_) return;

      Base::addMesh (mesh_model_, "model", Eigen::Isometry3d (T.inverse ().cast <double> ()));
      Base::calcFPS (computation_fps_);
    }
  }
  Base::setPivot ("model");
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OfflineIntegration::getFilesFromDirectory (const std::string          path_dir,
                                                     const std::string          extension,
                                                     std::vector <std::string>& files) const
{
  if (path_dir == "" || !boost::filesystem::exists (path_dir))
  {
    std::cerr << "ERROR in offline_integration.cpp: Invalid path\n  '" << path_dir << "'\n";
    return (false);
  }

  boost::filesystem::directory_iterator it_end;
  for (boost::filesystem::directory_iterator it (path_dir); it != it_end; ++it)
  {
    if (!is_directory (it->status ()) &&
        boost::algorithm::to_upper_copy (boost::filesystem::extension (it->path ())) == boost::algorithm::to_upper_copy (extension))
    {
      files.push_back (it->path ().string ());
    }
  }

  if (files.empty ())
  {
    std::cerr << "ERROR in offline_integration.cpp: No '" << extension << "' files found\n";
    return (false);
  }

  sort (files.begin (), files.end ());

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OfflineIntegration::loadTransform (const std::string& filename,
                                             Eigen::Matrix4f&   transform) const
{
 Eigen::Matrix4d tr;
 std::ifstream file;
 file.open (filename.c_str (), std::ios::binary);

 if (!file.is_open ())
 {
   std::cerr << "Error in offline_integration.cpp: Could not open the file '" << filename << "'\n";
   return (false);
 }

 for (int i = 0; i < 4; ++i)
 {
   for (int j = 0; j < 4; ++j)
   {
     file.read (reinterpret_cast<char*>(&tr (i, j)), sizeof (double));

     if (!file.good ())
     {
       std::cerr << "Error in offline_integration.cpp: Could not read the transformation from the file.\n";
       return (false);
     }
   }
 }

 transform = tr.cast<float> ();

 return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OfflineIntegration::load (const std::string&    filename,
                                    CloudXYZRGBNormalPtr& cloud,
                                    Eigen::Matrix4f&      T) const
{
  if (!cloud)
  {
    cloud = CloudXYZRGBNormalPtr (new CloudXYZRGBNormal ());
  }

  // Load the cloud.
  CloudXYZRGBAPtr cloud_input (new CloudXYZRGBA ());

  pcl::PCDReader reader;
  if (reader.read (filename, *cloud_input) < 0)
  {
    std::cerr << "ERROR in offline_integration.cpp: Could not read the pcd file.\n";
    return (false);
  }

  // Normal estimation.
  normal_estimation_->setInputCloud (cloud_input);
  normal_estimation_->compute (*cloud);

  pcl::copyPointCloud (*cloud_input, *cloud);

  // Change the file extension of the file.
  // Load the transformation.
  std::string fn_transform = filename;

  size_t pos = fn_transform.find_last_of (".");
  if (pos == std::string::npos || pos == (fn_transform.size () - 1))
  {
    std::cerr << "ERROR in offline_integration.cpp: No file extension\n";
    return (false);
  }

  fn_transform.replace (pos, std::string::npos, ".transform");

  if (!this->loadTransform (fn_transform, T))
  {
    std::cerr << "ERROR in offline_integration.cpp: Could not load the transformation.\n";
    return (false);
  }

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OfflineIntegration::paintEvent (QPaintEvent* event)
{
  Base::paintEvent (event);

  QPainter painter (this);
  painter.setPen (Qt::white);
  QFont font;
  font.setPointSize (this->width () / 50);
  painter.setFont (font);

  std::string vis_fps ("Visualization: "), comp_fps ("Computation: ");
  {
    boost::mutex::scoped_lock lock (mutex_);
    this->calcFPS (visualization_fps_);

    vis_fps.append  (visualization_fps_.str ()).append (" fps");
    comp_fps.append (computation_fps_.str   ()).append (" fps");
  }

  const std::string str = std::string (comp_fps).append ("\n").append (vis_fps);

  painter.drawText (0, 0, this->width (), this->height (), Qt::AlignBottom | Qt::AlignLeft, str.c_str ());
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OfflineIntegration::keyPressEvent (QKeyEvent* event)
{
  if (event->key () == Qt::Key_Escape)
  {
    boost::mutex::scoped_lock lock (mutex_);
    QApplication::quit ();
  }

  switch (event->key ())
  {
    case Qt::Key_H:
    {
      std::cerr << "======================================================================\n"
                << "Help:\n"
                << "----------------------------------------------------------------------\n"
                << "ESC: Quit the application.\n"
                << "c  : Reset the camera.\n"
                << "k  : Toggle the coloring (rgb, one color, visibility-confidence).\n"
                << "s  : Toggle the mesh representation between points and faces.\n"
                << "======================================================================\n";
      break;
    }
    case Qt::Key_C: Base::resetCamera ();              break;
    case Qt::Key_K: Base::toggleColoring ();           break;
    case Qt::Key_S: Base::toggleMeshRepresentation (); break;
    default:                                           break;
  }
}

////////////////////////////////////////////////////////////////////////////////

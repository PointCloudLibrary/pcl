/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *	
 */

#include <pcl/proctor/detector_visualizer.h>
// QT4
#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>
#include <QListWidgetItem>
// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DetectorVisualizer::DetectorVisualizer () : cloud_pass_()
{
  // Create a timer and fire it up every 5ms
  vis_timer_ = new QTimer (this);
  vis_timer_->start (5);

  connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot ()));

  ui_ = new Ui::MainWindow;
  ui_->setupUi (this);

  this->setWindowTitle ("Proctor: Detector Visualizer");
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  ui_->qvtk_widget->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (ui_->qvtk_widget->GetInteractor (), ui_->qvtk_widget->GetRenderWindow ());
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  ui_->qvtk_widget->update (); 

  // Start the OpenNI data acquision
  boost::function<void (const CloudConstPtr&)> f = boost::bind (&DetectorVisualizer::cloud_cb, this, _1);

  //ui_->fieldValueSlider->setRange (5, 50);
  //ui_->fieldValueSlider->setValue (50);
  //connect (ui_->fieldValueSlider, SIGNAL (valueChanged (int)), this, SLOT (adjustPassThroughValues (int)));
  //
  //ui_->tabWidget->tab_training;
  connect (ui_->modelListWidget, SIGNAL (currentItemChanged(QListWidgetItem *, QListWidgetItem *)), this, SLOT (modelSelectionChanged(QListWidgetItem *, QListWidgetItem *)));

  cloud_pass_.reset (new Cloud());
  updated = false;
}

bool
DetectorVisualizer::addCloud(std::string id, CloudPtr cloud)
{
  QMutexLocker locker (&mtx_);
  if (cloud_database.count(id) == 0) {
    cloud_database[id] = cloud;
    new QListWidgetItem(tr(id.c_str()), ui_->modelListWidget);
    return true;
  } else {
    return false;
  }
}

bool
DetectorVisualizer::showCloud(std::string id)
{
  QMutexLocker locker (&mtx_);
  if (cloud_database.count(id) > 0) {
    // TODO Figure out if this is allowed (non main thread)
    cloud_pass_ = cloud_database[id];
    updated = true;
    return true;
  }

  return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void
DetectorVisualizer::cloud_cb (const CloudConstPtr& cloud)
{
  QMutexLocker locker (&mtx_);  
  FPS_CALC ("computation");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void
DetectorVisualizer::timeoutSlot ()
{
  QMutexLocker locker (&mtx_);
  if (!cloud_pass_)
  {
    boost::this_thread::sleep (boost::posix_time::milliseconds (1));
    return;
  }

  if (updated) {
    updated = false;
    vis_->removePointCloud("cloud_pass");
    vis_->removePointCloud("cloud_pass_normals");

    vis_->addPointCloud <pcl::PointNormal> (cloud_pass_, "cloud_pass");
    vis_->addPointCloudNormals <pcl::PointNormal> (cloud_pass_, 50, 0.02, "cloud_pass_normals");
    vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud_pass_normals");
    vis_->addCoordinateSystem (1.0);
    int num_x = 5;
    int num_y = 5;
    int num_z = 5;

    //double width = 0.2;
    //double height = 0.2;
    //double depth = 0.2;

    for (int i = 0; i < num_x; i++)
    {
      for (int j = 0; j < num_y; j++)
      {
        for (int k = 0; k < num_z; k++)
        {
          //std::stringstream stream;
          //stream << "cube" << i << j << k;
          //std::string id = stream.str();
          //cout << "ID: " << id << endl;
          //double opacity = float(i + j + k) / (num_x + num_y + num_z) ;
          //vis_->addCube(Eigen::Vector3f(i * width, j * height, k * depth), Eigen::Quaternionf(), opacity *  width - 0.01, opacity * height - 0.01, opacity * depth - 0.01, id);
          //vis_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, id);
          //vis_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
          //vis_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);
        }
      }
    }

    vis_->resetCameraViewpoint ("cloud_pass");
  }

  ui_->qvtk_widget->update ();
}

void
DetectorVisualizer::modelSelectionChanged (QListWidgetItem *current, QListWidgetItem *previous)
{
  std::string id = current->text().toStdString();
  if (cloud_database.count(id) > 0) {
    showCloud(current->text().toStdString());
  }
  else
  {
    assert(false);
  }
}


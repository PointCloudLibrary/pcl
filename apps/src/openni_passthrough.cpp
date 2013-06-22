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

#include <pcl/apps/openni_passthrough.h>
// QT4
#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>
// PCL
#include <pcl/console/parse.h>

#include <vtkRenderWindow.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OpenNIPassthrough::OpenNIPassthrough (pcl::OpenNIGrabber& grabber) 
  : vis_ ()
  , grabber_(grabber)
  , device_id_ ()
  , cloud_pass_()
  , pass_ ()
  , mtx_ ()
  , ui_ (new Ui::MainWindow)
  , vis_timer_ (new QTimer (this))
{
  // Create a timer and fire it up every 5ms
  vis_timer_->start (5);

  connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot ()));

  ui_->setupUi (this);

  this->setWindowTitle ("PCL OpenNI PassThrough Viewer");
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  ui_->qvtk_widget->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (ui_->qvtk_widget->GetInteractor (), ui_->qvtk_widget->GetRenderWindow ());
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  ui_->qvtk_widget->update (); 

  // Start the OpenNI data acquision
  boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIPassthrough::cloud_cb, this, _1);
  boost::signals2::connection c = grabber_.registerCallback (f);

  grabber_.start ();

  // Set defaults
  pass_.setFilterFieldName ("z");
  pass_.setFilterLimits (0.5, 5.0);
  
  ui_->fieldValueSlider->setRange (5, 50);
  ui_->fieldValueSlider->setValue (50);
  connect (ui_->fieldValueSlider, SIGNAL (valueChanged (int)), this, SLOT (adjustPassThroughValues (int)));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void
OpenNIPassthrough::cloud_cb (const CloudConstPtr& cloud)
{
  QMutexLocker locker (&mtx_);  
  FPS_CALC ("computation");

  // Computation goes here
  cloud_pass_.reset (new Cloud);
  pass_.setInputCloud (cloud);
  pass_.filter (*cloud_pass_);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void
OpenNIPassthrough::timeoutSlot ()
{
  if (!cloud_pass_)
  {
    boost::this_thread::sleep (boost::posix_time::milliseconds (1));
    return;
  }

  CloudPtr temp_cloud;
  {
    QMutexLocker locker (&mtx_);
    temp_cloud.swap (cloud_pass_); 
  }
  // Add to the 3D viewer
  if (!vis_->updatePointCloud (temp_cloud, "cloud_pass"))
  {
    vis_->addPointCloud (temp_cloud, "cloud_pass");
    vis_->resetCameraViewpoint ("cloud_pass");
  }
  FPS_CALC ("visualization");
  ui_->qvtk_widget->update ();
}

int 
main (int argc, char ** argv)
{
  // Initialize QT
  QApplication app (argc, argv); 

  // Open the first available camera
  pcl::OpenNIGrabber grabber ("#1");
  // Check if an RGB stream is provided
  if (!grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  {
    PCL_ERROR ("Device #1 does not provide an RGB stream!\n");
    return (-1);
  }

  OpenNIPassthrough v (grabber);
  v.show ();
  return (app.exec ());
}

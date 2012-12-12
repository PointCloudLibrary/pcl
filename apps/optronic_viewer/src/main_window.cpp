/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/apps/optronic_viewer/main_window.h>
#include <pcl/apps/optronic_viewer/openni_grabber.h>

#include <pcl/io/openni_grabber.h>

#include <QFileInfo>
#include <vtkActor.h>
#include <vtkRenderer.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
MainWindow::
MainWindow ()
  : grabber_ (NULL)
  , grabber_thread_ (NULL)
{
  cloud_.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  
  
  this->setWindowTitle ("PCL Optronic Viewer");


  // setup menu
  this->createFileMenu ();


  // setup widgets
  QWidget * central_widget = new QWidget (this);
  
  QLabel * sensor_label = new QLabel (tr ("Sensor"));
  QComboBox * sensor_selection_combo_box = new QComboBox ();
  sensor_selection_combo_box->addItem (tr ("none"));
  this->findConnectedDevices ();
  for (size_t device_idx = 0; device_idx < connected_openni_devices_.size (); ++device_idx)
  {
    sensor_selection_combo_box->addItem (tr (connected_openni_devices_[device_idx].name.c_str ()));
  }
  connect (sensor_selection_combo_box, SIGNAL(activated (int)), this, SLOT(selectedSensorChanged (int)));
  
  processing_list_view_ = new QListView (central_widget);

  pcl_visualizer_ = new pcl::visualization::PCLVisualizer ("", false);
  qvtk_widget_ = new QVTKWidget (central_widget);
  qvtk_widget_->SetRenderWindow (pcl_visualizer_->getRenderWindow ());
  pcl_visualizer_->setupInteractor (qvtk_widget_->GetInteractor (), qvtk_widget_->GetRenderWindow ());
  qvtk_widget_->update ();


  // setup layouts
  QHBoxLayout * sensor_layout = new QHBoxLayout ();
  sensor_layout->addWidget (sensor_label, 0, Qt::AlignLeft);
  sensor_layout->addWidget (sensor_selection_combo_box, 1);

  QVBoxLayout * sensor_processing_list_layout = new QVBoxLayout ();
  sensor_processing_list_layout->addLayout (sensor_layout);
  sensor_processing_list_layout->addWidget (processing_list_view_);

  QHBoxLayout * main_layout = new QHBoxLayout (central_widget);
  main_layout->addLayout (sensor_processing_list_layout, 0);
  main_layout->addWidget (qvtk_widget_, 1);

  this->resize (1024, 600);
  this->setCentralWidget (central_widget);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
MainWindow::
~MainWindow ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
MainWindow::
createFileMenu ()
{
  //QAction * open_pcd_file_act = new QAction (tr ("&Open PCD file"), this);
  //open_pcd_file_act->setStatusTip(tr("Opens a PCD file"));
  //connect(newAct, SIGNAL(triggered()), this, SLOT(newFile()));

  QAction * exit_act = new QAction (tr ("E&xit"), this);
  exit_act->setStatusTip(tr("Closes the viewer"));
  //connect(newAct, SIGNAL(triggered()), this, SLOT(newFile()));

  QMenu * file_menu = this->menuBar ()->addMenu (tr ("&File"));
  //file_menu->addAction (open_pcd_file_act);
  file_menu->addAction (exit_act);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
MainWindow::
findConnectedDevices ()
{
  connected_openni_devices_.clear ();

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
  if (driver.getNumberDevices() > 0)
  {
    std::cerr << "Connected devices:" << std::endl;
    for (unsigned device_idx = 0; device_idx < driver.getNumberDevices(); ++device_idx)
    {
      std::cout << "  Device: " << device_idx + 1 << ", vendor: " << driver.getVendorName(device_idx) << ", product: " << driver.getProductName(device_idx)
        << ", connected: " << driver.getBus(device_idx) << " @ " << driver.getAddress(device_idx) << ", serial number: \'" << driver.getSerialNumber(device_idx) << "\'" << endl;

      OpenNIDevice device;
      device.device_id = device_idx + 1;
      device.name = driver.getProductName (device_idx);
      device.vendor = driver.getVendorName(device_idx);
      device.serial_number = driver.getSerialNumber(device_idx);

      connected_openni_devices_.push_back (device);
    }

  }
  else
    std::cout << "No devices connected." << endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
selectedSensorChanged (int index)
{
  std::cerr << "selected sensor changed to " << index << std::endl;

  // stop old grabber
  if (grabber_ != NULL)
  {
    if (grabber_->isRunning ())
      grabber_->stop ();
  }

  // stop old grabber thread
  if (grabber_thread_ != NULL)
  {
    disconnect (grabber_thread_, SIGNAL(cloudReceived (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr)), this, SLOT(cloud_callback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr)));
    grabber_thread_->terminate ();

    delete grabber_thread_;
    grabber_thread_ = NULL;
  }

  // delete old grabber
  if (grabber_ != NULL)
  {
    delete grabber_;
    grabber_ = NULL;
  }

  if (index != 0)
  {
    std::stringstream ss;
    ss << "#" << connected_openni_devices_[index-1].device_id;

    grabber_ = new pcl::OpenNIGrabber (ss.str ());

    grabber_thread_ = new pcl::apps::optronic_viewer::OpenNIGrabber (grabber_);
    grabber_thread_->start ();
    connect (grabber_thread_, SIGNAL(cloudReceived (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr)), this, SLOT(cloud_callback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr)));

    QTimer::singleShot (30, this, SLOT(refresh ()));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
refresh ()
{
	QTime now;
	now.start();

  if (cloud_mutex_.try_lock ())
  {
    if (!pcl_visualizer_->updatePointCloud (cloud_, "OpenNICloud"))
    {
      pcl_visualizer_->addPointCloud (cloud_, "OpenNICloud");
      pcl_visualizer_->resetCameraViewpoint ("OpenNICloud");
    }          
    cloud_mutex_.unlock ();
  }
  qvtk_widget_->GetRenderWindow ()->Render ();

  int elapsed_time = now.elapsed ();
  int time = 30 - elapsed_time;

  std::cerr << "elapsed time: " << elapsed_time << std::endl;

  if (time < 0)
    time = 0;

  QTimer::singleShot (time, this, SLOT(refresh ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
cloud_callback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
  //std::cerr << "cloud received" << std::endl;

  if (cloud_mutex_.try_lock ())
  {
    cloud_ = cloud;
    cloud_mutex_.unlock ();
  }
}



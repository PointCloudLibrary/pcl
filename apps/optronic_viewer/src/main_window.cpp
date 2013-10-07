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
#include <pcl/apps/optronic_viewer/filter_window.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/fotonic_grabber.h>

#include <fz_api.h>
#include <fz_internal.h>

#include <QFileInfo>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
MainWindow::
MainWindow ()
  : grabber_ (NULL)
  , grabber_thread_ (NULL)
{
  // initialize API
  pcl::FotonicGrabber::initAPI ();


  // setup filters
  filter_factories_.push_back (new VoxelGridCFF2 ());
  filter_factories_.push_back (new PassThroughCFF2 ());
  filter_factories_.push_back (new RadiusOutlierCFF2 ());
  filter_factories_.push_back (new FastBilateralCFF2 ());
  filter_factories_.push_back (new MedianCFF2 ());
  filter_factories_.push_back (new RandomSampleCFF2 ());
  filter_factories_.push_back (new PlaneCFF2 ());


  // reset point cloud
  cloud_.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  

  // set window title
  this->setWindowTitle ("PCL Optronic Viewer");


  // setup menu
  this->createFileMenu ();


  // setup widgets
  QWidget * central_widget = new QWidget (this);
  
  QLabel * sensor_label = new QLabel (tr ("Sensor"));
  QComboBox * sensor_selection_combo_box = new QComboBox ();
  sensor_selection_combo_box->addItem (tr ("none"));
  this->findConnectedDevices ();
  for (size_t device_idx = 0; device_idx < connected_devices_.size (); ++device_idx)
  {
    if (connected_devices_[device_idx]->device_type == OPENNI_DEVICE)
      sensor_selection_combo_box->addItem (tr (reinterpret_cast<OpenNIDevice*> (connected_devices_[device_idx])->name.c_str ()));
    if (connected_devices_[device_idx]->device_type == FOTONIC_DEVICE)
      sensor_selection_combo_box->addItem (tr (reinterpret_cast<FotonicDevice*> (connected_devices_[device_idx])->device_info.szPath));
  }
  connect (sensor_selection_combo_box, SIGNAL(activated (int)), this, SLOT(selectedSensorChanged (int)));
  
  processing_list_ = new QListWidget (central_widget);
  connect (processing_list_, SIGNAL (itemDoubleClicked (QListWidgetItem*)), this, SLOT (updateFilter (QListWidgetItem*)));
  connect (processing_list_, SIGNAL (itemSelectionChanged ()), this, SLOT (filterSelectionChanged ()));


  up_ = new QPushButton ("up");
  down_ = new QPushButton ("down");
  remove_ = new QPushButton ("remove");

  connect (up_, SIGNAL (clicked ()), this, SLOT (moveFilterUp ()));
  connect (down_, SIGNAL (clicked ()), this, SLOT (moveFilterDown ()));
  connect (remove_, SIGNAL (clicked ()), this, SLOT (removeFilter ()));

  up_->setEnabled (false);
  down_->setEnabled (false);
  remove_->setEnabled (false);

  pcl_visualizer_ = new pcl::visualization::PCLVisualizer ("", false);
  qvtk_widget_ = new QVTKWidget (central_widget);
  qvtk_widget_->SetRenderWindow (pcl_visualizer_->getRenderWindow ());
  pcl_visualizer_->setupInteractor (qvtk_widget_->GetInteractor (), qvtk_widget_->GetRenderWindow ());
  qvtk_widget_->update ();


  // setup layouts
  QHBoxLayout * sensor_layout = new QHBoxLayout ();
  sensor_layout->addWidget (sensor_label, 0, Qt::AlignLeft);
  sensor_layout->addWidget (sensor_selection_combo_box, 1);

  QHBoxLayout * button_layout = new QHBoxLayout ();
  button_layout->addWidget (up_);
  button_layout->addWidget (down_);
  button_layout->addWidget (remove_);

  QVBoxLayout * sensor_processing_list_layout = new QVBoxLayout ();
  sensor_processing_list_layout->addLayout (sensor_layout, 0);
  sensor_processing_list_layout->addWidget (processing_list_);
  sensor_processing_list_layout->addLayout (button_layout, 0);

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
  // exit Fotonic API
  pcl::FotonicGrabber::exitAPI ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
MainWindow::
createFileMenu ()
{
  // File menu
  QAction * exit_act = new QAction (tr ("E&xit"), this);
  exit_act->setStatusTip (tr ("Closes the viewer"));
  connect(exit_act, SIGNAL(triggered()), this, SLOT(close ()));

  QMenu * file_menu = this->menuBar ()->addMenu (tr ("&File"));
  file_menu->addAction (exit_act);


  // Process menu
  QAction * add_filter_act = new QAction (tr ("Add &Filter"), this);
  add_filter_act->setStatusTip (tr ("Adds a filter"));
  connect(add_filter_act, SIGNAL (triggered ()), this, SLOT (addFilter ()));

  QMenu * process_menu = this->menuBar ()->addMenu (tr ("&Process"));
  process_menu->addAction (add_filter_act);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
MainWindow::
findConnectedDevices ()
{
  connected_devices_.clear ();

  std::cerr << "Check for OpenNI devices..." << std::endl;
  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
  if (driver.getNumberDevices() > 0)
  {
    std::cerr << "Connected devices:" << std::endl;
    for (unsigned device_idx = 0; device_idx < driver.getNumberDevices(); ++device_idx)
    {
      std::cout << "  Device: " << device_idx + 1 << ", vendor: " << driver.getVendorName(device_idx) << ", product: " << driver.getProductName(device_idx)
        << ", connected: " << driver.getBus(device_idx) << " @ " << driver.getAddress(device_idx) << ", serial number: \'" << driver.getSerialNumber(device_idx) << "\'" << endl;

      OpenNIDevice * device = new OpenNIDevice ();
      device->device_id = device_idx + 1;
      device->name = driver.getProductName (device_idx);
      device->vendor = driver.getVendorName(device_idx);
      device->serial_number = driver.getSerialNumber(device_idx);

      connected_devices_.push_back (device);
    }

  }
  else
    std::cout << "No devices connected." << endl;


  std::cerr << "Check for Fotonic devices..." << std::endl;
  std::vector<FZ_DEVICE_INFO> fotonic_devices = pcl::FotonicGrabber::enumDevices ();
  for (int device_index = 0; device_index < fotonic_devices.size (); ++device_index)
  {
    FotonicDevice * device = new FotonicDevice ();

    device->device_info.iDeviceType = fotonic_devices[device_index].iDeviceType;
    memcpy (device->device_info.szPath, fotonic_devices[device_index].szPath, sizeof (fotonic_devices[device_index].szPath[0])*512);
    memcpy (device->device_info.szSerial, fotonic_devices[device_index].szSerial, sizeof (fotonic_devices[device_index].szSerial[0])*16);
    memcpy (device->device_info.szShortName, fotonic_devices[device_index].szShortName, sizeof (fotonic_devices[device_index].szShortName[0])*32);

    std::cerr << "device id " << device_index << std::endl;
    std::cerr << "  device type: " << fotonic_devices[device_index].iDeviceType << std::endl;
    std::cerr << "  device path: " << fotonic_devices[device_index].szPath << std::endl;
    std::cerr << "  device serial: " << fotonic_devices[device_index].szSerial << std::endl;
    std::cerr << "  device short name: " << fotonic_devices[device_index].szShortName << std::endl;

    connected_devices_.push_back (device);
  }
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
    if (connected_devices_[index-1]->device_type == OPENNI_DEVICE)
    {
      std::stringstream ss;
      ss << "#" << reinterpret_cast<OpenNIDevice*> (connected_devices_[index-1])->device_id;

      grabber_ = new pcl::OpenNIGrabber (ss.str ());

    }

    if (connected_devices_[index-1]->device_type == FOTONIC_DEVICE)
    {
      grabber_ = new pcl::FotonicGrabber (reinterpret_cast<FotonicDevice*> (connected_devices_[index-1])->device_info);
    }

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
    // apply filters
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_in = cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    for (int i = 0; i < active_cloud_filters_.size (); ++i)
    {
      active_cloud_filters_[i]->filter (cloud_in, cloud_out);
      cloud_in = cloud_out;
      cloud_out.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    }

    // only update if cloud is not empty, otherwise the reset of the camera 
    // viewpoint won't work when initially selecting a sensor
    if (!cloud_in->empty ()) 
    {
      // visualize
      if (!pcl_visualizer_->updatePointCloud (cloud_in, "OpenNICloud"))
      {
        pcl_visualizer_->addPointCloud (cloud_in, "OpenNICloud");
        pcl_visualizer_->resetCameraViewpoint ("OpenNICloud");
      }  
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

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
addFilter ()
{
  std::cerr << "add filter..." << std::endl;

  // open window to select new filter
  pcl::apps::optronic_viewer::FilterWindow * filter_dialog = new pcl::apps::optronic_viewer::FilterWindow (filter_factories_, active_cloud_filters_);
  filter_dialog->setWindowTitle (tr ("Add Filter..."));

  connect (filter_dialog, SIGNAL (filterCreated ()), this, SLOT (refreshFilterList ()));

  filter_dialog->show ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
updateFilter (QListWidgetItem * item)
{
  int id = processing_list_->row (item);

  QWizard * wizard = new QWizard ();
  wizard->addPage (active_cloud_filters_[id]->getParameterPage ());
  wizard->show ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
filterSelectionChanged ()
{
  std::cerr << "filter selection changed..." << std::endl;

  QList<QListWidgetItem*> selected_items = processing_list_->selectedItems ();

  std::cerr << "number of selected items: " << selected_items.size () << std::endl;

  if (selected_items.size () != 0)
  {
    int row = processing_list_->row (selected_items[0]);
    std::cerr << "selected filter: " << row << std::endl;

    if (row != 0)
      up_->setEnabled (true);
    else
      up_->setEnabled (false);
    
    const int list_size = active_cloud_filters_.size ();
    if (row != list_size-1)
      down_->setEnabled (true);
    else
      down_->setEnabled (false);

    remove_->setEnabled (true);
  }
  else
  {
    up_->setEnabled (false);
    down_->setEnabled (false);
    remove_->setEnabled (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
moveFilterUp ()
{
  std::cerr << "move filter up" << std::endl;

  QList<QListWidgetItem*> selected_items = processing_list_->selectedItems ();
  int change_id = processing_list_->row (selected_items[0]);

  pcl::apps::optronic_viewer::CloudFilter* tmp = active_cloud_filters_[change_id];
  active_cloud_filters_[change_id] = active_cloud_filters_[change_id-1];
  active_cloud_filters_[change_id-1] = tmp;

  refreshFilterList ();

  processing_list_->setCurrentRow (change_id-1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
moveFilterDown ()
{
  std::cerr << "move filter down" << std::endl;

  QList<QListWidgetItem*> selected_items = processing_list_->selectedItems ();
  int change_id = processing_list_->row (selected_items[0]);

  pcl::apps::optronic_viewer::CloudFilter* tmp = active_cloud_filters_[change_id];
  active_cloud_filters_[change_id] = active_cloud_filters_[change_id+1];
  active_cloud_filters_[change_id+1] = tmp;

  refreshFilterList ();

  processing_list_->setCurrentRow (change_id+1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
removeFilter ()
{
  std::cerr << "remove filter" << std::endl;

  QList<QListWidgetItem*> selected_items = processing_list_->selectedItems ();
  int remove_id = processing_list_->row (selected_items[0]);

  pcl::apps::optronic_viewer::CloudFilter* tmp = active_cloud_filters_[remove_id];
  active_cloud_filters_.erase (active_cloud_filters_.begin()+remove_id);
  refreshFilterList ();

  cloud_mutex_.lock ();
  delete tmp;
  cloud_mutex_.unlock ();

  if (active_cloud_filters_.size () > remove_id)
    processing_list_->setCurrentRow (remove_id);
  else
    processing_list_->setCurrentRow (remove_id-1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::apps::optronic_viewer::
MainWindow::
refreshFilterList ()
{
  processing_list_->clear ();
  std::cerr << "filters: " << std::endl;
  for (int i = 0; i < active_cloud_filters_.size (); ++i)
  {
    std::cerr << "  " << active_cloud_filters_[i]->getName () << std::endl;
    processing_list_->addItem (QString (active_cloud_filters_[i]->getName ().c_str ()));
  }
}

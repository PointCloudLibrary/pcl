/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: $
 *
 * @author: Koen Buys - KU Leuven
 */

//PCL
#include <pcl/apps/pcd_video_player.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//STD
#include <iostream>
#include <fstream>

//BOOST
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

//QT4
#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>
#include <QFileDialog>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>

// VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

using namespace pcl;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
PCDVideoPlayer::PCDVideoPlayer ()
{
  cloud_present_ = false;
  cloud_modified_ = false;
  play_mode_ = false;
  speed_counter_ = 0;
  speed_value_ = 5;

  //Create a timer
  vis_timer_ = new QTimer (this);
  vis_timer_->start (5);//5ms

  connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot()));

  ui_ = new Ui::MainWindow;
  ui_->setupUi (this);
  
  this->setWindowTitle ("PCL PCD Video Player");

  // Setup the cloud pointer
  cloud_.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);

  // Set up the qvtk window
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  ui_->qvtkWidget->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (ui_->qvtkWidget->GetInteractor (), ui_->qvtkWidget->GetRenderWindow ());
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  ui_->qvtkWidget->update ();

  // Connect all buttons
  connect (ui_->playButton, SIGNAL(clicked()), this, SLOT(playButtonPressed()));
  connect (ui_->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonPressed()));
  connect (ui_->backButton, SIGNAL(clicked()), this, SLOT(backButtonPressed()));
  connect (ui_->nextButton, SIGNAL(clicked()), this, SLOT(nextButtonPressed()));

  connect (ui_->selectFolderButton, SIGNAL(clicked()), this, SLOT(selectFolderButtonPressed()));
  connect (ui_->selectFilesButton, SIGNAL(clicked()), this, SLOT(selectFilesButtonPressed()));
  
  connect (ui_->indexSlider, SIGNAL(valueChanged(int)), this, SLOT(indexSliderValueChanged(int)));
}

void
PCDVideoPlayer::playButtonPressed()
{
  play_mode_ = true;
}

void 
PCDVideoPlayer::stopButtonPressed()
{
  play_mode_= false;
}

void 
PCDVideoPlayer::backButtonPressed()
{
  if(current_frame_ == 0) // Allready in the beginning
  {
    PCL_DEBUG ("[PCDVideoPlayer::nextButtonPressed] : reached the end\n");
    current_frame_ = nr_of_frames_-1; // reset to end
  }
  else
  {
    current_frame_--;
    cloud_modified_ = true;
    ui_->indexSlider->setSliderPosition(current_frame_);        // Update the slider position
  }
}

void 
PCDVideoPlayer::nextButtonPressed()
{
  if(current_frame_ == (nr_of_frames_-1)) // Reached the end
  {
    PCL_DEBUG ("[PCDVideoPlayer::nextButtonPressed] : reached the end\n");
    current_frame_ = 0; // reset to beginning
  }
  else
  {
    current_frame_++;
    cloud_modified_ = true;
    ui_->indexSlider->setSliderPosition(current_frame_);        // Update the slider position
  }
}

void
PCDVideoPlayer::selectFolderButtonPressed()
{
  pcd_files_.clear();     // Clear the std::vector
  pcd_paths_.clear();     // Clear the boost filesystem paths

  dir_ = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

  boost::filesystem::directory_iterator end_itr;

  if(boost::filesystem::is_directory(dir_.toStdString()))
  {
    for (boost::filesystem::directory_iterator itr(dir_.toStdString()); itr != end_itr; ++itr)
    {
      std::string ext = itr->path().extension().string();
      if(ext.compare(".pcd") == 0)
      {
        pcd_files_.push_back (itr->path ().string ());
        pcd_paths_.push_back (itr->path ());
      }
      else
      {
        // Found non pcd file
        PCL_DEBUG ("[PCDVideoPlayer::selectFolderButtonPressed] : found a different file\n");
      }
    }
  }
  else
  {
    PCL_ERROR("Path is not a directory\n");
    exit(-1);
  }
  nr_of_frames_ = pcd_files_.size();
  PCL_DEBUG ("[PCDVideoPlayer::selectFolderButtonPressed] : found %d files\n", nr_of_frames_ );

  if(nr_of_frames_ == 0)
  {
    PCL_ERROR("Please select valid pcd folder\n");
    cloud_present_ = false;
    return;
  }
  else
  {
    // Reset the Slider
    ui_->indexSlider->setValue(0);                // set cursor back in the beginning
    ui_->indexSlider->setRange(0,nr_of_frames_-1);  // rescale the slider

    current_frame_ = 0;

    cloud_present_ = true;
    cloud_modified_ = true;
  }
}

void 
PCDVideoPlayer::selectFilesButtonPressed()
{
  pcd_files_.clear();  // Clear the std::vector
  pcd_paths_.clear();     // Clear the boost filesystem paths

  QStringList qt_pcd_files = QFileDialog::getOpenFileNames(this, "Select one or more PCD files to open", "/home", "PointClouds (*.pcd)");
  nr_of_frames_ = qt_pcd_files.size();
  std::cout << "[PCDVideoPlayer::selectFilesButtonPressed] : selected " << nr_of_frames_ << " files" << std::endl;

  if(nr_of_frames_ == 0)
  {
    PCL_ERROR("Please select valid pcd files\n");
    cloud_present_ = false;
    return;
  }
  else
  {
    for(int i = 0; i < qt_pcd_files.size(); i++)
    {
      pcd_files_.push_back(qt_pcd_files.at(i).toStdString());
    }

    current_frame_ = 0;

    // Reset the Slider
    ui_->indexSlider->setValue(0);                // set cursor back in the beginning
    ui_->indexSlider->setRange(0,nr_of_frames_-1);  // rescale the slider

    cloud_present_ = true;
    cloud_modified_ = true;
  }
}

void 
PCDVideoPlayer::timeoutSlot ()
{
  if(play_mode_)
  {
    if(speed_counter_ == speed_value_)
    {
      if(current_frame_ == (nr_of_frames_-1)) // Reached the end
      {
        current_frame_ = 0; // reset to beginning
      }
      else
      {
        current_frame_++;
        cloud_modified_ = true;
        ui_->indexSlider->setSliderPosition(current_frame_);        // Update the slider position
      }
    }
    else
    {
      speed_counter_++;
    }
  }

  if(cloud_present_ && cloud_modified_)
  {
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcd_files_[current_frame_], *cloud_) == -1) //* load the file
    {
      PCL_ERROR ("[PCDVideoPlayer::timeoutSlot] : Couldn't read file %s\n");
    }

    if(!vis_->updatePointCloud(cloud_, "cloud_"))
    {
      vis_->addPointCloud (cloud_, "cloud_");
      vis_->resetCameraViewpoint("cloud_");
    }
    cloud_modified_ = false;
  }
  ui_->qvtkWidget->update();
}

void
PCDVideoPlayer::indexSliderValueChanged(int value)
{
  PCL_DEBUG ("[PCDVideoPlayer::indexSliderValueChanged] : (I) : value %d\n", value);
  current_frame_ = value;
  cloud_modified_ = true;
}

void
print_usage ()
{
  PCL_INFO ("PCDVideoPlayer V0.1\n");
  PCL_INFO ("-------------------\n");
  PCL_INFO ("\tThe slider accepts focus on Tab and provides both a mouse wheel and a keyboard interface. The keyboard interface is the following:\n");
  PCL_INFO ("\t  Left/Right move a horizontal slider by one single step.\n");
  PCL_INFO ("\t  Up/Down move a vertical slider by one single step.\n");
  PCL_INFO ("\t  PageUp moves up one page.\n");
  PCL_INFO ("\t  PageDown moves down one page.\n");
  PCL_INFO ("\t  Home moves to the start (mininum).\n");
  PCL_INFO ("\t  End moves to the end (maximum).\n");
}

int
main (int argc, char** argv)
{
  QApplication app(argc, argv);

  PCDVideoPlayer VideoPlayer;

  VideoPlayer.show();

  return (app.exec());
}

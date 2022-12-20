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
 */

#include <pcl/apps/pcd_video_player.h>
#include <pcl/point_types.h>

#include <QApplication>
#include <QButtonGroup>
#include <QEvent>
#include <QFileDialog>
#include <QGroupBox>
#include <QMutexLocker>
#include <QObject>
#include <QRadioButton>
#if VTK_MAJOR_VERSION >= 9 || (VTK_MAJOR_VERSION == 8 && VTK_MINOR_VERSION >= 2)
#define HAS_QVTKOPENGLWINDOW_H
#include <QVTKOpenGLWindow.h>
#endif
#include <ui_pcd_video_player.h>

#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>

#include <fstream>
#include <iostream>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
PCDVideoPlayer::PCDVideoPlayer()
{
  cloud_present_ = false;
  cloud_modified_ = false;
  play_mode_ = false;
  speed_counter_ = 0;
  speed_value_ = 5;

  // Create a timer
  vis_timer_ = new QTimer(this);
  vis_timer_->start(5); // 5ms

  connect(vis_timer_, SIGNAL(timeout()), this, SLOT(timeoutSlot()));

  ui_ = new Ui::MainWindow;
  ui_->setupUi(this);

  this->setWindowTitle("PCL PCD Video Player_mod");

  // Setup the cloud pointer
  cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
  new_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

  // Create the QVTKWidget
#if VTK_MAJOR_VERSION > 8
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vis_.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "", false));
#else
  vis_.reset(new pcl::visualization::PCLVisualizer("", false));
#endif // VTK_MAJOR_VERSION > 8
  vis_->setBackgroundColor(0.7, 0.7, 0.7);

  // Draw and label the axes
  PointXYZ pt_origin{0.0, 0.0, 0.0};
  PointXYZ pt_x{5, 0, 0};
  PointXYZ pt_y{0, 5, 0};
  PointXYZ pt_z{0, 0, 5};
  vis_->addArrow<PointXYZ,PointXYZ>(pt_x, pt_origin, 1.0 ,0.0, 0.0, false, "axisX");
  vis_->addArrow<PointXYZ,PointXYZ>(pt_y, pt_origin, 0.0 ,1.0, 0.0, false, "axisY");
  vis_->addArrow<PointXYZ,PointXYZ>(pt_z, pt_origin, 0.0 ,0.0, 1.0, false, "axisZ");

  vis_->addText3D<PointXYZ>("X", pt_x, 1, 1.0, 0.0, 0.0);
  vis_->addText3D<PointXYZ>("Y", pt_y, 1, 0.0, 1.0, 0.0);
  vis_->addText3D<PointXYZ>("Z", pt_z, 1, 0.0, 0.0, 1.0);

  setRenderWindowCompat(*(ui_->qvtk_widget), *(vis_->getRenderWindow()));
  vis_->setupInteractor(getInteractorCompat(*(ui_->qvtk_widget)),
                        getRenderWindowCompat(*(ui_->qvtk_widget)));

  vis_->getInteractorStyle()->setKeyboardModifier(
      pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

  refreshView();

  // Connect all buttons
  connect(ui_->playButton, SIGNAL(clicked()), this, SLOT(playButtonPressed()));
  connect(ui_->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonPressed()));
  connect(ui_->backButton, SIGNAL(clicked()), this, SLOT(backButtonPressed()));
  connect(ui_->nextButton, SIGNAL(clicked()), this, SLOT(nextButtonPressed()));

  connect(ui_->selectFolderButton,
          SIGNAL(clicked()),
          this,
          SLOT(selectFolderButtonPressed()));
  connect(ui_->selectFilesButton,
          SIGNAL(clicked()),
          this,
          SLOT(selectFilesButtonPressed()));

  connect(ui_->indexSlider,
          SIGNAL(valueChanged(int)),
          this,
          SLOT(indexSliderValueChanged(int)));

  std::cout << "\n Created instance for PCDVideoPlayer \n";
}

void
PCDVideoPlayer::backButtonPressed()
{
  if (current_frame_ == 0) // Already in the beginning
  {
    PCL_DEBUG("[PCDVideoPlayer::nextButtonPressed] : reached the end\n");
    current_frame_ = nr_of_frames_ - 1; // reset to end
  }
  else {
    current_frame_--;
    cloud_modified_ = true;
    ui_->indexSlider->setSliderPosition(current_frame_); // Update the slider position
  }
}

void
PCDVideoPlayer::nextButtonPressed()
{
  if (current_frame_ == (nr_of_frames_ - 1)) // Reached the end
  {
    PCL_DEBUG("[PCDVideoPlayer::nextButtonPressed] : reached the end\n");
    current_frame_ = 0; // reset to beginning
  }
  else {
    current_frame_++;
    cloud_modified_ = true;
    ui_->indexSlider->setSliderPosition(current_frame_); // Update the slider position
  }
}

void
PCDVideoPlayer::selectFolderButtonPressed()
{
    std::cout << "\n Selecting folder for PCDVideoPlayer \n";

  pcd_files_.clear(); // Clear the std::vector
  pcd_paths_.clear(); // Clear the boost filesystem paths

  dir_ = QFileDialog::getExistingDirectory(this,
                                           tr("Open Directory"),
                                           "/home",
                                           QFileDialog::ShowDirsOnly |
                                               QFileDialog::DontResolveSymlinks);

  boost::filesystem::directory_iterator end_itr;

  if (boost::filesystem::is_directory(dir_.toStdString())) {
    for (boost::filesystem::directory_iterator itr(dir_.toStdString()); itr != end_itr; ++itr) {
      std::string ext = itr->path().extension().string();
      if (boost::filesystem::is_regular_file(itr->path()) && ext == ".pcd") {
        std::cout << "\t Read file:" << itr->path().string() << "\n";
        pcd_files_.push_back(itr->path().string());
        pcd_paths_.push_back(itr->path());
      }
      else {
        // Found non pcd file
        std::cerr << "\t (!) Found non pcd file..." << std::endl;
        PCL_DEBUG(
            "[PCDVideoPlayer::selectFolderButtonPressed] : found a different file\n");
      }
    }
    // Sort files by name to play them back properly
    static const auto strcomp = [](const std::string& s1, const std::string& s2) { return s1 < s2;};
    std::sort<decltype(pcd_files_.begin()), decltype(strcomp)>(pcd_files_.begin(), pcd_files_.end(), strcomp);
  }
  else {
    PCL_ERROR("Path is not a directory\n");
    exit(-1);
  }
  nr_of_frames_ = pcd_files_.size();
  PCL_DEBUG("[PCDVideoPlayer::selectFolderButtonPressed] : found %d files\n",
            nr_of_frames_);

  if (nr_of_frames_ == 0) {
    PCL_ERROR("Please select valid pcd folder\n");
    cloud_present_ = false;
    return;
  }
  // Reset the Slider
  ui_->indexSlider->setValue(0);                    // set cursor back in the beginning
  ui_->indexSlider->setRange(0, nr_of_frames_ - 1); // rescale the slider

  current_frame_ = 0;

  cloud_present_ = true;
  cloud_modified_ = true;
}

void
PCDVideoPlayer::selectFilesButtonPressed()
{
  pcd_files_.clear(); // Clear the std::vector
  pcd_paths_.clear(); // Clear the boost filesystem paths

  QStringList qt_pcd_files = QFileDialog::getOpenFileNames(
      this, "Select one or more PCD files to open", "/home", "PointClouds (*.pcd)");
  nr_of_frames_ = qt_pcd_files.size();
  PCL_INFO("[PCDVideoPlayer::selectFilesButtonPressed] : selected %ld files\n",
           nr_of_frames_);

  if (nr_of_frames_ == 0) {
    PCL_ERROR("Please select valid pcd files\n");
    cloud_present_ = false;
    return;
  }

  for (int i = 0; i < qt_pcd_files.size(); i++) {
    pcd_files_.push_back(qt_pcd_files.at(i).toStdString());
  }

  // FIXME : Look up for .pcd files and sort them by name

  current_frame_ = 0;

  // Reset the Slider
  ui_->indexSlider->setValue(0);                    // set cursor back in the beginning
  ui_->indexSlider->setRange(0, nr_of_frames_ - 1); // rescale the slider

  cloud_present_ = true;
  cloud_modified_ = true;
}

void
PCDVideoPlayer::timeoutSlot()
{
  //std::cout << "\t Initialiting playback now... \n";

  if (play_mode_) {
    if (speed_counter_ == speed_value_) {
      if (current_frame_ == (nr_of_frames_ - 1)) // Reached the end
      {
        current_frame_ = 0; // reset to beginning
      }
      else {
        current_frame_++;
        cloud_modified_ = true;
        ui_->indexSlider->setSliderPosition(
            current_frame_); // Update the slider position
      }
    }
    else {
      speed_counter_++;
    }
  }

  if (cloud_present_ && cloud_modified_) {
    auto current_file = pcd_files_[current_frame_];
    std::cout << "Reading " << current_file << endl;

    new_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    if(pcdReader.read(current_file, *new_cloud) == -1)
    {
      std::cout << "[PCDVideoPlayer::timeoutSlot] : Couldn't read file" << current_file << std::endl;
      PCL_ERROR("[PCDVideoPlayer::timeoutSlot] : Couldn't read file %s\n");
    }
    else{
      std::cout << "Done." << endl;
    }
    
    // Update cloud_ptr to visualize
    cloud_ptr = new_cloud;

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> colorh(cloud_ptr, "intensity");
    pcl::PointCloud<PointXYZI>::ConstPtr const_cloud_ptr{cloud_ptr};

    if (!vis_->updatePointCloud<pcl::PointXYZI>(cloud_ptr, colorh, "cloud")) {
      // Setting up visualizer for first time
      vis_->resetCameraViewpoint("cloud");
      vis_->addPointCloud<pcl::PointXYZI>(cloud_ptr, colorh, "cloud");
      vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
      vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT, 
                                             pcl::visualization::PCL_VISUALIZER_LUT_JET, "cloud");
      vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_RANGE, 0, 255, "cloud");
      vis_->addText("current_file_path", 0, 0, 30, 0.f, 0.f, 0.f, "message");
    }
    else{
      // Visualizer is set
      vis_->updateText(current_file, 0, 0, 30, 0.f, 0.f, 0.f, "message");
    }
    cloud_modified_ = false;
  }

  refreshView();
}

void
PCDVideoPlayer::indexSliderValueChanged(int value)
{
  PCL_DEBUG("[PCDVideoPlayer::indexSliderValueChanged] : (I) : value %d\n", value);
  current_frame_ = value;
  cloud_modified_ = true;
}

void
PCDVideoPlayer::refreshView()
{
#if VTK_MAJOR_VERSION > 8
  ui_->qvtk_widget->renderWindow()->Render();
#else
  ui_->qvtk_widget->update();
#endif // VTK_MAJOR_VERSION > 8
}

void
print_usage()
{
  // clang-format off
  PCL_INFO ("PCDVideoPlayer V0.1\n");
  PCL_INFO ("-------------------\n");
  PCL_INFO ("\tThe slider accepts focus on Tab and provides both a mouse wheel and a keyboard interface. The keyboard interface is the following:\n");
  PCL_INFO ("\t  Left/Right move a horizontal slider by one single step.\n");
  PCL_INFO ("\t  Up/Down move a vertical slider by one single step.\n");
  PCL_INFO ("\t  PageUp moves up one page.\n");
  PCL_INFO ("\t  PageDown moves down one page.\n");
  PCL_INFO ("\t  Home moves to the start (minimum).\n");
  PCL_INFO ("\t  End moves to the end (maximum).\n");
  // clang-format on
}

int
main(int argc, char** argv)
{
#ifdef HAS_QVTKOPENGLWINDOW_H
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWindow::defaultFormat());
#endif
  QApplication app(argc, argv);

  PCDVideoPlayer VideoPlayer;

  VideoPlayer.show();

  return QApplication::exec();
}

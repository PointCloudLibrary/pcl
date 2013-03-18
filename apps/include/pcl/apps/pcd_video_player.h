/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#include <ui_pcd_video_player.h>

#include <iostream>
#include <time.h>

// QT4
#include <QMainWindow>
#include <QMutex>
#include <QTimer>

// Boost
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>

// PCL
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/registration/transformation_estimation_svd.h>

typedef pcl::PointXYZRGBA PointT;

#define CURRENT_VERSION 0.2

// Useful macros
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

namespace Ui
{
  class MainWindow;
}

class PCDVideoPlayer : public QMainWindow
{
  Q_OBJECT
  public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

    PCDVideoPlayer ();

    ~PCDVideoPlayer () {}

  protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer>  vis_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr               cloud_;

    QMutex          mtx_;
    QMutex          vis_mtx_;
    Ui::MainWindow  *ui_;
    QTimer          *vis_timer_;

    QString         dir_;

    std::vector<std::string>              pcd_files_;
    std::vector<boost::filesystem::path>  pcd_paths_;

    //QStringList     pcd_files_;

    unsigned int    current_frame_;         // The current displayed frame
    unsigned int    nr_of_frames_;          // Store the number of loaded frames

    bool            cloud_present_;         // Indicate that pointclouds were loaded
    bool            cloud_modified_;        // Indicate that the timeoutSlot needs to reload the pointcloud

    bool            play_mode_;             // Indicate that files should play continiously
    unsigned int    speed_counter_;         // In play mode only update if speed_counter_ == speed_value
    unsigned int    speed_value_;           // Fixes the speed in steps of 5ms, default 5, gives 5+1 * 5ms = 30ms = 33,3 Hz playback speed

  public slots:
    void 
    playButtonPressed();
    void 
    stopButtonPressed();
    void
    backButtonPressed();
    void 
    nextButtonPressed();
    void 
    selectFolderButtonPressed();
    void
    selectFilesButtonPressed();

    void
    indexSliderValueChanged(int value);

  private slots:
    void
    timeoutSlot();

};

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, O R PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: openni_viewer.cpp 5059 2012-03-14 02:12:17Z gedikli $
 *
 */

#define SHOW_FPS 1
#include <pcl/apps/timer.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace std;

typedef PointUV KeyPointT;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class AGASTDemo
{
  public:
    typedef PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    AGASTDemo (Grabber& grabber)
      : cloud_viewer_ ("AGAST 2D Keypoints -- PointCloud")
      , grabber_ (grabber)
      , image_viewer_ ("AGAST 2D Keypoints -- Image")
      , bmax_ (255)
      , threshold_ (30)
      , detector_type_ (0)
      , timer_ ()
    {
    }

    /////////////////////////////////////////////////////////////////////////
    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);

      // Compute AGAST keypoints 
      AgastKeypoint2D<PointT> agast;
      agast.setNonMaxSuppression (true);
      agast.setThreshold (threshold_);
      agast.setMaxDataValue (bmax_);
      agast.setInputCloud (cloud);

      keypoints_.reset (new PointCloud<KeyPointT>);

      // Select the detector type
      switch (detector_type_)
      {
        case 1:
        default:
        {
          timer_.reset ();
          pcl::keypoints::agast::AgastDetector7_12s::Ptr detector (new pcl::keypoints::agast::AgastDetector7_12s (cloud->width, cloud->height, threshold_, bmax_));
          agast.setAgastDetector (detector);
          agast.compute (*keypoints_);
          PCL_DEBUG ("AGAST 7_12s computation took %f ms.\n", timer_.getTime ());
          break;
        }
        case 2:
        {
          timer_.reset ();
          pcl::keypoints::agast::AgastDetector5_8::Ptr detector (new pcl::keypoints::agast::AgastDetector5_8 (cloud->width, cloud->height, threshold_, bmax_));
          agast.setAgastDetector (detector);
          agast.compute (*keypoints_);
          PCL_DEBUG ("AGAST 5_8 computation took %f ms.\n", timer_.getTime ());
          break;
        }
        case 3:
        {
          timer_.reset ();
          pcl::keypoints::agast::OastDetector9_16::Ptr detector (new pcl::keypoints::agast::OastDetector9_16 (cloud->width, cloud->height, threshold_, bmax_));
          agast.setAgastDetector (detector);
          agast.compute (*keypoints_);
          PCL_DEBUG ("OAST 9_6 computation took %f ms.\n", timer_.getTime ());
          break;
        }
      }
      cloud_ = cloud;
    }

    /////////////////////////////////////////////////////////////////////////
    void 
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
    {
      AGASTDemo* obj = static_cast<AGASTDemo*> (cookie);
      
      if (event.getKeyCode ())
      {
        std::stringstream ss; ss << event.getKeyCode ();
        obj->detector_type_ = atoi (ss.str ().c_str ());
        return;
      }

      if (event.getKeySym () == "Up")
      {
        if (obj->threshold_ <= 0.9)
        {
          PCL_INFO ("[keyboard_callback] Increase AGAST threshold from %f to %f.\n", obj->threshold_, obj->threshold_ + 0.01);
          obj->threshold_ += 0.01;
          return;
        }
        PCL_INFO ("[keyboard_callback] Increase AGAST threshold from %f to %f.\n", obj->threshold_, obj->threshold_ + 1);
        obj->threshold_ += 1;
        return;
      }

      if (event.getKeySym () == "Down")
      {
        if (obj->threshold_ <= 0)
          return;
        if (obj->threshold_ <= 1)
        {
          PCL_INFO ("[keyboard_callback] Decrease AGAST threshold from %f to %f.\n", obj->threshold_, obj->threshold_ - 0.01);
          obj->threshold_ -= 0.01;
          return;
        }
        PCL_INFO ("[keyboard_callback] Decrease AGAST threshold from %f to %f.\n", obj->threshold_, obj->threshold_ - 1);
        obj->threshold_ -= 1;
        return;
      }

      if (event.getKeySym () == "Right")
      {
        PCL_INFO ("[keyboard_callback] Increase AGAST BMAX from %f to %f.\n", obj->bmax_, obj->bmax_ + 1);
        obj->bmax_ += 1;
        return;
      }

      if (event.getKeySym () == "Left")
      {
        if (obj->bmax_ <= 0)
          return;
        PCL_INFO ("[keyboard_callback] Decrease AGAST BMAX from %f to %f.\n", obj->bmax_, obj->bmax_ - 1);
        obj->bmax_ -= 1;
        return;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void
    init ()
    {
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&AGASTDemo::cloud_callback, this, _1);
      cloud_connection = grabber_.registerCallback (cloud_cb);      
    }

    /////////////////////////////////////////////////////////////////////////
    string
    getStrBool (bool state)
    {
      stringstream ss;
      ss << state;
      return (ss.str ());
    }

    /////////////////////////////////////////////////////////////////////////
    void
    get3DKeypoints (
        const CloudConstPtr &cloud,
        const PointCloud<KeyPointT>::Ptr &keypoints, PointCloud<PointT> &keypoints3d)
    {
      if (!cloud || !keypoints || cloud->points.empty () || keypoints->points.empty ())
        return;

      keypoints3d.resize (keypoints->size ());
      keypoints3d.width = keypoints->width;
      keypoints3d.height = keypoints->height;
      keypoints3d.is_dense = true;

      size_t j = 0;
      for (size_t i = 0; i < keypoints->size (); ++i)
      {
        const PointT &pt = (*cloud)(static_cast<long unsigned int> (keypoints->points[i].u), 
                                    static_cast<long unsigned int> (keypoints->points[i].v));
        if (!pcl_isfinite (pt.x) || !pcl_isfinite (pt.y) || !pcl_isfinite (pt.z))
          continue;

        keypoints3d.points[j].x = pt.x;
        keypoints3d.points[j].y = pt.y;
        keypoints3d.points[j].z = pt.z;
        ++j;
      }

      if (j != keypoints->size ())
      {
        keypoints3d.resize (j);
        keypoints3d.width = j;
        keypoints3d.height = 1;
      }
    }
    
    /////////////////////////////////////////////////////////////////////////
    void
    run ()
    {
      cloud_viewer_.registerKeyboardCallback (&AGASTDemo::keyboard_callback, *this, static_cast<AGASTDemo*> (this));
      image_viewer_.registerKeyboardCallback (&AGASTDemo::keyboard_callback, *this, static_cast<AGASTDemo*> (this));

      grabber_.start ();
      
      bool image_init = false, cloud_init = false;
      bool keypts = true;

      CloudPtr keypoints3d (new Cloud);

      while (!cloud_viewer_.wasStopped () && !image_viewer_.wasStopped ())
      {
        PointCloud<KeyPointT>::Ptr keypoints;
        CloudConstPtr cloud;

        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          keypoints_.swap (keypoints);
        
          cloud_mutex_.unlock ();
        }

        if (cloud)
        {
          if (!cloud_init)
          {
            cloud_viewer_.setPosition (0, 0);
            cloud_viewer_.setSize (cloud->width, cloud->height);
            cloud_init = !cloud_init;
          }

          if (!cloud_viewer_.updatePointCloud (cloud, "OpenNICloud"))
          {
            cloud_viewer_.addPointCloud (cloud, "OpenNICloud");
            cloud_viewer_.resetCameraViewpoint ("OpenNICloud");
          }

          if (!image_init)
          {
            image_viewer_.setPosition (cloud->width, 0);
            image_viewer_.setSize (cloud->width, cloud->height);
            image_init = !image_init;
          }

          image_viewer_.addRGBImage<PointT> (cloud);

          if (keypoints && !keypoints->empty ())
          {
            image_viewer_.removeLayer (getStrBool (keypts));
            for (size_t i = 0; i < keypoints->size (); ++i)
            {
              int u = int (keypoints->points[i].u);
              int v = int (keypoints->points[i].v);
              image_viewer_.markPoint (u, v, visualization::red_color, visualization::blue_color, 10, getStrBool (!keypts));
            }
            keypts = !keypts;

            get3DKeypoints (cloud, keypoints, *keypoints3d);
            visualization::PointCloudColorHandlerCustom<PointT> blue (keypoints3d, 0, 0, 255);
            if (!cloud_viewer_.updatePointCloud (keypoints3d, blue, "keypoints"))
              cloud_viewer_.addPointCloud (keypoints3d, blue, "keypoints");
            cloud_viewer_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 20, "keypoints");
            cloud_viewer_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_OPACITY, 0.5, "keypoints");
          }
        }

        cloud_viewer_.spinOnce ();
        image_viewer_.spinOnce ();
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();
      cloud_connection.disconnect ();
    }
    
    visualization::PCLVisualizer cloud_viewer_;
    Grabber& grabber_;
    boost::mutex cloud_mutex_;
    CloudConstPtr cloud_;
    
    visualization::ImageViewer image_viewer_;

    PointCloud<KeyPointT>::Ptr keypoints_;

    double bmax_;
    double threshold_;
    int detector_type_;
  private:
    boost::signals2::connection cloud_connection;
    StopWatch timer_;
};

/* ---[ */
int
main (int argc, char** argv)
{
  bool debug = false;
  pcl::console::parse_argument (argc, argv, "-debug", debug);
  if (debug)
    pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);
  else
    pcl::console::setVerbosityLevel (pcl::console::L_INFO);

  string device_id ("#1");
  OpenNIGrabber grabber (device_id);
  AGASTDemo<PointXYZRGBA> openni_viewer (grabber);

  openni_viewer.init ();
  openni_viewer.run ();
  
  return (0);
}
/* ]--- */

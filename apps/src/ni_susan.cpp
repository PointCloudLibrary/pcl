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
 * $Id$
 *
 */

#define SHOW_FPS 1
#include <pcl/apps/timer.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/susan.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace std;

typedef PointXYZRGBA PointT;
typedef PointXYZRGBL KeyPointT;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class SUSANDemo
{
  public:
    typedef PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

    SUSANDemo (Grabber& grabber)
      : cloud_viewer_ ("SUSAN 2D Keypoints -- PointCloud")
      , grabber_ (grabber)
      , image_viewer_ ("SUSAN 2D Keypoints -- Image")
    {
    }

    /////////////////////////////////////////////////////////////////////////
    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;

      // Compute SUSAN keypoints 
      SUSANKeypoint<PointT, KeyPointT> susan;
      susan.setInputCloud (cloud);
      susan.setNumberOfThreads (6);
      susan.setNonMaxSupression (true);
      keypoints_.reset (new PointCloud<KeyPointT>);
      susan.compute (*keypoints_);
    }

    /////////////////////////////////////////////////////////////////////////
    void
    init ()
    {
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&SUSANDemo::cloud_callback, this, _1);
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
    run ()
    {
      grabber_.start ();
      
      bool image_init = false, cloud_init = false;
      bool keypts = true;

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
              int u = int (keypoints->points[i].label % cloud->width);
              int v = cloud->height - int (keypoints->points[i].label / cloud->width);
              image_viewer_.markPoint (u, v, visualization::red_color, visualization::blue_color, 10, getStrBool (!keypts));
            }
            keypts = !keypts;

            visualization::PointCloudColorHandlerCustom<KeyPointT> blue (keypoints, 0, 0, 255);
            if (!cloud_viewer_.updatePointCloud (keypoints, blue, "keypoints"))
              cloud_viewer_.addPointCloud (keypoints, blue, "keypoints");
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
        
  private:
    boost::signals2::connection cloud_connection;
};

/* ---[ */
int
main (int, char**)
{
  string device_id ("#1");
  OpenNIGrabber grabber (device_id);
  SUSANDemo openni_viewer (grabber);

  openni_viewer.init ();
  openni_viewer.run ();
  
  return (0);
}
/* ]--- */

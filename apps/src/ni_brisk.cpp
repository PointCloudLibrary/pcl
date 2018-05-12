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
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
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
#include <pcl/keypoints/brisk_2d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace std;

typedef PointXYZRGBA PointT;
typedef PointWithScale KeyPointT;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BRISKDemo
{
  public:
    typedef PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

    BRISKDemo (Grabber& grabber)
      : cloud_viewer_ ("BRISK 2D Keypoints -- PointCloud")
      , grabber_ (grabber)
      , image_viewer_ ("BRISK 2D Keypoints -- Image")
    {
    }

    /////////////////////////////////////////////////////////////////////////
    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;

      // Compute BRISK keypoints 
      BriskKeypoint2D<PointT> brisk;
      brisk.setThreshold (60);
      brisk.setOctaves (4);
      brisk.setInputCloud (cloud);

      keypoints_.reset (new PointCloud<KeyPointT>);
      brisk.compute (*keypoints_);
    }

    /////////////////////////////////////////////////////////////////////////
    void
    init ()
    {
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&BRISKDemo::cloud_callback, this, _1);
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
    inline PointT
    bilinearInterpolation (const CloudConstPtr &cloud, float x, float y)
    {
      int u = int (x);
      int v = int (y);
      
      PointT pt;
      pt.x = pt.y = pt.z = 0;

      const PointT &p1 = (*cloud)(u,   v);
      const PointT &p2 = (*cloud)(u+1, v);
      const PointT &p3 = (*cloud)(u,   v+1);
      const PointT &p4 = (*cloud)(u+1, v+1);
      
      float fx = x - float (u), fy = y - float (v);
      float fx1 = 1.0f - fx, fy1 = 1.0f - fy;

      float w1 = fx1 * fy1, w2 = fx * fy1, w3 = fx1 * fy, w4 = fx * fy;
      float weight = 0;
      
      if (isFinite (p1))
      {
        pt.x += p1.x * w1;
        pt.y += p1.y * w1;
        pt.z += p1.z * w1;
        weight += w1;
      }
      if (isFinite (p2))
      {
        pt.x += p2.x * w2;
        pt.y += p2.y * w2;
        pt.z += p2.z * w2;
        weight += w2;
      }
      if (isFinite (p3))
      {
        pt.x += p3.x * w3;
        pt.y += p3.y * w3;
        pt.z += p3.z * w3;
        weight += w3;
      }
      if (isFinite (p4))
      {
        pt.x += p4.x * w4;
        pt.y += p4.y * w4;
        pt.z += p4.z * w4;
        weight += w4;
      }

      if (weight == 0)
        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
      else
      {
        weight = 1.0f / weight;
        pt.x *= weight; pt.y *= weight; pt.z *= weight;
      }
 
      return (pt);
    }

    /////////////////////////////////////////////////////////////////////////
    void
    get3DKeypoints (
        const CloudConstPtr &cloud,
        const PointCloud<KeyPointT>::Ptr &keypoints, PointCloud<PointT> &keypoints3d)
    {
      keypoints3d.resize (keypoints->size ());
      keypoints3d.width = keypoints->width;
      keypoints3d.height = keypoints->height;
      keypoints3d.is_dense = true;

      size_t j = 0;
      for (size_t i = 0; i < keypoints->size (); ++i)
      {
        PointT pt = bilinearInterpolation (cloud, keypoints->points[i].x, keypoints->points[i].y);

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
      grabber_.start ();
      
      bool image_init = false, cloud_init = false;
      bool keypts = true;

      PointCloud<KeyPointT>::Ptr keypoints;
      CloudConstPtr cloud;
      CloudPtr keypoints3d (new Cloud);

      while (!cloud_viewer_.wasStopped () && !image_viewer_.wasStopped ())
      {
        if (cloud_mutex_.try_lock ())
        {
          if (cloud_)
            cloud_.swap (cloud);

          if (keypoints_)
            keypoints_.swap (keypoints);

          cloud_mutex_.unlock ();

          if (!cloud)
            continue;

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

          image_viewer_.showRGBImage<PointT> (cloud);

          image_viewer_.removeLayer (getStrBool (keypts));
          for (size_t i = 0; i < keypoints->size (); ++i)
          {
            int u = int (keypoints->points[i].x);
            int v = int (keypoints->points[i].y);
            image_viewer_.markPoint (u, v, visualization::red_color, visualization::blue_color, 10, getStrBool (!keypts));
          }
          keypts = !keypts;

          
          get3DKeypoints (cloud, keypoints, *keypoints3d);
          visualization::PointCloudColorHandlerCustom<PointT> blue (keypoints3d, 0, 0, 255);
          if (!cloud_viewer_.updatePointCloud (keypoints3d, blue, "keypoints"))
            cloud_viewer_.addPointCloud (keypoints3d, blue, "keypoints");
          cloud_viewer_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");
          cloud_viewer_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_OPACITY, 0.5, "keypoints");
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
  BRISKDemo openni_viewer (grabber);

  openni_viewer.init ();
  openni_viewer.run ();
  
  return (0);
}
/* ]--- */

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
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/agast_2d.h>
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
      BriskKeypoint2D<PointT> agast;
      agast.setThreshold (60);
      agast.setOctaves (4);
      agast.setInputCloud (cloud);

      keypoints_.reset (new PointCloud<KeyPointT>);
      agast.compute (*keypoints_);
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
    void
    get3DKeypoints (
        const CloudConstPtr &cloud,
        const PointCloud<KeyPointT>::Ptr &keypoints, PointCloud<PointT> &keypoints3d)
    {
      keypoints3d.resize (keypoints->size ());
      keypoints3d.width = keypoints->width;
      keypoints3d.height = keypoints->height;
      keypoints3d.is_dense = false;

      vector<int> indices (1);
      vector<float> distances (1);

      // Create a dummy XY cloud
      PointCloud<KeyPointT>::Ptr cloudxy (new PointCloud<KeyPointT>);
      copyPointCloud<PointT, KeyPointT> (*cloud, *cloudxy);
      KdTreeFLANN<KeyPointT> tree;
      tree.setInputCloud (cloudxy);

      KeyPointT pt;
      // Search for nearest neighbors
      for (size_t i = 0; i < keypoints->size (); ++i)
      {
        PointT pt3d = (*cloud)(static_cast<long unsigned int> (keypoints->points[i].x), 
                               static_cast<long unsigned int> (keypoints->points[i].y));
        pt.x = pt3d.x; pt.y = pt3d.y;

        if (!pcl_isfinite (pt.x) || !pcl_isfinite (pt.y))
          continue;

        tree.nearestKSearch (pt, 1, indices, distances);
        
        keypoints3d.points[i].x = pt.x;
        keypoints3d.points[i].y = pt.y;
        keypoints3d.points[i].z = cloud->points[indices[0]].z;
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
            int v = cloud->height - int (keypoints->points[i].y);
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

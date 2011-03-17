/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * $Id: pcl_visualizer.cpp 35278 2011-01-17 01:20:57Z rusu $
 *
 */

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>

struct pcl_visualization::CloudViewer::CloudViewer_impl
{
  CloudViewer_impl (const std::string& window_name) :
    window_name_ (window_name), has_cloud_ (false), quit_ (false), cloud_ (0), gray_cloud_ (0)
  {
    viewer_thread_ = boost::thread (boost::ref (*this));
    while (!viewer_)
    {
      boost::thread::yield ();
    }
  }
  ~CloudViewer_impl ()
  {
    quit_ = true;
    viewer_thread_.join ();
  }

  void
  block_post_cloud (const CloudViewer::ColorCloud* cloud, const std::string& name)
  {
    {
      boost::mutex::scoped_lock (mtx_);
      cloud_ = cloud;
      color_name_ = name;
      has_cloud_ = true;
    }
    while (cloud_ != NULL)
    {
      boost::thread::yield ();
    }
  }
  void
  block_post_cloud (const CloudViewer::GrayCloud* cloud, const std::string& name)
  {
    {
      boost::mutex::scoped_lock (mtx_);
      gray_cloud_ = cloud;
      gray_name_ = name;
      has_cloud_ = true;
    }
    while (gray_cloud_ != NULL)
    {
      boost::thread::yield ();
    }

  }
  void
  operator() ()
  {
    viewer_ = boost::shared_ptr<pcl_visualization::PCLVisualizer> (new pcl_visualization::PCLVisualizer (window_name_));
    viewer_->setBackgroundColor (0.1, 0.1, 0.1);
    viewer_->addCoordinateSystem (0.1);

    while (!quit_)
    {
      viewer_thread_.yield ();
//      if (!has_cloud_)
//        continue;
      {
        {
          boost::mutex::scoped_lock (mtx_);
          if (gray_cloud_ != NULL)
          {
            pcl_visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler (*gray_cloud_, 255, 0, 255);
            viewer_->removePointCloud (gray_name_);
            viewer_->addPointCloud (*gray_cloud_, handler, gray_name_);
            gray_cloud_ = 0;
          }
          else if (cloud_ != NULL)
          {
            pcl_visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (*cloud_);
            viewer_->removePointCloud (color_name_);
            viewer_->addPointCloud (*cloud_, handler, color_name_);
            cloud_ = 0;
          }
        }

        {
          boost::mutex::scoped_lock (c_mtx);
          BOOST_FOREACH(CallableMap::value_type& x, callables)
                {
                  std::cerr << "calling " << x.first << std::endl;
                  (x.second) (*viewer_);
                }
        }
        if (viewer_->wasStopped ())
        {
          return; //todo handle this better
        }
        {
          boost::mutex::scoped_lock (spin_mtx_);
          //TODO some smart waitkey like stuff here, so that wasStoped() can hold for a long time
          //maybe a counter
          viewer_->spinOnce (); // Give the GUI millis to handle events, then return
        }
      }
    }
  }
  void
  post (VizCallable x, const std::string& key)
  {
    boost::mutex::scoped_lock (c_mtx);
    callables[key] = x;
  }
  void
  remove (const std::string& key)
  {
    boost::mutex::scoped_lock (c_mtx);
    if (callables.find (key) != callables.end ())
    {
      callables.erase (key);
    }
  }
  std::string window_name_;
  boost::shared_ptr<pcl_visualization::PCLVisualizer> viewer_;
  boost::mutex mtx_, spin_mtx_, c_mtx;
  boost::thread viewer_thread_;
  bool has_cloud_;
  bool quit_;
  const CloudViewer::ColorCloud* cloud_;
  const CloudViewer::GrayCloud* gray_cloud_;
  std::string gray_name_, color_name_;
  typedef std::map<std::string, VizCallable> CallableMap;
  CallableMap callables;

};

pcl_visualization::CloudViewer::CloudViewer (const std::string& window_name) :
  impl_ (new CloudViewer_impl (window_name))
{

}

pcl_visualization::CloudViewer::~CloudViewer ()
{
  delete impl_;
}

void
pcl_visualization::CloudViewer::showCloud (const CloudViewer::ColorCloud& cloud, const std::string& cloudname)
{
  if (!impl_->viewer_ || impl_->viewer_->wasStopped ())
    return;
  impl_->block_post_cloud (&cloud, cloudname);
}

void
pcl_visualization::CloudViewer::showCloud (const CloudViewer::GrayCloud& cloud, const std::string& cloudname)
{
  if (!impl_->viewer_ || impl_->viewer_->wasStopped ())
    return;
  impl_->block_post_cloud (&cloud, cloudname);
}
void
pcl_visualization::CloudViewer::runOnVisualizationThread (VizCallable x, const std::string& key)
{
  impl_->post (x, key);
}

void
pcl_visualization::CloudViewer::removeVisualizationCallable (const std::string& key)
{
  impl_->remove (key);
}

bool
pcl_visualization::CloudViewer::wasStopped (int millis)
{
  boost::thread::yield (); //allow this to be called in a loop
  if (impl_->viewer_)
    return (impl_->viewer_->wasStopped ());
  else
    return false;
}

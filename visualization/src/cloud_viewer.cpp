/**
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
  * $Id$
  *
  */

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>

namespace pcl
{
  struct cloud_show_base
  {
    virtual void pop () = 0;
    virtual bool popped () const = 0;
    typedef boost::shared_ptr<cloud_show_base> Ptr;
  };

  template <typename CloudT> 
  struct cloud_show : cloud_show_base
  {
    cloud_show (const std::string &cloud_name, typename CloudT::ConstPtr cloud,
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) :
      cloud_name (cloud_name), cloud (cloud), viewer (viewer),popped_ (false)
    {}

    template <typename Handler> void
    pop (const Handler &handler)
    {
      double psize = 1.0, opacity = 1.0, linesize =1.0;
      viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, cloud_name);
      viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cloud_name);
      viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, cloud_name);

      if (!viewer->updatePointCloud (cloud, handler, cloud_name))
      {
        viewer->addPointCloud (cloud, handler, cloud_name);
        viewer->resetCameraViewpoint (cloud_name);
      }

      // viewer->removePointCloud (cloud_name);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, cloud_name);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cloud_name);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, cloud_name);
      popped_ = true;
    }

    virtual void pop ();
    
    virtual bool
    popped () const
    {
      return popped_;
    }
    
    std::string cloud_name;
    typename CloudT::ConstPtr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    bool popped_;
  };
  
  typedef pcl::PointCloud<pcl::PointXYZRGBA> cca;
  typedef pcl::PointCloud<pcl::PointXYZRGB> cc;
  typedef pcl::PointCloud<pcl::PointXYZI> gc;
  typedef pcl::PointCloud<pcl::PointXYZ> mc;

  template <> void
  cloud_show<cca>::pop ()
  {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> handler (cloud);
    pop (handler);
  }
  
  template <> void
  cloud_show<cc>::pop ()
  {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (cloud);
    pop (handler);
  }
  
  template <> void
  cloud_show<gc>::pop ()
  {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler (cloud, "intensity");
    pop (handler);
  }
  
  template <> void
  cloud_show<mc>::pop ()
  {
    pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZ> handler (cloud);
    pop (handler);
  }
}

struct pcl::visualization::CloudViewer::CloudViewer_impl
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  CloudViewer_impl (const std::string& window_name) :
    window_name_ (window_name), has_cloud_ (false), quit_ (false)
  {
    viewer_thread_ = boost::thread (boost::ref (*this));
    while (!viewer_)
    {
      boost::thread::yield ();
    }
  }

  ~CloudViewer_impl ()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  template <typename T> void
  block_post_cloud (const typename T::ConstPtr &cloud, const std::string &name)
  {
    cloud_show_base::Ptr cs (new cloud_show<T>(name,cloud,viewer_));
    {
      boost::mutex::scoped_lock lock (mtx_);
      cloud_shows_.push_back (cs);
    }
    while (!cs->popped ())
    {
      boost::thread::yield ();
    }
  }

  template <typename T> void
  nonblock_post_cloud (const typename T::ConstPtr &cloud, const std::string &name)
  {
    cloud_show_base::Ptr cs (new cloud_show<T>(name,cloud,viewer_));
    {
      boost::mutex::scoped_lock lock (mtx_);

      cloud_shows_.push_back (cs);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  void
  operator() ()
  {
    using namespace pcl::visualization;

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
    viewer_ = boost::shared_ptr<PCLVisualizer>(new PCLVisualizer (window_name_));
#else
    viewer_ = boost::shared_ptr<PCLVisualizer>(new PCLVisualizer (window_name_, true));
#endif
    viewer_->setBackgroundColor (0.1, 0.1, 0.1);
    viewer_->addCoordinateSystem (0.1, "global");

    while (!quit_)
    {
      {
        boost::mutex::scoped_lock lock (mtx_);
        while (!cloud_shows_.empty ())
        {
          cloud_shows_.back ()->pop ();
          cloud_shows_.pop_back ();
        }
      }
      {
        boost::mutex::scoped_lock lock (once_mtx);
        BOOST_FOREACH (CallableList::value_type& x, callables_once)
        {
          (x)(*viewer_);
        }
        callables_once.clear ();
      }
      {
        boost::mutex::scoped_lock lock (c_mtx);
        BOOST_FOREACH (CallableMap::value_type& x, callables)
        {
          (x.second)(*viewer_);
        }
      }
      if (viewer_->wasStopped ())
      {
          quit_ = true;
      }else
      {
        boost::mutex::scoped_lock lock (spin_mtx_);
        //TODO some smart waitkey like stuff here, so that wasStoped() can hold for a long time
        //maybe a counter
        viewer_->spinOnce (10); // Give the GUI millis to handle events, then return
      }

    }
    viewer_.reset ();
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  void
  post (VizCallable x, const std::string &key)
  {
    boost::mutex::scoped_lock lock (c_mtx);
    callables[key] = x;
  }

  void
  post (VizCallable x)
  {
    boost::mutex::scoped_lock lock (once_mtx);
    callables_once.push_back (x);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  void
  remove (const std::string &key)
  {
    boost::mutex::scoped_lock lock (c_mtx);
    if (callables.find (key) != callables.end ())
      callables.erase (key);
  }

  std::string window_name_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  boost::mutex mtx_, spin_mtx_, c_mtx, once_mtx;
  boost::thread viewer_thread_;
  bool has_cloud_;
  bool quit_;
  std::list<boost::shared_ptr<cloud_show_base> > cloud_shows_;
  typedef std::map<std::string, VizCallable> CallableMap;
  CallableMap callables;
  typedef std::list<VizCallable> CallableList;
  CallableList callables_once;
};

////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::CloudViewer::CloudViewer (const std::string &window_name) :
  impl_ (new CloudViewer_impl (window_name))
{}

pcl::visualization::CloudViewer::~CloudViewer ()
{
  impl_->quit_ = true;
  impl_->viewer_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::CloudViewer::showCloud (const ColorACloud::ConstPtr &cloud,
                                            const std::string &cloudname)
{
  if (!impl_->viewer_ || impl_->viewer_->wasStopped ())
    return;
  impl_->block_post_cloud<ColorACloud>(cloud, cloudname);
}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::CloudViewer::showCloud (const ColorCloud::ConstPtr &cloud,
                                            const std::string &cloudname)
{
  if (!impl_->viewer_ || impl_->viewer_->wasStopped ())
    return;
  impl_->block_post_cloud<ColorCloud>(cloud, cloudname);
}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::CloudViewer::showCloud (const GrayCloud::ConstPtr &cloud,
                                            const std::string &cloudname)
{
  if (!impl_->viewer_ || impl_->viewer_->wasStopped ())
    return;
  impl_->block_post_cloud<GrayCloud>(cloud, cloudname);
}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::CloudViewer::showCloud (const MonochromeCloud::ConstPtr &cloud,
                                            const std::string &cloudname)
{
  if (!impl_->viewer_ || impl_->viewer_->wasStopped ())
    return;
  impl_->block_post_cloud<MonochromeCloud>(cloud, cloudname);
}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::CloudViewer::runOnVisualizationThread (VizCallable x, const std::string &key)
{
  impl_->post (x, key);
}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::CloudViewer::runOnVisualizationThreadOnce (VizCallable x)
{
  impl_->post (x);
}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::CloudViewer::removeVisualizationCallable (const std::string &key)
{
  impl_->remove (key);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::CloudViewer::wasStopped (int)
{
  boost::thread::yield (); //allow this to be called in a loop
  return !impl_->viewer_;
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::CloudViewer::registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)> callback)
{
  return impl_->viewer_->registerKeyboardCallback (callback);
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::CloudViewer::registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)> callback)
{
  return impl_->viewer_->registerMouseCallback (callback);
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::CloudViewer::registerPointPickingCallback (boost::function<void (const pcl::visualization::PointPickingEvent&)> callback)
{
  return (impl_->viewer_->registerPointPickingCallback (callback));
}


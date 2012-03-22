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
 * $Id: openni_viewer.cpp 5059 2012-03-14 02:12:17Z gedikli $
 *
 */

#include <boost/thread/thread.hpp>
#include <pcl/apps/timer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/search/organized.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using namespace pcl;

#define SHOW_FPS 1

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class NILinemod
{
  public:
    typedef PointCloud<PointXYZRGBA> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    bool added;

    NILinemod (Grabber& grabber)
      : cloud_viewer_ ("PointCloud")
      , grabber_ (grabber)
      , image_viewer_ ("Image")
    {
      added = false;
    }

    /////////////////////////////////////////////////////////////////////////
//    void
//    image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
//    {
//      boost::mutex::scoped_lock lock (image_mutex_);
//      image_ = image;
//    }
//    
//    boost::shared_ptr<openni_wrapper::Image>
//    getLatestImage ()
//    {
//      // Lock while we swap our image and reset it.
//      boost::mutex::scoped_lock lock (image_mutex_);
//      boost::shared_ptr<openni_wrapper::Image> temp_image;
//      temp_image.swap (image_);
//      return (temp_image);
//    }
    
    /////////////////////////////////////////////////////////////////////////
    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
      search_.setInputCloud (cloud);
    }

    CloudConstPtr
    getLatestCloud ()
    {
      // Lock while we swap our cloud and reset it.
      boost::mutex::scoped_lock lock (cloud_mutex_);
      CloudConstPtr temp_cloud;
      temp_cloud.swap (cloud_);
      return (temp_cloud);
    }


    /////////////////////////////////////////////////////////////////////////
    void 
    keyboard_callback (const visualization::KeyboardEvent& event, void*)
    {
      //if (event.getKeyCode())
      //  cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
      //else
      //  cout << "the special key \'" << event.getKeySym() << "\' was";
      //if (event.keyDown())
      //  cout << " pressed" << endl;
      //else
      //  cout << " released" << endl;
    }
    
    void 
    mouse_callback (const visualization::MouseEvent& mouse_event, void*)
    {
      if (mouse_event.getType() == visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == visualization::MouseEvent::LeftButton)
      {
        cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void
    segment ()
    {
    }

    /////////////////////////////////////////////////////////////////////////
    void 
    pp_callback (const pcl::visualization::PointPickingEvent& event, void*)
    {
      int idx = event.getPointIndex ();
      if (idx == -1)
        return;

      std::vector<int> indices (1);
      std::vector<float> distances (1);

      {
        // Use mutices to make sure we get the right cloud
        boost::mutex::scoped_lock lock1 (cloud_mutex_);
        //boost::mutex::scoped_lock lock2 (image_mutex_);

        pcl::PointXYZRGBA pos;
        event.getPoint (pos.x, pos.y, pos.z);

        std::stringstream ss;
        ss << "sphere_" << idx;
        cloud_viewer_.addSphere (pos, 0.01, 1.0, 0.0, 0.0, ss.str ());

        if (!search_.isValid ())
          return;

        search_.nearestKSearch (pos, 1, indices, distances);

        uint32_t width  = search_.getInputCloud ()->width,
                 height = search_.getInputCloud ()->height;
 
        int v = indices[0] / width,
            u = indices[0] % width;
        //image_viewer_.addCircle (u, height - v, 5, 1.0, 0.0, 0.0, "circles", 1.0);
        //image_viewer_.addBox (u-5, u+5, height-v-5, height-v+5, 0.0, 1.0, 0.0, "boxes", 0.5);
        image_viewer_.markPoint (u, v, pcl::visualization::red_color, pcl::visualization::blue_color, 10);
        added = !added;

        if (!added)
          //cloud_viewer_.close ();
          image_viewer_.removeLayer ("circles");
        //image_viewer_.markPoint (u, v, pcl::visualization::Vector3ub (1, 0, 0), pcl::visualization::Vector3ub (1, 0, 0), 0.1);//0.01, 1.0, 0.0, 0.0, ss.str ());
      }
    }
    
    /////////////////////////////////////////////////////////////////////////
    void
    init ()
    {
      cloud_viewer_.registerMouseCallback (&NILinemod::mouse_callback, *this);
      cloud_viewer_.registerKeyboardCallback(&NILinemod::keyboard_callback, *this);
      cloud_viewer_.registerPointPickingCallback (&NILinemod::pp_callback, *this);
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&NILinemod::cloud_callback, this, _1);
      cloud_connection = grabber_.registerCallback (cloud_cb);
      
      image_viewer_.registerMouseCallback (&NILinemod::mouse_callback, *this);
      image_viewer_.registerKeyboardCallback(&NILinemod::keyboard_callback, *this);
//      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&NILinemod::image_callback, this, _1);
//      image_connection = grabber_.registerCallback (image_cb);
    }

    /////////////////////////////////////////////////////////////////////////
    void
    run ()
    {
      grabber_.start ();
      
      bool image_init = false, cloud_init = false;
      unsigned char* rgb_data = 0;
      unsigned rgb_data_size = 0;

      while (!cloud_viewer_.wasStopped () && !image_viewer_.wasStopped ())
      {
        if (cloud_)
        {
          CloudConstPtr cloud = getLatestCloud ();
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

          image_viewer_.showRGBImage<PointXYZRGBA> (cloud);
        }

/*        if (image_)
        {
          boost::shared_ptr<openni_wrapper::Image> image = getLatestImage ();
          if (!image_init)
          {
            image_viewer_.setPosition (image->getWidth (), 0);
            image_viewer_.setSize (image->getWidth (), image->getHeight ());
            image_init = !image_init;
          }

          if (image->getEncoding() == openni_wrapper::Image::RGB)
            image_viewer_.showRGBImage (image->getMetaData ().Data (), image->getWidth (), image->getHeight ());
          else
          {
            if (rgb_data_size < image->getWidth () * image->getHeight ())
            {
              if (rgb_data)
                delete [] rgb_data;
              rgb_data_size = image->getWidth () * image->getHeight ();
              rgb_data = new unsigned char [rgb_data_size * 3];
            }
            image->fillRGB (image->getWidth (), image->getHeight (), rgb_data);
            image_viewer_.showRGBImage (rgb_data, image->getWidth (), image->getHeight ());
          }
        }*/

        cloud_viewer_.spinOnce ();
        image_viewer_.spinOnce ();
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();
      
//      if (rgb_data)
//        delete [] rgb_data;
    
      std::cerr << "finishing..." << std::endl;
      cloud_connection.disconnect ();
//      image_connection.disconnect ();
    }
    
    visualization::PCLVisualizer cloud_viewer_;
    Grabber& grabber_;
    boost::mutex cloud_mutex_;
    CloudConstPtr cloud_;
    
//    boost::mutex image_mutex_;
//    boost::shared_ptr<openni_wrapper::Image> image_;
    visualization::ImageViewer image_viewer_;

    search::OrganizedNeighbor<PointXYZRGBA> search_;
  private:
    boost::signals2::connection cloud_connection, image_connection;
};

/* ---[ */
int
main (int, char**)
{
  std::string device_id ("#1");
  OpenNIGrabber grabber (device_id);
  NILinemod openni_viewer (grabber);

  openni_viewer.init ();
  openni_viewer.run ();
  
  return (0);
}
/* ]--- */

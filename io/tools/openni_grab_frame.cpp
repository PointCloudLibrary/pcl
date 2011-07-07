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
 * Author: Nico Blodow (blodow@cs.tum.edu)
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>

class OpenNIGrabFrame
{
  public:
    OpenNIGrabFrame () : no_frame(true) {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
    {
      if (no_frame)
      {

        std::string output_dir;
        if (output_dir.empty ())
          output_dir = ".";

        std::string filename;
        if (filename.empty ())
        {
          std::stringstream ss;
          ss << output_dir << "/frame_" << boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::local_time()) << ".pcd";
          filename = ss.str ();
        }
        pcl::io::savePCDFileASCII(filename, *cloud);
        std::cerr << "Data saved to " << filename << std::endl;
        no_frame = false;
      }
    }
    
    void run ()
    {
      // create a new grabber for OpenNI devices
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      // make callback function from member function
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
        boost::bind (&OpenNIGrabFrame::cloud_cb_, this, _1);

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = interface->registerCallback (f);

      // start receiving point clouds
      interface->start ();

      // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
      while (no_frame)
        boost::this_thread::sleep (boost::posix_time::seconds (1));

      // stop the grabber
      interface->stop ();
    }
    bool no_frame;
};

int 
main ()
{
  OpenNIGrabFrame v;
  v.run ();
  return 0;
}


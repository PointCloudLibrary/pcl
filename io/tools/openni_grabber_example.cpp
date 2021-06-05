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
 * Author: Suat Gedikli (gedikli@willowgarage.com)
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <iomanip> // for setprecision

class SimpleOpenNIProcessor
{
  public:
    bool save;
    openni_wrapper::OpenNIDevice::DepthMode mode;

    SimpleOpenNIProcessor (openni_wrapper::OpenNIDevice::DepthMode depth_mode = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth) : mode (depth_mode) {}

    void 
    cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) const
    {
      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30)
      {
        double now = pcl::getTime ();
        std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        count = 0;
        last = now;
      }

      if (save)
      {
        std::stringstream ss;
        ss << std::setprecision (12) << pcl::getTime () * 100 << ".pcd";
        pcl::PCDWriter w;
        w.writeBinaryCompressed (ss.str (), *cloud);
        std::cout << "wrote point clouds to file " << ss.str () << std::endl;
      }
    }

    void 
    imageDepthImageCallback (const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr& d_img, float constant)
    {
      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30)
      {
        double now = pcl::getTime ();
        std::cout << "got synchronized image x depth-image with constant factor: " << constant << ". Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        std::cout << "Depth baseline: " << d_img->getBaseline () << " and focal length: " << d_img->getFocalLength () << std::endl;
        count = 0;
        last = now;
      }
    }

    void 
    run ()
    {
      save = false;

      // create a new grabber for OpenNI devices
      pcl::OpenNIGrabber interface;

      // Set the depth output format
      interface.getDevice ()->setDepthOutputFormat (mode);

      // make callback function from member function
      std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        [this] (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) { cloud_cb_ (cloud); };

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = interface.registerCallback (f);

      // make callback function from member function
      std::function<void (const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, float constant)> f2 =
        [this] (const openni_wrapper::Image::Ptr& img, const openni_wrapper::DepthImage::Ptr& depth, float constant)
        {
          imageDepthImageCallback (img, depth, constant);
        };

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c2 = interface.registerCallback (f2);

      // start receiving point clouds
      interface.start ();

      std::cout << R"(<Esc>, 'q', 'Q': quit the program)" << std::endl;
      std::cout << "\' \': pause" << std::endl;
      std::cout << "\'s\': save" << std::endl;
      char key;
      do
      {
        key = static_cast<char> (getchar ());
        switch (key)
        {
          case ' ':
            if (interface.isRunning ())
              interface.stop ();
            else
              interface.start ();
            break;
          case 's':
            save = !save;
        }
      } while (key != 27 && key != 'q' && key != 'Q');

      // stop the grabber
      interface.stop ();
    }
};

int
main (int argc, char **argv)
{
  int mode = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth;
  pcl::console::parse_argument (argc, argv, "-mode", mode);

  SimpleOpenNIProcessor v (static_cast<openni_wrapper::OpenNIDevice::DepthMode> (mode));
  v.run ();
  return (0);
}

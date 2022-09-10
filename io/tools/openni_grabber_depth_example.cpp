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
 */

#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>

class SimpleOpenNIProcessor
{
  public:
    bool save;
    openni_wrapper::OpenNIDevice::DepthMode mode;

    SimpleOpenNIProcessor (openni_wrapper::OpenNIDevice::DepthMode depth_mode = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth) : mode (depth_mode) {}

    void 
    imageDepthImageCallback (const openni_wrapper::DepthImage::Ptr& d_img)
    {
      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30)
      {
        double now = pcl::getTime ();
        std::cout << "got depth-image. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
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
      std::function<void (const openni_wrapper::DepthImage::Ptr&)> f2 = [this] (const openni_wrapper::DepthImage::Ptr& depth)
      {
        imageDepthImageCallback (depth);
      };

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c2 = interface.registerCallback (f2);

      // start receiving point clouds
      interface.start ();

      std::cout << R"(<Esc>, 'q', 'Q': quit the program)" << std::endl;
      std::cout << "\' \': pause" << std::endl;
      char key;
      do
      {
        key = static_cast<char> (getchar ());
        if (key == ' ')
        {
          interface.toggle ();
        }
      } while ((key != 27) && (key != 'q') && (key != 'Q'));

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

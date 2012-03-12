/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <pcl_cuda/io/cloud_to_pcl.h>
#include <pcl_cuda/io/debayering.h>

#include <pcl/io/kinect_grabber.h>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl_cuda/time_cpu.h>

class SimpleKinectTool
{
  public:
     //SimpleKinectTool () : viewer ("KinectGrabber"), init_(false) {}

    void cloud_cb_ (const boost::shared_ptr<openni_wrapper::Image>& image)
    {
    	thrust::host_vector<pcl_cuda::OpenNIRGB> rgb_image(image->getWidth () * image->getHeight ());
    	cv::Mat cv_image( image->getHeight (), image->getWidth (), CV_8UC3 );
    	{
    	pcl::ScopeTime t ("computeBilinear+memcpy");
    	debayering.computeBilinear (image, rgb_image);
    	//debayering.computeEdgeAware (image, rgb_image);
    	// now fill image and show!
    	pcl::ScopeTime t2 ("memcpy");
    	memcpy (cv_image.data, &rgb_image[0], image->getWidth () * image->getHeight () * 3);
    	}
    	imshow ("test", cv_image);
    }
    
    void run (const std::string& device_id)
    {
	    cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
      pcl::Grabber* interface = new pcl::OpenNIGrabber(device_id);

      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>& image)> f = boost::bind (&SimpleKinectTool::cloud_cb_, this, _1);

      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();
      
      while (true)
      {
        //sleep (1);
        cv::waitKey(10);
      }

      interface->stop ();
    }

    pcl_cuda::Debayering<pcl_cuda::Host> debayering;
    boost::mutex mutex_;
    bool init_;
};

int main (int argc, char** argv)
{
	std::string device_id = "#1";
	if (argc == 2)
	{
		device_id = argv[1];
	}
  SimpleKinectTool v;
  v.run (device_id);
  return 0;
}

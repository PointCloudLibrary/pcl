/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


/*
 * hdl_grabber_example.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: keven
 */

#include <string>
#include <iostream>
#include <iomanip>

#include <pcl/io/hdl_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>

class SimpleHDLGrabber 
{
  public:
    std::string calibrationFile, pcapFile;

    SimpleHDLGrabber (std::string& calibFile, std::string& pcapFile) 
      : calibrationFile (calibFile)
      , pcapFile (pcapFile) 
    {
    }

    void 
    sectorScan (
        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&, 
        float,
        float) 
    {
      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30) 
      {
        double now = pcl::getTime();
        std::cout << "got sector scan.  Avg Framerate " << double(count) / double(now - last) << " Hz" << std::endl;
        count = 0;
        last = now;
      }
    }

    void 
    sweepScan (
        const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >& sweep)
    {
      static unsigned count = 0;
      static double last = pcl::getTime();

      if (sweep->header.seq == 0) {
        pcl::uint64_t stamp;
        stamp = sweep->header.stamp;
        time_t systemTime = static_cast<time_t>(((stamp & 0xffffffff00000000l) >> 32) & 0x00000000ffffffff);
        pcl::uint32_t usec = static_cast<pcl::uint32_t>(stamp & 0x00000000ffffffff);
        std::cout << std::hex << stamp << "  " << ctime(&systemTime) << " usec: " << usec << std::endl;
      }

      if (++count == 30) 
      {
        double now = pcl::getTime ();
        std::cout << "got sweep.  Avg Framerate " << double(count) / double(now - last) << " Hz" << std::endl;
        count = 0;
        last = now;
      }
    }

    void 
    run () 
    {
      pcl::HDLGrabber interface (calibrationFile, pcapFile);
      // make callback function from member function
      boost::function<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&, float, float)> f =
          boost::bind(&SimpleHDLGrabber::sectorScan, this, _1, _2, _3);

      // connect callback function for desired signal. In this case its a sector with XYZ and intensity information
      //boost::signals2::connection c = interface.registerCallback(f);

      // Register a callback function that gets complete 360 degree sweeps.
      boost::function<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&)> f2 = boost::bind(
          &SimpleHDLGrabber::sweepScan, this, _1);
      boost::signals2::connection c2 = interface.registerCallback(f2);

      //interface.filterPackets(boost::asio::ip::address_v4::from_string("192.168.18.38"));

      // start receiving point clouds
      interface.start ();

      std::cout << R"(<Esc>, 'q', 'Q': quit the program)" << std::endl;
      char key;
      do 
      {
        key = static_cast<char> (getchar ());
      } while (key != 27 && key != 'q' && key != 'Q');

      // stop the grabber
      interface.stop ();
    }
};

int 
main (int argc, char **argv) 
{
	std::string hdlCalibration, pcapFile;

	pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);

	SimpleHDLGrabber grabber (hdlCalibration, pcapFile);
	grabber.run ();
	return (0);
}

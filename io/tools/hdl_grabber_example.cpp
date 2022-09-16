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
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&,
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
    sweepScan (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& sweep)
    {
      static unsigned count = 0;
      static double last = pcl::getTime();

      if (sweep->header.seq == 0) {
        std::uint64_t stamp;
        stamp = sweep->header.stamp;
        time_t systemTime = static_cast<time_t>(((stamp & 0xffffffff00000000l) >> 32) & 0x00000000ffffffff);
        auto usec = static_cast<std::uint32_t>(stamp & 0x00000000ffffffff);
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
      std::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&, float, float)> f =
          [this] (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& p1, float p2, float p3) { sectorScan (p1, p2, p3); };

      // connect callback function for desired signal. In this case its a sector with XYZ and intensity information
      //boost::signals2::connection c = interface.registerCallback(f);

      // Register a callback function that gets complete 360 degree sweeps.
      std::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f2 =
          [this] (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& sweep) { sweepScan (sweep); };
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

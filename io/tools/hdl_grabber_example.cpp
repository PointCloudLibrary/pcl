/*
 * hdl_grabber_example.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: keven
 */

#include <string>
#include <iostream>

#include <pcl/io/hdl_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>

class SimpleHDLGrabber {
public:
	std::string calibrationFile, pcapFile;

	SimpleHDLGrabber(std::string& calibFile, std::string& pcapFile) :
			calibrationFile(calibFile), pcapFile(pcapFile) {
	}

	void sectorScan(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& sector, float startAngle,
			float endAngle) {
		static unsigned count = 0;
		static double last = pcl::getTime();
		if (++count == 30) {
			double now = pcl::getTime();
			std::cout << "got sector scan.  Avg Framerate " << double(count) / double(now - last) << " Hz" << std::endl;
			count = 0;
			last = now;
		}
	}

	void sweepScan(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >& scan) {
		static unsigned count = 0;
		static double last = pcl::getTime();

		if (++count == 30) {
			double now = pcl::getTime();
			std::cout << "got sweep.  Avg Framerate " << double(count) / double(now - last) << " Hz" << std::endl;
			count = 0;
			last = now;
		}
	}

	void run() {
		pcl::HDL_Grabber interface(calibrationFile, pcapFile);

		// make callback function from member function
		boost::function<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&, float, float)> f =
				boost::bind(&SimpleHDLGrabber::sectorScan, this, _1, _2, _3);

		// connect callback function for desired signal. In this case its a sector with XYZ and intensity information
		//boost::signals2::connection c = interface.registerCallback(f);

		// Register a callback function that gets complete 360 degree sweeps.
		boost::function<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&)> f2 = boost::bind(
				&SimpleHDLGrabber::sweepScan, this, _1);
		boost::signals2::connection c2 = interface.registerCallback(f2);

		interface.filterPackets(boost::asio::ip::address_v4::from_string("192.168.18.38"));

		// start receiving point clouds
		interface.start();

		std::cout << "<Esc>, \'q\', \'Q\': quit the program" << std::endl;
		char key;
		do {
			key = static_cast<char>(getchar());
		} while (key != 27 && key != 'q' && key != 'Q');

		// stop the grabber
		interface.stop();
	}
};

int main(int argc, char **argv) {
	std::string hdlCalibration, pcapFile;

	pcl::console::parse_argument(argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument(argc, argv, "-pcapFile", pcapFile);

	SimpleHDLGrabber grabber(hdlCalibration, pcapFile);
	grabber.run();

	return (0);
}

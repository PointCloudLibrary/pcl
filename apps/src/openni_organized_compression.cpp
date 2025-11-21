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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#include <pcl/common/time.h>
#include <pcl/compression/organized_pointcloud_compression.h>
#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/asio.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <vector>

using boost::asio::ip::tcp;

using namespace pcl;
using namespace pcl::io;

using namespace std::chrono_literals;

char usage[] = "\n"
               "  PCL organized point cloud stream compression\n"
               "\n"
               "  usage: ./pcl_openni_organized_compression [mode] [parameters]\n"
               "\n"
               "  I/O: \n"
               "      -f file  : file name \n"
               "\n"
               "  file compression mode:\n"
               "      -x: encode point cloud stream to file\n"
               "      -d: decode from file and display point cloud stream\n"
               "\n"
               "  network streaming mode:\n"
               "      -s       : start server on localhost\n"
               "      -c host  : connect to server and display decoded cloud stream\n"
               "\n"
               "  optional compression parameters:\n"
               "      -a       : enable color coding\n"
               "      -t       : output statistics\n"
               "      -e       : show input cloud during encoding\n"
               "      -r       : raw encoding of disparity maps\n"
               "      -g       : gray scale conversion\n"

               "\n"
               "  example:\n"
               "      ./pcl_openni_organized_compression -x -t -f pc_compressed.pcc \n"
               "\n";

// clang-format off
#define FPS_CALC(_WHAT_)                                                               \
  do {                                                                                 \
    static unsigned count = 0;                                                         \
    static double last = pcl::getTime();                                               \
    double now = pcl::getTime();                                                       \
    ++count;                                                                           \
    if (now - last >= 1.0) {                                                           \
      std::cout << "Average framerate(" << _WHAT_ << "): "                             \
                << double(count) / double(now - last) << " Hz" << std::endl;           \
      count = 0;                                                                       \
      last = now;                                                                      \
    }                                                                                  \
  } while (false)
// clang-format on

void
print_usage(const std::string& msg)
{
  std::cerr << msg << std::endl;
  std::cout << usage << std::endl;
}

class SimpleOpenNIViewer {
public:
  SimpleOpenNIViewer(ostream& outputFile_arg,
                     OrganizedPointCloudCompression<PointXYZRGBA>* octreeEncoder_arg,
                     bool doColorEncoding_arg,
                     bool bShowStatistics_arg,
                     bool bRawImageEncoding_arg,
                     bool bGrayScaleConversion_arg,
                     int pngLevel_arg = -1)
  : viewer("Input Point Cloud - PCL Compression Viewer")
  , outputFile_(outputFile_arg)
  , organizedEncoder_(octreeEncoder_arg)
  , doColorEncoding_(doColorEncoding_arg)
  , bShowStatistics_(bShowStatistics_arg)
  , bRawImageEncoding_(bRawImageEncoding_arg)
  , bGrayScaleConversion_(bGrayScaleConversion_arg)
  , pngLevel_(pngLevel_arg)
  {}

  void
  cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
  {
    if (!viewer.wasStopped()) {
      organizedEncoder_->encodePointCloud(cloud,
                                          outputFile_,
                                          doColorEncoding_,
                                          bGrayScaleConversion_,
                                          bShowStatistics_,
                                          pngLevel_);

      viewer.showCloud(cloud);
    }
  }

  void
  run()
  {

    if (!bRawImageEncoding_) {
      // create a new grabber for OpenNI devices
      pcl::OpenNIGrabber interface;

      // make callback function from member function
      std::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
          [this](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) {
            cloud_cb_(cloud);
          };

      // connect callback function for desired signal. In this case its a point cloud
      // with color values
      boost::signals2::connection c = interface.registerCallback(f);

      // start receiving point clouds
      interface.start();

      while (!outputFile_.fail()) {
        std::this_thread::sleep_for(1s);
      }

      interface.stop();
    }
  }

  pcl::visualization::CloudViewer viewer;
  ostream& outputFile_;
  OrganizedPointCloudCompression<PointXYZRGBA>* organizedEncoder_;
  bool doColorEncoding_;
  bool bShowStatistics_;
  bool bRawImageEncoding_;
  bool bGrayScaleConversion_;
  int pngLevel_;
};

struct EventHelper {
  EventHelper(ostream& outputFile_arg,
              OrganizedPointCloudCompression<PointXYZRGBA>* octreeEncoder_arg,
              bool doColorEncoding_arg,
              bool bShowStatistics_arg,
              bool bRawImageEncoding_arg,
              bool bGrayScaleConversion_arg,
              int pngLevel_arg = -1)
  : outputFile_(outputFile_arg)
  , organizedEncoder_(octreeEncoder_arg)
  , doColorEncoding_(doColorEncoding_arg)
  , bShowStatistics_(bShowStatistics_arg)
  , bRawImageEncoding_(bRawImageEncoding_arg)
  , bGrayScaleConversion_(bGrayScaleConversion_arg)
  , pngLevel_(pngLevel_arg)
  {}

  void
  cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
  {
    if (!outputFile_.fail()) {
      organizedEncoder_->encodePointCloud(cloud,
                                          outputFile_,
                                          doColorEncoding_,
                                          bGrayScaleConversion_,
                                          bShowStatistics_,
                                          pngLevel_);
    }
  }

  void
  image_callback(const openni_wrapper::Image::Ptr& image,
                 const openni_wrapper::DepthImage::Ptr& depth_image,
                 float)
  {

    std::vector<std::uint16_t> disparity_data;
    std::vector<std::uint8_t> rgb_data;

    std::uint32_t width = depth_image->getWidth();
    std::uint32_t height = depth_image->getHeight();

    disparity_data.resize(width * height);
    depth_image->fillDepthImageRaw(
        width,
        height,
        &disparity_data[0],
        static_cast<unsigned int>(width * sizeof(std::uint16_t)));

    if (image->getEncoding() != openni_wrapper::Image::RGB) {
      rgb_data.resize(width * height * 3);
      image->fillRGB(width,
                     height,
                     &rgb_data[0],
                     static_cast<unsigned int>(width * sizeof(std::uint8_t) * 3));
    }

    organizedEncoder_->encodeRawDisparityMapWithColorImage(disparity_data,
                                                           rgb_data,
                                                           width,
                                                           height,
                                                           outputFile_,
                                                           doColorEncoding_,
                                                           bGrayScaleConversion_,
                                                           bShowStatistics_,
                                                           pngLevel_);
  }

  void
  run()
  {
    if (!bRawImageEncoding_) {
      // create a new grabber for OpenNI devices
      pcl::OpenNIGrabber interface;

      // make callback function from member function
      std::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
          [this](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) {
            cloud_cb_(cloud);
          };

      // connect callback function for desired signal. In this case its a point cloud
      // with color values
      boost::signals2::connection c = interface.registerCallback(f);

      // start receiving point clouds
      interface.start();

      while (!outputFile_.fail()) {
        std::this_thread::sleep_for(1s);
      }

      interface.stop();
    }
    else {
      pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
      int depthformat = openni_wrapper::OpenNIDevice::OpenNI_shift_values;

      pcl::OpenNIGrabber grabber(
          "", pcl::OpenNIGrabber::OpenNI_Default_Mode, image_mode);

      // Set the depth output format
      grabber.getDevice()->setDepthOutputFormat(
          static_cast<openni_wrapper::OpenNIDevice::DepthMode>(depthformat));

      std::function<void(const openni_wrapper::Image::Ptr&,
                         const openni_wrapper::DepthImage::Ptr&,
                         float)>
          image_cb = [this](const openni_wrapper::Image::Ptr& img,
                            const openni_wrapper::DepthImage::Ptr& depth,
                            float f) { image_callback(img, depth, f); };
      boost::signals2::connection image_connection = grabber.registerCallback(image_cb);

      grabber.start();
      while (!outputFile_.fail()) {
        std::this_thread::sleep_for(1s);
      }
      grabber.stop();
    }
  }

  std::ostream& outputFile_;
  OrganizedPointCloudCompression<PointXYZRGBA>* organizedEncoder_;
  bool doColorEncoding_;
  bool bShowStatistics_;
  bool bRawImageEncoding_;
  bool bGrayScaleConversion_;
  int pngLevel_;
};

int
main(int argc, char** argv)
{
  OrganizedPointCloudCompression<PointXYZRGBA>* organizedCoder;

  bool showStatistics;
  bool doColorEncoding;
  bool bShowInputCloud;
  bool bRawImageEncoding;
  bool bGrayScaleConversion;

  std::string fileName = "pc_compressed.pcc";
  std::string hostName = "localhost";

  bool bServerFileMode;
  bool bEnDecode;
  bool validArguments;

  validArguments = false;
  bServerFileMode = false;
  bEnDecode = false;

  showStatistics = false;
  doColorEncoding = false;
  bShowInputCloud = false;
  bRawImageEncoding = false;
  bGrayScaleConversion = false;

  if (pcl::console::find_argument(argc, argv, "-e") > 0)
    bShowInputCloud = true;

  if (pcl::console::find_argument(argc, argv, "-r") > 0)
    bRawImageEncoding = true;

  if (pcl::console::find_argument(argc, argv, "-g") > 0)
    bGrayScaleConversion = true;

  if (pcl::console::find_argument(argc, argv, "-s") > 0) {
    bEnDecode = true;
    bServerFileMode = true;
    validArguments = true;
  }

  if (pcl::console::parse_argument(argc, argv, "-c", hostName) > 0) {
    bEnDecode = false;
    bServerFileMode = true;
    validArguments = true;
  }

  if (pcl::console::find_argument(argc, argv, "-a") > 0) {
    doColorEncoding = true;
  }

  if (pcl::console::find_argument(argc, argv, "-x") > 0) {
    bEnDecode = true;
    bServerFileMode = false;
    validArguments = true;
  }

  if (pcl::console::find_argument(argc, argv, "-d") > 0) {
    bEnDecode = false;
    bServerFileMode = false;
    validArguments = true;
  }

  if (pcl::console::find_argument(argc, argv, "-t") > 0)
    showStatistics = true;

  pcl::console::parse_argument(argc, argv, "-f", fileName);

  if (pcl::console::find_argument(argc, argv, "-?") > 0) {
    print_usage("");
    return 1;
  }

  if (!validArguments) {
    print_usage("Please specify compression mode..\n");
    return -1;
  }

  organizedCoder = new OrganizedPointCloudCompression<PointXYZRGBA>();

  if (!bServerFileMode) {
    if (bEnDecode) {
      // ENCODING
      std::ofstream compressedPCFile;
      compressedPCFile.open(fileName.c_str(),
                            std::ios::out | std::ios::trunc | std::ios::binary);

      if (!bShowInputCloud) {
        EventHelper v(compressedPCFile,
                      organizedCoder,
                      doColorEncoding,
                      showStatistics,
                      bRawImageEncoding,
                      bGrayScaleConversion);
        v.run();
      }
      else {
        SimpleOpenNIViewer v(compressedPCFile,
                             organizedCoder,
                             doColorEncoding,
                             showStatistics,
                             bRawImageEncoding,
                             bGrayScaleConversion);
        v.run();
      }
    }
    else {
      // DECODING
      std::ifstream compressedPCFile;
      compressedPCFile.open(fileName.c_str(), std::ios::in | std::ios::binary);
      compressedPCFile.seekg(0);
      compressedPCFile.unsetf(std::ios_base::skipws);

      pcl::visualization::CloudViewer viewer("PCL Compression Viewer");

      while (!compressedPCFile.eof()) {
        PointCloud<PointXYZRGBA>::Ptr cloudOut(new PointCloud<PointXYZRGBA>());
        organizedCoder->decodePointCloud(compressedPCFile, cloudOut);
        viewer.showCloud(cloudOut);
      }
    }
  }
  else {
    if (bEnDecode) {
      // ENCODING
      try {
        boost::asio::io_service io_service;
        tcp::endpoint endpoint(tcp::v4(), 6666);
        tcp::acceptor acceptor(io_service, endpoint);

        tcp::iostream socketStream;

        std::cout << "Waiting for connection.." << std::endl;

        acceptor.accept(*socketStream.rdbuf());

        std::cout << "Connected!" << std::endl;

        if (!bShowInputCloud) {
          EventHelper v(socketStream,
                        organizedCoder,
                        doColorEncoding,
                        showStatistics,
                        bRawImageEncoding,
                        bGrayScaleConversion);
          v.run();
        }
        else {
          SimpleOpenNIViewer v(socketStream,
                               organizedCoder,
                               doColorEncoding,
                               showStatistics,
                               bRawImageEncoding,
                               bGrayScaleConversion);
          v.run();
        }

        std::cout << "Disconnected!" << std::endl;

        std::this_thread::sleep_for(3s);

      } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
      }
    }
    else {
      // DECODING
      std::cout << "Connecting to: " << hostName << ".." << std::endl;

      try {
        tcp::iostream socketStream(hostName.c_str(), "6666");

        std::cout << "Connected!" << std::endl;

        pcl::visualization::CloudViewer viewer(
            "Decoded Point Cloud - PCL Compression Viewer");

        while (!socketStream.fail()) {
          FPS_CALC("drawing");
          PointCloud<PointXYZRGBA>::Ptr cloudOut(new PointCloud<PointXYZRGBA>());
          organizedCoder->decodePointCloud(socketStream, cloudOut);
          viewer.showCloud(cloudOut);
        }

      } catch (std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
      }
    }
  }

  delete organizedCoder;
  return 0;
}

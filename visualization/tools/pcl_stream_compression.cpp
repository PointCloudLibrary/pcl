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


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include "pcl/compression/octree_pointcloud_compression.h"

#include <iostream>
#include <vector>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include "stdio.h"
#include <ctype.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include <boost/asio.hpp>

using boost::asio::ip::tcp;

using namespace pcl;
using namespace pcl::octree;

using namespace std;

char usage[] = "\n"
  "  PCL point cloud stream compression\n"
  "\n"
  "  usage: ./pcl_stream_compression [mode] [profile] [parameters]\n"
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
  "  optional compression profile: \n"
  "      -p profile : select compression profile:       \n"
  "                     -\"lowC\"  Low resolution with color\n"
  "                     -\"lowNC\" Low resolution without color\n"
  "                     -\"medC\" Medium resolution with color\n"
  "                     -\"medNC\" Medium resolution without color\n"
  "                     -\"highC\" High resolution with color\n"
  "                     -\"highNC\" High resolution without color\n"
  "\n"
  "  optional compression parameters:\n"
  "      -r prec  : point precision Hz\n"
  "      -o prec  : octree voxel size\n"
  "      -v       : enable voxel-grid downsampling\n"
  "      -a       : enable color coding\n"
  "      -i rate  : i-frame rate\n"
  "      -b bits  : bits/color component\n"
  "      -t       : output statistics\n"
  "\n"
  "  example:\n"
  "      ./pcl_stream_compression -x -p highC -t -f pc_compressed.pcc \n"
  "\n";

void
print_usage (std::string msg)
{
  std::cerr << msg << std::endl;
  std::cout << usage << std::endl;
}

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer ( ostream& outputFile_arg, PointCloudCompression<PointXYZRGB>* octreeEncoder_arg ) : viewer ("Input Point Cloud - PCL Compression Viewer"),
    outputFile_(outputFile_arg), octreeEncoder_(octreeEncoder_arg){}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped()) {

        PointCloud<PointXYZRGB>::Ptr cloudOut (new PointCloud<PointXYZRGB> ());

        octreeEncoder_->encodePointCloud ( cloud, outputFile_);

        viewer.showCloud (cloud);
      }
    }

    void run ()
    {

      // create a new grabber for OpenNI devices
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      // make callback function from member function
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = interface->registerCallback (f);

      // start receiving point clouds
      interface->start ();


      while (!outputFile_.fail())
      {
        sleep (1);
      }

      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
    ostream& outputFile_;
    PointCloudCompression<PointXYZRGB>* octreeEncoder_;



};


int main (int argc, char **argv)
{
  int c;
  PointCloudCompression<PointXYZRGB>* octreeCoder;

  pcl::octree::compression_Profiles_e compressionProfile;

  bool showStatistics;
  double pointResolution;
  double octreeResolution;
  bool doVoxelGridDownDownSampling;
  unsigned int iFrameRate;
  bool doColorEncoding;
  char colorBitResolution;

  // default values
  showStatistics = false;
  pointResolution = 0.001;
  octreeResolution = 0.01;
  doVoxelGridDownDownSampling = false;
  iFrameRate = 30;
  doColorEncoding = false;
  colorBitResolution = 6;
  compressionProfile = pcl::octree::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;

  std::string fileName = "pc_compressed.pcc";
  std::string hostName = "localhost";

  bool bServerFileMode;
  bool bEnDecode;
  bool validArguments;

  validArguments = false;
  bServerFileMode = false;
  bEnDecode = false;

  // decode command line arguments
  while ((c = getopt (argc, argv, "sc:taoxdr:o:vcb:i:sp:f:")) != -1)

    switch (c)
    {
      case 's':
        bEnDecode = true;
        bServerFileMode = true;
        validArguments = true;
        break;
      case 'c':
        bEnDecode = false;
        bServerFileMode = true;
        validArguments = true;
        hostName = optarg;
        break;
      case 'x':
        bEnDecode = true;
        bServerFileMode = false;
        validArguments = true;
        break;
      case 'd':
        bEnDecode = false;
        bServerFileMode = false;
        validArguments = true;
        break;
      case 't':
        showStatistics = true;
        break;
      case 'a':
        doColorEncoding = true;
        compressionProfile = pcl::octree::MANUAL_CONFIGURATION;
        break;
      case 'v':
        doVoxelGridDownDownSampling = true;
        compressionProfile = pcl::octree::MANUAL_CONFIGURATION;
        break;
      case 'f':
        fileName = optarg;
        break;
      case 'r':
        compressionProfile = pcl::octree::MANUAL_CONFIGURATION;
        pointResolution = atof (optarg);
        if (pointResolution <= 0)
        {
          print_usage ("Invalid resolution parameter..\n");
          return -1;
        }
        break;
      case 'i':
        compressionProfile = pcl::octree::MANUAL_CONFIGURATION;
        iFrameRate = atoi (optarg);
        if (iFrameRate <= 0)
        {
          print_usage ("Invalid iframe rate..\n");
          return -1;
        }
        break;
      case 'o':
        compressionProfile = pcl::octree::MANUAL_CONFIGURATION;
        octreeResolution = atof (optarg);
        if (octreeResolution <= 0)
        {
          print_usage ("Invalid octree voxel size parameter..\n");
          return -1;
        }
        break;
      case 'b':
        compressionProfile = pcl::octree::MANUAL_CONFIGURATION;
        colorBitResolution = atoi (optarg);
        if (colorBitResolution <= 0)
        {
          print_usage ("Invalid octree voxel size parameter..\n");
          return -1;
        }
        break;
      case 'p':
        if (strcmp (optarg, "lowC") == 0)
        {
          compressionProfile = pcl::octree::LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR;
        }
        else if (strcmp (optarg, "lowNC") == 0)
        {
          compressionProfile = pcl::octree::LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
        }
        else if (strcmp (optarg, "medC") == 0)
        {
          compressionProfile = pcl::octree::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR;
        }
        else if (strcmp (optarg, "medNC") == 0)
        {
          compressionProfile = pcl::octree::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
        }
        else if (strcmp (optarg, "highC") == 0)
        {
          compressionProfile = pcl::octree::HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR;
        }
        else if (strcmp (optarg, "highNC") == 0)
        {
          compressionProfile = pcl::octree::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
        }
        else
        {
          print_usage ("Unknown profile parameter..\n");
          return -1;
        }

        if (compressionProfile != MANUAL_CONFIGURATION)
        {
          // apply selected compression profile

          // retrieve profile settings
          const pcl::octree::configurationProfile_t selectedProfile = pcl::octree::compressionProfiles_[compressionProfile];

          // apply profile settings
          pointResolution = selectedProfile.pointResolution;
          octreeResolution = selectedProfile.octreeResolution;
          doVoxelGridDownDownSampling = selectedProfile.doVoxelGridDownSampling;
          iFrameRate = selectedProfile.iFrameRate;
          doColorEncoding = selectedProfile.doColorEncoding;
          colorBitResolution = selectedProfile.colorBitResolution;

        }

        break;
      case '?':
        if (isprint (optopt))
          fprintf (stderr, "Unknown option `-%c'.\n", optopt);
        else
          fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);

        print_usage ("");
        return 1;
      default:
        abort ();
    }

  if (!validArguments)
  {
    print_usage ("Please specify compression mode..\n");
    return -1;
  }

  octreeCoder = new PointCloudCompression<PointXYZRGB> (compressionProfile, showStatistics, pointResolution,
                                                          octreeResolution, doVoxelGridDownDownSampling, iFrameRate,
                                                          doColorEncoding, colorBitResolution);


  if (!bServerFileMode) {
    if (bEnDecode) {
      // ENCODING

      ofstream compressedPCFile;
      compressedPCFile.open (fileName.c_str(), ios::out | ios::trunc | ios::binary);

      SimpleOpenNIViewer v(compressedPCFile, octreeCoder);
      v.run ();
    } else
    {
      // DECODING

      ifstream compressedPCFile;
      compressedPCFile.open (fileName.c_str(), ios::in | ios::binary);
      compressedPCFile.seekg (0);
      compressedPCFile.unsetf (ios_base::skipws);

      pcl::visualization::CloudViewer viewer ("PCL Compression Viewer");

      while (!compressedPCFile.eof())
      {
        PointCloud<PointXYZRGB>::Ptr cloudOut (new PointCloud<PointXYZRGB> ());
        octreeCoder->decodePointCloud ( compressedPCFile, cloudOut);
        viewer.showCloud (cloudOut);
      }
    }
  } else
  {

    // switch to ONLINE profiles
    if (compressionProfile == pcl::octree::LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR)
    {
      compressionProfile = pcl::octree::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR;
    }
    else if (compressionProfile == pcl::octree::LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR)
    {
      compressionProfile = pcl::octree::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    }
    else if (compressionProfile == pcl::octree::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR)
    {
      compressionProfile = pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
    }
    else if (compressionProfile == pcl::octree::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR)
    {
      compressionProfile = pcl::octree::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    }
    else if (compressionProfile == pcl::octree::HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR)
    {
      compressionProfile = pcl::octree::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR;
    }
    else if (compressionProfile == pcl::octree::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR)
    {
      compressionProfile = pcl::octree::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    }

    if (bEnDecode)
    {
      // ENCODING
      try
      {
        boost::asio::io_service io_service;
        tcp::endpoint endpoint (tcp::v4 (), 6666);
        tcp::acceptor acceptor (io_service, endpoint);

        tcp::iostream socketStream;

        std::cout << "Waiting for connection.." << std::endl;

        acceptor.accept (*socketStream.rdbuf ());

        std::cout << "Connected!" << std::endl;

        SimpleOpenNIViewer v (socketStream, octreeCoder);
        // start receiving point clouds

        v.run ();

        std::cout << "Disconnected!" << std::endl;

        sleep (3);

      }
      catch (std::exception& e)
      {
        std::cerr << e.what () << std::endl;
      }
    }
    else
    {
      // DECODING

      std::cout << "Connecting to: " << hostName << ".." << std::endl;

      try
      {
        tcp::iostream socketStream (hostName.c_str (), "6666");

        std::cout << "Connected!" << std::endl;

        pcl::visualization::CloudViewer viewer ("Decoded Point Cloud - PCL Compression Viewer");

        while (!socketStream.fail()) {
          PointCloud<PointXYZRGB>::Ptr cloudOut (new PointCloud<PointXYZRGB> ());
          octreeCoder->decodePointCloud (socketStream, cloudOut);
          viewer.showCloud (cloudOut);
        }

      }
      catch (std::exception& e)
      {
        std::cout << "Exception: " << e.what () << std::endl;
      }
    }
  }


  delete(octreeCoder);

  return 0;
}


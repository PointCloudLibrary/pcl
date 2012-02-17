/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/convolution.h>
#include <pcl/visualization/pcl_visualizer.h>

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <filename> <-r|-c|-s> [-p <borders policy>] [-t <number of threads>] [-d <distance>]\n" << std::endl;
  std::cout << "Where options are:" << std::endl;
  std::cout << "\t\t\t-r convolve rows" << std::endl;
  std::cout << "\t\t\t-c convolve columns" << std::endl;
  std::cout << "\t\t\t-s convolve separate" << std::endl;
  std::cout << "\t\t\t-p borders policy" << std::endl;
  std::cout << "\t\t\t\t Z zero padding, default" << std::endl;
  std::cout << "\t\t\t\t D duplicate borders" << std::endl;
  std::cout << "\t\t\t\t M mirror borders" << std::endl;
  std::cout << "\t\t\t-t optional, number of threads, default 1" << std::endl;
  std::cout << "\t\t\t-d optional, distance threshold, default 0.001" << std::endl;
}

int
main (int argc, char ** argv)
{
  int direction = -1;
  int nb_threads = 0;
  char border_policy = 'Z';
  double threshold = 0.001;
	pcl::common::Convolution<pcl::PointXYZRGB> convolution;
	Eigen::ArrayXf gaussian_kernel(5);
	gaussian_kernel << 1.f/16, 1.f/4, 3.f/8, 1.f/4, 1.f/16;
  std::cout << "convolution kernel " << gaussian_kernel.transpose () << std::endl;

  if (argc < 3)
  {
    usage (argv);
    return 1;
  }

  // check if user is requesting help
  std::string arg (argv[1]);

  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }

  // user don't need help find convolving direction
  // convolve row
  if (pcl::console::find_switch (argc, argv, "-r") != -1)
    direction = 0;
  else 
  {
    // convolve column
    if (pcl::console::find_switch (argc, argv, "-c") != -1)
      direction = 1;
    else
      // convolve both
      if (pcl::console::find_switch (argc, argv, "-s") != -1)
        direction = 2;    
      else
      {
        // wrong direction given print usage
        usage (argv);
        return 1;
      } 
  }

  // number of threads if any
  if (pcl::console::parse_argument (argc, argv, "-t", nb_threads) != -1 )
  {
    if (nb_threads <= 0)
      nb_threads = 1;
  }
  convolution.setNumberOfThreads (nb_threads);
  
  // borders policy if any
  if (pcl::console::parse_argument (argc, argv, "-p", border_policy) != -1 )
  {
    switch (border_policy)
    {
      case 'Z' : convolution.setBordersPolicy (pcl::common::Convolution<pcl::PointXYZRGB>::IGNORE);
        break;
      case 'M' : convolution.setBordersPolicy (pcl::common::Convolution<pcl::PointXYZRGB>::MIRROR);
        break;
      case 'D' : convolution.setBordersPolicy (pcl::common::Convolution<pcl::PointXYZRGB>::DUPLICATE);
        break;
      default : 
      {
        usage (argv);
        return (1);
      }      
    }
  }
  else
    convolution.setBordersPolicy (pcl::common::Convolution<pcl::PointXYZRGB>::IGNORE);

  // distance threshold if any
  if (pcl::console::parse_argument (argc, argv, "-d", threshold) == -1 )
  {
    threshold = 0.001;
  }
  convolution.setDistanceThreshold ((float)threshold);  

  // all set
  // we have file name and convolving direction
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
  {
    std::cerr << "Couldn't read file " << argv[1] << std::endl;
    return (-1);
  }
	cloud->is_dense = false;
	convolution.setInputCloud (cloud);
	convolution.setKernel (gaussian_kernel);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convolved (new pcl::PointCloud<pcl::PointXYZRGB> ());

  double t0;
  std::cout << "convolving " << argv[1] << " along ";
  
  switch (direction)
  {
    case 0: 
    {
      std::cout << "rows... ";
      t0 = pcl::getTime ();
      convolution.convolveRows (*convolved);
    }    
    break;
    case 1: 
    {
      std::cout << "columns... ";
      t0 = pcl::getTime ();
      convolution.convolveCols (*convolved);
    }
    break;
    case 2: 
    { 
      std::cout << "rows and columns... ";
      t0 = pcl::getTime ();
      convolution.convolve (*convolved);
    }
    break;      
  }

  std::cout << "in " << pcl::getTime () - t0 << "s" << std::endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Convolution"));
  viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> convolved_color (convolved);
  viewer->addPointCloud<pcl::PointXYZRGB> (convolved, convolved_color, "convolved");	
	viewer->spin ();
}

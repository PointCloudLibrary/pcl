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
#include <pcl/console/print.h>
#include <pcl/filters/convolution.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>

void
usage (char ** argv)
{
  pcl::console::print_info ("usage: %s <filename> <-r|-c|-s> [-p <borders policy>] [-t <number of threads>] [-d <distance>]\n\n", argv[0]);
  pcl::console::print_info ("Where options are:\n");
  pcl::console::print_info ("\t\t\t-r convolve rows\n");
  pcl::console::print_info ("\t\t\t-c convolve columns\n");
  pcl::console::print_info ("\t\t\t-s convolve separate\n");
  pcl::console::print_info ("\t\t\t-p borders policy\n");
  pcl::console::print_info ("\t\t\t\t Z zero padding, default\n");
  pcl::console::print_info ("\t\t\t\t D duplicate borders\n");
  pcl::console::print_info ("\t\t\t\t M mirror borders\n");
  pcl::console::print_info ("\t\t\t-t optional, number of threads, default 1\n");
  pcl::console::print_info ("\t\t\t-d optional, distance threshold, default 0.001\n");
}

int
main (int argc, char ** argv)
{
  int viewport_source, viewport_convolved = 0;
  int direction = -1;
  int nb_threads = 1;
  char border_policy = 'Z';
  double threshold = 0.001;
  pcl::filters::Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB> convolution;
  Eigen::ArrayXf gaussian_kernel(5);
  gaussian_kernel << 1.f/16, 1.f/4, 3.f/8, 1.f/4, 1.f/16;
  pcl::console::print_info ("convolution kernel:");
  for (int i = 0; i < gaussian_kernel.size (); ++i)
    pcl::console::print_info (" %f", gaussian_kernel[i]);
  pcl::console::print_info ("\n");

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
  if (pcl::console::find_switch (argc, argv, "-r"))
    direction = 0;
  else
  {
    // convolve column
    if (pcl::console::find_switch (argc, argv, "-c"))
      direction = 1;
    else
      // convolve both
      if (pcl::console::find_switch (argc, argv, "-s"))
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
#ifndef _OPENMP
    if (nb_threads > 1)
    {
      pcl::console::print_info ("OpenMP not activated. Number of threads: 1\n");
      nb_threads = 1;
    }
#endif
  }
#ifdef _OPENMP
  else
  {
    nb_threads = omp_get_num_procs();
  }
#endif
  convolution.setNumberOfThreads (nb_threads);

  // borders policy if any
  if (pcl::console::parse_argument (argc, argv, "-p", border_policy) != -1 )
  {
    switch (border_policy)
    {
      case 'Z' : convolution.setBordersPolicy (pcl::filters::Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::BORDERS_POLICY_IGNORE);
        break;
      case 'M' : convolution.setBordersPolicy (pcl::filters::Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::BORDERS_POLICY_MIRROR);
        break;
      case 'D' : convolution.setBordersPolicy (pcl::filters::Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::BORDERS_POLICY_DUPLICATE);
        break;
      default :
      {
        usage (argv);
        return (1);
      }
    }
  }
  else
    convolution.setBordersPolicy (pcl::filters::Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::BORDERS_POLICY_IGNORE);

  // distance threshold if any
  if (pcl::console::parse_argument (argc, argv, "-d", threshold) == -1 )
  {
    threshold = 0.01;
  }
  convolution.setDistanceThreshold (static_cast<float> (threshold));

  // all set
  // we have file name and convolving direction
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
  {
    pcl::console::print_error ("Couldn't read file %s \n", argv[1]);
    return (-1);
  }
  cloud->is_dense = false;
  convolution.setInputCloud (cloud);
  convolution.setKernel (gaussian_kernel);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convolved (new pcl::PointCloud<pcl::PointXYZRGB> ());
  double t0;
  pcl::console::print_info ("convolving %s along \n", argv[1]);
  std::ostringstream convolved_label;
  convolved_label << "convolved along ";
  switch (direction)
  {
    case 0:
    {
      convolved_label << "rows... ";
      t0 = pcl::getTime ();
      convolution.convolveRows (*convolved);
      break;
    }
    case 1:
    {
      convolved_label << "columns... ";
      t0 = pcl::getTime ();
      convolution.convolveCols (*convolved);
      break;
    }
    case 2:
    {
      convolved_label << "rows and columns... ";
      t0 = pcl::getTime ();
      convolution.convolve (*convolved);
      break;
    }
  }
  convolved_label << pcl::getTime () - t0 << "s";
#ifdef _OPENMP
  convolved_label << "\ncpu cores: " << omp_get_num_procs() << " ";
#else
  convolved_label << "\n";
#endif  
  convolved_label << "threads: " << nb_threads;
  // Display
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Convolution"));
  // viewport stuff
  viewer->createViewPort (0, 0, 0.5, 1, viewport_source);
  viewer->createViewPort (0.5, 0, 1, 1, viewport_convolved);
  viewer->setBackgroundColor (0, 0, 0);

  // Source
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler_source (cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, color_handler_source, "source", viewport_source);
  viewer->addText ("source", 10, 10, "source_label", viewport_source);

  // Convolved
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler_convolved (convolved);
  viewer->addPointCloud<pcl::PointXYZRGB> (convolved, color_handler_convolved, "convolved", viewport_convolved);
  viewer->addText (convolved_label.str (), 10, 10, "convolved_label", viewport_convolved);
  viewer->spin ();
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("convolved.pcd", *convolved, false);
}

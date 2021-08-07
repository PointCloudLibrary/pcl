/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *
 */
#include <pcl/io/image_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp> // for exists

using pcl::console::print_error;
using pcl::console::print_info;
using pcl::console::print_value;

pcl::ImageGrabber<pcl::PointXYZRGBA>::Ptr grabber;
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_;
std::string out_folder;
int counter;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s <options>\n", argv[0]);
  print_info (" where options are:\n");
  print_info ("\t-rgb_dir   \t<directory_path>    \t= directory path to RGB images to be read from\n");
  print_info ("\t-depth_dir \t<directory_path>    \t= directory path to Depth images to be read from\n");
  print_info ("\t-out_dir   \t<directory_path>    \t= directory path to put the pcd files\n");
  //print_info ("\t-fps frequency           = frames per second\n");
  print_info ("\n");
}

struct EventHelper
{
  void 
  cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
  {
    const std::string filepath = out_folder + '/' + grabber->getPrevDepthFileName() + ".pcd";
    pcl::io::savePCDFileASCII (filepath, *cloud);
  }
};

/* ---[ */
int
main (int argc, char** argv)
{
  counter = 0;
  out_folder.clear();

  if (argc > 1)
  {
    for (int i = 1; i < argc; i++)
    {
      if (std::string (argv[i]) == "-h")
      {
        printHelp (argc, argv);
        return (-1);
      }
    }
  }

  float frames_per_second = 0; // 0 means only if triggered!
  pcl::console::parse (argc, argv, "-fps", frames_per_second);
  if (frames_per_second < 0)
    frames_per_second = 0.0;

  float focal_length = 525.0;
  pcl::console::parse (argc, argv, "-focal", focal_length);

  std::string depth_path;
  pcl::console::parse_argument (argc, argv, "-depth_dir", depth_path);

  std::string rgb_path;
  pcl::console::parse_argument (argc, argv, "-rgb_dir", rgb_path);

  pcl::console::parse_argument (argc, argv, "-out_dir", out_folder);

  if (out_folder.empty() || !boost::filesystem::exists (out_folder))
  {
    PCL_INFO("No correct directory was given with the -out_dir flag. Setting to current dir\n");
    out_folder = "./";
  }
  else
    PCL_INFO("Using %s as output dir", out_folder.c_str());

  if (!rgb_path.empty() && !depth_path.empty() && boost::filesystem::exists (rgb_path) && boost::filesystem::exists (depth_path))
  {
    grabber.reset (new pcl::ImageGrabber<pcl::PointXYZRGBA> (depth_path, rgb_path, frames_per_second, false));
  }
  else
  {
    PCL_INFO("No directory was given with the -<rgb/depth>_dir flag.");
    printHelp (argc, argv);
    return (-1);
  }
  grabber->setDepthImageUnits (float (1E-3));
  //grabber->setFocalLength(focal_length); // FIXME

  EventHelper h;
  std::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f = [&] (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
  {
    h.cloud_cb (cloud);
  };
  boost::signals2::connection c1 = grabber->registerCallback (f);

  do
  {
    grabber->trigger();
  }
  while (!grabber->atLastFrame());
  grabber->trigger(); // Attempt to process the last frame
  grabber->stop ();
}
/* ]--- */

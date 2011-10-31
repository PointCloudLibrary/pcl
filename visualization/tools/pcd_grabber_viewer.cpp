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
 * Author: Radu B. Rusu, Suat Gedikli
 *
 */
#include <pcl/io/pcd_grabber.h>
#include <pcl/console/parse.h>
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/console/print.h>
#include <pcl/visualization/cloud_viewer.h>

using pcl::console::print_error;
using pcl::console::print_info;
using pcl::console::print_value;

boost::mutex mutex_;

void
printHelp (int argc, char **argv)
{
  //print_error ("Syntax is: %s <file_name 1..N>.pcd <options>\n", argv[0]);
  print_error ("Syntax is: %s <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -file file_name          = PCD file to be read from\n");
  print_info ("                     -dir directory_path      = directory path to PCD file(s) to be read from\n");
  print_info ("                     -fps frequency           = frames per second\n");
  print_info ("                     -repeat                  = optional parameter that tells wheter the PCD file(s) should be \"grabbed\" in a endless loop.\n");
  print_info ("\n");
  print_info ("                     -cam (*)                 = use given camera settings as initial view\n");
  print_info (stderr, " (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Window Size / Window Pos] or use a <filename.cam> that contains the same information.\n");
}

// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> p;
std::vector<double> fcolor_r, fcolor_b, fcolor_g;
bool fcolorparam = false;

struct EventHelper
{
  void 
  cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud)
  {
    // Add the cloud to the renderer
    boost::mutex::scoped_lock lock (mutex_);
    if (!cloud)
      return;
    if (!p->updatePointCloud (cloud, "PCDCloud"))
    {
      p->addPointCloud (cloud, "PCDCloud");
      p->resetCameraViewpoint ("PCDCloud");
    }
  }
};

void 
keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  std::string* message = (std::string*)cookie;
  cout << (*message) << " :: ";
  if (event.getKeyCode())
    cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
  else
    cout << "the special key \'" << event.getKeySym() << "\' was";
  if (event.keyDown())
    cout << " pressed" << endl;
  else
    cout << " released" << endl;
}

void mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void* cookie)
{
  std::string* message = (std::string*) cookie;
  if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
  {
    cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  srand (time (0));

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

  // Command line parsing
  double bcolor[3] = {0, 0, 0};
  pcl::console::parse_3x_arguments (argc, argv, "-bc", bcolor[0], bcolor[1], bcolor[2]);

  fcolorparam = pcl::console::parse_multiple_3x_arguments (argc, argv, "-fc", fcolor_r, fcolor_g, fcolor_b);

  int psize = 0;
  pcl::console::parse_argument (argc, argv, "-ps", psize);

  double opaque;
  pcl::console::parse_argument (argc, argv, "-opaque", opaque);

  p.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));

  //  // Change the cloud rendered point size
  //  if (psize > 0)
  //    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "OpenNICloud");
  //
  //  // Change the cloud rendered opacity
  //  if (opaque >= 0)
  //    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque, "OpenNICloud");

  p->setBackgroundColor (bcolor[0], bcolor[1], bcolor[2]);

  // Read axes settings
  double axes = 0.0;
  pcl::console::parse_argument (argc, argv, "-ax", axes);
  if (axes != 0.0 && p)
  {
    double ax_x = 0.0, ax_y = 0.0, ax_z = 0.0;
    pcl::console::parse_3x_arguments (argc, argv, "-ax_pos", ax_x, ax_y, ax_z, false);
    // Draw XYZ axes if command-line enabled
    p->addCoordinateSystem (axes, ax_x, ax_y, ax_z);
  }

  pcl::Grabber* grabber = 0;

  float frames_per_second = 0; // 0 means only if triggered!
  pcl::console::parse (argc, argv, "-fps", frames_per_second);
  if (frames_per_second < 0)
    frames_per_second = 0.0;

  std::cout << pcl::console::find_argument (argc, argv, "-repeat") << " : repaet" << std::endl;
  bool repeat = (pcl::console::find_argument (argc, argv, "-repeat") != -1);

  std::cout << "fps: " << frames_per_second << " , repeat: " << repeat << std::endl;
  std::string path = "";
  pcl::console::parse_argument (argc, argv, "-file", path);
  std::cout << "path: " << path << std::endl;
  if (path != "" && boost::filesystem::exists (path))
  {
    grabber = new pcl::PCDGrabber<pcl::PointXYZRGB> (path, frames_per_second, repeat);
  }
  else
  {
    std::vector<std::string> pcd_files;
    pcl::console::parse_argument (argc, argv, "-dir", path);
    std::cout << "path: " << path << std::endl;
    if (path != "" && boost::filesystem::exists (path))
    {
      boost::filesystem::directory_iterator end_itr;
      for (boost::filesystem::directory_iterator itr (path); itr != end_itr; ++itr)
      {
        if (!is_directory (itr->status()) && boost::algorithm::to_upper_copy(boost::filesystem::extension (itr->leaf())) == ".PCD" )
        {
          pcd_files.push_back (itr->path ().string());
          std::cout << "added: " << itr->path ().string() << std::endl;
        }
      }
    }
    else
    {
      std::cout << "Neither a pcd file given using the \"-file\" option, nor given a directory containing pcd files using the \"-dir\" option." << std::endl;
    }
    grabber = new pcl::PCDGrabber<pcl::PointXYZRGB> (pcd_files, frames_per_second, repeat);
  }

  EventHelper h;
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&) > f = boost::bind (&EventHelper::cloud_cb, &h, _1);
  boost::signals2::connection c1 = grabber->registerCallback (f);

  std::string mouseMsg3D ("Mouse coordinates in PCL Visualizer");
  std::string keyMsg3D ("Key event for PCL Visualizer");

  p->registerMouseCallback (&mouse_callback, (void*)(&mouseMsg3D));    
  p->registerKeyboardCallback(&keyboard_callback, (void*)(&keyMsg3D));
        
  grabber->start ();
  while (true)
  {
    boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    {
      boost::mutex::scoped_lock lock (mutex_);
      p->spinOnce ();
      if (p->wasStopped ())
        break;
    }
  }

  grabber->stop ();
}
/* ]--- */

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
#include <pcl/io/pcd_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>

#if (PCL_LINEAR_VERSION(VTK_MAJOR_VERSION,VTK_MINOR_VERSION,0)<=PCL_LINEAR_VERSION(5,4,0))
  #define DISPLAY_IMAGE
#endif

using pcl::console::print_error;
using pcl::console::print_info;
using pcl::console::print_value;

boost::mutex mutex_;
boost::shared_ptr<pcl::PCDGrabber<pcl::PointXYZRGBA> > grabber;
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_;

void
printHelp (int, char **argv)
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
boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer;
#ifdef DISPLAY_IMAGE
boost::shared_ptr<pcl::visualization::ImageViewer> img_viewer;
#endif

std::vector<double> fcolor_r, fcolor_b, fcolor_g;
bool fcolorparam = false;

struct EventHelper
{
  void 
  cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
  {
    if (mutex_.try_lock ())
    {
      cloud_ = cloud;
      mutex_.unlock ();
    }
  }
};

void 
keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
{
  /// If SPACE is pressed, trigger new cloud callback (only works if framerate is set to 0)
  if (event.getKeyCode() == ' ' && grabber)
    grabber->trigger ();
}

void 
mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void* cookie)
{
  std::string* message = static_cast<std::string*> (cookie);
  if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
  {
    cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  srand (unsigned (time (0)));

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

  cloud_viewer.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));

#ifdef DISPLAY_IMAGE
  img_viewer.reset (new pcl::visualization::ImageViewer ("OpenNI Viewer"));
#endif

  //  // Change the cloud rendered point size
  //  if (psize > 0)
  //    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "OpenNICloud");
  //
  //  // Change the cloud rendered opacity
  //  if (opaque >= 0)
  //    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque, "OpenNICloud");

  cloud_viewer->setBackgroundColor (bcolor[0], bcolor[1], bcolor[2]);

  // Read axes settings
  double axes = 0.0;
  pcl::console::parse_argument (argc, argv, "-ax", axes);
  if (axes != 0.0 && cloud_viewer)
  {
    float ax_x = 0.0, ax_y = 0.0, ax_z = 0.0;
    pcl::console::parse_3x_arguments (argc, argv, "-ax_pos", ax_x, ax_y, ax_z, false);
    // Draw XYZ axes if command-line enabled
    cloud_viewer->addCoordinateSystem (axes, ax_x, ax_y, ax_z, "global");
  }

  float frames_per_second = 0; // 0 means only if triggered!
  pcl::console::parse (argc, argv, "-fps", frames_per_second);
  if (frames_per_second < 0)
    frames_per_second = 0.0;


  bool repeat = (pcl::console::find_argument (argc, argv, "-repeat") != -1);

  std::cout << "fps: " << frames_per_second << " , repeat: " << repeat << std::endl;
  std::string path = "";
  pcl::console::parse_argument (argc, argv, "-file", path);
  std::cout << "path: " << path << std::endl;
  if (path != "" && boost::filesystem::exists (path))
  {
    grabber.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> (path, frames_per_second, repeat));
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
#if BOOST_FILESYSTEM_VERSION == 3
        if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".PCD" )
#else
        if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->leaf ())) == ".PCD" )
#endif
        {
#if BOOST_FILESYSTEM_VERSION == 3
          pcd_files.push_back (itr->path ().string ());
          std::cout << "added: " << itr->path ().string () << std::endl;
#else
          pcd_files.push_back (itr->path ().string ());
          std::cout << "added: " << itr->path () << std::endl;
#endif
        }
      }
    }
    else
    {
      std::cout << "Neither a pcd file given using the \"-file\" option, nor given a directory containing pcd files using the \"-dir\" option." << std::endl;
    }

    // Sort the read files by name
    sort (pcd_files.begin (), pcd_files.end ());
    grabber.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> (pcd_files, frames_per_second, repeat));
  }

  EventHelper h;
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f = boost::bind (&EventHelper::cloud_cb, &h, _1);
  boost::signals2::connection c1 = grabber->registerCallback (f);

  std::string mouse_msg_3D ("Mouse coordinates in PCL Visualizer");
  std::string key_msg_3D ("Key event for PCL Visualizer");

  cloud_viewer->registerMouseCallback (&mouse_callback, static_cast<void*> (&mouse_msg_3D));
  cloud_viewer->registerKeyboardCallback(&keyboard_callback, static_cast<void*> (&key_msg_3D));

  std::string mouse_msg_2D ("Mouse coordinates in image viewer");
  std::string key_msg_2D ("Key event for image viewer");

#ifdef DISPLAY_IMAGE
  img_viewer->registerMouseCallback (&mouse_callback, static_cast<void*> (&mouse_msg_2D));
  img_viewer->registerKeyboardCallback(&keyboard_callback, static_cast<void*> (&key_msg_2D));
#endif

  grabber->start ();
  while (!cloud_viewer->wasStopped ())
  {
    cloud_viewer->spinOnce ();

#ifdef DISPLAY_IMAGE
    img_viewer->spinOnce ();
#endif

    if (!cloud_)
    {
      boost::this_thread::sleep(boost::posix_time::microseconds(10000));
      continue;
    }
    else if (mutex_.try_lock ())
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr temp_cloud;
      temp_cloud.swap (cloud_);
      mutex_.unlock ();

#ifdef DISPLAY_IMAGE
      img_viewer->showRGBImage (*temp_cloud);
#endif

      if (!cloud_viewer->updatePointCloud (temp_cloud, "PCDCloud"))
      {
        cloud_viewer->addPointCloud (temp_cloud, "PCDCloud");
        cloud_viewer->resetCameraViewpoint ("PCDCloud");
      }
    }
  }

  grabber->stop ();
}
/* ]--- */

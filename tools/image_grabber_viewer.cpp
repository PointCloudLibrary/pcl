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
#include <pcl/visualization/boost.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/pcd_io.h>

#include <mutex>
#include <thread>

using namespace std::chrono_literals;
using pcl::console::print_error;
using pcl::console::print_info;
using pcl::console::print_value;

std::mutex mutex_;
boost::shared_ptr<pcl::ImageGrabber<pcl::PointXYZRGBA> > grabber;
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_;

void
printHelp (int, char **argv)
{
  //print_error ("Syntax is: %s <file_name 1..N>.tiff <options>\n", argv[0]);
  print_error ("Syntax is: %s <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("\t-dir directory_path      = directory path to image or pclzf file(s) to be read from\n");
  print_info ("\t-fps frequency           = frames per second\n");
  print_info ("\t-pclzf                   = Load pclzf files instead\n");
  print_info ("\t-repeat                  = optional parameter that tells whether the TIFF file(s) should be \"grabbed\" in a endless loop.\n");
  print_info ("\n");
  print_info ("\t-cam (*)                 = use given camera settings as initial view\n");
  print_info (stderr, " (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Window Size / Window Pos] or use a <filename.cam> that contains the same information.\n");

  print_info ("Additional options:\n");
  print_info ("\t-bc col1 col2 col3       = background color\n");
  print_info ("\t-ax                      = use custom coordinate system\n");
  print_info ("\t-ax_pos pos1 pos2 pos3   = the axes coordinates\n");
  print_info ("\t-focal focal_length      = ImageGrabber focal length\n");
  // Following ones are not implemented
//  print_info ("\t-fc col1 col2 col3       = foreground color\n");
//  print_info ("\t-pc                      = point size\n");
//  print_info ("\t-opaque                  = point opacity\n");
}

// Create the PCLVisualizer object
pcl::visualization::PCLVisualizer::Ptr cloud_viewer;
#ifdef DISPLAY_IMAGE
pcl::visualization::ImageViewer::Ptr img_viewer;
#endif

std::vector<double> fcolor_r, fcolor_b, fcolor_g;
bool fcolorparam = false;

struct EventHelper
{
  void 
  cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
  {
    pcl::uint64_t timestamp;
    timestamp = cloud->header.stamp;
    if (timestamp > 0)
      PCL_INFO ("Acquired cloud with timestamp of %lu\n", timestamp);
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
  srand (unsigned (time (nullptr)));

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

  bool use_pclzf = (pcl::console::find_argument (argc, argv, "-pclzf") != -1);

  std::cout << "fps: " << frames_per_second << " , repeat: " << repeat << std::endl;
  std::string path;
  pcl::console::parse_argument (argc, argv, "-dir", path);
  std::cout << "path: " << path << std::endl;
  if (!path.empty() && boost::filesystem::exists (path))
  {
    grabber.reset (new pcl::ImageGrabber<pcl::PointXYZRGBA> (path, frames_per_second, repeat, use_pclzf));
  }
  else
  {
    std::cout << "No directory was given with the -dir flag." << std::endl;
    printHelp (argc, argv);
    return (-1);
  }
  grabber->setDepthImageUnits (float (1E-3));

  // Before manually setting
  double fx, fy, cx, cy;
  grabber->getCameraIntrinsics (fx, fy, cx, cy);
  PCL_INFO ("Factory default intrinsics: %f, %f, %f, %f\n", 
      fx, fy, cx, cy);
  float focal_length;
  if (pcl::console::parse (argc, argv, "-focal", focal_length) != -1)
    grabber->setCameraIntrinsics (focal_length, focal_length, 320, 240);
  
  

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

  grabber->getCameraIntrinsics (fx, fy, cx, cy);
  PCL_INFO ("Grabber is using intrinsics: %f, %f, %f, %f\n", 
      fx, fy, cx, cy);

  while (!cloud_viewer->wasStopped ())
  {
    cloud_viewer->spinOnce ();

#ifdef DISPLAY_IMAGE
    img_viewer->spinOnce ();
#endif

    if (!cloud_)
    {
      std::this_thread::sleep_for(10ms);
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
        cloud_viewer->setCameraPosition (0, 0, 0, 0, 0, 1, 0, -1, 0);
      }
    }
  }

  grabber->stop ();
}
/* ]--- */

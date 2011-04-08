/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: pcd_viewer.cpp 35932 2011-02-10 03:55:03Z rusu $
 *
 */
// PCL
#include <boost/thread/thread.hpp>
#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/kinect_grabber.h>
#include <cfloat>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/terminal_tools/print.h>
#include <pcl/terminal_tools/parse.h>
#include <pcl/terminal_tools/time.h>

using terminal_tools::print_color;
using terminal_tools::print_error;
using terminal_tools::print_error;
using terminal_tools::print_warn;
using terminal_tools::print_info;
using terminal_tools::print_debug;
using terminal_tools::print_value;
using terminal_tools::print_highlight;
using terminal_tools::TT_BRIGHT;
using terminal_tools::TT_RED;
using terminal_tools::TT_GREEN;
using terminal_tools::TT_BLUE;

typedef pcl_visualization::PointCloudColorHandler<pcl::PointCloud<pcl::PointXYZ> > ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef pcl_visualization::PointCloudGeometryHandler<pcl::PointCloud<pcl::PointXYZ> > GeometryHandler;
typedef GeometryHandler::Ptr GeometryHandlerPtr;
typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;
boost::mutex mutex_;

#define NORMALS_SCALE 0.01
#define PC_SCALE 0.001

void
  printHelp (int argc, char **argv)
{
  //print_error ("Syntax is: %s <file_name 1..N>.pcd <options>\n", argv[0]);
  print_error ("Syntax is: %s <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -dev device_id           = device to be used\n");
  print_info ("                                                maybe \"#n\", with n being the number of the device in device list.\n");
  print_info ("                                                maybe \"bus@addr\", with bus and addr being the usb bus and address where device is connected.\n");
  print_info ("                                                maybe \"serial\", with serial being the serial number of the device.\n");
  print_info ("                     -bc r,g,b                = background color\n");
  print_info ("                     -fc r,g,b                = foreground color\n");
  print_info ("                     -ps X                    = point size ("); print_value ("1..64"); print_info (") \n");
  print_info ("                     -opaque X                = rendered point cloud opacity ("); print_value ("0..1"); print_info (")\n");

  print_info ("                     -ax "); print_value ("n"); print_info ("                    = enable on-screen display of "); 
  print_color (stdout, TT_BRIGHT, TT_RED, "X"); print_color (stdout, TT_BRIGHT, TT_GREEN, "Y"); print_color (stdout, TT_BRIGHT, TT_BLUE, "Z");
  print_info (" axes and scale them to "); print_value ("n\n");
  print_info ("                     -ax_pos X,Y,Z            = if axes are enabled, set their X,Y,Z position in space (default "); print_value ("0,0,0"); print_info (")\n");

  print_info ("\n");
  print_info ("                     -cam (*)                 = use given camera settings as initial view\n");
  print_info (stderr, " (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Window Size / Window Pos] or use a <filename.cam> that contains the same information.\n");

  print_info ("\n");
  print_info ("                     -multiview 0/1           = enable/disable auto-multi viewport rendering (default "); print_value ("disabled"); print_info (")\n");
  print_info ("\n");

  print_info ("\n");
  print_info ("                     -normals 0/X             = disable/enable the display of every Xth point's surface normal as lines (default "); print_value ("disabled"); print_info (")\n");
  print_info ("                     -normals_scale X         = resize the normal unit vector size to X (default "); print_value ("0.02"); print_info (")\n");
  print_info ("\n");
  print_info ("                     -pc 0/X                  = disable/enable the display of every Xth point's principal curvatures as lines (default "); print_value ("disabled"); print_info (")\n");
  print_info ("                     -pc_scale X              = resize the principal curvatures vectors size to X (default "); print_value ("0.02"); print_info (")\n");
  print_info ("\n");

  print_info ("\n(Note: for multiple .pcd files, provide multiple -{fc,ps,opaque} parameters; they will be automatically assigned to the right file)\n");
}

// Create the PCLVisualizer object
boost::shared_ptr<pcl_visualization::PCLVisualizer> p;
ColorHandlerPtr color_handler;
GeometryHandlerPtr geometry_handler;
std::vector<double> fcolor_r, fcolor_b, fcolor_g;
bool fcolorparam = false;

struct EventHelper
{
  void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
      //std::cout << __PRETTY_FUNCTION__ << " " << cloud->width << std::endl;
    // Add the dataset with a XYZ and a random handler 
  //  geometry_handler.reset (new pcl_visualization::PointCloudGeometryHandlerXYZ<pcl::PointCloud<pcl::PointXYZRGB> > (*cloud));

    //// If color was given, ues that
    //if (fcolorparam)
    //  color_handler.reset (new pcl_visualization::PointCloudColorHandlerCustom<pcl::PointCloud<pcl::PointXYZRGB> > (cloud, fcolor_r, fcolor_g, fcolor_b));
    //else
    //  color_handler.reset (new pcl_visualization::PointCloudColorHandlerRandom<pcl::PointCloud<pcl::PointXYZRGB> > (cloud));

    // Add the cloud to the renderer

    boost::mutex::scoped_lock lock(mutex_);
    if (!cloud)
      return;
    p->removePointCloud ("KinectCloud");
    p->addPointCloud (*cloud, "KinectCloud");
  }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  srand (time (0));

  if (argc > 1)
  {
    for (int i = 1; i < argc; i++)
    {
      if (std::string(argv[i]) == "-h")
      {
        printHelp (argc, argv);
        return (-1);
      }
    }
  }

  // Command line parsing
  double bcolor[3] = {0, 0, 0};
  terminal_tools::parse_3x_arguments (argc, argv, "-bc", bcolor[0], bcolor[1], bcolor[2]);

  fcolorparam = terminal_tools::parse_multiple_3x_arguments (argc, argv, "-fc", fcolor_r, fcolor_g, fcolor_b);

  int psize = 0;
  terminal_tools::parse_argument (argc, argv, "-ps", psize);

  double opaque;
  terminal_tools::parse_argument (argc, argv, "-opaque", opaque);

  p.reset (new pcl_visualization::PCLVisualizer (argc, argv, "PCD viewer"));

//  // Change the cloud rendered point size
//  if (psize > 0)
//    p->setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "KinectCloud");
//
//  // Change the cloud rendered opacity
//  if (opaque >= 0)
//    p->setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_OPACITY, opaque, "KinectCloud");

  p->setBackgroundColor (bcolor[0], bcolor[1], bcolor[2]);

  

  //boost::signals2::connection c = interface->registerCallback (boost::bind (&bla::blatestpointcloudrgb, *this, _1));
  //boost::function<void (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >)> f = boost::bind (&bla::blatestpointcloudrgb, this, _1);
  //boost::signals2::connection c =
  //  interface->registerCallback <void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >)> (boost::bind (&bla::blatestpointcloudrgb, *this, _1).);

  // Read axes settings
  double axes  = 0.0;
  terminal_tools::parse_argument (argc, argv, "-ax", axes);
  if (axes != 0.0 && p)
  {
    double ax_x = 0.0, ax_y = 0.0, ax_z = 0.0;
    terminal_tools::parse_3x_arguments (argc, argv, "-ax_pos", ax_x, ax_y, ax_z, false);
    // Draw XYZ axes if command-line enabled
    p->addCoordinateSystem (axes, ax_x, ax_y, ax_z);
  }


  std::string device_id = "";
  terminal_tools::parse_argument (argc, argv, "-dev", device_id);

  pcl::Grabber* interface = new pcl::OpenNIGrabber(device_id);
  
  EventHelper h;
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind(&EventHelper::cloud_cb, &h, _1);
  boost::signals2::connection c1 = interface->registerCallback (f);
  
  interface->start ();
  while (true)
  {
    boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    {
      boost::mutex::scoped_lock lock(mutex_);
      p->spinOnce ();
      if (p->wasStopped ())
        break;
    }
  }

  interface->stop ();
}
/* ]--- */

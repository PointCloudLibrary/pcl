/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
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
 */

#include <pcl/memory.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/io_exception.h>
#include <pcl/io/real_sense_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/format.hpp>

#include <iostream>
#include <mutex>


using namespace pcl::console;

void
printHelp (int, char **argv)
{
  std::cout << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "*                        REAL SENSE VIEWER - Usage Guide                   *" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << argv[0] << " [Options] device_id" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << std::endl;
  std::cout << "     --help, -h  : Show this help"                                            << std::endl;
  std::cout << "     --list, -l  : List connected RealSense devices and supported modes"      << std::endl;
  std::cout << "     --mode <id> : Use capture mode <id> from the list of supported modes"    << std::endl;
  std::cout << std::endl;
  std::cout << "Keyboard commands:"                                                           << std::endl;
  std::cout << std::endl;
  std::cout << "   When the focus is on the viewer window, the following keyboard commands"   << std::endl;
  std::cout << "   are available:"                                                            << std::endl;
  std::cout << "     * t/T : increase or decrease depth data confidence threshold"            << std::endl;
  std::cout << "     * k   : enable next temporal filtering method"                           << std::endl;
  std::cout << "     * s   : save the last grabbed cloud to disk"                             << std::endl;
  std::cout << "     * h   : print the list of standard PCL viewer commands"                  << std::endl;
  std::cout << std::endl;
  std::cout << "Notes:"                                                                       << std::endl;
  std::cout << std::endl;
  std::cout << "   The device to grab data from is selected using device_id argument. It"     << std::endl;
  std::cout << "   could be either:"                                                          << std::endl;
  std::cout << "     * serial number (e.g. 231400041-03)"                                     << std::endl;
  std::cout << "     * device index (e.g. #2 for the second connected device)"                << std::endl;
  std::cout << std::endl;
  std::cout << "   If device_id is not given, then the first available device will be used."  << std::endl;
  std::cout << std::endl;
  std::cout << "   If capture mode is not given, then the grabber will try to enable both"    << std::endl;
  std::cout << "   depth and color streams at VGA resolution and 30 Hz framerate. If this"    << std::endl;
  std::cout << "   particular mode is not available, the one that most closely matches this"  << std::endl;
  std::cout << "   specification will be chosen."                                             << std::endl;
  std::cout << std::endl;
}

void
printDeviceList ()
{
  std::vector<RealSenseGrabber::Ptr> grabbers;
  std::cout << "Connected devices: ";
  boost::format fmt ("\n  #%i  %s");
  boost::format fmt_dm ("\n        %2i) %d Hz  %dx%d Depth");
  boost::format fmt_dcm ("\n        %2i) %d Hz  %dx%d Depth  %dx%d Color");
  while (true)
  {
    try
    {
      grabbers.push_back (RealSenseGrabber::Ptr (new pcl::RealSenseGrabber));
      std::cout << boost::str (fmt % grabbers.size () % grabbers.back ()->getDeviceSerialNumber ());
      std::vector<pcl::RealSenseGrabber::Mode> xyz_modes = grabbers.back ()->getAvailableModes (true);
      std::cout << "\n      Depth modes:";
      if (xyz_modes.size ())
        for (std::size_t i = 0; i < xyz_modes.size (); ++i)
          std::cout << boost::str (fmt_dm % (i + 1) % xyz_modes[i].fps % xyz_modes[i].depth_width % xyz_modes[i].depth_height);
      else
      {
        std::cout << " none";
      }
      std::vector<pcl::RealSenseGrabber::Mode> xyzrgba_modes = grabbers.back ()->getAvailableModes (false);
      std::cout << "\n      Depth + color modes:";
      if (xyz_modes.size ())
        for (std::size_t i = 0; i < xyzrgba_modes.size (); ++i)
        {
          const pcl::RealSenseGrabber::Mode& m = xyzrgba_modes[i];
          std::cout << boost::str (fmt_dcm % (i + xyz_modes.size () + 1) % m.fps % m.depth_width % m.depth_height % m.color_width % m.color_height);
        }
      else
        std::cout << " none";
    }
    catch (pcl::io::IOException& e)
    {
      break;
    }
  }
  if (grabbers.size ())
    std::cout << std::endl;
  else
    std::cout << "none" << std::endl;
}

template <typename PointT>
class RealSenseViewer
{

  public:

    using PointCloudT = pcl::PointCloud<PointT>;

    RealSenseViewer (pcl::RealSenseGrabber& grabber)
    : grabber_ (grabber)
    , viewer_ ("RealSense Viewer")
    , window_ (3)
    , threshold_ (6)
    , temporal_filtering_ (pcl::RealSenseGrabber::RealSense_None)
    {
      viewer_.setCameraFieldOfView (0.785398); // approximately 45 degrees
      viewer_.setCameraPosition (0, 0, 0, 0, 0, 1, 0, 1, 0);
      viewer_.registerKeyboardCallback (&RealSenseViewer::keyboardCallback, *this);
      viewer_.registerPointPickingCallback (&RealSenseViewer::pointPickingCallback, *this);
    }

    ~RealSenseViewer ()
    {
      connection_.disconnect ();
    }

    void
    run ()
    {
      std::function<void (const typename PointCloudT::ConstPtr&)> f = [this] (const typename PointCloudT::ConstPtr& cloud) { cloudCallback (cloud); };
      connection_ = grabber_.registerCallback (f);
      grabber_.start ();
      printMode (grabber_.getMode ());
      viewer_.setSize (grabber_.getMode ().depth_width, grabber_.getMode ().depth_height);
      while (!viewer_.wasStopped ())
      {
        if (new_cloud_)
        {
          std::lock_guard<std::mutex> lock (new_cloud_mutex_);
          if (!viewer_.updatePointCloud (new_cloud_, "cloud"))
          {
            viewer_.addPointCloud (new_cloud_, "cloud");
            viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
          }
          displaySettings ();
          last_cloud_ = new_cloud_;
          new_cloud_.reset ();
        }
        viewer_.spinOnce (1, true);
      }
      grabber_.stop ();
    }

  private:

    void
    cloudCallback (typename PointCloudT::ConstPtr cloud)
    {
      if (!viewer_.wasStopped ())
      {
        std::lock_guard<std::mutex> lock (new_cloud_mutex_);
        new_cloud_ = cloud;
      }
    }

    void
    keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      if (event.keyDown ())
      {
        if (event.getKeyCode () == 'w' || event.getKeyCode () == 'W')
        {
          window_ += event.getKeyCode () == 'w' ? 1 : -1;
          if (window_ < 1)
            window_ = 1;
          pcl::console::print_info ("Temporal filtering window size: ");
          pcl::console::print_value ("%i\n", window_);
          grabber_.enableTemporalFiltering (temporal_filtering_, window_);
        }
        if (event.getKeyCode () == 't' || event.getKeyCode () == 'T')
        {
          threshold_ += event.getKeyCode () == 't' ? 1 : -1;
          if (threshold_ < 0)
            threshold_ = 0;
          if (threshold_ > 15)
            threshold_ = 15;
          pcl::console::print_info ("Confidence threshold: ");
          pcl::console::print_value ("%i\n", threshold_);
          grabber_.setConfidenceThreshold (threshold_);
        }
        if (event.getKeyCode () == 'k')
        {
          pcl::console::print_info ("Temporal filtering: ");
          switch (temporal_filtering_)
          {
            case pcl::RealSenseGrabber::RealSense_None:
              {
                temporal_filtering_ = pcl::RealSenseGrabber::RealSense_Median;
                pcl::console::print_value ("median\n");
                break;
              }
            case pcl::RealSenseGrabber::RealSense_Median:
              {
                temporal_filtering_ = pcl::RealSenseGrabber::RealSense_Average;
                pcl::console::print_value ("average\n");
                break;
              }
            case pcl::RealSenseGrabber::RealSense_Average:
              {
                temporal_filtering_ = pcl::RealSenseGrabber::RealSense_None;
                pcl::console::print_value ("none\n");
                break;
              }
          }
          grabber_.enableTemporalFiltering (temporal_filtering_, window_);
        }
        if (event.getKeyCode () == 's')
        {
          boost::format fmt ("RS_%s_%u.pcd");
          std::string fn = boost::str (fmt % grabber_.getDeviceSerialNumber ().c_str () % last_cloud_->header.stamp);
          pcl::io::savePCDFileBinaryCompressed (fn, *last_cloud_);
          pcl::console::print_info ("Saved point cloud: ");
          pcl::console::print_value (fn.c_str ());
          pcl::console::print_info ("\n");
        }
        displaySettings ();
      }
    }

    void
    pointPickingCallback (const pcl::visualization::PointPickingEvent& event, void*)
    {
      float x, y, z;
      event.getPoint (x, y, z);
      pcl::console::print_info ("Picked point at ");
      pcl::console::print_value ("%.3f", x); pcl::console::print_info (", ");
      pcl::console::print_value ("%.3f", y); pcl::console::print_info (", ");
      pcl::console::print_value ("%.3f\n", z);
    }

    void
    displaySettings ()
    {
      const int dx = 5;
      const int dy = 14;
      const int fs = 10;
      boost::format name_fmt ("text%i");
      const char* TF[] = {"off", "median", "average"};
      std::vector<boost::format> entries;
      // Framerate
      entries.push_back (boost::format ("framerate: %.1f") % grabber_.getFramesPerSecond ());
      // Confidence threshold
      entries.push_back (boost::format ("confidence threshold: %i") % threshold_);
      // Temporal filter settings
      std::string tfs = boost::str (boost::format (", window size %i") % window_);
      entries.push_back (boost::format ("temporal filtering: %s%s") % TF[temporal_filtering_] % (temporal_filtering_ == pcl::RealSenseGrabber::RealSense_None ? "" : tfs));
      for (std::size_t i = 0; i < entries.size (); ++i)
      {
        std::string name = boost::str (name_fmt % i);
        std::string entry = boost::str (entries[i]);
        if (!viewer_.updateText (entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name))
          viewer_.addText (entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name);
      }
    }

    void
    printMode (const pcl::RealSenseGrabber::Mode& mode)
    {
      print_info ("Capturing mode: ");
      print_value ("%i", mode.fps);
      print_info (" Hz  ");
      print_value ("%dx%d  ", mode.depth_width, mode.depth_height);
      print_info ("Depth");
      if (pcl::traits::has_color<PointT>::value)
      {
        print_value ("  %dx%d  ", mode.color_width, mode.color_height);
        print_info ("Color");
      }
      print_value ("\n");
    }

    pcl::RealSenseGrabber& grabber_;
    pcl::visualization::PCLVisualizer viewer_;
    boost::signals2::connection connection_;

    int window_;
    int threshold_;
    pcl::RealSenseGrabber::TemporalFilteringType temporal_filtering_;

    mutable std::mutex new_cloud_mutex_;
    typename PointCloudT::ConstPtr new_cloud_;
    typename PointCloudT::ConstPtr last_cloud_;

};

int
main (int argc, char** argv)
{
  print_info ("Viewer for RealSense devices (run with --help for more information)\n", argv[0]);

  if (find_switch (argc, argv, "--help") || find_switch (argc, argv, "-h"))
  {
    printHelp (argc, argv);
    return (0);
  }

  if (find_switch (argc, argv, "--list") || find_switch (argc, argv, "-l"))
  {
    printDeviceList ();
    return (0);
  }

  unsigned int mode_id = 0;
  bool with_mode = find_argument(argc, argv, "--mode") != -1;
  parse_argument(argc, argv, "--mode", mode_id);

  std::string device_id;

  if (argc == 1 ||              // no arguments
     (argc == 3 && with_mode))  // single argument, and it is --mode <id> 
  {
    device_id = "";
    print_info ("Creating a grabber for the first available device\n");
  }
  else
  {
    device_id = argv[argc - 1];
    print_info ("Creating a grabber for device "); print_value ("%s\n", device_id.c_str ());
  }

  try
  {
    pcl::RealSenseGrabber grabber (device_id);
    std::vector<pcl::RealSenseGrabber::Mode> xyz_modes = grabber.getAvailableModes (true);
    std::vector<pcl::RealSenseGrabber::Mode> xyzrgba_modes = grabber.getAvailableModes (false);
    if (mode_id == 0)
    {
      RealSenseViewer<pcl::PointXYZRGBA> viewer (grabber);
      viewer.run ();
    }
    else if (mode_id <= xyz_modes.size ())
    {
      grabber.setMode (xyz_modes[mode_id - 1], true);
      RealSenseViewer<pcl::PointXYZ> viewer (grabber);
      viewer.run ();
    }
    else if (mode_id <= xyz_modes.size () + xyzrgba_modes.size ())
    {
      grabber.setMode (xyzrgba_modes[mode_id - xyz_modes.size () - 1], true);
      RealSenseViewer<pcl::PointXYZRGBA> viewer (grabber);
      viewer.run ();
    }
    else
    {
      print_error ("Requested a mode (%i) that is not in the list of supported by this device\n", mode_id);
      return (1);
    }
  }
  catch (pcl::io::IOException& e)
  {
    print_error ("Failed to create a grabber: %s\n", e.what ());
    return (1);
  }

  return (0);
}


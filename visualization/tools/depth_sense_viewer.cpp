/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#include <iostream>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/io_exception.h>
#include <pcl/io/depth_sense_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::console;

void
printHelp (int, char **argv)
{
  std::cout << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "*                       DEPTH SENSE VIEWER - Usage Guide                   *" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << argv[0] << " [Options] device_id" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << std::endl;
  std::cout << "     --help, -h : Show this help"                                             << std::endl;
  std::cout << "     --list, -l : List connected DepthSense devices"                          << std::endl;
  std::cout << "     --xyz      : View XYZ-only clouds"                                       << std::endl;
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
  std::cout << "     * serial number (e.g. YZVF0780239000261D)"                               << std::endl;
  std::cout << "     * device index (e.g. #2 for the second connected device)"                << std::endl;
  std::cout << std::endl;
  std::cout << "   If device_id is not given, then the first available device will be used."  << std::endl;
  std::cout << std::endl;
}

void
printDeviceList ()
{
  typedef boost::shared_ptr<pcl::DepthSenseGrabber> DepthSenseGrabberPtr;
  std::vector<DepthSenseGrabberPtr> grabbers;
  std::cout << "Connected devices: ";
  boost::format fmt ("\n  #%i  %s");
  while (true)
  {
    try
    {
      grabbers.push_back (DepthSenseGrabberPtr (new pcl::DepthSenseGrabber));
      std::cout << boost::str (fmt % grabbers.size () % grabbers.back ()->getDeviceSerialNumber ());
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
class DepthSenseViewer
{

  public:

    typedef pcl::PointCloud<PointT> PointCloudT;

    DepthSenseViewer (pcl::DepthSenseGrabber& grabber)
    : grabber_ (grabber)
    , viewer_ ("DepthSense Viewer")
    , threshold_ (50)
    , window_ (5)
    , temporal_filtering_ (pcl::DepthSenseGrabber::DepthSense_None)
    {
      viewer_.registerKeyboardCallback (&DepthSenseViewer::keyboardCallback, *this);
    }

    ~DepthSenseViewer ()
    {
      connection_.disconnect ();
    }

    void
    run ()
    {
      boost::function<void (const typename PointCloudT::ConstPtr&)> f = boost::bind (&DepthSenseViewer::cloudCallback, this, _1);
      connection_ = grabber_.registerCallback (f);
      grabber_.start ();
      while (!viewer_.wasStopped ())
      {
        if (new_cloud_)
        {
          boost::mutex::scoped_lock lock (new_cloud_mutex_);
          if (!viewer_.updatePointCloud (new_cloud_, "cloud"))
          {
            viewer_.addPointCloud (new_cloud_, "cloud");
            viewer_.resetCamera ();
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
        boost::mutex::scoped_lock lock (new_cloud_mutex_);
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
          threshold_ += event.getKeyCode () == 't' ? 10 : -10;
          if (threshold_ < 0)
            threshold_ = 0;
          pcl::console::print_info ("Confidence threshold: ");
          pcl::console::print_value ("%i\n", threshold_);
          grabber_.setConfidenceThreshold (threshold_);
        }
        if (event.getKeyCode () == 'k')
        {
          pcl::console::print_info ("Temporal filtering: ");
          switch (temporal_filtering_)
          {
            case pcl::DepthSenseGrabber::DepthSense_None:
              {
                temporal_filtering_ = pcl::DepthSenseGrabber::DepthSense_Median;
                pcl::console::print_value ("median\n");
                break;
              }
            case pcl::DepthSenseGrabber::DepthSense_Median:
              {
                temporal_filtering_ = pcl::DepthSenseGrabber::DepthSense_Average;
                pcl::console::print_value ("average\n");
                break;
              }
            case pcl::DepthSenseGrabber::DepthSense_Average:
              {
                temporal_filtering_ = pcl::DepthSenseGrabber::DepthSense_None;
                pcl::console::print_value ("none\n");
                break;
              }
          }
          grabber_.enableTemporalFiltering (temporal_filtering_, window_);
        }
        if (event.getKeyCode () == 's')
        {
          boost::format fmt ("DS_%s_%u.pcd");
          std::string fn = boost::str (fmt % grabber_.getDeviceSerialNumber ().c_str () % last_cloud_->header.stamp);
          pcl::io::savePCDFileBinaryCompressed (fn, *last_cloud_);
          pcl::console::print_info ("Saved point cloud: ");
          pcl::console::print_value (fn.c_str ());
          pcl::console::print_info ("\n");
        }
        displaySettings ();
      }
    }

    void displaySettings ()
    {
      const int dx = 5;
      const int dy = 14;
      const int fs = 10;
      boost::format name_fmt ("text%i");
      const char* TF[] = {"off", "median", "average"};
      std::vector<boost::format> entries;
      // Framerate
      entries.push_back(boost::format("framerate: %.1f") % grabber_.getFramesPerSecond());
      // Confidence threshold
      entries.push_back (boost::format ("confidence threshold: %i") % threshold_);
      // Temporal filter settings
      std::string tfs = boost::str (boost::format (", window size %i") % window_);
      entries.push_back (boost::format ("temporal filtering: %s%s") % TF[temporal_filtering_] % (temporal_filtering_ == pcl::DepthSenseGrabber::DepthSense_None ? "" : tfs));
      for (size_t i = 0; i < entries.size (); ++i)
      {
        std::string name = boost::str (name_fmt % i);
        std::string entry = boost::str (entries[i]);
        if (!viewer_.updateText (entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name))
          viewer_.addText (entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name);
      }
    }

    pcl::DepthSenseGrabber& grabber_;
    pcl::visualization::PCLVisualizer viewer_;
    boost::signals2::connection connection_;

    int threshold_;
    int window_;
    pcl::DepthSenseGrabber::TemporalFilteringType temporal_filtering_;

    mutable boost::mutex new_cloud_mutex_;
    typename PointCloudT::ConstPtr new_cloud_;
    typename PointCloudT::ConstPtr last_cloud_;

};


int
main (int argc, char** argv)
{
  print_info ("Viewer for DepthSense devices (run with --help for more information)\n", argv[0]);

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

  bool xyz_only = find_switch (argc, argv, "--xyz");

  std::string device_id;

  if (argc == 1 ||             // no arguments
     (argc == 2 && xyz_only))  // single argument, and it is --xyz
  {
    device_id = "";
    print_info ("Creating a grabber for the first available device\n");
  }
  else
  {
    device_id = argv[argc - 1];
    print_info ("Creating a grabber for device \"%s\"\n", device_id.c_str ());
  }

  try
  {
    pcl::DepthSenseGrabber grabber (device_id);
    if (xyz_only)
    {
      DepthSenseViewer<pcl::PointXYZ> viewer (grabber);
      viewer.run ();
    }
    else
    {
      DepthSenseViewer<pcl::PointXYZRGBA> viewer (grabber);
      viewer.run ();
    }
  }
  catch (pcl::io::IOException& e)
  {
    print_error ("Failed to create a grabber: %s\n", e.what ());
    return (1);
  }

  return (0);
}


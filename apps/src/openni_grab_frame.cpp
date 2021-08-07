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
 * Author: Nico Blodow (blodow@cs.tum.edu)
 *         Christian Potthast (potthast@usc.edu)
 */

#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>

#include <mutex>

using namespace std::chrono_literals;

#define SHOW_FPS 1
#if SHOW_FPS
// clang-format off
#define FPS_CALC(_WHAT_)                                                               \
  do {                                                                                 \
    static unsigned count = 0;                                                         \
    static double last = pcl::getTime();                                               \
    double now = pcl::getTime();                                                       \
    ++count;                                                                           \
    if (now - last >= 1.0) {                                                           \
      std::cout << "Average framerate(" << _WHAT_ << "): "                             \
                << double(count) / double(now - last) << " Hz" << std::endl;           \
      count = 0;                                                                       \
      last = now;                                                                      \
    }                                                                                  \
  } while (false)
// clang-format on
#else
#define FPS_CALC(_WHAT_)                                                               \
  do {                                                                                 \
  } while (false)
#endif

using namespace pcl::console;
using namespace boost::filesystem;

template <typename PointType>
class OpenNIGrabFrame {
  using Cloud = pcl::PointCloud<PointType>;
  using CloudConstPtr = typename Cloud::ConstPtr;

public:
  OpenNIGrabFrame(pcl::OpenNIGrabber& grabber)
  : visualizer_(new pcl::visualization::PCLVisualizer("OpenNI Viewer"))
  , writer_()
  , quit_(false)
  , continuous_(false)
  , trigger_(false)
  , file_name_("")
  , dir_name_("")
  , format_(4)
  , grabber_(grabber)
  , visualizer_enable_(true)
  {}

  void
  cloud_cb_(const CloudConstPtr& cloud)
  {
    if (quit_)
      return;

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_ = cloud;

    if (continuous_ || trigger_)
      saveCloud();

    trigger_ = false;
  }

  void
  keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
  {
    if (event.keyUp()) {
      switch (event.getKeyCode()) {
      case 27:
      case 'Q':
      case 'q':
        quit_ = true;
        visualizer_->close();
        break;
      case 'V':
      case 'v':
        visualizer_enable_ = !visualizer_enable_;
        break;
      case ' ':
        continuous_ = !continuous_;
        break;
      }
    }
  }

  void
  mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*)
  {
    if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
        mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
      trigger_ = true;
    }
  }

  CloudConstPtr
  getLatestCloud()
  {
    // lock while we swap our cloud and reset it.
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    CloudConstPtr temp_cloud;
    temp_cloud.swap(cloud_); // here we set cloud_ to null, so that
    // it is safe to set it again from our
    // callback
    return temp_cloud;
  }

  void
  saveCloud()
  {
    FPS_CALC("I/O");
    const std::string time = boost::posix_time::to_iso_string(
        boost::posix_time::microsec_clock::local_time());
    const std::string filepath = dir_name_ + '/' + file_name_ + '_' + time + ".pcd";

    if (format_ & 1) {
      writer_.writeBinary<PointType>(filepath, *cloud_);
      // std::cerr << "Data saved in BINARY format to " << ss.str () << std::endl;
    }

    if (format_ & 2) {
      writer_.writeBinaryCompressed<PointType>(filepath, *cloud_);
    }

    if (format_ & 4) {
      writer_.writeBinaryCompressed<PointType>(filepath, *cloud_);
    }
  }

  void
  run()
  {
    // register the keyboard and mouse callback for the visualizer
    visualizer_->registerMouseCallback(&OpenNIGrabFrame::mouse_callback, *this);
    visualizer_->registerKeyboardCallback(&OpenNIGrabFrame::keyboard_callback, *this);

    // make callback function from member function
    std::function<void(const CloudConstPtr&)> f = [this](const CloudConstPtr& cloud) {
      cloud_cb_(cloud);
    };

    // connect callback function for desired signal. In this case its a point cloud with
    // color values
    boost::signals2::connection c = grabber_.registerCallback(f);

    // start receiving point clouds
    grabber_.start();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (!visualizer_->wasStopped()) {
      std::this_thread::sleep_for(100us);

      visualizer_->spinOnce();

      if (!visualizer_enable_)
        continue;

      if (cloud_) {
        CloudConstPtr cloud = getLatestCloud();
        if (!visualizer_->updatePointCloud(cloud, "OpenNICloud")) {
          visualizer_->addPointCloud(cloud, "OpenNICloud");
          visualizer_->resetCameraViewpoint("OpenNICloud");
        }
      }
    }

    // stop the grabber
    grabber_.stop();
  }

  void
  setOptions(const std::string& filename,
             const std::string& pcd_format,
             bool paused,
             bool visualizer)
  {
    boost::filesystem::path path(filename);

    if (filename.empty()) {
      dir_name_ = ".";
      file_name_ = "frame";
    }
    else {
      dir_name_ = path.parent_path().string();

      if (!dir_name_.empty() && !boost::filesystem::exists(path.parent_path())) {
        std::cerr << "directory \"" << path.parent_path() << "\" does not exist!\n";
        exit(1);
      }
      file_name_ = path.stem().string();
    }

    std::cout << "dir: " << dir_name_ << " :: " << path.parent_path() << std::endl;
    std::cout << "file: " << file_name_ << " :: " << path.stem().string() << std::endl;

    if (pcd_format == "b" || pcd_format == "all")
      format_ |= 1;
    else if (pcd_format == "ascii" || pcd_format == "all")
      format_ |= 2;
    else if (pcd_format == "bc" || pcd_format == "all")
      format_ |= 4;

    continuous_ = !paused;
    visualizer_enable_ = visualizer;
  }

  pcl::visualization::PCLVisualizer::Ptr visualizer_;
  pcl::PCDWriter writer_;
  bool quit_;
  bool continuous_;
  bool trigger_;
  std::string file_name_;
  std::string dir_name_;
  unsigned format_;
  CloudConstPtr cloud_;
  mutable std::mutex cloud_mutex_;
  pcl::OpenNIGrabber& grabber_;
  bool visualizer_enable_;
};

void
usage(char** argv)
{
  std::cout << "usage: " << argv[0] << " <filename> <options>\n\n";

  // clang-format off
  print_info ("  filename: if no filename is provided a generic timestamp will be set as filename\n\n");
  print_info ("  where options are:\n");
  print_info ("                    -format = PCD file format (b=binary; bc=binary compressed; ascii=ascii; all=all) (default: bc)\n");
  print_info ("                    -XYZ  = store just a XYZ cloud\n");
  print_info ("                    -paused = start grabber in paused mode. Toggle pause by pressing the space bar\n");
  print_info ("                              or grab single frames by just pressing the left mouse button.\n");
  print_info ("                    -imagemode = select the image mode (resolution, fps) for the grabber, see pcl::OpenNIGrabber::Mode for details.\n");
  print_info ("                    -depthmode = select the depth mode (resolution, fps) for the grabber, see pcl::OpenNIGrabber::Mode for details.\n");
  print_info ("                    -visualizer 0/1 = turn OFF or ON the visualization of point clouds in the viewer (can also be changed using 'v'/'V' in the viewer).\n");
  // clang-format on
}

int
main(int argc, char** argv)
{
  std::string arg;
  if (argc > 1)
    arg = std::string(argv[1]);

  if (arg == "--help" || arg == "-h") {
    usage(argv);
    return 1;
  }

  std::string format = "bc";
  std::string filename;
  bool paused = false;
  bool xyz = false;
  bool visualizer = true;
  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

  if (argc > 1) {
    // Parse the command line arguments for .pcd file
    std::vector<int> p_file_indices;
    p_file_indices = parse_file_extension_argument(argc, argv, ".pcd");
    if (p_file_indices.size() > 0)
      filename = argv[p_file_indices[0]];

    std::cout << "fname: " << filename << std::endl;
    // Command line parsing
    parse_argument(argc, argv, "-format", format);
    xyz = find_switch(argc, argv, "-XYZ");
    paused = find_switch(argc, argv, "-paused");
    visualizer = find_switch(argc, argv, "-visualizer");

    unsigned mode;
    if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
      depth_mode = pcl::OpenNIGrabber::Mode(mode);

    if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
      image_mode = pcl::OpenNIGrabber::Mode(mode);
  }

  pcl::OpenNIGrabber grabber("#1", depth_mode, image_mode);

  if (xyz) {
    OpenNIGrabFrame<pcl::PointXYZ> grab_frame(grabber);
    grab_frame.setOptions(filename, format, paused, visualizer);
    grab_frame.run();
  }
  else {
    OpenNIGrabFrame<pcl::PointXYZRGBA> grab_frame(grabber);
    grab_frame.setOptions(filename, format, paused, visualizer);
    grab_frame.run();
  }
  return 0;
}

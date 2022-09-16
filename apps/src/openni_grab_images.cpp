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
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>

#include <mutex>

using namespace pcl::console;
using namespace boost::filesystem;

class OpenNIGrabFrame {
public:
  OpenNIGrabFrame(pcl::OpenNIGrabber& grabber, bool paused)
  : grabber_(grabber)
  , image_viewer_("RGB Image")
  , depth_image_viewer_("Depth Image")
  , quit_(false)
  , continuous_(!paused)
  , trigger_(false)
  , importer_(vtkSmartPointer<vtkImageImport>::New())
  , depth_importer_(vtkSmartPointer<vtkImageImport>::New())
  , writer_(vtkSmartPointer<vtkTIFFWriter>::New())
  , flipper_(vtkSmartPointer<vtkImageFlip>::New())
  {
    importer_->SetNumberOfScalarComponents(3);
    importer_->SetDataScalarTypeToUnsignedChar();
    depth_importer_->SetNumberOfScalarComponents(1);
    depth_importer_->SetDataScalarTypeToUnsignedShort();
    writer_->SetCompressionToPackBits();
    flipper_->SetFilteredAxes(1);
  }

  void
  image_callback(const openni_wrapper::Image::Ptr& image,
                 const openni_wrapper::DepthImage::Ptr& depth_image,
                 float)
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    image_ = image;
    depth_image_ = depth_image;
    lock.unlock();
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
        break;
      case ' ':
        continuous_ = !continuous_;
        break;
      case 'G':
      case 'g':
        trigger_ = true;
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

  void
  saveImages()
  {
    std::string time = boost::posix_time::to_iso_string(
        boost::posix_time::microsec_clock::local_time());
    openni_wrapper::Image::Ptr image;
    openni_wrapper::DepthImage::Ptr depth_image;

    image_mutex_.lock();
    image.swap(image_);
    depth_image.swap(depth_image_);
    image_mutex_.unlock();

    if (image) {
      const void* data;
      if (image->getEncoding() != openni_wrapper::Image::RGB) {
        static unsigned char* rgb_data = 0;
        static unsigned rgb_data_size = 0;

        if (rgb_data_size < image->getWidth() * image->getHeight()) {
          delete[] rgb_data;
          rgb_data_size = image->getWidth() * image->getHeight();
          rgb_data = new unsigned char[rgb_data_size * 3];
        }
        image->fillRGB(image->getWidth(), image->getHeight(), rgb_data);
        data = reinterpret_cast<const void*>(rgb_data);
      }
      else
        data = reinterpret_cast<const void*>(image->getMetaData().Data());

      importer_->SetWholeExtent(
          0, image->getWidth() - 1, 0, image->getHeight() - 1, 0, 0);
      importer_->SetDataExtentToWholeExtent();

      const std::string rgb_frame_filename = "frame_" + time + "_rgb.tiff";
      importer_->SetImportVoidPointer(const_cast<void*>(data), 1);
      importer_->Update();
      flipper_->SetInputConnection(importer_->GetOutputPort());
      flipper_->Update();
      writer_->SetFileName(rgb_frame_filename.c_str());
      writer_->SetInputConnection(flipper_->GetOutputPort());
      writer_->Write();
    }
    if (depth_image) {
      const std::string depth_frame_filename = "frame_" + time + "_depth.tiff";

      depth_importer_->SetWholeExtent(
          0, depth_image->getWidth() - 1, 0, depth_image->getHeight() - 1, 0, 0);
      depth_importer_->SetDataExtentToWholeExtent();
      depth_importer_->SetImportVoidPointer(
          const_cast<void*>(
              reinterpret_cast<const void*>(depth_image->getDepthMetaData().Data())),
          1);
      depth_importer_->Update();
      flipper_->SetInputConnection(depth_importer_->GetOutputPort());
      flipper_->Update();
      writer_->SetFileName(depth_frame_filename.c_str());
      writer_->SetInputConnection(flipper_->GetOutputPort());
      writer_->Write();
    }

    trigger_ = false;
  }

  int
  run()
  {
    // register the keyboard and mouse callback for the visualizer
    image_viewer_.registerMouseCallback(&OpenNIGrabFrame::mouse_callback, *this);
    image_viewer_.registerKeyboardCallback(&OpenNIGrabFrame::keyboard_callback, *this);
    depth_image_viewer_.registerMouseCallback(&OpenNIGrabFrame::mouse_callback, *this);
    depth_image_viewer_.registerKeyboardCallback(&OpenNIGrabFrame::keyboard_callback,
                                                 *this);

    std::function<void(const openni_wrapper::Image::Ptr&,
                       const openni_wrapper::DepthImage::Ptr&,
                       float)>
        image_cb = [this](const openni_wrapper::Image::Ptr& img,
                          const openni_wrapper::DepthImage::Ptr& depth,
                          float f) { image_callback(img, depth, f); };
    boost::signals2::connection image_connection = grabber_.registerCallback(image_cb);

    // start receiving point clouds
    grabber_.start();
    unsigned char* rgb_data = 0;
    unsigned rgb_data_size = 0;
    unsigned char* depth_data = 0;

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (!image_viewer_.wasStopped() && !quit_) {
      std::string time = boost::posix_time::to_iso_string(
          boost::posix_time::microsec_clock::local_time());
      openni_wrapper::Image::Ptr image;
      openni_wrapper::DepthImage::Ptr depth_image;

      if (image_mutex_.try_lock()) {
        image.swap(image_);
        depth_image.swap(depth_image_);
        image_mutex_.unlock();
      }

      if (image) {
        const void* data;
        if (image->getEncoding() != openni_wrapper::Image::RGB) {
          if (rgb_data_size < image->getWidth() * image->getHeight()) {
            delete[] rgb_data;
            rgb_data_size = image->getWidth() * image->getHeight();
            rgb_data = new unsigned char[rgb_data_size * 3];
          }
          image->fillRGB(image->getWidth(), image->getHeight(), rgb_data);
          data = reinterpret_cast<const void*>(rgb_data);
        }
        else
          data = reinterpret_cast<const void*>(image->getMetaData().Data());
        image_viewer_.addRGBImage(
            (const unsigned char*)data, image->getWidth(), image->getHeight());

        if (continuous_ || trigger_) {
          importer_->SetWholeExtent(
              0, image->getWidth() - 1, 0, image->getHeight() - 1, 0, 0);
          importer_->SetDataExtentToWholeExtent();

          const std::string rgb_frame_filename = "frame_" + time + "_rgb.tiff";
          importer_->SetImportVoidPointer(const_cast<void*>(data), 1);
          importer_->Update();
          flipper_->SetInputConnection(importer_->GetOutputPort());
          flipper_->Update();
          writer_->SetFileName(rgb_frame_filename.c_str());
          writer_->SetInputConnection(flipper_->GetOutputPort());
          writer_->Write();
          std::cout << "writing rgb frame: " << rgb_frame_filename << std::endl;
        }
      }

      if (depth_image) {
        delete[] depth_data;
        depth_data = pcl::visualization::FloatImageUtils::getVisualImage(
            reinterpret_cast<const unsigned short*>(
                depth_image->getDepthMetaData().Data()),
            depth_image->getWidth(),
            depth_image->getHeight(),
            std::numeric_limits<unsigned short>::min(),
            // Scale so that the colors look brigher on screen
            std::numeric_limits<unsigned short>::max() / 10,
            true);

        depth_image_viewer_.addRGBImage(
            depth_data, depth_image->getWidth(), depth_image->getHeight());
        if (continuous_ || trigger_) {
          const std::string depth_frame_filename = "frame_" + time + "_depth.tiff";

          depth_importer_->SetWholeExtent(
              0, depth_image->getWidth() - 1, 0, depth_image->getHeight() - 1, 0, 0);
          depth_importer_->SetDataExtentToWholeExtent();
          depth_importer_->SetImportVoidPointer(
              const_cast<void*>(reinterpret_cast<const void*>(
                  depth_image->getDepthMetaData().Data())),
              1);
          depth_importer_->Update();
          flipper_->SetInputConnection(depth_importer_->GetOutputPort());
          flipper_->Update();
          writer_->SetFileName(depth_frame_filename.c_str());
          writer_->SetInputConnection(flipper_->GetOutputPort());
          writer_->Write();
          std::cout << "writing depth frame: " << depth_frame_filename << std::endl;
        }
      }
      trigger_ = false;
      image_viewer_.spinOnce();
      depth_image_viewer_.spinOnce();
    }

    image_mutex_.lock();
    // stop the grabber
    grabber_.stop();
    image_mutex_.unlock();
    return 0;
  }

  pcl::OpenNIGrabber& grabber_;
  pcl::visualization::ImageViewer image_viewer_;
  pcl::visualization::ImageViewer depth_image_viewer_;
  bool quit_;
  bool continuous_;
  bool trigger_;
  mutable std::mutex image_mutex_;
  openni_wrapper::Image::Ptr image_;
  openni_wrapper::DepthImage::Ptr depth_image_;
  vtkSmartPointer<vtkImageImport> importer_, depth_importer_;
  vtkSmartPointer<vtkTIFFWriter> writer_;
  vtkSmartPointer<vtkImageFlip> flipper_;
};

void
usage(char** argv)
{
  std::cout << "usage: " << argv[0]
            << " [((<device_id> | <path-to-oni-file>) [-imagemode <mode>] "
               "[-depthformat <format>] [-paused] | -l [<device_id>]| -h | --help)]"
            << std::endl;
  std::cout << argv[0] << " -h | --help : shows this help" << std::endl;
  std::cout << argv[0] << " -l : list all available devices" << std::endl;
  std::cout << argv[0]
            << " -l <device-id> : list all available modes for specified device"
            << std::endl;

  std::cout << "                 device_id may be #1, #2, ... for the first, second "
               "etc device in the list"
#ifndef _WIN32
            << " or" << std::endl
            << "                 bus@address for the device connected to a specific "
               "usb-bus / address combination or"
            << std::endl
            << "                 <serial-number>"
#endif
            << std::endl;
  std::cout << "                 -paused : start grabber in paused mode. Toggle pause "
               "by pressing the space bar\n";
  std::cout << "                           or grab single frames by just pressing the "
               "left mouse button.\n";
  std::cout << std::endl;
  std::cout << "examples:" << std::endl;
  std::cout << argv[0] << " \"#1\"" << std::endl;
  std::cout << "    uses the first device." << std::endl;
  std::cout << argv[0] << " \"./temp/test.oni\"" << std::endl;
  std::cout << "    uses the oni-player device to play back oni file given by path."
            << std::endl;
  std::cout << argv[0] << " -l" << std::endl;
  std::cout << "    lists all available devices." << std::endl;
  std::cout << argv[0] << " -l \"#2\"" << std::endl;
  std::cout << "    lists all available modes for the second device" << std::endl;
#ifndef _WIN32
  std::cout << argv[0] << " A00361800903049A" << std::endl;
  std::cout << "    uses the device with the serial number \'A00361800903049A\'."
            << std::endl;
  std::cout << argv[0] << " 1@16" << std::endl;
  std::cout << "    uses the device on address 16 at USB bus 1." << std::endl;
#endif
}

int
main(int argc, char** argv)
{
  std::string device_id("");
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

  if (argc >= 2) {
    device_id = argv[1];
    if (device_id == "--help" || device_id == "-h") {
      usage(argv);
      return 0;
    }
    else if (device_id == "-l") {
      if (argc >= 3) {
        pcl::OpenNIGrabber grabber(argv[2]);
        auto device = grabber.getDevice();
        std::vector<std::pair<int, XnMapOutputMode>> modes;

        if (device->hasImageStream()) {
          std::cout << std::endl
                    << "Supported image modes for device: " << device->getVendorName()
                    << " , " << device->getProductName() << std::endl;
          modes = grabber.getAvailableImageModes();
          for (const auto& mode : modes) {
            std::cout << mode.first << " = " << mode.second.nXRes << " x "
                      << mode.second.nYRes << " @ " << mode.second.nFPS << std::endl;
          }
        }
      }
      else {
        openni_wrapper::OpenNIDriver& driver =
            openni_wrapper::OpenNIDriver::getInstance();
        if (driver.getNumberDevices() > 0) {
          for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices();
               ++deviceIdx) {
            std::cout << "Device: " << deviceIdx + 1
                      << ", vendor: " << driver.getVendorName(deviceIdx)
                      << ", product: " << driver.getProductName(deviceIdx)
                      << ", connected: " << driver.getBus(deviceIdx) << " @ "
                      << driver.getAddress(deviceIdx) << ", serial number: \'"
                      << driver.getSerialNumber(deviceIdx) << "\'" << std::endl;
          }
        }
        else
          std::cout << "No devices connected." << std::endl;

        std::cout << "Virtual Devices available: ONI player" << std::endl;
      }
      return 0;
    }
  }
  else {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
    if (driver.getNumberDevices() > 0)
      std::cout << "Device Id not set, using first device." << std::endl;
  }

  unsigned mode;
  if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
    image_mode = static_cast<pcl::OpenNIGrabber::Mode>(mode);

  int depthformat = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth;
  pcl::console::parse_argument(argc, argv, "-depthformat", depthformat);

  pcl::OpenNIGrabber grabber(
      device_id, pcl::OpenNIGrabber::OpenNI_Default_Mode, image_mode);
  // Set the depth output format
  grabber.getDevice()->setDepthOutputFormat(
      static_cast<openni_wrapper::OpenNIDevice::DepthMode>(depthformat));

  bool paused = find_switch(argc, argv, "-paused");

  OpenNIGrabFrame grabFrame(grabber, paused);
  return grabFrame.run();
}

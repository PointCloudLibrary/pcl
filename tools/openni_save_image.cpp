/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkSmartPointer.h>
#include <vtkImageImport.h>
#include <vtkTIFFWriter.h>
#include <vtkImageFlip.h>

#include <mutex>
#include <string>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp> // for to_iso_string, local_time


#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer (pcl::OpenNIGrabber& grabber)
      : grabber_ (grabber)
      , importer_ (vtkSmartPointer<vtkImageImport>::New ())
      , depth_importer_ (vtkSmartPointer<vtkImageImport>::New ())
      , writer_ (vtkSmartPointer<vtkTIFFWriter>::New ())
      , flipper_ (vtkSmartPointer<vtkImageFlip>::New ())
    {
      importer_->SetNumberOfScalarComponents (3);
      importer_->SetDataScalarTypeToUnsignedChar ();
      depth_importer_->SetNumberOfScalarComponents (1);
      depth_importer_->SetDataScalarTypeToUnsignedShort ();
      writer_->SetCompressionToPackBits ();
      flipper_->SetFilteredAxes (1);
    }

    void
    image_callback (const openni_wrapper::Image::Ptr &image, 
                    const openni_wrapper::DepthImage::Ptr &depth_image, float)
    {
      FPS_CALC ("image callback");
      std::lock_guard<std::mutex> lock (image_mutex_);
      image_ = image;
      depth_image_ = depth_image;
    }
    
    void
    run ()
    {
      std::function<
        void (const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, float)
      > image_cb = [this] (const openni_wrapper::Image::Ptr& img, const openni_wrapper::DepthImage::Ptr& depth, float f)
      {
        image_callback (img, depth, f);
      };
      boost::signals2::connection image_connection = grabber_.registerCallback (image_cb);
      
      grabber_.start ();
      
      unsigned char* rgb_data = nullptr;
      unsigned rgb_data_size = 0;
      const void* data;
       
      while (true)
      {
        std::lock_guard<std::mutex> lock (image_mutex_);

        std::string time = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());
        if (image_)
        {
          FPS_CALC ("writer callback");
          openni_wrapper::Image::Ptr image;
          image.swap (image_);

          if (image->getEncoding() == openni_wrapper::Image::RGB)
          {
            data = reinterpret_cast<const void*> (image->getMetaData ().Data ());
            importer_->SetWholeExtent (0, image->getWidth () - 1, 0, image->getHeight () - 1, 0, 0);
            importer_->SetDataExtentToWholeExtent ();
          }
          else
          {
            if (rgb_data_size < image->getWidth () * image->getHeight ())
            {
              rgb_data_size = image->getWidth () * image->getHeight ();
              rgb_data = new unsigned char [rgb_data_size * 3];
              importer_->SetWholeExtent (0, image->getWidth () - 1, 0, image->getHeight () - 1, 0, 0);
              importer_->SetDataExtentToWholeExtent ();
            }
            image->fillRGB (image->getWidth (), image->getHeight (), rgb_data);
            data = reinterpret_cast<const void*> (rgb_data);
          }

          const std::string filename = "frame_" + time + "_rgb.tiff";
          importer_->SetImportVoidPointer (const_cast<void*>(data), 1);
          importer_->Update ();
          flipper_->SetInputConnection (importer_->GetOutputPort ());
          flipper_->Update ();
          writer_->SetFileName (filename.c_str ());
          writer_->SetInputConnection (flipper_->GetOutputPort ());
          writer_->Write ();
        }

        if (depth_image_)
        {
          openni_wrapper::DepthImage::Ptr depth_image;
          depth_image.swap (depth_image_);

          const std::string filename = "frame_" + time + "_depth.tiff";

          depth_importer_->SetWholeExtent (0, depth_image->getWidth () - 1, 0, depth_image->getHeight () - 1, 0, 0);
          depth_importer_->SetDataExtentToWholeExtent ();
          depth_importer_->SetImportVoidPointer (const_cast<void*> (reinterpret_cast<const void*> (depth_image->getDepthMetaData ().Data ())), 1);
          depth_importer_->Update ();
          flipper_->SetInputConnection (depth_importer_->GetOutputPort ());
          flipper_->Update ();
          writer_->SetFileName (filename.c_str ());
          writer_->SetInputConnection (flipper_->GetOutputPort ());
          writer_->Write ();
        }
      }

      grabber_.stop ();
      image_connection.disconnect ();
      delete[] rgb_data;
    }

    pcl::OpenNIGrabber& grabber_;
    std::mutex image_mutex_;
    openni_wrapper::Image::Ptr image_;
    openni_wrapper::DepthImage::Ptr depth_image_;
    vtkSmartPointer<vtkImageImport> importer_, depth_importer_;
    vtkSmartPointer<vtkTIFFWriter> writer_;
    vtkSmartPointer<vtkImageFlip> flipper_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " [((<device_id> | <path-to-oni-file>) [-imagemode <mode>] | -l [<device_id>]| -h | --help)]" << std::endl;
  std::cout << argv[0] << " -h | --help : shows this help" << std::endl;
  std::cout << argv[0] << " -l : list all available devices" << std::endl;
  std::cout << argv[0] << " -l <device-id> : list all available modes for specified device" << std::endl;

  std::cout << "                 device_id may be #1, #2, ... for the first, second etc device in the list"
#ifndef _WIN32
       << " or" << std::endl
       << "                 bus@address for the device connected to a specific usb-bus / address combination or" << std::endl
       << "                 <serial-number>"
#endif
       << std::endl;
  std::cout << std::endl;
  std::cout << "examples:" << std::endl;
  std::cout << argv[0] << " \"#1\"" << std::endl;
  std::cout << "    uses the first device." << std::endl;
  std::cout << argv[0] << " \"./temp/test.oni\"" << std::endl;
  std::cout << "    uses the oni-player device to play back oni file given by path." << std::endl;
  std::cout << argv[0] << " -l" << std::endl;
  std::cout << "    lists all available devices." << std::endl;
  std::cout << argv[0] << " -l \"#2\"" << std::endl;
  std::cout << "    lists all available modes for the second device" << std::endl;
#ifndef _WIN32
  std::cout << argv[0] << " A00361800903049A" << std::endl;
  std::cout << "    uses the device with the serial number \'A00361800903049A\'." << std::endl;
  std::cout << argv[0] << " 1@16" << std::endl;
  std::cout << "    uses the device on address 16 at USB bus 1." << std::endl;
#endif
  return;
}

int
main(int argc, char ** argv)
{
  std::string device_id;
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  
  if (argc >= 2)
  {
    device_id = argv[1];
    if (device_id == "--help" || device_id == "-h")
    {
      usage(argv);
      return 0;
    }
    if (device_id == "-l")
    {
      if (argc >= 3)
      {
        pcl::OpenNIGrabber grabber (argv[2]);
        openni_wrapper::OpenNIDevice::Ptr device = grabber.getDevice ();
        std::vector<std::pair<int, XnMapOutputMode> > modes;

        if (device->hasImageStream ())
        {
          std::cout << std::endl << "Supported image modes for device: " << device->getVendorName () << " , " << device->getProductName () << std::endl;
          modes = grabber.getAvailableImageModes ();
          for (const auto& mode : modes)
          {
            std::cout << mode.first << " = " << mode.second.nXRes << " x " << mode.second.nYRes << " @ " << mode.second.nFPS << std::endl;
          }
        }
      }
      else
      {
        openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
        if (driver.getNumberDevices () > 0)
        {
          for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
          {
            std::cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << std::endl;
          }

        }
        else
          std::cout << "No devices connected." << std::endl;
        
        std::cout <<"Virtual Devices available: ONI player" << std::endl;
      }
      return (0);
    }
  }
  else
  {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
    if (driver.getNumberDevices () > 0)
      std::cout << "Device Id not set, using first device." << std::endl;
  }
  
  unsigned imagemode;
  if (pcl::console::parse (argc, argv, "-imagemode", imagemode) != -1)
    image_mode = pcl::OpenNIGrabber::Mode (imagemode);
  unsigned depthmode;
  if (pcl::console::parse (argc, argv, "-depthmode", depthmode) != -1)
    depth_mode = pcl::OpenNIGrabber::Mode (depthmode);
  

  pcl::OpenNIGrabber grabber (device_id, depth_mode, image_mode);
  SimpleOpenNIViewer v (grabber);
  v.run ();

  return (0);
}

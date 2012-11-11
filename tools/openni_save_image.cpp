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
#include <vector>
#include <string>

#include <pcl/visualization/vtk.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "boost.h"

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
      , image_mutex_ ()
      , image_ ()
      , depth_image_ ()
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
    image_callback (const boost::shared_ptr<openni_wrapper::Image> &image, 
                    const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image, float)
    {
      FPS_CALC ("image callback");
      boost::mutex::scoped_lock lock (image_mutex_);
      image_ = image;
      depth_image_ = depth_image;
    }
    
    void
    run ()
    {
      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float) > image_cb = boost::bind (&SimpleOpenNIViewer::image_callback, this, _1, _2, _3);
      boost::signals2::connection image_connection = grabber_.registerCallback (image_cb);
      
      grabber_.start ();
      
      unsigned char* rgb_data = 0;
      unsigned rgb_data_size = 0;
      const void* data;
       
      while (true)
      {
        boost::mutex::scoped_lock lock (image_mutex_);

        std::string time = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());
        if (image_)
        {
          FPS_CALC ("writer callback");
          boost::shared_ptr<openni_wrapper::Image> image;
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

          std::stringstream ss;
          ss << "frame_" + time + "_rgb.tiff";
          importer_->SetImportVoidPointer (const_cast<void*>(data), 1);
          importer_->Update ();
          flipper_->SetInputConnection (importer_->GetOutputPort ());
          flipper_->Update ();
          writer_->SetFileName (ss.str ().c_str ());
          writer_->SetInputConnection (flipper_->GetOutputPort ());
          writer_->Write ();
        }

        if (depth_image_)
        {
          boost::shared_ptr<openni_wrapper::DepthImage> depth_image;
          depth_image.swap (depth_image_);

          std::stringstream ss;
          ss << "frame_" + time + "_depth.tiff";

          depth_importer_->SetWholeExtent (0, depth_image->getWidth () - 1, 0, depth_image->getHeight () - 1, 0, 0);
          depth_importer_->SetDataExtentToWholeExtent ();
          depth_importer_->SetImportVoidPointer (const_cast<void*> (reinterpret_cast<const void*> (depth_image->getDepthMetaData ().Data ())), 1);
          depth_importer_->Update ();
          flipper_->SetInputConnection (depth_importer_->GetOutputPort ());
          flipper_->Update ();
          writer_->SetFileName (ss.str ().c_str ());
          writer_->SetInputConnection (flipper_->GetOutputPort ());
          writer_->Write ();
        }
      }

      grabber_.stop ();
      
      image_connection.disconnect ();
      
      if (rgb_data)
        delete[] rgb_data;
    }

    pcl::OpenNIGrabber& grabber_;
    boost::mutex image_mutex_;
    boost::shared_ptr<openni_wrapper::Image> image_;
    boost::shared_ptr<openni_wrapper::DepthImage> depth_image_;
    vtkSmartPointer<vtkImageImport> importer_, depth_importer_;
    vtkSmartPointer<vtkTIFFWriter> writer_;
    vtkSmartPointer<vtkImageFlip> flipper_;
};

void
usage (char ** argv)
{
  cout << "usage: " << argv[0] << " [((<device_id> | <path-to-oni-file>) [-imagemode <mode>] | -l [<device_id>]| -h | --help)]" << endl;
  cout << argv[0] << " -h | --help : shows this help" << endl;
  cout << argv[0] << " -l : list all available devices" << endl;
  cout << argv[0] << " -l <device-id> : list all available modes for specified device" << endl;

  cout << "                 device_id may be #1, #2, ... for the first, second etc device in the list"
#ifndef _WIN32
       << " or" << endl
       << "                 bus@address for the device connected to a specific usb-bus / address combination or" << endl
       << "                 <serial-number>"
#endif
       << endl;
  cout << endl;
  cout << "examples:" << endl;
  cout << argv[0] << " \"#1\"" << endl;
  cout << "    uses the first device." << endl;
  cout << argv[0] << " \"./temp/test.oni\"" << endl;
  cout << "    uses the oni-player device to play back oni file given by path." << endl;
  cout << argv[0] << " -l" << endl;
  cout << "    lists all available devices." << endl;
  cout << argv[0] << " -l \"#2\"" << endl;
  cout << "    lists all available modes for the second device" << endl;
#ifndef _WIN32
  cout << argv[0] << " A00361800903049A" << endl;
  cout << "    uses the device with the serial number \'A00361800903049A\'." << endl;
  cout << argv[0] << " 1@16" << endl;
  cout << "    uses the device on address 16 at USB bus 1." << endl;
#endif
  return;
}

int
main(int argc, char ** argv)
{
  std::string device_id ("");
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
    else if (device_id == "-l")
    {
      if (argc >= 3)
      {
        pcl::OpenNIGrabber grabber (argv[2]);
        boost::shared_ptr<openni_wrapper::OpenNIDevice> device = grabber.getDevice ();
        std::vector<std::pair<int, XnMapOutputMode> > modes;

        if (device->hasImageStream ())
        {
          cout << endl << "Supported image modes for device: " << device->getVendorName () << " , " << device->getProductName () << endl;
          modes = grabber.getAvailableImageModes ();
          for (std::vector<std::pair<int, XnMapOutputMode> >::const_iterator it = modes.begin (); it != modes.end (); ++it)
          {
            cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
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
            cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
          }

        }
        else
          cout << "No devices connected." << endl;
        
        cout <<"Virtual Devices available: ONI player" << endl;
      }
      return (0);
    }
  }
  else
  {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
    if (driver.getNumberDevices () > 0)
      cout << "Device Id not set, using first device." << endl;
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

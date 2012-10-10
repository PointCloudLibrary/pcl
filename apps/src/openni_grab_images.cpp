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

#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include <pcl/visualization/vtk.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/common/float_image_utils.h>

using namespace pcl::console;
using namespace boost::filesystem;

class OpenNIGrabFrame
{
  public:
    OpenNIGrabFrame (pcl::OpenNIGrabber& grabber, bool paused) 
    : grabber_ (grabber)
    , image_viewer_ ("RGB Image")
    , depth_image_viewer_ ("Depth Image")
    , quit_ (false)
    , continuous_ (!paused)
    , trigger_ (false)
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
      boost::mutex::scoped_lock lock (image_mutex_);
      image_ = image;
      depth_image_ = depth_image;
      lock.unlock ();
    }
    
    void 
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      if (event.keyUp ())
      {
        switch (event.getKeyCode ())
        {
          case 27:
          case 'Q':
          case 'q': quit_ = true;
            break;
          case ' ': continuous_ = !continuous_;
            break;
          case 'G':
          case 'g': trigger_ = true;
            break;
        }
      }
    }
    
    void 
    mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*)
    {
      if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
      {
        trigger_ = true;
      }
    }
    
    void saveImages ()
    {
      std::string time = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());
      boost::shared_ptr<openni_wrapper::Image> image;
      boost::shared_ptr<openni_wrapper::DepthImage> depth_image;

      image_mutex_.lock ();
      image.swap (image_);
      depth_image.swap (depth_image_);
      image_mutex_.unlock ();

      static unsigned char* rgb_data = 0;
      static unsigned rgb_data_size = 0;
      
      if (image)
      {
        const void* data;
        if (image->getEncoding() != openni_wrapper::Image::RGB)
        {
          if (rgb_data_size < image->getWidth () * image->getHeight ())
          {
            if (rgb_data)
              delete [] rgb_data;
            rgb_data_size = image->getWidth () * image->getHeight ();
            rgb_data = new unsigned char [rgb_data_size * 3];
          }
          image->fillRGB (image->getWidth (), image->getHeight (), rgb_data);
          data = reinterpret_cast<const void*> (rgb_data);
        }
        else
          data = reinterpret_cast<const void*> (image->getMetaData ().Data ());

        importer_->SetWholeExtent (0, image->getWidth () - 1, 0, image->getHeight () - 1, 0, 0);
        importer_->SetDataExtentToWholeExtent ();
        
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
      if (depth_image)
      {
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

      trigger_ = false;
    }
    
    int 
    run ()
    {
      // register the keyboard and mouse callback for the visualizer
      image_viewer_.registerMouseCallback (&OpenNIGrabFrame::mouse_callback, *this);
      image_viewer_.registerKeyboardCallback(&OpenNIGrabFrame::keyboard_callback, *this);
      depth_image_viewer_.registerMouseCallback (&OpenNIGrabFrame::mouse_callback, *this);
      depth_image_viewer_.registerKeyboardCallback(&OpenNIGrabFrame::keyboard_callback, *this);
      
      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float) > image_cb = boost::bind (&OpenNIGrabFrame::image_callback, this, _1, _2, _3);
      boost::signals2::connection image_connection = grabber_.registerCallback (image_cb);
      
      // start receiving point clouds
      grabber_.start ();
      unsigned char* rgb_data = 0;
      unsigned rgb_data_size = 0;
      unsigned char* depth_data = 0;

      // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
      while (!image_viewer_.wasStopped() && !quit_)
      {
        std::string time = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());
        boost::shared_ptr<openni_wrapper::Image> image;
        boost::shared_ptr<openni_wrapper::DepthImage> depth_image;
        
        if (image_mutex_.try_lock ())
        {
          image.swap (image_);
          depth_image.swap (depth_image_);
          image_mutex_.unlock ();
        }
        
        if (image)
        {
          const void* data;
          if (image->getEncoding() != openni_wrapper::Image::RGB)
          {
            if (rgb_data_size < image->getWidth () * image->getHeight ())
            {
              if (rgb_data)
                delete [] rgb_data;
              rgb_data_size = image->getWidth () * image->getHeight ();
              rgb_data = new unsigned char [rgb_data_size * 3];
            }
            image->fillRGB (image->getWidth (), image->getHeight (), rgb_data);
            data = reinterpret_cast<const void*> (rgb_data);
          }
          else
            data = reinterpret_cast<const void*> (image->getMetaData ().Data ());            
          image_viewer_.addRGBImage ((const unsigned char*) data, image->getWidth (), image->getHeight ());
          
          if (continuous_ || trigger_)
          {
            importer_->SetWholeExtent (0, image->getWidth () - 1, 0, image->getHeight () - 1, 0, 0);
            importer_->SetDataExtentToWholeExtent ();

            std::stringstream ss;
            ss << "frame_" + time + "_rgb.tiff";
            importer_->SetImportVoidPointer (const_cast<void*>(data), 1);
            importer_->Update ();
            flipper_->SetInputConnection (importer_->GetOutputPort ());
            flipper_->Update ();
            writer_->SetFileName (ss.str ().c_str ());
            writer_->SetInputConnection (flipper_->GetOutputPort ());
            writer_->Write ();
            cout << "writing rgb frame: " << ss.str () << endl;
          }
        } // if (image_)
        
        
        if (depth_image)
        {
          if (depth_data)
            delete[] depth_data;
          depth_data = pcl::visualization::FloatImageUtils::getVisualImage (
              reinterpret_cast<const unsigned short*> (depth_image->getDepthMetaData ().Data ()),
                depth_image->getWidth (), depth_image->getHeight (),
                std::numeric_limits<unsigned short>::min (), 
                // Scale so that the colors look brigher on screen
                std::numeric_limits<unsigned short>::max () / 10, 
                true);
          
          depth_image_viewer_.addRGBImage (depth_data, depth_image->getWidth (), depth_image->getHeight ());
          if (continuous_ || trigger_)
          {
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
            cout << "writing depth frame: " << ss.str () << endl;
          }
        }
        trigger_ = false;
        image_viewer_.spinOnce ();
        depth_image_viewer_.spinOnce ();
        //boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }
      
      image_mutex_.lock ();
      // stop the grabber
      grabber_.stop ();
      image_mutex_.unlock ();
      return 0;
    }

    pcl::OpenNIGrabber& grabber_;
    pcl::visualization::ImageViewer image_viewer_;
    pcl::visualization::ImageViewer depth_image_viewer_;
    bool quit_;
    bool continuous_;
    bool trigger_;
    mutable boost::mutex image_mutex_;
    boost::shared_ptr<openni_wrapper::Image> image_;
    boost::shared_ptr<openni_wrapper::DepthImage> depth_image_;
    vtkSmartPointer<vtkImageImport> importer_, depth_importer_;
    vtkSmartPointer<vtkTIFFWriter> writer_;
    vtkSmartPointer<vtkImageFlip> flipper_;
};

void
usage (char ** argv)
{
  cout << "usage: " << argv[0] << " [((<device_id> | <path-to-oni-file>) [-imagemode <mode>] [-depthformat <format>] [-paused] | -l [<device_id>]| -h | --help)]" << endl;
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
  cout << "                 -paused : start grabber in paused mode. Toggle pause by pressing the space bar\n";
  cout << "                           or grab single frames by just pressing the left mouse button.\n";
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
}

int 
main (int argc, char** argv)
{
  std::string device_id ("");
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  
  if (argc >= 2)
  {
    device_id = argv[1];
    if (device_id == "--help" || device_id == "-h")
    {
      usage (argv);
      return (0);
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
    if (driver.getNumberDevices() > 0)
      cout << "Device Id not set, using first device." << endl;
  }
  
  unsigned mode;
  if (pcl::console::parse (argc, argv, "-imagemode", mode) != -1)
    image_mode = static_cast<pcl::OpenNIGrabber::Mode> (mode);
  
  int depthformat = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth;
  pcl::console::parse_argument (argc, argv, "-depthformat", depthformat);

  pcl::OpenNIGrabber grabber (device_id, pcl::OpenNIGrabber::OpenNI_Default_Mode, image_mode);
  // Set the depth output format
  grabber.getDevice ()->setDepthOutputFormat (static_cast<openni_wrapper::OpenNIDevice::DepthMode> (depthformat));

  bool paused = find_switch (argc, argv, "-paused");
  
  OpenNIGrabFrame grabFrame (grabber, paused);
  return grabFrame.run ();
}


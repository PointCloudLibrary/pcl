/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/io/lzf.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/mouse_event.h>
#include <vtkImageViewer.h>
#include <vtkImageImport.h>
#include <vtkJPEGWriter.h>
#include <vtkBMPWriter.h>
#include <vtkPNMWriter.h>
#include <vtkTIFFWriter.h>
#include "snappy.h"

#ifdef _WIN32
# include <io.h>
# include <windows.h>
# define pcl_open                    _open
# define pcl_close(fd)               _close(fd)
# define pcl_lseek(fd,offset,origin) _lseek(fd,offset,origin)
#else
# include <sys/mman.h>
# define pcl_open                    open
# define pcl_close(fd)               close(fd)
# define pcl_lseek(fd,offset,origin) lseek(fd,offset,origin)
#endif

using namespace std;

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
    SimpleOpenNIViewer (
        pcl::OpenNIGrabber& grabber)
      : grabber_ (grabber)
      , image_viewer_ ("PCL/OpenNI RGB image viewer")
      , depth_image_viewer_ ("PCL/OpenNI depth image viewer")
      , image_cld_init_ (false), depth_image_cld_init_ (false)
      , rgb_data_ (0)
      , depth_data_ (0)
      , rgb_width_ (0), rgb_height_ (0)
      , depth_width_ (0), depth_height_ (0)
      , save_data_ (false)
      , importer_ (vtkSmartPointer<vtkImageImport>::New ())
      , depth_importer_ (vtkSmartPointer<vtkImageImport>::New ())
      , writer_ (vtkSmartPointer<vtkTIFFWriter>::New ())
      , nr_frames_total_ (0)
    {
      importer_->SetNumberOfScalarComponents (3);
      importer_->SetDataScalarTypeToUnsignedChar ();
      depth_importer_->SetNumberOfScalarComponents (1);
      depth_importer_->SetDataScalarTypeToUnsignedShort ();
      //writer_->SetCompressionToNoCompression ();
      //writer_->SetCompressionToPackBits ();
    }

    //////////////////////////////////////////////////////////////////////////
    // Realtime LZF compression
    uint32_t
    compress (const char* input, uint32_t input_size, char *output)
    {
      unsigned int compressed_size = pcl::lzfCompress (
          input,
          input_size,
          &output[8],
          uint32_t (float (input_size) * 1.5f));

      uint32_t compressed_final_size = 0;
      if (compressed_size)
      {
        char *header = &output[0];
        memcpy (&header[0], &compressed_size, sizeof (unsigned int));
        memcpy (&header[4], &input_size, sizeof (unsigned int));
        compressed_final_size = uint32_t (compressed_size + 8);
      }

      return (compressed_final_size);
    }

    //////////////////////////////////////////////////////////////////////////
    bool
    saveImage (const char* data, size_t data_size, 
               const std::string &filename)
    {
#if _WIN32
      HANDLE h_native_file = CreateFile (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
      if (h_native_file == INVALID_HANDLE_VALUE)
        return (false);
      HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, data_size, NULL);
      char *map = static_cast<char*> (MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_size));
      CloseHandle (fm);
      memcpy (&map[0], data, data_size);
      UnmapViewOfFile (map);
      CloseHandle (h_native_file);
#else
      int fd = pcl_open (filename.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
      if (fd < 0)
        return (false);
      // Stretch the file size to the size of the data
      off_t result = pcl_lseek (fd, data_size - 1, SEEK_SET);
      if (result < 0)
      {
        pcl_close (fd);
        return (false);
      }
      // Write a bogus entry so that the new file size comes in effect
      result = static_cast<int> (::write (fd, "", 1));
      if (result != 1)
      {
        pcl_close (fd);
        return (false);
      }
      char *map = static_cast<char*> (mmap (0, data_size, PROT_WRITE, MAP_SHARED, fd, 0));
      if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
      {
        pcl_close (fd);
        return (false);
      }
      memcpy (&map[0], data, data_size);
      if (munmap (map, (data_size)) == -1)
      {
        pcl_close (fd);
        return (false);
      }
      pcl_close (fd);
#endif
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////
    bool
    saveDepth (const char* data,
               int width, int height,
               const std::string &filename)
    {
      // Save compressed depth to disk
      unsigned int depth_size = width * height * 2;
      char* compressed_depth = static_cast<char*> (malloc (size_t (float (depth_size) * 1.5f + 8.0f)));
      size_t compressed_final_size = compress (
          data,
          depth_size,
          compressed_depth);

      saveImage (compressed_depth, compressed_final_size, filename);
      free (compressed_depth);
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////
    void
    saveRGB (const char *data, 
             int width, int height,
             const std::string &filename)
    {
      // Transform RGBRGB into RRGGBB for better compression
      vector<char> rrggbb (width * height * 3);
      int ptr1 = 0,
          ptr2 = width * height,
          ptr3 = ptr2 + width * height;
      for (int i = 0; i < width * height; ++i, ++ptr1, ++ptr2, ++ptr3)
      {
        rrggbb[ptr1] = data[i * 3 + 0];
        rrggbb[ptr2] = data[i * 3 + 1];
        rrggbb[ptr3] = data[i * 3 + 2];
      }

      char* compressed_rgb = static_cast<char*> (malloc (size_t (float (rrggbb.size ()) * 1.5f + 8.0f)));
      size_t compressed_final_size = compress (
          reinterpret_cast<const char*> (&rrggbb[0]), 
          uint32_t (rrggbb.size ()),
          compressed_rgb);

      // Save compressed RGB to disk
      saveImage (compressed_rgb, compressed_final_size, filename);
      free (compressed_rgb);
    }

    //////////////////////////////////////////////////////////////////////////
    void
    image_callback (const boost::shared_ptr<openni_wrapper::Image> &image, 
                    const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image, float)
    {
      {
        FPS_CALC ("image callback");
        boost::mutex::scoped_lock lock (image_mutex_);

        // Copy data
        rgb_width_  = image->getWidth ();
        rgb_height_ = image->getHeight ();
        rgb_data_.resize (rgb_width_ * rgb_height_ * 3);
        depth_width_  = depth_image->getWidth ();
        depth_height_ = depth_image->getHeight ();
        depth_data_.resize (depth_width_ * depth_height_ * 2);

        if (image->getEncoding () != openni_wrapper::Image::RGB)
          image->fillRGB (rgb_width_, rgb_height_, &rgb_data_[0]);
        else
          memcpy (&rgb_data_[0], image->getMetaData ().Data (), rgb_data_.size ());

        // Copy data
        memcpy (&depth_data_[0], reinterpret_cast<const unsigned char*> (&depth_image->getDepthMetaData ().Data ()[0]), depth_data_.size ());
      }

      // If save data is enabled
      if (save_data_)
      {
        pcl::console::TicToc tt;
        tt.tic ();
        nr_frames_total_++;
        std::string time = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());
        std::stringstream ss1, ss2;
        ss1 << "frame_" << time << "_rgb.pclzf";
        saveRGB (reinterpret_cast<char*> (&rgb_data_[0]), rgb_width_, rgb_height_, ss1.str ());
        ss2 << "frame_" + time + "_depth.pclzf";
        saveDepth (reinterpret_cast<char*> (&depth_data_[0]), depth_width_, depth_height_, ss2.str ());

#if 0
        importer_->SetWholeExtent (0, image->getWidth () - 1, 0, image->getHeight () - 1, 0, 0);
        importer_->SetDataExtentToWholeExtent ();
        importer_->SetImportVoidPointer (data, 1);
        importer_->Update ();

      writer_->SetCompressionToPackBits ();
        writer_->SetFileName (ss1.str ().c_str ());
        writer_->SetInputConnection (importer_->GetOutputPort ());
        writer_->Write ();
#endif   
#if 0
        // Compress the valid data
        unsigned int data_size = image->getWidth () * image->getHeight () * 3;
        char* compressed_rgb = static_cast<char*> (malloc (size_t (float (data_size) * 1.5f + 8.0f)));
        char *data2 = new char[data_size];
        char *data3 = (char*)data;
        for (int i = 0; i < image->getWidth () * image->getHeight (); ++i)
        {
          data2[i] = data3[i * 3];
          data2[i + image->getWidth () * image->getHeight ()] = data3[i * 3 + 1];
          data2[i + image->getWidth () * image->getHeight ()  * 2] = data3[i * 3 + 2];
        }
        // Compress the valid data
        pcl::console::TicToc tt;
        tt.tic ();
        size_t rgb_compressed_size = compress (
            reinterpret_cast<const char*> (data), 
            data_size,
            compressed_rgb);
        cerr << "RGB compressed 1 : " << rgb_compressed_size << " (" << tt.toc () << ")\n";
        free (compressed_rgb);
#endif

#if 0
        vector<uint16_t> depth_raw (depth_image->getWidth () * depth_image->getHeight ());
        memcpy (&depth_raw[0], &depth_image->getDepthMetaData ().Data ()[0], depth_raw.size ());
        vector<uint8_t> depth_compressed;
        pcl::io::encodeMonoImageToPNG (depth_raw, depth_image->getWidth (), 
                                       depth_image->getHeight (), depth_compressed, 
                                       depth_png_compression_);
      
        vector<uint8_t> rgb_raw (image->getWidth () * image->getHeight () );
        //memcpy (&rgb_raw[0], &image->getMetaData ().Data ()[0], rgb_raw.size ());
        //memcpy (&rgb_raw[0], reinterpret_cast<const char*> (&data[0]), rgb_raw.size ());
        //memcpy (&rgb_raw[0], &rgb_data_[0], rgb_raw.size ());
        for (int i = 0; i < image->getWidth () * image->getHeight (); ++i)
          rgb_raw[i] = rgb_data_[i * 3];
        vector<uint8_t> rgb_compressed;
#endif
#if 0
        char* output = new char[snappy::MaxCompressedLength(data_size)];
        tt.tic ();
        size_t output_length;
        snappy::RawCompress (
            reinterpret_cast<const char*> (data),
            data_size,
            output, 
            &output_length);
        cerr << "RGB compressed 2 : " << output_length << " (" << tt.toc () << ")\n";
        
        delete[] output;
#endif
        //for (int i = 0; i < 9; ++i)
        //{
        //  tt.tic ();
        //  pcl::io::encodeMonoImageToPNG (
        //      rgb_raw,
        //      image->getWidth (),
        //      image->getHeight (),
        //      rgb_compressed, i);
        //  cerr << "[" << i << "] RGB compressed : " << rgb_compressed.size () << " (" << tt.toc () << ")\n";
        //}

#if 0
        depth_importer_->SetWholeExtent (0, depth_image->getWidth () - 1, 0, depth_image->getHeight () - 1, 0, 0);
        depth_importer_->SetDataExtentToWholeExtent ();
        depth_importer_->SetImportVoidPointer ((void*)(depth_image->getDepthMetaData ().Data ()), 1);
        //depth_importer_->CopyImportVoidPointer (compressed_depth, depth_compressed_size);
        depth_importer_->Update ();
        writer_->SetFileName (ss2.str ().c_str ());
        writer_->SetInputConnection (depth_importer_->GetOutputPort ());
        writer_->SetCompressionToPackBits ();
        writer_->Write ();
#endif
        //tt.toc_print ();
      }
    }
    
    void 
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      // Space toggle saving the file
      if (event.getKeySym () == "space")
      {
        if (event.keyDown ())
        {
          save_data_ = !save_data_;
          PCL_INFO ("Toggled recording state: %s.\n", save_data_ ? "enabled" : "disabled");
        }
        return;
      }

      // Q quits
      if (event.getKeyCode () == 'q')
        return;

      //string* message = static_cast<string*> (cookie);
      //cout << (*message) << " :: ";
      //if (event.getKeyCode ())
      //  cout << "the key \'" << event.getKeyCode () << "\' (" << event.getKeyCode () << ") was";
      //else
      //  cout << "the special key \'" << event.getKeySym () << "\' was";
      //if (event.keyDown ())
      //  cout << " pressed" << endl;
      //else
      //  cout << " released" << endl;
    }
    
    void 
    mouse_callback (const pcl::visualization::MouseEvent&, void*)
    {
      //string* message = static_cast<string*> (cookie);
      //if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton)
      //{
      //  cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      //}
    }

    void
    run ()
    {
      string mouseMsg2D ("Mouse coordinates in image viewer");
      string keyMsg2D ("Key event for image viewer");

      image_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, static_cast<void*> (&mouseMsg2D));
      image_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, static_cast<void*> (&keyMsg2D));
      depth_image_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, static_cast<void*> (&mouseMsg2D));
      depth_image_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, static_cast<void*> (&keyMsg2D));
        
      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float) > image_cb = boost::bind (&SimpleOpenNIViewer::image_callback, this, _1, _2, _3);
      boost::signals2::connection image_connection = grabber_.registerCallback (image_cb);
      
      grabber_.start ();

      while (!image_viewer_.wasStopped () && !depth_image_viewer_.wasStopped ())
      {
        boost::this_thread::sleep (boost::posix_time::microseconds (100));

        if (!image_cld_init_)
        {
          image_viewer_.setPosition (0, 0);
          image_cld_init_ = !image_cld_init_;
        }

        image_viewer_.spinOnce ();
        depth_image_viewer_.spinOnce ();
        
        FPS_CALC ("visualization callback");
        // Add to renderer
        if (!rgb_data_.empty ())
          image_viewer_.addRGBImage (reinterpret_cast<unsigned char*> (&rgb_data_[0]), rgb_width_, rgb_height_);

        if (!depth_data_.empty ())
        {
          unsigned char* data = pcl::visualization::FloatImageUtils::getVisualImage (
              reinterpret_cast<unsigned short*> (&depth_data_[0]),
                depth_width_, depth_height_,
                std::numeric_limits<unsigned short>::min (), 
                // Scale so that the colors look brigher on screen
                std::numeric_limits<unsigned short>::max () / 10, 
                true);

          depth_image_viewer_.addRGBImage (data, depth_width_, depth_height_);
          if (!depth_image_cld_init_)
          {
            depth_image_viewer_.setPosition (depth_width_, 0);
            depth_image_cld_init_ = !depth_image_cld_init_;
          }
          delete[] data;
        }
      }

      grabber_.stop ();
      
      image_connection.disconnect ();
    
      PCL_INFO ("Total number of frames written: %d.\n", nr_frames_total_);
    }

    pcl::OpenNIGrabber& grabber_;
    boost::mutex image_mutex_;
    pcl::visualization::ImageViewer image_viewer_;
    pcl::visualization::ImageViewer depth_image_viewer_;
    bool image_cld_init_, depth_image_cld_init_;
    vector<unsigned char> rgb_data_, depth_data_;
    unsigned rgb_width_, rgb_height_, depth_width_, depth_height_;
    bool save_data_;
    vtkSmartPointer<vtkImageImport> importer_, depth_importer_;
    vtkSmartPointer<vtkTIFFWriter> writer_;
    int nr_frames_total_;
};

void
usage (char ** argv)
{
  cout << "usage: " << argv[0] << " [((<device_id> | <path-to-oni-file>) [-imagemode <mode>] | [-depthmode <mode>] | [-depthformat <format>] | -l [<device_id>]| -h | --help)]" << endl;
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
}

int
main (int argc, char ** argv)
{
  std::string device_id ("");
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  
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
        if (device->hasDepthStream ())
        {
          cout << endl << "Supported depth modes for device: " << device->getVendorName () << " , " << device->getProductName () << endl;
          modes = grabber.getAvailableDepthModes ();
          for (std::vector<std::pair<int, XnMapOutputMode> >::const_iterator it = modes.begin (); it != modes.end (); ++it)
          {
            cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
          }
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
  if (pcl::console::parse (argc, argv, "-depthmode", mode) != -1)
    depth_mode = static_cast<pcl::OpenNIGrabber::Mode> (mode);
  
  int depthformat = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth;
  pcl::console::parse_argument (argc, argv, "-depthformat", depthformat);

  pcl::OpenNIGrabber grabber (device_id, depth_mode, image_mode);
  // Set the depth output format
  grabber.getDevice ()->setDepthOutputFormat (static_cast<openni_wrapper::OpenNIDevice::DepthMode> (depthformat));

  SimpleOpenNIViewer v (grabber);
  v.run ();

  return (0);
}

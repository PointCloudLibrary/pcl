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

#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/io/lzf_image_io.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/mouse_event.h>

#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> // for ptime, to_iso_string, ...

#include <csignal>
#include <limits>
#include <mutex>
#include <thread>
#include <memory>

using namespace std::chrono_literals;
using namespace pcl;
using namespace pcl::console;

bool global_visualize = true, is_done = false, save_data = false, toggle_one_frame_capture = false, visualize = true;
std::mutex io_mutex;
int nr_frames_total = 0;

#if defined(__linux__) 
#include <unistd.h>
// Get the available memory size on Linux systems

size_t 
getTotalSystemMemory ()
{
  std::uint64_t pages = sysconf (_SC_AVPHYS_PAGES);
  std::uint64_t page_size = sysconf (_SC_PAGE_SIZE);
  print_info ("Total available memory size: %lluMB.\n", (pages * page_size) / 1048576);
  if (pages * page_size > std::uint64_t (std::numeric_limits<std::size_t>::max ()))
  {
    return std::numeric_limits<std::size_t>::max ();
  }
  return std::size_t (pages * page_size);
}

const int BUFFER_SIZE = int (getTotalSystemMemory () / (640 * 480) / 2);
#else

const int BUFFER_SIZE = 200;
#endif

int buff_size = BUFFER_SIZE;

#define FPS_CALC_WRITER(_WHAT_, buff) \
do \
{ \
    static unsigned count = 0;\
    static double last = getTime ();\
    double now = getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cerr << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz. Queue size: " << buff.getSize () << ", number of frames written so far: " << nr_frames_total << "\n"; \
      count = 0; \
      last = now; \
    } \
}while(false)

#define FPS_CALC_VIEWER(_WHAT_, buff) \
do \
{ \
    static unsigned count = 0;\
    static double last = getTime ();\
    double now = getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cerr << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz. Queue size: " << buff.getSize () << "\n"; \
      count = 0; \
      last = now; \
    } \
}while(false)

#define FPS_CALC_DRIVER(_WHAT_, buff1, buff2) \
do \
{ \
    static unsigned count = 0;\
    static double last = getTime ();\
    double now = getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      if (visualize && global_visualize) \
        std::cerr << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz. Queue size: " << buff1.getSize () << " (w) / " << buff2.getSize () << " (v)\n"; \
      else \
        std::cerr << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz. Queue size: " << buff1.getSize () << " (w)\n"; \
      count = 0; \
      last = now; \
    } \
}while(false)

//////////////////////////////////////////////////////////////////////////////////////////
struct Frame
{
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

  Frame (const openni_wrapper::Image::Ptr &_image,
         const openni_wrapper::DepthImage::Ptr &_depth_image,
         const io::CameraParameters &_parameters_rgb,
         const io::CameraParameters &_parameters_depth,
         const boost::posix_time::ptime &_time)
    : image (_image)
    , depth_image (_depth_image)
    , parameters_rgb (_parameters_rgb)
    , parameters_depth (_parameters_depth)
    , time (_time) 
  {}

  const openni_wrapper::Image::Ptr image;
  const openni_wrapper::DepthImage::Ptr depth_image;
        
  io::CameraParameters parameters_rgb, parameters_depth;

  boost::posix_time::ptime time;
};

//////////////////////////////////////////////////////////////////////////////////////////
class Buffer
{
	public:
    Buffer () = default;
    Buffer (const Buffer&) = delete;            // Disabled copy constructor
    Buffer& operator =(const Buffer&) = delete; // Disabled assignment operator

    bool 
    pushBack (Frame::ConstPtr frame)
    {
      bool retVal = false;
      {
        std::lock_guard<std::mutex> buff_lock (bmutex_);
        if (!buffer_.full ())
          retVal = true;
        buffer_.push_back (frame);
      }
      buff_empty_.notify_one ();
      return (retVal);
    }

    Frame::ConstPtr 
    popFront ()
    {
      Frame::ConstPtr cloud;
      {
        std::unique_lock<std::mutex> buff_lock (bmutex_);
        while (buffer_.empty ())
        {
          if (is_done)
            break;
          {
            std::lock_guard<std::mutex> io_lock (io_mutex);
            //std::cerr << "No data in buffer_ yet or buffer is empty." << std::endl;
          }
          buff_empty_.wait (buff_lock);
        }
        cloud = buffer_.front ();
        buffer_.pop_front ();
      }
      return (cloud);
    }
		
    inline bool 
    isFull ()
    {
      std::lock_guard<std::mutex> buff_lock (bmutex_);
      return (buffer_.full ());
    }
		
    inline bool
    isEmpty ()
    {
      std::lock_guard<std::mutex> buff_lock (bmutex_);
    	return (buffer_.empty ());
    }
		
    inline int 
    getSize ()
    {
      std::lock_guard<std::mutex> buff_lock (bmutex_);
      return (int (buffer_.size ()));
    }
		
    inline int 
    getCapacity ()
    {
	    return (int (buffer_.capacity ()));
    }
		
    inline void 
    setCapacity (int buff_size)
    {
      std::lock_guard<std::mutex> buff_lock (bmutex_);
      buffer_.set_capacity (buff_size);
    }

    inline void 
    clear ()
    {
      std::lock_guard<std::mutex> buff_lock (bmutex_);
      buffer_.clear ();
    }

	private:
    std::mutex bmutex_;
		std::condition_variable buff_empty_;
		boost::circular_buffer<Frame::ConstPtr> buffer_;
};

//////////////////////////////////////////////////////////////////////////////////////////
class Writer 
{
  private:
    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    writeToDisk (const Frame::ConstPtr& frame)
    {
      if (!frame)
        return;

      FPS_CALC_WRITER ("data write   ", buf_);
      nr_frames_total++;
      
      std::string time_string = boost::posix_time::to_iso_string (frame->time);
      // Save RGB data
      const std::string rgb_filename = "frame_" + time_string + "_rgb.pclzf";
      switch (frame->image->getEncoding ())
      {
        case openni_wrapper::Image::YUV422:
        {
          io::LZFYUV422ImageWriter lrgb;
          lrgb.write (reinterpret_cast<const char*> (&frame->image->getMetaData ().Data ()[0]), frame->image->getWidth (), frame->image->getHeight (), rgb_filename);
          break;
        }
        case openni_wrapper::Image::RGB:
        {
          io::LZFRGB24ImageWriter lrgb;
          lrgb.write (reinterpret_cast<const char*> (&frame->image->getMetaData ().Data ()[0]), frame->image->getWidth (), frame->image->getHeight (), rgb_filename);
          break;
        }
        case openni_wrapper::Image::BAYER_GRBG:
        {
          io::LZFBayer8ImageWriter lrgb;
          lrgb.write (reinterpret_cast<const char*> (&frame->image->getMetaData ().Data ()[0]), frame->image->getWidth (), frame->image->getHeight (), rgb_filename);
          break;
        }
      }

      // Save depth data
      const std::string depth_filename = "frame_" + time_string + "_depth.pclzf";
      io::LZFDepth16ImageWriter ld;
      //io::LZFShift11ImageWriter ld;
      ld.write (reinterpret_cast<const char*> (&frame->depth_image->getDepthMetaData ().Data ()[0]), frame->depth_image->getWidth (), frame->depth_image->getHeight (), depth_filename);
      
      // Save depth data
      const std::string xml_filename = "frame_" + time_string + ".xml";
         
      io::LZFRGB24ImageWriter lrgb;
      lrgb.writeParameters (frame->parameters_rgb, xml_filename);
      ld.writeParameters (frame->parameters_depth, xml_filename);
      // By default, the z-value depth multiplication factor is written as part of the LZFDepthImageWriter's writeParameters as 0.001
      // If you want to change that, uncomment the next line and change the value
      //ld.writeParameter (0.001, "depth.z_multiplication_factor", ss3.str ());

      toggle_one_frame_capture = false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    receiveAndProcess ()
    {
      while (!is_done)
      {
        if (save_data || toggle_one_frame_capture)
          writeToDisk (buf_.popFront ());
        else
          std::this_thread::sleep_for(100us);
      }

      if (save_data && buf_.getSize () > 0)
      {
        {
          std::lock_guard<std::mutex> io_lock (io_mutex);
          print_info ("Writing remaining %ld clouds in the buffer to disk...\n", buf_.getSize ());
        }
        while (!buf_.isEmpty ())
        {
          {
            std::lock_guard<std::mutex> io_lock (io_mutex);
            print_info ("Clearing buffer... %ld remaining...\n", buf_.getSize ());
          }
          writeToDisk (buf_.popFront ());
        }
      }
    }

  public:
    Writer (Buffer &buf)
      : buf_ (buf)
    {
      thread_.reset (new std::thread (&Writer::receiveAndProcess, this));
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      thread_->join ();
      std::lock_guard<std::mutex> io_lock (io_mutex);
      print_highlight ("Writer done.\n");
    }

  private:
    Buffer &buf_;
    std::shared_ptr<std::thread> thread_;
};


///////////////////////////////////////////////////////////////////////////////////////////
class Driver
{
  private:
    //////////////////////////////////////////////////////////////////////////
    void
    image_callback (const openni_wrapper::Image::Ptr &image, 
                    const openni_wrapper::DepthImage::Ptr &depth_image, 
                    float)
    {
      boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time ();
      FPS_CALC_DRIVER ("driver       ", buf_write_, buf_vis_);

      // Extract camera parameters
      io::CameraParameters parameters_rgb;
      parameters_rgb.focal_length_x = parameters_rgb.focal_length_y = grabber_.getDevice ()->getImageFocalLength (depth_image->getWidth ());
      parameters_rgb.principal_point_x = image->getWidth () >> 1;
      parameters_rgb.principal_point_y = image->getHeight () >> 1;

      io::CameraParameters parameters_depth;
      parameters_depth.focal_length_x = parameters_depth.focal_length_y = grabber_.getDevice ()->getDepthFocalLength (depth_image->getWidth ());
      parameters_depth.principal_point_x = depth_image->getWidth () >> 1;
      parameters_depth.principal_point_y = depth_image->getHeight () >> 1;

      // Create a new frame
      Frame::ConstPtr frame (new Frame (image, depth_image, 
                                        parameters_rgb, parameters_depth,
                                        time));

      if ((save_data || toggle_one_frame_capture) && !buf_write_.pushBack (frame))
      {
        std::lock_guard<std::mutex> io_lock (io_mutex);
        print_warn ("Warning! Write buffer was full, overwriting data!\n");
      }

      if (global_visualize && visualize && !buf_vis_.pushBack (frame))
      {
        std::lock_guard<std::mutex> io_lock (io_mutex);
        print_warn ("Warning! Visualization buffer was full, overwriting data!\n");
      }
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    grabAndSend ()
    {
      std::function<
        void (const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, float)
      > image_cb = [this] (const openni_wrapper::Image::Ptr& img, const openni_wrapper::DepthImage::Ptr& depth, float f)
      {
        image_callback (img, depth, f);
      };
      boost::signals2::connection image_connection = grabber_.registerCallback (image_cb);

      grabber_.start ();
      
      while (!is_done)
      {
        std::this_thread::sleep_for(1s);
      }
      grabber_.stop ();
      image_connection.disconnect ();
    }

  public:
    Driver (OpenNIGrabber& grabber, Buffer &buf_write, Buffer &buf_vis)
      : grabber_ (grabber)
      , buf_write_ (buf_write)
      , buf_vis_ (buf_vis)
    {
      thread_.reset (new std::thread (&Driver::grabAndSend, this));
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      thread_->join ();
      std::lock_guard<std::mutex> io_lock (io_mutex);
      print_highlight ("Grabber done.\n");
    }

  private:
    
    OpenNIGrabber& grabber_;
    Buffer &buf_write_, &buf_vis_;
    std::shared_ptr<std::thread> thread_;
};

///////////////////////////////////////////////////////////////////////////////////////////
class Viewer
{
  private:
    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    receiveAndView ()
    {
      std::string mouseMsg2D ("Mouse coordinates in image viewer");
      std::string keyMsg2D ("Key event for image viewer");

      image_viewer_->registerMouseCallback (&Viewer::mouse_callback, *this, static_cast<void*> (&mouseMsg2D));
      image_viewer_->registerKeyboardCallback(&Viewer::keyboard_callback, *this, static_cast<void*> (&keyMsg2D));
      depth_image_viewer_->registerMouseCallback (&Viewer::mouse_callback, *this, static_cast<void*> (&mouseMsg2D));
      depth_image_viewer_->registerKeyboardCallback(&Viewer::keyboard_callback, *this, static_cast<void*> (&keyMsg2D));

      // Position the first window (RGB)
      if (!image_cld_init_)
      {
        image_viewer_->setPosition (0, 0);
        image_cld_init_ = !image_cld_init_;
      }

      // Process until stopped
      while (!image_viewer_->wasStopped () && 
             !depth_image_viewer_->wasStopped () && 
             !is_done)
      {
        std::this_thread::sleep_for(100us);

        if (!visualize)
        {
          image_viewer_->spinOnce ();
          depth_image_viewer_->spinOnce ();
          continue;
        }

        while (!buf_.isEmpty () && !is_done)
        {
          Frame::ConstPtr frame = buf_.popFront ();

          FPS_CALC_VIEWER ("visualization", buf_);
          // Add to renderer
          if (frame->image)
          {
            // Copy RGB data for visualization
            static std::vector<unsigned char> rgb_data (frame->image->getWidth () * frame->image->getHeight () * 3);
            if (frame->image->getEncoding () != openni_wrapper::Image::RGB)
            {
              frame->image->fillRGB (frame->image->getWidth (), 
                                     frame->image->getHeight (), 
                                     &rgb_data[0]);
            }
            else
              memcpy (&rgb_data[0],
                      frame->image->getMetaData ().Data (),
                      rgb_data.size ());

            image_viewer_->addRGBImage (reinterpret_cast<unsigned char*> (&rgb_data[0]), 
                                        frame->image->getWidth (),
                                        frame->image->getHeight (),
                                        "rgb_image");
          }

          if (frame->depth_image)
          {
            unsigned char* data = visualization::FloatImageUtils::getVisualImage (
                reinterpret_cast<const unsigned short*> (&frame->depth_image->getDepthMetaData ().Data ()[0]),
                  frame->depth_image->getWidth (), frame->depth_image->getHeight (),
                  std::numeric_limits<unsigned short>::min (), 
                  // Scale so that the colors look brigher on screen
                  std::numeric_limits<unsigned short>::max () / 10, 
                  true);

            depth_image_viewer_->addRGBImage (data, 
                                              frame->depth_image->getWidth (),
                                              frame->depth_image->getHeight (),
                                              "rgb_image");
            if (!depth_image_cld_init_)
            {
              depth_image_viewer_->setPosition (frame->depth_image->getWidth (), 0);
              depth_image_cld_init_ = !depth_image_cld_init_;
            }
            delete[] data;
          }
          image_viewer_->spinOnce ();
          depth_image_viewer_->spinOnce ();
        }
      }
    }

  public:
    ///////////////////////////////////////////////////////////////////////////////////////
    Viewer (Buffer &buf)
      : buf_ (buf)
      , image_cld_init_ (false), depth_image_cld_init_ (false)
    {
      image_viewer_.reset (new visualization::ImageViewer ("PCL/OpenNI RGB image viewer"));
      depth_image_viewer_.reset (new visualization::ImageViewer ("PCL/OpenNI depth image viewer"));

      receiveAndView ();
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      std::lock_guard<std::mutex> io_lock (io_mutex);
      print_highlight ("Viewer done.\n");
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    keyboard_callback (const visualization::KeyboardEvent& event, void*)
    {
      // Space toggle saving the file
      if (event.getKeySym () == "space")
      {
        if (event.keyDown ())
        {
          save_data = !save_data;
          PCL_INFO ("Toggled recording state: %s.\n", save_data ? "enabled" : "disabled");
        }
        return;
      }

      // V turns visualization on/off
      if (event.getKeyCode () == 'v' && event.keyDown ())
      {
        visualize = !visualize;
        if (!visualize)
          buf_.clear ();
        PCL_INFO ("Visualization state: %s.\n", (visualize ? "on" : "off"));
        return;
      }

      // S saves one frame
      if (event.getKeyCode () == 's' && event.keyDown ())
      {
        toggle_one_frame_capture = true;
        PCL_INFO ("Toggled single frame capture state.\n");
        return;
      }

      // Q quits
      if (event.getKeyCode () == 'q')
      {
        is_done = true;
        return;
      }
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    mouse_callback (const visualization::MouseEvent& mouse_event, void*)
    {
      if (mouse_event.getType () == visualization::MouseEvent::MouseButtonPress && mouse_event.getButton () == visualization::MouseEvent::LeftButton)
      {
        toggle_one_frame_capture = true;
        PCL_INFO ("Toggled single frame capture state.\n");
      }
    }

    Buffer &buf_;
    visualization::ImageViewer::Ptr image_viewer_;
    visualization::ImageViewer::Ptr depth_image_viewer_;
    bool image_cld_init_, depth_image_cld_init_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " [(<device_id> [-visualize | -imagemode <mode>] | [-depthmode <mode>] | [-depthformat <format>] | -l [<device_id>]| -h | --help)]\n";
  std::cout << argv[0] << " -h | --help : shows this help\n";
  std::cout << argv[0] << " -l : list all available devices\n";
  std::cout << argv[0] << " -buf X         : use a buffer size of X frames (default: " << buff_size << ")\n";
  std::cout << argv[0] << " -visualize 0/1 : turn the visualization off/on (WARNING: when visualization is disabled, data writing is enabled by default!)\n";
  std::cout << argv[0] << " -l <device-id> : list all available modes for specified device\n";

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
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
ctrlC (int)
{
	std::lock_guard<std::mutex> io_lock (io_mutex);
	print_info ("\nCtrl-C detected, exit condition set to true.\n");
	is_done = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char ** argv)
{
  print_highlight ("PCL OpenNI Image Viewer/Recorder. See %s -h for options.\n", argv[0]);
  bool debug = false;
  console::parse_argument (argc, argv, "-debug", debug);
  if (debug)
    console::setVerbosityLevel (console::L_DEBUG);

  if (parse_argument (argc, argv, "-buf", buff_size) != -1)
    print_highlight ("Setting buffer size to %d frames.\n", buff_size);
  else
    print_highlight ("Using default buffer size of %d frames.\n", buff_size);

  std::string device_id;
  OpenNIGrabber::Mode image_mode = OpenNIGrabber::OpenNI_Default_Mode;
  OpenNIGrabber::Mode depth_mode = OpenNIGrabber::OpenNI_Default_Mode;
  
  if (argc >= 2)
  {
    device_id = argv[1];
    if (device_id == "--help" || device_id == "-h")
    {
      usage (argv);
      return (0);
    }
    if (device_id == "-l")
    {
      if (argc >= 3)
      {
        OpenNIGrabber grabber (argv[2]);
        auto device = grabber.getDevice ();
        std::vector<std::pair<int, XnMapOutputMode> > modes;

        if (device->hasImageStream ())
        {
          std::cout << std::endl << "Supported image modes for device: " << device->getVendorName () << " , " << device->getProductName () << std::endl;
          modes = grabber.getAvailableImageModes ();
          for (const auto& mode : modes)
          {
            std::cout << mode.first << " = " << mode.second.nXRes << " x " << mode.second.nYRes << " @ " << mode.second.nFPS << std::endl;
          }
        if (device->hasDepthStream ())
        {
          std::cout << std::endl << "Supported depth modes for device: " << device->getVendorName () << " , " << device->getProductName () << std::endl;
          modes = grabber.getAvailableDepthModes ();
          for (const auto& mode : modes)
          {
            std::cout << mode.first << " = " << mode.second.nXRes << " x " << mode.second.nYRes << " @ " << mode.second.nFPS << std::endl;
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
    if (driver.getNumberDevices() > 0)
      std::cout << "Device Id not set, using first device." << std::endl;
  }
  
  unsigned mode;
  if (console::parse (argc, argv, "-imagemode", mode) != -1)
    image_mode = static_cast<OpenNIGrabber::Mode> (mode);
  if (console::parse (argc, argv, "-depthmode", mode) != -1)
    depth_mode = static_cast<OpenNIGrabber::Mode> (mode);
  
  int depthformat = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth;
  console::parse_argument (argc, argv, "-depthformat", depthformat);
  console::parse_argument (argc, argv, "-visualize", global_visualize);

  OpenNIGrabber ni_grabber (device_id, depth_mode, image_mode);
  // Set the depth output format
  ni_grabber.getDevice ()->setDepthOutputFormat (static_cast<openni_wrapper::OpenNIDevice::DepthMode> (depthformat));

  //int imageformat = 0;
  //console::parse_argument (argc, argv, "-imageformat", imageformat);

  Buffer buf_write, buf_vis;
  buf_write.setCapacity (buff_size);
  buf_vis.setCapacity (buff_size);
  
  signal (SIGINT, ctrlC);

  Driver driver (ni_grabber, buf_write, buf_vis);
  Writer writer (buf_write);
  std::shared_ptr<Viewer> viewer;
  if (global_visualize)
    viewer.reset (new Viewer (buf_vis));
  else
    save_data = true;

  // Exit here when the viewer ends (if enabled)
  driver.stop ();
  if (global_visualize)
    viewer->stop ();
  writer.stop ();

  print_highlight ("Total number of frames written: %d.\n", nr_frames_total);
  return (0);
}

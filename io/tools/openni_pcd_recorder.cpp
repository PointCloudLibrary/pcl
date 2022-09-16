/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Sudarshan Srinivasan <sudarshan85@gmail.com>
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
#include <pcl/io/openni_grabber.h>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> // for to_iso_string, local_time
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h> //fps calculations

#include <csignal>
#include <limits>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using namespace pcl;
using namespace pcl::console;

bool is_done = false;
std::mutex io_mutex;

#if defined(__linux__) || defined (TARGET_OS_MAC)
#include <unistd.h>
// Get the available memory size on Linux/BSD systems

size_t 
getTotalSystemMemory ()
{
  std::uint64_t memory = std::numeric_limits<std::size_t>::max ();

#ifdef _SC_AVPHYS_PAGES
  std::uint64_t pages = sysconf (_SC_AVPHYS_PAGES);
  std::uint64_t page_size = sysconf (_SC_PAGE_SIZE);
  
  memory = pages * page_size;
  
#elif defined(HAVE_SYSCTL) && defined(HW_PHYSMEM)
  // This works on *bsd and darwin.
  unsigned int physmem;
  std::size_t len = sizeof physmem;
  static int mib[2] = { CTL_HW, HW_PHYSMEM };

  if (sysctl (mib, ARRAY_SIZE (mib), &physmem, &len, NULL, 0) == 0 && len == sizeof (physmem))
  {
    memory = physmem;
  }
#endif

  if (memory > std::uint64_t (std::numeric_limits<std::size_t>::max ()))
  {
    memory = std::numeric_limits<std::size_t>::max ();
  }
  
  print_info ("Total available memory size: %lluMB.\n", memory / 1048576ull);
  return std::size_t (memory);
}

const std::size_t BUFFER_SIZE = std::size_t (getTotalSystemMemory () / (640 * 480 * sizeof (pcl::PointXYZRGBA)));
#else

const std::size_t BUFFER_SIZE = 200;
#endif

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class PCDBuffer
{
  public:
    PCDBuffer () = default;
    PCDBuffer (const PCDBuffer&) = delete; // Disabled copy constructor
    PCDBuffer& operator = (const PCDBuffer&) = delete; // Disabled assignment operator

    bool 
    pushBack (typename PointCloud<PointT>::ConstPtr); // thread-save wrapper for push_back() method of ciruclar_buffer

    typename PointCloud<PointT>::ConstPtr 
    getFront (); // thread-save wrapper for front() method of ciruclar_buffer

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

  private:
    std::mutex bmutex_;
    std::condition_variable buff_empty_;
    boost::circular_buffer<typename PointCloud<PointT>::ConstPtr> buffer_;
};

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool 
PCDBuffer<PointT>::pushBack (typename PointCloud<PointT>::ConstPtr cloud)
{
  bool retVal = false;
  {
    std::lock_guard<std::mutex> buff_lock (bmutex_);
    if (!buffer_.full ())
      retVal = true;
    buffer_.push_back (cloud);
  }
  buff_empty_.notify_one ();
  return (retVal);
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename PointCloud<PointT>::ConstPtr 
PCDBuffer<PointT>::getFront ()
{
  typename PointCloud<PointT>::ConstPtr cloud;
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

#define FPS_CALC(_WHAT_, buff) \
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

//////////////////////////////////////////////////////////////////////////////////////////
// Producer thread class
template <typename PointT>
class Producer
{
  private:
    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    grabberCallBack (const typename PointCloud<PointT>::ConstPtr& cloud)
    {
      if (!buf_.pushBack (cloud))
      {
        {
          std::lock_guard<std::mutex> io_lock (io_mutex);
          print_warn ("Warning! Buffer was full, overwriting data!\n");
        }
      }
      FPS_CALC ("cloud callback.", buf_);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    grabAndSend ()
    {
      auto* grabber = new OpenNIGrabber ();
      grabber->getDevice ()->setDepthOutputFormat (depth_mode_);

      Grabber* interface = grabber;
      std::function<void (const typename PointCloud<PointT>::ConstPtr&)> f = [this] (const typename PointCloud<PointT>::ConstPtr& cloud)
      {
        grabberCallBack (cloud);
      };
      interface->registerCallback (f);
      interface->start ();

      while (true)
      {
        if (is_done)
          break;
        std::this_thread::sleep_for(1s);
      }
      interface->stop ();
    }

  public:
    Producer (PCDBuffer<PointT> &buf, openni_wrapper::OpenNIDevice::DepthMode depth_mode)
      : buf_ (buf),
        depth_mode_ (depth_mode)
    {
      thread_.reset (new std::thread (&Producer::grabAndSend, this));
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      thread_->join ();
      std::lock_guard<std::mutex> io_lock (io_mutex);
      print_highlight ("Producer done.\n");
    }

  private:
    PCDBuffer<PointT> &buf_;
    openni_wrapper::OpenNIDevice::DepthMode depth_mode_;
    std::shared_ptr<std::thread> thread_;
};

//////////////////////////////////////////////////////////////////////////////////////////
// Consumer thread class
template <typename PointT>
class Consumer
{
  private:
    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    writeToDisk (const typename PointCloud<PointT>::ConstPtr& cloud)
    {
      std::stringstream ss;
      std::string time = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());
      ss << "frame-" << time << ".pcd";
      writer_.writeBinaryCompressed (ss.str (), *cloud);
      FPS_CALC ("cloud write.", buf_);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Consumer thread function
    void 
    receiveAndProcess ()
    {
      while (true)
      {
        if (is_done)
          break;
        writeToDisk (buf_.getFront ());
      }

      {
        std::lock_guard<std::mutex> io_lock (io_mutex);
        print_info ("Writing remaining %ld clouds in the buffer to disk...\n", buf_.getSize ());
      }
      while (!buf_.isEmpty ())
        writeToDisk (buf_.getFront ());
    }

  public:
    Consumer (PCDBuffer<PointT> &buf)
      : buf_ (buf)
    {
      thread_.reset (new std::thread (&Consumer::receiveAndProcess, this));
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      thread_->join ();
      std::lock_guard<std::mutex> io_lock (io_mutex);
      print_highlight ("Consumer done.\n");
    }

  private:
    PCDBuffer<PointT> &buf_;
    std::shared_ptr<std::thread> thread_;
    PCDWriter writer_;
};

//////////////////////////////////////////////////////////////////////////////////////////
void 
ctrlC (int)
{
  std::lock_guard<std::mutex> io_lock (io_mutex);
  print_info ("\nCtrl-C detected, exit condition set to true.\n");
  is_done = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
void
printHelp (int default_buff_size, int, char **argv)
{
  using pcl::console::print_error;
  using pcl::console::print_info;

  print_error ("Syntax is: %s ((<device_id> | <path-to-oni-file>) [-xyz] [-shift] [-buf X]  | -l [<device_id>] | -h | --help)]\n", argv [0]);
  print_info ("%s -h | --help : shows this help\n", argv [0]);
  print_info ("%s -xyz : save only XYZ data, even if the device is RGB capable\n", argv [0]);
  print_info ("%s -shift : use OpenNI shift values rather than 12-bit depth\n", argv [0]);
  print_info ("%s -buf X ; use a buffer size of X frames (default: ", argv [0]);
  print_value ("%d", default_buff_size); print_info (")\n");
  print_info ("%s -l : list all available devices\n", argv [0]);
  print_info ("%s -l <device-id> :list all available modes for specified device\n", argv [0]);
  print_info ("\t\t<device_id> may be \"#1\", \"#2\", ... for the first, second etc device in the list\n");
#ifndef _WIN32
  print_info ("\t\t                   bus@address for the device connected to a specific usb-bus / address combination\n");
  print_info ("\t\t                   <serial-number>\n");
#endif
  print_info ("\n\nexamples:\n");
  print_info ("%s \"#1\"\n", argv [0]);
  print_info ("\t\t uses the first device.\n");
  print_info ("%s  \"./temp/test.oni\"\n", argv [0]);
  print_info ("\t\t uses the oni-player device to play back oni file given by path.\n");
  print_info ("%s -l\n", argv [0]);
  print_info ("\t\t list all available devices.\n");
  print_info ("%s -l \"#2\"\n", argv [0]);
  print_info ("\t\t list all available modes for the second device.\n");
  #ifndef _WIN32
  print_info ("%s A00361800903049A\n", argv [0]);
  print_info ("\t\t uses the device with the serial number \'A00361800903049A\'.\n");
  print_info ("%s 1@16\n", argv [0]);
  print_info ("\t\t uses the device on address 16 at USB bus 1.\n");
  #endif
}

//////////////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  print_highlight ("PCL OpenNI Recorder for saving buffered PCD (binary compressed to disk). See %s -h for options.\n", argv[0]);

  std::string device_id;
  int buff_size = BUFFER_SIZE;

  if (argc >= 2)
  {
    device_id = argv[1];
    if (device_id == "--help" || device_id == "-h")
    {
      printHelp (buff_size, argc, argv);
      return 0;
    }
    if (device_id == "-l")
    {
      if (argc >= 3)
      {
        pcl::OpenNIGrabber grabber (argv[2]);
        openni_wrapper::OpenNIDevice::Ptr device = grabber.getDevice ();
        std::cout << "Supported depth modes for device: " << device->getVendorName () << " , " << device->getProductName () << std::endl;
        std::vector<std::pair<int, XnMapOutputMode > > modes = grabber.getAvailableDepthModes ();
        for (const auto& mode : modes)
        {
          std::cout << mode.first << " = " << mode.second.nXRes << " x " << mode.second.nYRes << " @ " << mode.second.nFPS << std::endl;
        }

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
        if (driver.getNumberDevices() > 0)
        {
          for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
          {
            std::cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus(deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << std::endl;
          }

        }
        else
          std::cout << "No devices connected." << std::endl;

        std::cout <<"Virtual Devices available: ONI player" << std::endl;
      }
      return 0;
    }
  }
  else
  {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
    if (driver.getNumberDevices () > 0)
      std::cout << "Device Id not set, using first device." << std::endl;
  }

  bool just_xyz = find_switch (argc, argv, "-xyz");
  openni_wrapper::OpenNIDevice::DepthMode depth_mode = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth;
  if (find_switch (argc, argv, "-shift"))
    depth_mode = openni_wrapper::OpenNIDevice::OpenNI_shift_values;

  if (parse_argument (argc, argv, "-buf", buff_size) != -1)
    print_highlight ("Setting buffer size to %d frames.\n", buff_size);
  else
    print_highlight ("Using default buffer size of %d frames.\n", buff_size);

  print_highlight ("Starting the producer and consumer threads... Press Ctrl+C to end\n");
 
  OpenNIGrabber grabber (device_id);
  if (grabber.providesCallback<OpenNIGrabber::sig_cb_openni_point_cloud_rgba> () && 
      !just_xyz)
  {
    print_highlight ("PointXYZRGBA enabled.\n");
    PCDBuffer<PointXYZRGBA> buf;
    buf.setCapacity (buff_size);
    Producer<PointXYZRGBA> producer (buf, depth_mode);
    std::this_thread::sleep_for(2s);
    Consumer<PointXYZRGBA> consumer (buf);

    signal (SIGINT, ctrlC);
    producer.stop ();
    consumer.stop ();
  }
  else
  {
    print_highlight ("PointXYZ enabled.\n");
    PCDBuffer<PointXYZ> buf;
    buf.setCapacity (buff_size);
    Producer<PointXYZ> producer (buf, depth_mode);
    std::this_thread::sleep_for(2s);
    Consumer<PointXYZ> consumer (buf);

    signal (SIGINT, ctrlC);
    producer.stop ();
    consumer.stop ();
  }
  return (0);
}


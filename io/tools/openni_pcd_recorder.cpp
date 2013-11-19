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
#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>
#include <csignal>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h> //fps calculations

using namespace std;
using namespace pcl;
using namespace pcl::console;

bool is_done = false;
boost::mutex io_mutex;

#if defined(__linux__) || defined (TARGET_OS_MAC)
#include <unistd.h>
// Get the available memory size on Linux/BSD systems

size_t 
getTotalSystemMemory ()
{
  size_t memory = std::numeric_limits<size_t>::max ();

#ifdef _SC_AVPHYS_PAGES
  uint64_t pages = sysconf (_SC_AVPHYS_PAGES);
  uint64_t page_size = sysconf (_SC_PAGE_SIZE);
  
  memory = pages * page_size;
  
#elif defined(HAVE_SYSCTL) && defined(HW_PHYSMEM)
  // This works on *bsd and darwin.
  unsigned int physmem;
  size_t len = sizeof physmem;
  static int mib[2] = { CTL_HW, HW_PHYSMEM };

  if (sysctl (mib, ARRAY_SIZE (mib), &physmem, &len, NULL, 0) == 0 && len == sizeof (physmem))
  {
    memory = physmem;
  }
#endif

  if (memory > std::numeric_limits<size_t>::max ())
  {
    memory = std::numeric_limits<size_t>::max ();
  }
  
  print_info ("Total available memory size: %lluMB.\n", memory / 1048576);
  return memory;
}

const int BUFFER_SIZE = int (getTotalSystemMemory () / (640 * 480));
#else

const int BUFFER_SIZE = 200;
#endif

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class PCDBuffer
{
  public:
    PCDBuffer () {}

    bool 
    pushBack (typename PointCloud<PointT>::ConstPtr); // thread-save wrapper for push_back() method of ciruclar_buffer

    typename PointCloud<PointT>::ConstPtr 
    getFront (); // thread-save wrapper for front() method of ciruclar_buffer

    inline bool 
    isFull ()
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      return (buffer_.full ());
    }

    inline bool
    isEmpty ()
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      return (buffer_.empty ());
    }

    inline int 
    getSize ()
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
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
      boost::mutex::scoped_lock buff_lock (bmutex_);
      buffer_.set_capacity (buff_size);
    }

  private:
    PCDBuffer (const PCDBuffer&); // Disabled copy constructor
    PCDBuffer& operator =(const PCDBuffer&); // Disabled assignment operator

    boost::mutex bmutex_;
    boost::condition_variable buff_empty_;
    boost::circular_buffer<typename PointCloud<PointT>::ConstPtr> buffer_;
};

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool 
PCDBuffer<PointT>::pushBack (typename PointCloud<PointT>::ConstPtr cloud)
{
  bool retVal = false;
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
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
    boost::mutex::scoped_lock buff_lock (bmutex_);
    while (buffer_.empty ())
    {
      if (is_done)
        break;
      {
        boost::mutex::scoped_lock io_lock (io_mutex);
        //cerr << "No data in buffer_ yet or buffer is empty." << endl;
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
      cerr << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz. Queue size: " << buff.getSize () << "\n"; \
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
          boost::mutex::scoped_lock io_lock (io_mutex);
          print_warn ("Warning! Buffer was full, overwriting data!\n");
        }
      }
      FPS_CALC ("cloud callback.", buf_);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    grabAndSend ()
    {
      OpenNIGrabber* grabber = new OpenNIGrabber ();
      grabber->getDevice ()->setDepthOutputFormat (depth_mode_);

      Grabber* interface = grabber;
      boost::function<void (const typename PointCloud<PointT>::ConstPtr&)> f = boost::bind (&Producer::grabberCallBack, this, _1);
      interface->registerCallback (f);
      interface->start ();

      while (true)
      {
        if (is_done)
          break;
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }
      interface->stop ();
    }

  public:
    Producer (PCDBuffer<PointT> &buf, openni_wrapper::OpenNIDevice::DepthMode depth_mode)
      : buf_ (buf),
        depth_mode_ (depth_mode)
    {
      thread_.reset (new boost::thread (boost::bind (&Producer::grabAndSend, this)));
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      thread_->join ();
      boost::mutex::scoped_lock io_lock (io_mutex);
      print_highlight ("Producer done.\n");
    }

  private:
    PCDBuffer<PointT> &buf_;
    openni_wrapper::OpenNIDevice::DepthMode depth_mode_;
    boost::shared_ptr<boost::thread> thread_;
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
      stringstream ss;
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
        boost::mutex::scoped_lock io_lock (io_mutex);
        print_info ("Writing remaing %ld clouds in the buffer to disk...\n", buf_.getSize ());
      }
      while (!buf_.isEmpty ())
        writeToDisk (buf_.getFront ());
    }

  public:
    Consumer (PCDBuffer<PointT> &buf)
      : buf_ (buf)
    {
      thread_.reset (new boost::thread (boost::bind (&Consumer::receiveAndProcess, this)));
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      thread_->join ();
      boost::mutex::scoped_lock io_lock (io_mutex);
      print_highlight ("Consumer done.\n");
    }

  private:
    PCDBuffer<PointT> &buf_;
    boost::shared_ptr<boost::thread> thread_;
    PCDWriter writer_;
};

//////////////////////////////////////////////////////////////////////////////////////////
void 
ctrlC (int)
{
  boost::mutex::scoped_lock io_lock (io_mutex);
  print_info ("\nCtrl-C detected, exit condition set to true.\n");
  is_done = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
int 
main (int argc, char** argv)
{
  print_highlight ("PCL OpenNI Recorder for saving buffered PCD (binary compressed to disk). See %s -h for options.\n", argv[0]);

  int buff_size = BUFFER_SIZE;
  
  if (find_switch (argc, argv, "-h") || find_switch (argc, argv, "--help"))
  {
    print_info ("Options are: \n"
              "             -xyz    = save only XYZ data, even if the device is RGB capable\n"
              "             -shift  = use OpenNI shift values rather than 12-bit depth\n"
              "             -buf X  = use a buffer size of X frames (default: "); 
    print_value ("%d", buff_size); print_info (")\n");
    return (0);
  }


  bool just_xyz = find_switch (argc, argv, "-xyz");
  openni_wrapper::OpenNIDevice::DepthMode depth_mode = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth;
  if (find_switch (argc, argv, "-shift"))
    depth_mode = openni_wrapper::OpenNIDevice::OpenNI_shift_values;

  if (parse_argument (argc, argv, "-buf", buff_size) != -1)
    print_highlight ("Setting buffer size to %d frames.\n", buff_size);
  else
    print_highlight ("Using default buffer size of %d frames.\n", buff_size);

  print_highlight ("Starting the producer and consumer threads... Press Cltr+C to end\n");
 
  OpenNIGrabber grabber ("");
  if (grabber.providesCallback<OpenNIGrabber::sig_cb_openni_point_cloud_rgba> () && 
      !just_xyz)
  {
    print_highlight ("PointXYZRGBA enabled.\n");
    PCDBuffer<PointXYZRGBA> buf;
    buf.setCapacity (buff_size);
    Producer<PointXYZRGBA> producer (buf, depth_mode);
    boost::this_thread::sleep (boost::posix_time::seconds (2));
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
    boost::this_thread::sleep (boost::posix_time::seconds (2));
    Consumer<PointXYZ> consumer (buf);

    signal (SIGINT, ctrlC);
    producer.stop ();
    consumer.stop ();
  }
  return (0);
}


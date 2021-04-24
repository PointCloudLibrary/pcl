/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012 Sudarshan Srinivasan <sudarshan85@gmail.com>
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
 *  Author: Raphael Favier, Technical University Eindhoven, (r.mysurname < aT > tue.nl)
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/memory.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/gpu/containers/kernel_containers.h>
#include <pcl/io/png_io.h>
#include <pcl/console/print.h>

#include <boost/circular_buffer.hpp>

#include <condition_variable>
#include <csignal>
#include <ctime>
#include <functional>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cerr << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

bool is_done = false;
std::mutex io_mutex;

const int BUFFER_SIZE = 1000;
static int counter = 1;
//////////////////////////////////////////////////////////////////////////////////////////
class MapsBuffer
{
  public:
    
    struct PixelRGB
    {
      unsigned char r, g, b;
    };
    
    struct MapsRgb
    {
      using Ptr = pcl::shared_ptr<MapsRgb>;
      using ConstPtr = pcl::shared_ptr<const MapsRgb>;

      pcl::gpu::PtrStepSz<const PixelRGB> rgb_;
      pcl::gpu::PtrStepSz<const unsigned short> depth_;      
      double time_stamp_;
    };
    
    MapsBuffer () {}    
    
    bool 
    pushBack (MapsRgb::ConstPtr); // thread-save wrapper for push_back() method of ciruclar_buffer

    MapsRgb::ConstPtr
    getFront (bool); // thread-save wrapper for front() method of ciruclar_buffer
                
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
    MapsBuffer (const MapsBuffer&) = delete; // Disabled copy constructor
    MapsBuffer& operator =(const MapsBuffer&) = delete; // Disabled assignment operator

    std::mutex bmutex_;
    std::condition_variable buff_empty_;
    boost::circular_buffer<MapsRgb::ConstPtr> buffer_;

};


//////////////////////////////////////////////////////////////////////////////////////////
bool 
MapsBuffer::pushBack(MapsRgb::ConstPtr maps_rgb )
{
  bool retVal = false;
  {
    std::lock_guard<std::mutex> buff_lock (bmutex_);
    if (!buffer_.full ())
      retVal = true;
    buffer_.push_back (maps_rgb);
  }
  buff_empty_.notify_one ();
  return (retVal);
}


//////////////////////////////////////////////////////////////////////////////////////////
MapsBuffer::MapsRgb::ConstPtr
MapsBuffer::getFront(bool print)
{
  MapsBuffer::MapsRgb::ConstPtr depth_rgb;
  {
    std::unique_lock<std::mutex> buff_lock (bmutex_);
    while (buffer_.empty ())
    {
      if (is_done)
        break;
      {
        std::lock_guard<std::mutex> io_lock (io_mutex);
              //std::cout << "No data in buffer_ yet or buffer is empty." << std::endl;
      }
      buff_empty_.wait (buff_lock);
    }
    depth_rgb = buffer_.front ();
    buffer_.pop_front ();
  }
  
  if(print)
    PCL_INFO("%d maps left in the buffer...\n", buffer_.size ());
  
  return (depth_rgb);
}


MapsBuffer buff;
std::vector<unsigned short> source_depth_data_;
std::vector<MapsBuffer::PixelRGB> source_image_data_;


//////////////////////////////////////////////////////////////////////////////////////////
void 
writeToDisk (const MapsBuffer::MapsRgb::ConstPtr& map_rbg)
{
  //save rgb
  std::stringstream ss;
  ss.precision(std::numeric_limits<double>::digits10 + 2);
  ss.width(20);
  ss << map_rbg->time_stamp_;
    
  std::string prefix = "./frame-";
  std::string ext = " -rgb.png";
  std::string fname = prefix + ss.str () + ext;
  pcl::io::saveRgbPNGFile (fname, (unsigned char*)map_rbg->rgb_.data, 640,480);
  
  // save depth map
  ext = " -depth.png";
  fname = prefix + ss.str () + ext;
  pcl::io::saveShortPNGFile (fname, (short unsigned int*)map_rbg->depth_.data, 640,480,1);
  
  counter++;
  FPS_CALC ("maps write");
}

void
grabberMapsCallBack(const openni_wrapper::Image::Ptr& image_wrapper, const openni_wrapper::DepthImage::Ptr& depth_wrapper, float)
{
  MapsBuffer::MapsRgb rgb_depth;
  rgb_depth.time_stamp_ = pcl::getTime();  
  
  // fill in depth values
  rgb_depth.depth_.cols = depth_wrapper->getWidth();
  rgb_depth.depth_.rows = depth_wrapper->getHeight();
  rgb_depth.depth_.step = rgb_depth.depth_.cols * rgb_depth.depth_.elemSize();
  source_depth_data_.resize(rgb_depth.depth_.cols * rgb_depth.depth_.rows);
  depth_wrapper->fillDepthImageRaw(rgb_depth.depth_.cols, rgb_depth.depth_.rows, &source_depth_data_[0]);
  rgb_depth.depth_.data = &source_depth_data_[0];      
  
  // fill in rgb values
  rgb_depth.rgb_.cols = image_wrapper->getWidth();
  rgb_depth.rgb_.rows = image_wrapper->getHeight();
  rgb_depth.rgb_.step = rgb_depth.rgb_.cols * rgb_depth.rgb_.elemSize(); 
  source_image_data_.resize(rgb_depth.rgb_.cols * rgb_depth.rgb_.rows);
  image_wrapper->fillRGB(rgb_depth.rgb_.cols, rgb_depth.rgb_.rows, (unsigned char*)&source_image_data_[0]);
  rgb_depth.rgb_.data = &source_image_data_[0];    

  // push to buffer
  if (!buff.pushBack (pcl::make_shared<MapsBuffer::MapsRgb> (rgb_depth)))
  {
    {
      std::lock_guard<std::mutex> io_lock(io_mutex);
      PCL_WARN ("Warning! Buffer was full, overwriting data\n");
    }
  }
  FPS_CALC ("kinect callback");  
}


//////////////////////////////////////////////////////////////////////////////////////////
// Procuder thread function
void 
grabAndSend ()
{
  pcl::Grabber* interface = new pcl::OpenNIGrabber ();

  std::function<void (const openni_wrapper::Image::Ptr&,
                      const openni_wrapper::DepthImage::Ptr&,
                      float)> f = [] (const openni_wrapper::Image::Ptr& img,
                                      const openni_wrapper::DepthImage::Ptr& depth,
                                      float constant)
  {
    grabberMapsCallBack (img, depth, constant);
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


//////////////////////////////////////////////////////////////////////////////////////////
// Consumer thread function
void 
receiveAndProcess ()
{
  while (true)
  {
    if (is_done)
      break;
    writeToDisk (buff.getFront (false));
  }

  {
    std::lock_guard<std::mutex> io_lock (io_mutex);
    PCL_INFO ("Writing remaining %d maps in the buffer to disk...\n", buff.getSize ());
  }
  while (!buff.isEmpty ())
  {
    writeToDisk (buff.getFront (true));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
ctrlC (int)
{
  std::lock_guard<std::mutex> io_lock (io_mutex);
  std::cout << std::endl;
  PCL_WARN ("Ctrl-C detected, exit condition set to true\n");
  is_done = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
int 
main (int argc, char** argv)
{
  int buff_size = BUFFER_SIZE;
  if (argc == 2)
  {
    buff_size = atoi (argv[1]);
    std::cout << "Setting buffer size to " << buff_size << " frames " << std::endl;
  }
  else
  {
    std::cout << "Using default buffer size of " << buff_size << " frames " << std::endl;
  }
  buff.setCapacity (buff_size);
  std::cout << "Starting the producer and consumer threads..." << std::endl;
  std::cout << "Press Ctrl-C to end" << std::endl;
  std::thread producer (grabAndSend);
  std::this_thread::sleep_for(2s);
  std::thread consumer (receiveAndProcess);
  std::thread consumer2 (receiveAndProcess);
  std::thread consumer3 (receiveAndProcess);
  signal (SIGINT, ctrlC);
  producer.join ();
  {
    std::lock_guard<std::mutex> io_lock (io_mutex);
    PCL_WARN ("Producer done\n");
  }
  consumer.join ();
  consumer2.join();
  consumer3.join();

  PCL_WARN ("Consumers done\n");
  return (0);
}


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
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>
#include <csignal>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h> //fps calculations

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
boost::mutex io_mutex;

const int BUFFER_SIZE = 100;

//////////////////////////////////////////////////////////////////////////////////////////
class PCDBuffer
{
	public:
    PCDBuffer () {}

    bool 
    pushBack (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr); // thread-save wrapper for push_back() method of ciruclar_buffer

		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr 
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
		boost::circular_buffer<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> buffer_;
};

//////////////////////////////////////////////////////////////////////////////////////////
bool 
PCDBuffer::pushBack (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
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
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr 
PCDBuffer::getFront ()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
	{
		boost::mutex::scoped_lock buff_lock (bmutex_);
		while (buffer_.empty ())
		{
			if (is_done)
				break;
			{
				boost::mutex::scoped_lock io_lock (io_mutex);
				//std::cout << "No data in buffer_ yet or buffer is empty." << std::endl;
			}
			buff_empty_.wait (buff_lock);
		}
		cloud = buffer_.front ();
		buffer_.pop_front ();
	}
	return (cloud);
}

PCDBuffer buff;

//////////////////////////////////////////////////////////////////////////////////////////
void 
grabberCallBack (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
	if (!buff.pushBack (cloud))
	{
		{
			boost::mutex::scoped_lock io_lock(io_mutex);
			std::cout << "Warning! Buffer was full, overwriting data" << std::endl;
		}
	}
  FPS_CALC ("cloud callback");
}

//////////////////////////////////////////////////////////////////////////////////////////
// Procuder thread function
void 
grabAndSend ()
{
	pcl::Grabber* interface = new pcl::OpenNIGrabber ();
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& )> f = boost::bind(&grabberCallBack, _1);
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

//////////////////////////////////////////////////////////////////////////////////////////
void 
writeToDisk (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
	static int counter = 1;
	pcl::PCDWriter w;
	std::stringstream ss;
	ss << counter;
	std::string prefix = "./frame-";
	std::string ext = ".pcd";
	std::string fname = prefix + ss.str () + ext;
	w.writeBinaryCompressed (fname, *cloud);
	counter++;
  FPS_CALC ("cloud write");
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
		writeToDisk (buff.getFront ());
	}

	{
		boost::mutex::scoped_lock io_lock (io_mutex);
		std::cout << "Writing remaing " << buff.getSize () << " clouds in the buffer to disk..." << std::endl;
	}
	while (!buff.isEmpty ())
	{
		writeToDisk (buff.getFront ());
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
ctrlC (int)
{
	boost::mutex::scoped_lock io_lock (io_mutex);
	std::cout << std::endl << "Ctrl-C detected, exit condition set to true" << std::endl;
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
	boost::thread producer (grabAndSend);
	boost::this_thread::sleep (boost::posix_time::seconds (2));
	boost::thread consumer (receiveAndProcess);
	signal (SIGINT, ctrlC);
	producer.join ();
	{
		boost::mutex::scoped_lock io_lock (io_mutex);
		std::cout << "Producer done" << std::endl;
	}
	consumer.join ();
	std::cout << "Consumer done" << std::endl;
	return (0);
}


/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


//http://www.paulbridger.com/monitor_object/#ixzz2CeN1rr4P

#ifndef PCL_OUTOFCORE_MONITOR_QUEUE_IMPL_H_
#define PCL_OUTOFCORE_MONITOR_QUEUE_IMPL_H_

#include <queue>

template<typename DataT>
class MonitorQueue : boost::noncopyable
{
public:
  void
  push (const DataT& newData)
  {
    boost::mutex::scoped_lock lock (monitor_mutex_);
    queue_.push (newData);
    item_available_.notify_one ();
  }

  DataT
  pop ()
  {
    boost::mutex::scoped_lock lock (monitor_mutex_);

    if (queue_.empty ())
    {
      item_available_.wait (lock);
    }

    DataT temp (queue_.front ());
    queue_.pop ();

    return temp;
  }

private:
  std::queue<DataT> queue_;
  boost::mutex monitor_mutex_;
  boost::condition item_available_;
};

#endif //PCL_OUTOFCORE_MONITOR_QUEUE_IMPL_H_

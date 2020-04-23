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
 *  Author: Nico Blodow (blodow@in.tum.de), Suat Gedikli (gedikli@willowgarage.com)
 */

#pragma once

#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <utility>

namespace pcl
{
  /** /brief This template class synchronizes two data streams of different types.
   *         The data can be added using add0 and add1 methods which expects also a timestamp of type unsigned long.
   *         If two matching data objects are found, registered callback functions are invoked with the objects and the time stamps.
   *         The only assumption of the timestamp is, that they are in the same unit, linear and strictly monotonic increasing.
   *         If filtering is desired, e.g. thresholding of time differences, the user can do that in the callback method.
   *         This class is thread safe.
   * /ingroup common
   */
  template <typename T1, typename T2>
  class Synchronizer
  {
    using T1Stamped = std::pair<unsigned long, T1>;
    using T2Stamped = std::pair<unsigned long, T2>;
    std::mutex mutex1_;
    std::mutex mutex2_;
    std::mutex publish_mutex_;
    std::deque<T1Stamped> queueT1;
    std::deque<T2Stamped> queueT2;

    using CallbackFunction = std::function<void(T1, T2, unsigned long, unsigned long)>;

    std::map<int, CallbackFunction> cb_;
    int callback_counter = 0;
  public:

    int
    addCallback (const CallbackFunction& callback)
    {
      std::unique_lock<std::mutex> publish_lock (publish_mutex_);
      cb_[callback_counter] = callback;
      return callback_counter++;
    }

    void
    removeCallback (int i)
    {
      std::unique_lock<std::mutex> publish_lock (publish_mutex_);
      cb_.erase (i);
    }

    void
    add0 (const T1& t, unsigned long time)
    {
      mutex1_.lock ();
      queueT1.push_back (T1Stamped (time, t));
      mutex1_.unlock ();
      publish ();
    }

    void
    add1 (const T2& t, unsigned long time)
    {
      mutex2_.lock ();
      queueT2.push_back (T2Stamped (time, t));
      mutex2_.unlock ();
      publish ();
    }

  private:

    void
    publishData ()
    {
      std::unique_lock<std::mutex> lock1 (mutex1_);
      std::unique_lock<std::mutex> lock2 (mutex2_);

      for (const auto& cb: cb_)
      {
        if (cb.second)
        {
          cb.second.operator()(queueT1.front ().second, queueT2.front ().second, queueT1.front ().first, queueT2.front ().first);
        }
      }

      queueT1.pop_front ();
      queueT2.pop_front ();
    }

    void
    publish ()
    {
      // only one publish call at once allowed
      std::unique_lock<std::mutex> publish_lock (publish_mutex_);

      std::unique_lock<std::mutex> lock1 (mutex1_);
      if (queueT1.empty ())
        return;
      T1Stamped t1 = queueT1.front ();
      lock1.unlock ();

      std::unique_lock<std::mutex> lock2 (mutex2_);
      if (queueT2.empty ())
        return;
      T2Stamped t2 = queueT2.front ();
      lock2.unlock ();

      bool do_publish = false;

      if (t1.first <= t2.first)
      { // iterate over queue1
        lock1.lock ();
        while (queueT1.size () > 1 && queueT1[1].first <= t2.first)
          queueT1.pop_front ();

        if (queueT1.size () > 1)
        { // we have at least 2 measurements; first in past and second in future -> find out closer one!
          if ( (t2.first << 1) > (queueT1[0].first + queueT1[1].first) )
            queueT1.pop_front ();

          do_publish = true;
        }
        lock1.unlock ();
      }
      else
      { // iterate over queue2
        lock2.lock ();
        while (queueT2.size () > 1 && (queueT2[1].first <= t1.first) )
          queueT2.pop_front ();

        if (queueT2.size () > 1)
        { // we have at least 2 measurements; first in past and second in future -> find out closer one!
          if ( (t1.first << 1) > queueT2[0].first + queueT2[1].first )
            queueT2.pop_front ();

          do_publish = true;
        }
        lock2.unlock ();
      }

      if (do_publish)
        publishData ();
    }
  } ;
} // namespace

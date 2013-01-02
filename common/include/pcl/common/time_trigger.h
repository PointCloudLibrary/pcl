/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_COMMON_TIME_TRIGGER_H_
#define PCL_COMMON_TIME_TRIGGER_H_

#include <pcl/pcl_macros.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/signals2.hpp>

namespace pcl
{
  /** \brief Timer class that invokes registered callback methods periodically.
    * \ingroup common
    */
  class PCL_EXPORTS TimeTrigger
  {
    public:
      typedef boost::function<void() > callback_type;

      /** \brief Timer class that calls a callback method periodically. Due to possible blocking calls, only one callback method can be registered per instance.
        * \param[in] interval_seconds interval in seconds
        * \param[in] callback callback to be invoked periodically
        */
      TimeTrigger (double interval_seconds, const callback_type& callback);

      /** \brief Timer class that calls a callback method periodically. Due to possible blocking calls, only one callback method can be registered per instance.
        * \param[in] interval_seconds interval in seconds
        */
      TimeTrigger (double interval_seconds = 1.0);

      /** \brief Destructor. */
      ~TimeTrigger ();

      /** \brief registeres a callback
        * \param[in] callback callback function to the list of callbacks. signature has to be boost::function<void()>
        * \return connection the connection, which can be used to disable/enable and remove callback from list
        */
      boost::signals2::connection registerCallback (const callback_type& callback);

      /** \brief Resets the timer interval
        * \param[in] interval_seconds interval in seconds
        */
      void 
      setInterval (double interval_seconds);

      /** \brief Start the Trigger. */
      void 
      start ();

      /** \brief Stop the Trigger. */
      void 
      stop ();
    private:
      void 
      thread_function ();
      boost::signals2::signal <void() > callbacks_;

      double interval_;

      bool quit_;
      bool running_;

      boost::thread timer_thread_;
      boost::condition_variable condition_;
      boost::mutex condition_mutex_;
  };
}

#endif

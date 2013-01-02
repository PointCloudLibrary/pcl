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
#ifndef TERMINAL_TOOLS_TIME_H_
#define TERMINAL_TOOLS_TIME_H_

#ifdef __GNUC__
#pragma GCC system_header 
#endif

#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/console/print.h>

namespace pcl
{
  namespace console
  {
    class TicToc
    {
      public:

        TicToc () : tictic (), toctoc () {}

        void 
        tic ()
        {
          tictic = boost::posix_time::microsec_clock::local_time ();
        };

        inline double 
        toc ()
        {
          toctoc = boost::posix_time::microsec_clock::local_time ();
          return (static_cast<double> ((toctoc - tictic).total_milliseconds ()));
        };
        
        inline void 
        toc_print ()
        {
          double milliseconds = toc ();
          //int minutes = (int) floor ( seconds / 60.0 );
          //seconds -= minutes * 60.0;
          //if (minutes != 0)
          //{
          //  print_value ("%i", minutes);
          //  print_info (" minutes, ");
          //}
          print_value ("%g", milliseconds);
          print_info (" ms\n");
        };
      
      private:
        boost::posix_time::ptime tictic;
        boost::posix_time::ptime toctoc;
    };
  } 
}

#endif

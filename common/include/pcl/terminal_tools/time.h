/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: time.h 30706 2010-07-09 18:20:31Z rusu $
 *
 */
#ifndef TERMINAL_TOOLS_TIME_H_
#define TERMINAL_TOOLS_TIME_H_

#include <sys/time.h>
#include "terminal_tools/print.h"

namespace terminal_tools
{
  class TicToc
  {
    public:
      void tic ()
      {
        gettimeofday (&tictic,NULL);
      };

      inline double toc ()
      {
        gettimeofday (&toctoc,NULL);
        return (toctoc.tv_sec + ((double)toctoc.tv_usec / 1000000.0))
             - (tictic.tv_sec + ((double)tictic.tv_usec / 1000000.0));
      };
      
      inline void toc_print ()
      {
        double seconds = toc ();
        int minutes = (int) floor ( seconds / 60.0 );
        seconds -= minutes * 60.0;
        if (minutes != 0)
        {
          print_value ("%i", minutes);
          print_info (" minutes, ");
        }
        print_value ("%g", seconds);
        print_info (" seconds\n");
      };
    
    private:
      timeval tictic;
      timeval toctoc;
  };
} 

#endif

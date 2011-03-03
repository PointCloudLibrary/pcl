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
 */

#ifndef PCL_ROS_MACROS_H_
#define PCL_ROS_MACROS_H_

#include <stdarg.h>
#include <stdio.h>

#define ROS_DEBUG(...) ros::console::print (stdout, 2, 2, __VA_ARGS__)
#define ROS_INFO(...) ros::console::print (stdout, 2, 7, __VA_ARGS__)
#define ROS_WARN(...) ros::console::print (stderr, 2, 3, __VA_ARGS__)
#define ROS_ERROR(...) ros::console::print (stderr, 1, 1, __VA_ARGS__)

#define ROS_VERSION_MAJOR 1
#define ROS_VERSION_MINOR 2
#define ROS_VERSION_PATCH 2
#define ROS_VERSION_COMBINED(major, minor, patch) (((major) << 20) | ((minor) << 10) | (patch))
#define ROS_VERSION ROS_VERSION_COMBINED(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH)

#define ROS_VERSION_GE(major1, minor1, patch1, major2, minor2, patch2) (ROS_VERSION_COMBINED(major1, minor1, patch1) >= ROS_VERSION_COMBINED(major2, minor2, patch2))
#define ROS_VERSION_MINIMUM(major, minor, patch) ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, major, minor, patch)

namespace ros
{
  namespace console
  {
    inline void 
      print (FILE *stream, int attribute, int fg, const char* format, ...)
    {
      char command[13];
      // Command is the control command to the terminal        
      sprintf (command, "%c[%d;%dm", 0x1B, attribute, fg + 30);
      fprintf (stream, "%s", command); 

      va_list ap; 

      va_start (ap, format);
      vfprintf (stream, format, ap);
      va_end (ap);

      // Command is the control command to the terminal
      sprintf (command, "%c[0;m", 0x1B);
      fprintf (stream, "%s", command);
    }

  }
}


#endif  //#ifndef PCL_ROS_MACROS_H_

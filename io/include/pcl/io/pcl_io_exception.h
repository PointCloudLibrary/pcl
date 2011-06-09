/*********************************************************************
*
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
* Date: 31. March 2011
* Author: Suat Gedikli (gedikli@willowgarage.com)
*********************************************************************/

#ifndef __PCL_IO_EXCEPTION__
#define __PCL_IO_EXCEPTION__
#include <pcl/exceptions.h>
#include <pcl/win32_macros.h>
#include <cstdarg>
#include <stdio.h>

#define THROW_PCL_IO_EXCEPTION(format,...) throwPCLIOException( __PRETTY_FUNCTION__, __FILE__, __LINE__, format , ##__VA_ARGS__ )
namespace pcl
{
/** /brief
  * /ingroup io
  */
class PCLIOException : public PCLException
{
  public:
    PCLIOException(const std::string& error_description,
                   const std::string& file_name = "",
                   const std::string& function_name = "",
                   unsigned line_number = 0)
    : PCLException (error_description, file_name, function_name, line_number)
    {
    }
};

/** /brief
  * /ingroup io
  */
inline void throwPCLIOException (const char* function, const char* file, unsigned line, const char* format, ...)
{
  char msg[1024];
  va_list args;
  va_start (args, format);
  vsprintf (msg, format, args);

  throw PCLIOException (msg, file, function, line);
}
}
#endif

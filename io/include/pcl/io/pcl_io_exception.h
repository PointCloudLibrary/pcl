/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_IO_EXCEPTION
#define PCL_IO_EXCEPTION

#include <pcl/exceptions.h>
#include <pcl/pcl_macros.h>
#include <cstdarg>
#include <stdio.h>

#define THROW_PCL_IO_EXCEPTION(format,...) throwPCLIOException( __PRETTY_FUNCTION__, __FILE__, __LINE__, format , ##__VA_ARGS__ )

namespace pcl
{
  /** \brief Base exception class for I/O operations
    * \ingroup io
    */
  class PCLIOException : public PCLException
  {
    public:
      /** \brief Constructor
        * \param[in] error_description a string describing the error
        * \param[in] file_name the name of the file where the exception was caused
        * \param[in] function_name the name of the method where the exception was caused
        * \param[in] line_number the number of the line where the exception was caused
        */
      PCLIOException (const std::string& error_description,
                      const std::string& file_name = "",
                      const std::string& function_name = "",
                      unsigned line_number = 0)
      : PCLException (error_description, file_name, function_name, line_number)
      {
      }
  };

  /** \brief
    * \param[in] function_name the name of the method where the exception was caused
    * \param[in] file_name the name of the file where the exception was caused
    * \param[in] line_number the number of the line where the exception was caused
    * \param[in] format printf format
    * \ingroup io
    */
  inline void 
  throwPCLIOException (const char* function_name, const char* file_name, unsigned line_number, 
                       const char* format, ...)
  {
    char msg[1024];
    va_list args;
    va_start (args, format);
    vsprintf (msg, format, args);

    throw PCLIOException (msg, file_name, function_name, line_number);
  }
}

#endif


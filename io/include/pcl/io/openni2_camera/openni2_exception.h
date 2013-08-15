/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
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
 */
#ifndef __OPENNI2_EXCEPTION__
#define __OPENNI2_EXCEPTION__

#include <cstdarg>
#include <cstdio>
#include <exception>
#include <string>

#if defined _WIN32 && defined _MSC_VER
# define __PRETTY_FUNCTION__ __FUNCTION__
#endif
#define THROW_OPENNI_EXCEPTION(format,...) throwOpenNIException( __PRETTY_FUNCTION__, __FILE__, __LINE__, format , ##__VA_ARGS__ )

namespace openni2_wrapper
{
/**
 * @brief General exception class
 * @author Suat Gedikli
 * @date 02.january 2011
 */
class OpenNI2Exception : public std::exception
{
public:
  OpenNI2Exception(const std::string& function_name,
                   const std::string& file_name,
                   unsigned line_number,
                   const std::string& message) throw ();

  virtual ~OpenNI2Exception() throw ();
  OpenNI2Exception & operator=(const OpenNI2Exception& exception) throw ();
  virtual const char* what() const throw ();

  const std::string& getFunctionName() const throw ();
  const std::string& getFileName() const throw ();
  unsigned getLineNumber() const throw ();

protected:
  std::string function_name_;
  std::string file_name_;
  unsigned line_number_;
  std::string message_;
  std::string message_long_;
};

inline void throwOpenNIException(const char* function, const char* file, unsigned line, const char* format, ...)
{
  static char msg[1024];
  va_list args;
  va_start(args, format);
  vsprintf(msg, format, args);
  throw OpenNI2Exception(function, file, line, msg);
}
} // namespace openni_camera
#endif

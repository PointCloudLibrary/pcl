/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
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

#pragma once

#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI

#include <cstdarg>
#include <cstdio>
#include <exception>
#include <string>
//#include <pcl/pcl_macros.h> <-- because current header is included in NVCC-compiled code and contains <Eigen/Core>. Consider <pcl/pcl_exports.h>


//fom <pcl/pcl_macros.h>
#if defined _WIN32 && defined _MSC_VER && !defined __PRETTY_FUNCTION__
  #define __PRETTY_FUNCTION__ __FUNCTION__  
#endif


#define THROW_OPENNI_EXCEPTION(format,...) throwOpenNIException( __PRETTY_FUNCTION__, __FILE__, __LINE__, format , ##__VA_ARGS__ )


namespace openni_wrapper
{

  /**
   * @brief General exception class
   * @author Suat Gedikli
   * @date 02.january 2011
   * @ingroup io
   */
  class OpenNIException : public std::exception
  {
  public:
    /**
     * @brief Constructor
     * @note use the MACRO THROW_OPENNI_EXCEPTION, that takes care about the parameters function_name, file_name and line_number
     * @param[in] function_name the function in which this exception was created.
     * @param[in] file_name the file name in which this exception was created.
     * @param[in] line_number the line number where this exception was created.
     * @param[in] message the message of the exception
     */
    OpenNIException (const std::string& function_name, const std::string& file_name, unsigned line_number, const std::string& message) noexcept;

    /**
     * @brief virtual Destructor that never throws an exception
     */
    ~OpenNIException () noexcept override;

    /**
     * @brief Assignment operator to allow copying the message of another exception variable.
     * @param[in] exception left hand side
     * @return
     */
    OpenNIException & operator= (const OpenNIException& exception) noexcept;

    /**
     * @brief virtual method, derived from std::exception
     * @return the message of the exception.
     */
    const char* what () const throw () override;

    /**
     * @return the function name in which the exception was created.
     */
    const std::string& getFunctionName () const throw ();

    /**
     * @return the filename in which the exception was created.
     */
    const std::string& getFileName () const throw ();

    /**
     * @return the line number where the exception was created.
     */
    unsigned getLineNumber () const throw ();
  protected:
    std::string function_name_;
    std::string file_name_;
    unsigned line_number_;
    std::string message_;
    std::string message_long_;
  } ;

  /**
   * @brief inline function used by the macro THROW_OPENNI_EXCEPTION to create an instance of OpenNIException with correct values for function, file and line_number
   * @param[in] function_name the function name. Will be filled in by the macro THROW_OPENNI_EXCEPTION
   * @param[in] file_name the file name. Will be filled in by the macro THROW_OPENNI_EXCEPTION
   * @param[in] line_number the line number. Will be filled in by the macro THROW_OPENNI_EXCEPTION
   * @param[in] format the printf-style format string
   * @param[in] ... optional arguments for the printf style format.
   */
  inline void
  throwOpenNIException (const char* function_name, const char* file_name, unsigned line_number, const char* format, ...)
  {
    static char msg[1024];
    va_list args;
    va_start (args, format);
    vsprintf (msg, format, args);
    va_end (args);
    throw OpenNIException (function_name, file_name, line_number, msg);
  }
} // namespace openni_camera
#endif

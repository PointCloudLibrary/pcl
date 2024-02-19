/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2019-, Open Perception, Inc.
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

#include <stdexcept>
#include <sstream>
#include <boost/current_function.hpp>

/** POISSON_THROW_EXCEPTION is a helper macro to be used for throwing exceptions. e.g.
  * POISSON_THROW_EXCEPTION (PoissonBadArgumentException, "[ERROR] B-spline up-sampling not supported for degree " << Degree);
  * 
  * \note 
  * Adapted from PCL_THROW_EXCEPTION. We intentionally do not reuse PCL_THROW_EXCEPTION here
  * to avoid introducing any dependencies on PCL in this 3rd party module.       
  */
// NOLINTBEGIN(bugprone-macro-parentheses)
#define POISSON_THROW_EXCEPTION(ExceptionName, message)                     \
{                                                                           \
  std::ostringstream s;                                                     \
  s << message;                                                             \
  throw ExceptionName(s.str(), __FILE__, BOOST_CURRENT_FUNCTION, __LINE__); \
}
// NOLINTEND(bugprone-macro-parentheses)

namespace pcl
{
  namespace poisson
  {
    /** \class PoissonException
      * \brief A base class for all poisson exceptions which inherits from std::runtime_error
      * 
      * \note 
      * Adapted from PCLException. We intentionally do not reuse PCLException here
      * to avoid introducing any dependencies on PCL in this 3rd party module.       
      */
    class PoissonException : public std::runtime_error
    {
      public:
        PoissonException (const std::string& error_description,
                      const char* file_name = nullptr,
                      const char* function_name = nullptr,
                      unsigned line_number = 0)
          : std::runtime_error (createDetailedMessage (error_description,
                                                       file_name,
                                                       function_name,
                                                       line_number))
          , file_name_ (file_name)
          , function_name_ (function_name)
          , line_number_ (line_number)
        {}

      protected:
        static std::string
        createDetailedMessage (const std::string& error_description,
                               const char* file_name,
                               const char* function_name,
                               unsigned line_number)
        {
          std::ostringstream sstream;
          if (function_name)
            sstream << function_name << ' ';
          
          if (file_name)
          {
            sstream << "in " << file_name << ' ';
            if (line_number)
              sstream << "@ " << line_number << ' ';
          }
          sstream << ": " << error_description;
          
          return (sstream.str ());
        }
      
        const char* file_name_;
        const char* function_name_;
        unsigned line_number_;
    };

    /** \class PoissonBadArgumentException
      * \brief An exception that is thrown when the arguments number or type is wrong/unhandled.
      */
    class PoissonBadArgumentException : public PoissonException
    {
    public:
      PoissonBadArgumentException (const std::string& error_description,
        const char* file_name = nullptr,
        const char* function_name = nullptr,
        unsigned line_number = 0)
        : pcl::poisson::PoissonException (error_description, file_name, function_name, line_number) {}
    };

    /** \class PoissonOpenMPException
      * \brief An exception that is thrown when something goes wrong inside an openMP for loop.
      */
    class PoissonOpenMPException : public PoissonException
    {
    public:
      PoissonOpenMPException (const std::string& error_description,
        const char* file_name = nullptr,
        const char* function_name = nullptr,
        unsigned line_number = 0)
        : pcl::poisson::PoissonException (error_description, file_name, function_name, line_number) {}
    };

    /** \class PoissonBadInitException
      * \brief An exception that is thrown when initialization fails.
      */
    class PoissonBadInitException : public PoissonException
    {
    public:
      PoissonBadInitException (const std::string& error_description,
        const char* file_name = nullptr,
        const char* function_name = nullptr,
        unsigned line_number = 0)
        : pcl::poisson::PoissonException (error_description, file_name, function_name, line_number) {}
    };
  }
}

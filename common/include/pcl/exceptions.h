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

/** PCL_THROW_EXCEPTION a helper macro to be used for throwing exceptions.
  * This is an example on how to use:
  * PCL_THROW_EXCEPTION(IOException,
  *                     "encountered an error while opening " << filename << " PCD file");
  */
#define PCL_THROW_EXCEPTION(ExceptionName, message)                         \
{                                                                           \
  std::ostringstream s;                                                     \
  s << message;                                                             \
  throw ExceptionName(s.str(), __FILE__, BOOST_CURRENT_FUNCTION, __LINE__); \
}

namespace pcl
{

  /** \class PCLException
    * \brief A base class for all pcl exceptions which inherits from std::runtime_error
    * \author Eitan Marder-Eppstein, Suat Gedikli, Nizar Sallem
    */
  class PCLException : public std::runtime_error
  {
    public:

      PCLException (const std::string& error_description,
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
      
      const char*
      getFileName () const throw ()
      {
        return (file_name_);
      }

      const char*
      getFunctionName () const throw ()
      {
        return (function_name_);
      }

      unsigned
      getLineNumber () const throw ()
      {
        return (line_number_);
      }

      const char*
      detailedMessage () const throw ()
      {
        return (what ());
      }
    

    protected:
      static std::string
      createDetailedMessage (const std::string& error_description,
                             const char* file_name,
                             const char* function_name,
                             unsigned line_number)
      {
        std::ostringstream sstream;
        if (function_name != nullptr)
          sstream << function_name << " ";
        
        if (file_name != nullptr)
        {
          sstream << "in " << file_name << " ";
          if (line_number != 0)
            sstream << "@ " << line_number << " ";
        }
        sstream << ": " << error_description;
        
        return (sstream.str ());
      }
    
      const char* file_name_;
      const char* function_name_;
      unsigned line_number_;
  } ;

  /** \class InvalidConversionException
    * \brief An exception that is thrown when a PCLPointCloud2 message cannot be converted into a PCL type
    */
  class InvalidConversionException : public PCLException
  {
    public:

      InvalidConversionException (const std::string& error_description,
                                  const char* file_name = nullptr,
                                  const char* function_name = nullptr,
                                  unsigned line_number = 0)
        : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;

  /** \class IsNotDenseException
    * \brief An exception that is thrown when a PointCloud is not dense but is attempted to be used as dense
    */
  class IsNotDenseException : public PCLException
  {
    public:

      IsNotDenseException (const std::string& error_description,
                           const char* file_name = nullptr,
                           const char* function_name = nullptr,
                           unsigned line_number = 0)
        : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;

  /** \class InvalidSACModelTypeException
    * \brief An exception that is thrown when a sample consensus model doesn't
    * have the correct number of samples defined in model_types.h
    */
  class InvalidSACModelTypeException : public PCLException
  {
    public:

      InvalidSACModelTypeException (const std::string& error_description,
                                    const char* file_name = nullptr,
                                    const char* function_name = nullptr,
                                    unsigned line_number = 0)
        : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;

  /** \class IOException
    * \brief An exception that is thrown during an IO error (typical read/write errors)
    */
  class IOException : public PCLException
  {
    public:

      IOException (const std::string& error_description,
                   const char* file_name = nullptr,
                   const char* function_name = nullptr,
                   unsigned line_number = 0)
        : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;

  /** \class InitFailedException
    * \brief An exception thrown when init can not be performed should be used in all the
    * PCLBase class inheritants.
    */
  class InitFailedException : public PCLException
  {
    public:
      InitFailedException (const std::string& error_description = "",
                           const char* file_name = nullptr,
                           const char* function_name = nullptr,
                           unsigned line_number = 0)
        : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;

  /** \class UnorganizedPointCloudException
    * \brief An exception that is thrown when an organized point cloud is needed
    * but not provided.
    */
  class UnorganizedPointCloudException : public PCLException
  {
    public:
    
      UnorganizedPointCloudException (const std::string& error_description,
                                      const char* file_name = nullptr,
                                      const char* function_name = nullptr,
                                      unsigned line_number = 0)
        : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;

  /** \class KernelWidthTooSmallException
    * \brief An exception that is thrown when the kernel size is too small
    */
  class KernelWidthTooSmallException : public PCLException
  {
    public:
    
    KernelWidthTooSmallException (const std::string& error_description,
                                  const char* file_name = nullptr,
                                  const char* function_name = nullptr,
                                  unsigned line_number = 0)
      : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;

  class UnhandledPointTypeException : public PCLException
  {
    public:
    UnhandledPointTypeException (const std::string& error_description,
                                 const char* file_name = nullptr,
                                 const char* function_name = nullptr,
                                 unsigned line_number = 0)
      : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  };

  class ComputeFailedException : public PCLException
  {
    public:
    ComputeFailedException (const std::string& error_description,
                            const char* file_name = nullptr,
                            const char* function_name = nullptr,
                            unsigned line_number = 0)
      : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  };

  /** \class BadArgumentException
    * \brief An exception that is thrown when the arguments number or type is wrong/unhandled.
    */
  class BadArgumentException : public PCLException
  {
    public:
    BadArgumentException (const std::string& error_description,
                          const char* file_name = nullptr,
                          const char* function_name = nullptr,
                          unsigned line_number = 0)
      : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  };
}

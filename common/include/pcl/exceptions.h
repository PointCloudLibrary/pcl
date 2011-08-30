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
 * Author: Eitan Marder-Eppstein, Suat Gedikli (gedikli@willowgarage.com)
 */
#ifndef PCL_EXCEPTIONS_H_
#define PCL_EXCEPTIONS_H_

#include <stdexcept>

namespace pcl
{

  /** \class PCLException
    * \brief A base class for all pcl exceptions which inherits from std::runtime_error
    */
  class PCLException : public std::runtime_error
  {
    public:

      PCLException (const std::string& error_description,
                    const std::string& file_name = "",
                    const std::string& function_name = "" ,
                    unsigned line_number = 0) throw ()
      : std::runtime_error (error_description)
      , file_name_ (file_name)
      , function_name_ (function_name)
      , line_number_ (line_number)
      {}
      
      virtual ~PCLException () throw ()
      {}
      
      const std::string&
      getFileName () const throw ()
      {
        return file_name_;
      }

      const std::string&
      getFunctionName () const throw ()
      {
        return function_name_;
      }

      unsigned
      getLineNumber () const throw ()
      {
        return line_number_;
      }

      std::string 
      detailedMessage () const throw ()
      {
        std::stringstream sstream;
        if (function_name_ != "")
          sstream << function_name_ << " ";
        
        if (file_name_ != "")
        {
          sstream << "in " << file_name_ << " ";
          if (line_number_ != 0)
            sstream << "@ " << line_number_ << " ";
        }
        sstream << ":" << what ();
        
        return sstream.str ();
      }

    protected:
      std::string file_name_;
      std::string function_name_;
      unsigned line_number_;
  } ;

  /** \class InvalidConversionException
    * \brief An exception that is thrown when a PointCloud2 message cannot be converted into a PCL type
    */
  class InvalidConversionException : public PCLException
  {
    public:

      InvalidConversionException (const std::string& error_description,
                                  const std::string& file_name = "",
                                  const std::string& function_name = "" ,
                                  unsigned line_number = 0) throw ()
        : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;

  /**
   * @class IsNotDenseException
   * @brief An exception that is thrown when a PointCloud is not dense but is attemped to be used as dense
   */
  class IsNotDenseException : public PCLException
  {
    public:

      IsNotDenseException (const std::string& error_description,
                           const std::string& file_name = "",
                           const std::string& function_name = "" ,
                           unsigned line_number = 0) throw ()
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
                                    const std::string& file_name = "",
                                    const std::string& function_name = "" ,
                                    unsigned line_number = 0) throw ()
        : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;
}
#endif

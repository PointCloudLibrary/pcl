/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef PCL_EXCEPTIONS_H_
#define PCL_EXCEPTIONS_H_

#include <stdexcept>

namespace pcl
{
  /**
   * @class PCLException
   * @brief A base class for all pcl exceptions which inherits from std::runtime_error
   */
  class PCLException: public std::runtime_error
  {
    public:
      PCLException(const std::string error_description) : std::runtime_error (error_description) {}
  };

  /**
   * @class InvalidConversionException
   * @brief An exception that is thrown when a PointCloud2 message cannot be converted into a PCL type
   */
  class InvalidConversionException : public PCLException
  {
    public:
      InvalidConversionException (const std::string error_description) : pcl::PCLException (error_description) {}
  };

  /**
   * @class IsNotDenseException
   * @brief An exception that is thrown when a PointCloud is not dense but is attemped to be used as dense
   */
  class IsNotDenseException : public PCLException
  {
    public:
      IsNotDenseException (const std::string error_description) : pcl::PCLException (error_description) {}
  };
}
#endif

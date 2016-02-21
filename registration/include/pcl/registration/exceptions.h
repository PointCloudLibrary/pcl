/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_EXCEPTIONS_H_
#define PCL_REGISTRATION_EXCEPTIONS_H_

#include <pcl/exceptions.h>

namespace pcl
{
  /** \class SolverDidntConvergeException
    * \brief An exception that is thrown when the non linear solver didn't converge
    */
  class PCL_EXPORTS SolverDidntConvergeException : public PCLException
  {
    public:
    
    SolverDidntConvergeException (const std::string& error_description,
                                  const char* file_name = NULL,
                                  const char* function_name = NULL,
                                  unsigned line_number = 0)
      : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;

 /** \class NotEnoughPointsException
    * \brief An exception that is thrown when the number of correspondants is not equal
    * to the minimum required
    */
  class PCL_EXPORTS NotEnoughPointsException : public PCLException
  {
    public:
    
    NotEnoughPointsException (const std::string& error_description,
                              const char* file_name = NULL,
                              const char* function_name = NULL,
                              unsigned line_number = 0)
      : pcl::PCLException (error_description, file_name, function_name, line_number) { }
  } ;
}
#endif//PCL_REGISTRATION_EXCEPTIONS_H_

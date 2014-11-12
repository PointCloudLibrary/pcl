/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#ifndef PCL_ROSLIB_MESSAGE_HEADER_H
#define PCL_ROSLIB_MESSAGE_HEADER_H

#ifdef USE_ROS
   #error USE_ROS setup requires PCL to compile against ROS message headers, which is now deprecated
#endif 

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/pcl_macros.h>
#include <ostream>

namespace pcl
{
  struct PCLHeader
  {
    PCLHeader (): seq (0), stamp (), frame_id ()
    {}

    /** \brief Sequence number */
    pcl::uint32_t seq;
    /** \brief A timestamp associated with the time when the data was acquired
      *
      * The value represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch).
      */
    pcl::uint64_t stamp;
    /** \brief Coordinate frame ID */
    std::string frame_id;

    typedef boost::shared_ptr<PCLHeader> Ptr;
    typedef boost::shared_ptr<PCLHeader const> ConstPtr;
  }; // struct PCLHeader

  typedef boost::shared_ptr<PCLHeader> HeaderPtr;
  typedef boost::shared_ptr<PCLHeader const> HeaderConstPtr;

  inline std::ostream& operator << (std::ostream& out, const PCLHeader &h)
  {
    out << "seq: " << h.seq;
    out << " stamp: " << h.stamp;
    out << " frame_id: " << h.frame_id << std::endl;
    return (out);
  }

  inline bool operator== (const PCLHeader &lhs, const PCLHeader &rhs)
  {
    return (&lhs == &rhs) ||
      (lhs.seq == rhs.seq && lhs.stamp == rhs.stamp && lhs.frame_id == rhs.frame_id);
  }

} // namespace pcl

#endif // PCL_ROSLIB_MESSAGE_HEADER_H


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


#ifndef PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
#define PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H

#ifdef USE_ROS
   #error USE_ROS setup requires PCL to compile against ROS message headers, which is now deprecated
#endif 

#include <string>
#include <vector>
#include <ostream>
#include <boost/detail/endian.hpp>

// Include the correct Header path here
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointField.h>

namespace pcl
{

  struct PCLPointCloud2
  {
    PCLPointCloud2 () : header (), height (0), width (0), fields (),
                     is_bigendian (false), point_step (0), row_step (0),
                     data (), is_dense (false)
    {
#if defined(BOOST_BIG_ENDIAN)
      is_bigendian = true;
#elif defined(BOOST_LITTLE_ENDIAN)
      is_bigendian = false;
#else
#error "unable to determine system endianness"
#endif
    }

    ::pcl::PCLHeader header;

    pcl::uint32_t height;
    pcl::uint32_t width;

    std::vector< ::pcl::PCLPointField>  fields;

    pcl::uint8_t is_bigendian;
    pcl::uint32_t point_step;
    pcl::uint32_t row_step;

    std::vector<pcl::uint8_t> data;

    pcl::uint8_t is_dense;

  public:
    typedef boost::shared_ptr< ::pcl::PCLPointCloud2> Ptr;
    typedef boost::shared_ptr< ::pcl::PCLPointCloud2  const> ConstPtr;
  }; // struct PCLPointCloud2

  typedef boost::shared_ptr< ::pcl::PCLPointCloud2> PCLPointCloud2Ptr;
  typedef boost::shared_ptr< ::pcl::PCLPointCloud2 const> PCLPointCloud2ConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::PCLPointCloud2 &v)
  {
    s << "header: " << std::endl;
    s << v.header;
    s << "height: ";
    s << "  " << v.height << std::endl;
    s << "width: ";
    s << "  " << v.width << std::endl;
    s << "fields[]" << std::endl;
    for (size_t i = 0; i < v.fields.size (); ++i)
    {
      s << "  fields[" << i << "]: ";
      s << std::endl;
      s << "    " << v.fields[i] << std::endl;
    }
    s << "is_bigendian: ";
    s << "  " << v.is_bigendian << std::endl;
    s << "point_step: ";
    s << "  " << v.point_step << std::endl;
    s << "row_step: ";
    s << "  " << v.row_step << std::endl;
    s << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size (); ++i)
    {
      s << "  data[" << i << "]: ";
      s << "  " << v.data[i] << std::endl;
    }
    s << "is_dense: ";
    s << "  " << v.is_dense << std::endl;
    
    return (s);
  }

} // namespace pcl

#endif // PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H


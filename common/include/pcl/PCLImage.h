/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point CLoud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2020-, Open Perception
 *
 * All rights reserved
 */
#pragma once

#include <string>   // for string
#include <vector>   // for vector
#include <ostream>  // for ostream

#include <pcl/PCLHeader.h>   // for PCLHeader

namespace pcl
{
  struct PCLImage
  {
    PCLHeader  header;

    std::uint32_t height = 0;
    std::uint32_t width = 0;
    std::string encoding;

    std::uint8_t is_bigendian = 0;
    std::uint32_t step = 0;

    std::vector<std::uint8_t> data;

    using Ptr = shared_ptr<PCLImage>;
    using ConstPtr = shared_ptr<const PCLImage>;
  }; // struct PCLImage

  using PCLImagePtr = PCLImage::Ptr;
  using PCLImageConstPtr = PCLImage::ConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const PCLImage& v)
  {
    s << "header: " << std::endl;
    s << v.header;
    s << "height: ";
    s << "  " << v.height << std::endl;
    s << "width: ";
    s << "  " << v.width << std::endl;
    s << "encoding: ";
    s << "  " << v.encoding << std::endl;
    s << "is_bigendian: ";
    s << "  " << v.is_bigendian << std::endl;
    s << "step: ";
    s << "  " << v.step << std::endl;
    s << "data[]" << std::endl;
    for (std::size_t i = 0; i < v.data.size (); ++i)
    {
      s << "  data[" << i << "]: ";
      s << "  " << v.data[i] << std::endl;
    }
    return (s);
  }
} // namespace pcl

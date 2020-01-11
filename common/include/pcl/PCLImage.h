#pragma once

#include <string>
#include <vector>
#include <ostream>

#ifdef USE_ROS
   #error USE_ROS setup requires PCL to compile against ROS message headers, which is now deprecated
#endif   

// Include the correct Header path here
#include <pcl/PCLHeader.h>

namespace pcl
{
  struct PCLImage
  {
     ::pcl::PCLHeader  header;

    std::uint32_t height = 0;
    std::uint32_t width = 0;
    std::string encoding;

    std::uint8_t is_bigendian = 0;
    std::uint32_t step = 0;

    std::vector<std::uint8_t> data;

    using Ptr = shared_ptr< ::pcl::PCLImage>;
    using ConstPtr = shared_ptr<const ::pcl::PCLImage>;
  }; // struct PCLImage

  using PCLImagePtr = PCLImage::Ptr;
  using PCLImageConstPtr = PCLImage::ConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::PCLImage & v)
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

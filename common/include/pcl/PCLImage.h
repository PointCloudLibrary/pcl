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
    PCLImage () : height (0), width (0),
               is_bigendian (0), step (0)
    {}

     ::pcl::PCLHeader  header;

    pcl::uint32_t height;
    pcl::uint32_t width;
    std::string encoding;

    pcl::uint8_t is_bigendian;
    pcl::uint32_t step;

    std::vector<pcl::uint8_t> data;

    using Ptr = boost::shared_ptr< ::pcl::PCLImage>;
    using ConstPtr = boost::shared_ptr<const ::pcl::PCLImage>;
  }; // struct PCLImage

  using PCLImagePtr = boost::shared_ptr< ::pcl::PCLImage>;
  using PCLImageConstPtr = boost::shared_ptr<const ::pcl::PCLImage>;

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
    for (size_t i = 0; i < v.data.size (); ++i)
    {
      s << "  data[" << i << "]: ";
      s << "  " << v.data[i] << std::endl;
    }
    return (s);
  }
} // namespace pcl

#ifndef PCL_MESSAGE_IMAGE_H
#define PCL_MESSAGE_IMAGE_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include "std_msgs/Header.h"

namespace sensor_msgs
{
  struct Image
  {
    Image () : header (), height (0), width (0), encoding (), 
               is_bigendian (0), step (0), data ()
    {}

     ::std_msgs::Header  header;

    pcl::uint32_t height;
    pcl::uint32_t width;
    std::string encoding;

    pcl::uint8_t is_bigendian;
    pcl::uint32_t step;

    std::vector<pcl::uint8_t> data;

    typedef boost::shared_ptr< ::sensor_msgs::Image> Ptr;
    typedef boost::shared_ptr< ::sensor_msgs::Image  const> ConstPtr;
  }; // struct Image

  typedef boost::shared_ptr< ::sensor_msgs::Image> ImagePtr;
  typedef boost::shared_ptr< ::sensor_msgs::Image const> ImageConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const  ::sensor_msgs::Image & v)
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
} // namespace sensor_msgs

#endif // PCL_MESSAGE_IMAGE_H


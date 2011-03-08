#ifndef PCL_MESSAGE_IMAGE_H
#define PCL_MESSAGE_IMAGE_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include "Header.h"

namespace sensor_msgs
{
  template <class ContainerAllocator>
  struct Image_ 
  {
    typedef Image_<ContainerAllocator> Type;

    Image_()
    : header()
    , height(0)
    , width(0)
    , encoding()
    , is_bigendian(0)
    , step(0)
    , data()
    {
    }

    Image_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , height(0)
    , width(0)
    , encoding(_alloc)
    , is_bigendian(0)
    , step(0)
    , data(_alloc)
    {
    }

    typedef  ::roslib::Header_<ContainerAllocator>  _header_type;
     ::roslib::Header_<ContainerAllocator>  header;

    typedef uint32_t _height_type;
    uint32_t height;

    typedef uint32_t _width_type;
    uint32_t width;

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _encoding_type;
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  encoding;

    typedef uint8_t _is_bigendian_type;
    uint8_t is_bigendian;

    typedef uint32_t _step_type;
    uint32_t step;

    typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  data;

    typedef boost::shared_ptr< ::sensor_msgs::Image_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::sensor_msgs::Image_<ContainerAllocator>  const> ConstPtr;
  }; // struct Image
  typedef  ::sensor_msgs::Image_<std::allocator<void> > Image;

  typedef boost::shared_ptr< ::sensor_msgs::Image> ImagePtr;
  typedef boost::shared_ptr< ::sensor_msgs::Image const> ImageConstPtr;
} // namespace sensor_msgs

#endif // PCL_MESSAGE_IMAGE_H


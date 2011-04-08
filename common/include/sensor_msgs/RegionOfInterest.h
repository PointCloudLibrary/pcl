#ifndef PCL_MESSAGE_REGIONOFINTEREST_H
#define PCL_MESSAGE_REGIONOFINTEREST_H
#include <string>
#include <vector>
#include <ostream>

namespace sensor_msgs
{
  template <class ContainerAllocator>
  struct RegionOfInterest_ 
  {
    typedef RegionOfInterest_<ContainerAllocator> Type;

    RegionOfInterest_()
    : x_offset(0)
    , y_offset(0)
    , height(0)
    , width(0)
    {
    }

    RegionOfInterest_(const ContainerAllocator& _alloc)
    : x_offset(0)
    , y_offset(0)
    , height(0)
    , width(0)
    {
    }

    typedef uint32_t _x_offset_type;
    uint32_t x_offset;

    typedef uint32_t _y_offset_type;
    uint32_t y_offset;

    typedef uint32_t _height_type;
    uint32_t height;

    typedef uint32_t _width_type;
    uint32_t width;

    typedef boost::shared_ptr< ::sensor_msgs::RegionOfInterest_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::sensor_msgs::RegionOfInterest_<ContainerAllocator>  const> ConstPtr;
  }; // struct RegionOfInterest
  typedef  ::sensor_msgs::RegionOfInterest_<std::allocator<void> > RegionOfInterest;

  typedef boost::shared_ptr< ::sensor_msgs::RegionOfInterest> RegionOfInterestPtr;
  typedef boost::shared_ptr< ::sensor_msgs::RegionOfInterest const> RegionOfInterestConstPtr;

  template<typename ContainerAllocator>
  std::ostream& stream_with_indentation (std::ostream& s, const std::string& indent, 
                                         const RegionOfInterest_<ContainerAllocator> & v)
  {
    s << indent << "x_offset: ";
    s << indent << "  " << v.x_offset << std::endl;
    s << indent << "y_offset: ";
    s << indent << "  " << v.y_offset << std::endl;
    s << indent << "height: ";
    s << indent << "  " << v.height << std::endl;
    s << indent << "width: ";
    s << indent << "  " << v.width << std::endl;
    //s << indent << "do_rectify: ";
    //s << indent << "  " << v.do_rectify << std::endl;
    return (s);
  }

  template<typename ContainerAllocator>
  std::ostream& operator<<(std::ostream& s, const  ::sensor_msgs::RegionOfInterest_<ContainerAllocator> & v)
  {
    stream_with_indentation (s, "", v);
    return (s);
  }

} // namespace sensor_msgs

#endif // PCL_MESSAGE_REGIONOFINTEREST_H


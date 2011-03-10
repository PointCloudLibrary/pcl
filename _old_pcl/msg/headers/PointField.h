#ifndef PCL_SENSOR_MSGS_MESSAGE_POINTFIELD_H
#define PCL_SENSOR_MSGS_MESSAGE_POINTFIELD_H
#include <string>
#include <vector>
#include <ostream>
#include <stdint.h>
#include <boost/shared_ptr.hpp>

namespace sensor_msgs
{
  template <class ContainerAllocator>
  struct PointField_
  {
    typedef PointField_<ContainerAllocator> Type;

    PointField_()
    : name()
    , offset(0)
    , datatype(0)
    , count(0)
    {
    }

    PointField_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , offset(0)
    , datatype(0)
    , count(0)
    {
    }

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

    typedef uint32_t _offset_type;
    uint32_t offset;

    typedef uint8_t _datatype_type;
    uint8_t datatype;

    typedef uint32_t _count_type;
    uint32_t count;

    enum { INT8 = 1 };
    enum { UINT8 = 2 };
    enum { INT16 = 3 };
    enum { UINT16 = 4 };
    enum { INT32 = 5 };
    enum { UINT32 = 6 };
    enum { FLOAT32 = 7 };
    enum { FLOAT64 = 8 };

  public:
    typedef boost::shared_ptr< ::sensor_msgs::PointField_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::sensor_msgs::PointField_<ContainerAllocator>  const> ConstPtr;
  }; // struct PointField
  typedef  ::sensor_msgs::PointField_<std::allocator<void> > PointField;

  typedef boost::shared_ptr< ::sensor_msgs::PointField> PointFieldPtr;
  typedef boost::shared_ptr< ::sensor_msgs::PointField const> PointFieldConstPtr;

} // namespace sensor_msgs

#endif // PCL_SENSOR_MSGS_MESSAGE_POINTFIELD_H


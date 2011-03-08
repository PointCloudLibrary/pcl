#ifndef PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
#define PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include "Header.h"
#include "PointField.h"

namespace sensor_msgs
{
  template <class ContainerAllocator>
  struct PointCloud2_
  {
    typedef PointCloud2_<ContainerAllocator> Type;

    PointCloud2_()
    : header()
    , height(0)
    , width(0)
    , fields()
    , is_bigendian(false)
    , point_step(0)
    , row_step(0)
    , data()
    , is_dense(false)
    {
    }

    PointCloud2_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , height(0)
    , width(0)
    , fields(_alloc)
    , is_bigendian(false)
    , point_step(0)
    , row_step(0)
    , data(_alloc)
    , is_dense(false)
    {
    }

    typedef ::roslib::Header_<ContainerAllocator>  _header_type;
    ::roslib::Header_<ContainerAllocator>  header;

    typedef uint32_t _height_type;
    uint32_t height;

    typedef uint32_t _width_type;
    uint32_t width;

    typedef std::vector< ::sensor_msgs::PointField_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::PointField_<ContainerAllocator> >::other >  _fields_type;
    std::vector< ::sensor_msgs::PointField_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::PointField_<ContainerAllocator> >::other >  fields;

    typedef uint8_t _is_bigendian_type;
    uint8_t is_bigendian;

    typedef uint32_t _point_step_type;
    uint32_t point_step;

    typedef uint32_t _row_step_type;
    uint32_t row_step;

    typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  data;

    typedef uint8_t _is_dense_type;
    uint8_t is_dense;

  public:
    typedef boost::shared_ptr< ::sensor_msgs::PointCloud2_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::sensor_msgs::PointCloud2_<ContainerAllocator>  const> ConstPtr;
  }; // struct PointCloud2
  typedef  ::sensor_msgs::PointCloud2_<std::allocator<void> > PointCloud2;

  typedef boost::shared_ptr< ::sensor_msgs::PointCloud2> PointCloud2Ptr;
  typedef boost::shared_ptr< ::sensor_msgs::PointCloud2 const> PointCloud2ConstPtr;

} // namespace sensor_msgs

#endif // PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H


#ifndef PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
#define PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include "std_msgs/Header.h"
#include "sensor_msgs/PointField.h"

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

    typedef ::std_msgs::Header_<ContainerAllocator>  _header_type;
    ::std_msgs::Header_<ContainerAllocator>  header;

    typedef pcl::uint32_t _height_type;
    pcl::uint32_t height;

    typedef pcl::uint32_t _width_type;
    pcl::uint32_t width;

    typedef std::vector< ::sensor_msgs::PointField_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::PointField_<ContainerAllocator> >::other >  _fields_type;
    std::vector< ::sensor_msgs::PointField_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::PointField_<ContainerAllocator> >::other >  fields;

    typedef pcl::uint8_t _is_bigendian_type;
    pcl::uint8_t is_bigendian;

    typedef pcl::uint32_t _point_step_type;
    pcl::uint32_t point_step;

    typedef pcl::uint32_t _row_step_type;
    pcl::uint32_t row_step;

    typedef std::vector<pcl::uint8_t, typename ContainerAllocator::template rebind<pcl::uint8_t>::other >  _data_type;
    std::vector<pcl::uint8_t, typename ContainerAllocator::template rebind<pcl::uint8_t>::other >  data;

    typedef pcl::uint8_t _is_dense_type;
    pcl::uint8_t is_dense;

  public:
    typedef boost::shared_ptr< ::sensor_msgs::PointCloud2_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::sensor_msgs::PointCloud2_<ContainerAllocator>  const> ConstPtr;
  }; // struct PointCloud2
  typedef  ::sensor_msgs::PointCloud2_<std::allocator<void> > PointCloud2;

  typedef boost::shared_ptr< ::sensor_msgs::PointCloud2> PointCloud2Ptr;
  typedef boost::shared_ptr< ::sensor_msgs::PointCloud2 const> PointCloud2ConstPtr;

  template<typename ContainerAllocator>
  std::ostream& stream_with_indentation (std::ostream& s, const std::string& indent, 
                                         const ::sensor_msgs::PointCloud2_<ContainerAllocator> & v)
  {
    s << indent << "header: " << std::endl;
    stream_with_indentation (s, indent + "  ", v.header);
    s << indent << "height: ";
    s << indent << "  " << v.height << std::endl;
    s << indent << "width: ";
    s << indent << "  " << v.width << std::endl;
    s << indent << "fields[]" << std::endl;
    for (size_t i = 0; i < v.fields.size (); ++i)
    {
      s << indent << "  fields[" << i << "]: ";
      s << std::endl;
      s << indent;
      s << indent << "    " << v.fields[i] << std::endl;
    }
    s << indent << "is_bigendian: ";
    s << indent << "  " << v.is_bigendian << std::endl;
    s << indent << "point_step: ";
    s << indent << "  " << v.point_step << std::endl;
    s << indent << "row_step: ";
    s << indent << "  " << v.row_step << std::endl;
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size (); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      s << indent << "  " << v.data[i] << std::endl;
    }
    s << indent << "is_dense: ";
    s << indent << "  " << v.is_dense << std::endl;
    
    return (s);
  }


  template<typename ContainerAllocator>
  std::ostream& operator<<(std::ostream& s, const  ::sensor_msgs::PointCloud2_<ContainerAllocator> & v)
  {
    stream_with_indentation (s, "", v);
    return (s);
  }

} // namespace sensor_msgs

#endif // PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H


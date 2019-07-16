#pragma once

#ifdef USE_ROS
   #error USE_ROS setup requires PCL to compile against ROS message headers, which is now deprecated
#endif

#include <string>
#include <vector>
#include <ostream>
#include <boost/predef/other/endian.h>

// Include the correct Header path here
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointField.h>

namespace pcl
{

  struct PCLPointCloud2
  {
    PCLPointCloud2 () : height (0), width (0), 
                     is_bigendian (false), point_step (0), row_step (0),
                     is_dense (false)
    {
#if BOOST_ENDIAN_BIG_BYTE
      is_bigendian = true;
#elif BOOST_ENDIAN_LITTLE_BYTE
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
    using Ptr = boost::shared_ptr< ::pcl::PCLPointCloud2>;
    using ConstPtr = boost::shared_ptr<const ::pcl::PCLPointCloud2>;
  }; // struct PCLPointCloud2

  using PCLPointCloud2Ptr = boost::shared_ptr< ::pcl::PCLPointCloud2>;
  using PCLPointCloud2ConstPtr = boost::shared_ptr<const ::pcl::PCLPointCloud2>;

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

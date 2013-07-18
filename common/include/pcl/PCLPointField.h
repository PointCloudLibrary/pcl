#ifndef PCL_SENSOR_MSGS_MESSAGE_POINTFIELD_H
#define PCL_SENSOR_MSGS_MESSAGE_POINTFIELD_H

#ifdef USE_ROS
   #error USE_ROS setup requires PCL to compile against ROS message headers, which is now deprecated
#endif 

#include <string>
#include <vector>
#include <ostream>
#include <boost/shared_ptr.hpp>
#include <pcl/pcl_macros.h>

namespace pcl
{
  struct PCLPointField
  {
    PCLPointField () : name (), offset (0), datatype (0), count (0)
    {}

    std::string name;

    pcl::uint32_t offset;
    pcl::uint8_t datatype;
    pcl::uint32_t count;

    enum PointFieldTypes { INT8 = 1,
                           UINT8 = 2,
                           INT16 = 3,
                           UINT16 = 4,
                           INT32 = 5,
                           UINT32 = 6,
                           FLOAT32 = 7,
                           FLOAT64 = 8 };

  public:
    typedef boost::shared_ptr< ::pcl::PCLPointField> Ptr;
    typedef boost::shared_ptr< ::pcl::PCLPointField const> ConstPtr;
  }; // struct PCLPointField

  typedef boost::shared_ptr< ::pcl::PCLPointField> PCLPointFieldPtr;
  typedef boost::shared_ptr< ::pcl::PCLPointField const> PCLPointFieldConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::PCLPointField & v)
  {
    s << "name: ";
    s << "  " << v.name << std::endl;
    s << "offset: ";
    s << "  " << v.offset << std::endl;
    s << "datatype: ";
    s << "  " << v.datatype << std::endl;
    s << "count: ";
    s << "  " << v.count << std::endl;
    return (s);
  }
} // namespace pcl

#endif // PCL_SENSOR_MSGS_MESSAGE_POINTFIELD_H


#ifndef PCL_SENSOR_MSGS_MESSAGE_POINTFIELD_H
#define PCL_SENSOR_MSGS_MESSAGE_POINTFIELD_H
#include <string>
#include <vector>
#include <ostream>
#include <boost/shared_ptr.hpp>
#include <pcl/pcl_macros.h>

namespace sensor_msgs
{
  struct PointField
  {
    PointField () : name (), offset (0), datatype (0), count (0)
    {}

    std::string name;

    pcl::uint32_t offset;
    pcl::uint8_t datatype;
    pcl::uint32_t count;

    enum { INT8 = 1 };
    enum { UINT8 = 2 };
    enum { INT16 = 3 };
    enum { UINT16 = 4 };
    enum { INT32 = 5 };
    enum { UINT32 = 6 };
    enum { FLOAT32 = 7 };
    enum { FLOAT64 = 8 };

  public:
    typedef boost::shared_ptr< ::sensor_msgs::PointField> Ptr;
    typedef boost::shared_ptr< ::sensor_msgs::PointField const> ConstPtr;
  }; // struct PointField

  typedef boost::shared_ptr< ::sensor_msgs::PointField> PointFieldPtr;
  typedef boost::shared_ptr< ::sensor_msgs::PointField const> PointFieldConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const  ::sensor_msgs::PointField & v)
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
} // namespace sensor_msgs

#endif // PCL_SENSOR_MSGS_MESSAGE_POINTFIELD_H


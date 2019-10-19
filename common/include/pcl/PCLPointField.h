#pragma once

#ifdef USE_ROS
   #error USE_ROS setup requires PCL to compile against ROS message headers, which is now deprecated
#endif 

#include <string>
#include <vector>
#include <ostream>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <pcl/pcl_macros.h>

namespace pcl
{
  struct PCLPointField
  {
    std::string name;

    pcl::uint32_t offset = 0;
    pcl::uint8_t datatype = 0;
    pcl::uint32_t count = 0;

    enum PointFieldTypes { INT8 = 1,
                           UINT8 = 2,
                           INT16 = 3,
                           UINT16 = 4,
                           INT32 = 5,
                           UINT32 = 6,
                           FLOAT32 = 7,
                           FLOAT64 = 8 };

  public:
    using Ptr = std::shared_ptr< ::pcl::PCLPointField>;
    using ConstPtr = std::shared_ptr<const ::pcl::PCLPointField>;
  }; // struct PCLPointField

  using PCLPointFieldPtr = std::shared_ptr<::pcl::PCLPointField>;
  using PCLPointFieldConstPtr = std::shared_ptr<const ::pcl::PCLPointField>;

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

  inline bool operator==(const PCLPointField& f1, const PCLPointField& f2)
  {
      return ((f1.name == f2.name) ||
              (f1.name == "rgb" && f2.name == "rgba") ||
              (f1.name == "rgba" && f2.name == "rgb"));
  }
} // namespace pcl

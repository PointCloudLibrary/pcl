#pragma once

#include <string>   // for string
#include <ostream>  // for ostream

#include <pcl/pcl_macros.h>  // for shared_ptr

namespace pcl
{
  struct PCLPointField
  {
    std::string name;

    std::uint32_t offset = 0;
    std::uint8_t datatype = 0;
    std::uint32_t count = 0;

    enum PointFieldTypes { INT8 = 1,
                           UINT8 = 2,
                           INT16 = 3,
                           UINT16 = 4,
                           INT32 = 5,
                           UINT32 = 6,
                           FLOAT32 = 7,
                           FLOAT64 = 8 };

  public:
    using Ptr = shared_ptr< ::pcl::PCLPointField>;
    using ConstPtr = shared_ptr<const ::pcl::PCLPointField>;
  }; // struct PCLPointField

  using PCLPointFieldPtr = PCLPointField::Ptr;
  using PCLPointFieldConstPtr = PCLPointField::ConstPtr;

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

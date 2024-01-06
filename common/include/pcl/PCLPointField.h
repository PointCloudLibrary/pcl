#pragma once

#include <pcl/memory.h>       // for shared_ptr
#include <pcl/type_traits.h>  // for asEnum_v
#include <pcl/types.h>        // for index_t

#include <string>   // for string
#include <ostream>  // for ostream

namespace pcl
{
  struct PCLPointField
  {
    std::string name;

    uindex_t offset = 0;
    std::uint8_t datatype = 0;
    uindex_t count = 0;

    enum PointFieldTypes { BOOL = traits::asEnum_v<bool>,
                           INT8 = traits::asEnum_v<std::int8_t>,
                           UINT8 = traits::asEnum_v<std::uint8_t>,
                           INT16 = traits::asEnum_v<std::int16_t>,
                           UINT16 = traits::asEnum_v<std::uint16_t>,
                           INT32 = traits::asEnum_v<std::int32_t>,
                           UINT32 = traits::asEnum_v<std::uint32_t>,
                           INT64 = traits::asEnum_v<std::int64_t>,
                           UINT64 = traits::asEnum_v<std::uint64_t>,
                           FLOAT32 = traits::asEnum_v<float>,
                           FLOAT64 = traits::asEnum_v<double>};

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
    switch(v.datatype) {
      case ::pcl::PCLPointField::PointFieldTypes::INT8: s << "  INT8" << std::endl; break;
      case ::pcl::PCLPointField::PointFieldTypes::UINT8: s << "  UINT8" << std::endl; break;
      case ::pcl::PCLPointField::PointFieldTypes::INT16: s << "  INT16" << std::endl; break;
      case ::pcl::PCLPointField::PointFieldTypes::UINT16: s << "  UINT16" << std::endl; break;
      case ::pcl::PCLPointField::PointFieldTypes::INT32: s << "  INT32" << std::endl; break;
      case ::pcl::PCLPointField::PointFieldTypes::UINT32: s << "  UINT32" << std::endl; break;
      case ::pcl::PCLPointField::PointFieldTypes::FLOAT32: s << "  FLOAT32" << std::endl; break;
      case ::pcl::PCLPointField::PointFieldTypes::FLOAT64: s << "  FLOAT64" << std::endl; break;
      default: s << "  " << static_cast<int>(v.datatype) << std::endl;
    }
    s << "count: ";
    s << "  " << v.count << std::endl;
    return (s);
  }

  // Return true if the PCLPointField matches the expected name and data type.
  // Written as a struct to allow partially specializing on Tag.
  template<typename PointT, typename Tag>
  struct FieldMatches
  {
    bool operator() (const PCLPointField& field)
    {
      return ((field.name == traits::name<PointT, Tag>::value) &&
              (field.datatype == traits::datatype<PointT, Tag>::value) &&
              ((field.count == traits::datatype<PointT, Tag>::size) ||
               (field.count == 0 && traits::datatype<PointT, Tag>::size == 1 /* see bug #821 */)));
    }
  };

} // namespace pcl


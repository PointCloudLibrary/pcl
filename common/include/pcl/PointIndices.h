#pragma once

#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include <pcl/PCLHeader.h>

namespace pcl
{
  struct PointIndices
  {
    PointIndices ()
    {}

    ::pcl::PCLHeader header;

    std::vector<int> indices;

    public:
      using Ptr = boost::shared_ptr< ::pcl::PointIndices>;
      using ConstPtr = boost::shared_ptr<const ::pcl::PointIndices>;
  }; // struct PointIndices

  using PointIndicesPtr = boost::shared_ptr< ::pcl::PointIndices>;
  using PointIndicesConstPtr = boost::shared_ptr<const ::pcl::PointIndices>;

  inline std::ostream& operator << (std::ostream& s, const ::pcl::PointIndices &v)
  {
    s << "header: " << std::endl;
    s << "  " << v.header;
    s << "indices[]" << std::endl;
    for (size_t i = 0; i < v.indices.size (); ++i)
    {
      s << "  indices[" << i << "]: ";
      s << "  " << v.indices[i] << std::endl;
    }
    return (s);
  }
} // namespace pcl

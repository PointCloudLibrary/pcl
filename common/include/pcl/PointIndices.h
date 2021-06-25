#pragma once

#include <ostream>

// Include the correct Header path here
#include <pcl/PCLHeader.h>
#include <pcl/types.h>

namespace pcl
{
  struct PointIndices
  {
    using Ptr = shared_ptr< ::pcl::PointIndices>;
    using ConstPtr = shared_ptr<const ::pcl::PointIndices>;

    PointIndices () = default;

    ::pcl::PCLHeader header;

    Indices indices;
  }; // struct PointIndices

  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;

  inline std::ostream& operator << (std::ostream& s, const ::pcl::PointIndices &v)
  {
    s << "header: " << std::endl;
    s << "  " << v.header;
    s << "indices[]" << std::endl;
    for (std::size_t i = 0; i < v.indices.size (); ++i)
    {
      s << "  indices[" << i << "]: ";
      s << "  " << v.indices[i] << std::endl;
    }
    return (s);
  }
} // namespace pcl

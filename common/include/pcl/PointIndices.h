#pragma once

#include <pcl/PCLHeader.h>    // for PCLHeader
#include <pcl/make_shared.h>  // for shared_ptr

#include <vector>   // for vector
#include <ostream>  // for ostream

namespace pcl
{
  struct PointIndices
  {
    PointIndices ()
    {}

    ::pcl::PCLHeader header;

    std::vector<int> indices;

    public:
      using Ptr = shared_ptr< ::pcl::PointIndices>;
      using ConstPtr = shared_ptr<const ::pcl::PointIndices>;
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

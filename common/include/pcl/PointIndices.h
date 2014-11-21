#ifndef PCL_MESSAGE_POINTINDICES_H
#define PCL_MESSAGE_POINTINDICES_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include <pcl/PCLHeader.h>

namespace pcl
{
  struct PointIndices
  {
    PointIndices () : header (), indices ()
    {}

    ::pcl::PCLHeader header;

    std::vector<int> indices;

    public:
      typedef boost::shared_ptr< ::pcl::PointIndices> Ptr;
      typedef boost::shared_ptr< ::pcl::PointIndices const> ConstPtr;
  }; // struct PointIndices

  typedef boost::shared_ptr< ::pcl::PointIndices> PointIndicesPtr;
  typedef boost::shared_ptr< ::pcl::PointIndices const> PointIndicesConstPtr;

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

#endif // PCL_MESSAGE_POINTINDICES_H


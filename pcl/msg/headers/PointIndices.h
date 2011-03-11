#ifndef PCL_MESSAGE_POINTINDICES_H
#define PCL_MESSAGE_POINTINDICES_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include "Header.h"

namespace pcl
{
  template <class ContainerAllocator>
  struct PointIndices_
  {
    typedef PointIndices_<ContainerAllocator> Type;

    PointIndices_()
    : header()
    , indices()
    {
    }

    PointIndices_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , indices(_alloc)
    {
    }

    typedef ::roslib::Header_<ContainerAllocator>  _header_type;
    ::roslib::Header_<ContainerAllocator>  header;

    typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _indices_type;
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  indices;

    public:
      typedef boost::shared_ptr< ::pcl::PointIndices_<ContainerAllocator> > Ptr;
      typedef boost::shared_ptr< ::pcl::PointIndices_<ContainerAllocator>  const> ConstPtr;
  }; // struct PointIndices
  typedef  ::pcl::PointIndices_<std::allocator<void> > PointIndices;

  typedef boost::shared_ptr< ::pcl::PointIndices> PointIndicesPtr;
  typedef boost::shared_ptr< ::pcl::PointIndices const> PointIndicesConstPtr;
} // namespace pcl

#endif // PCL_MESSAGE_POINTINDICES_H


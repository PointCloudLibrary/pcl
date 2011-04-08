#ifndef PCL_MESSAGE_POINTINDICES_H
#define PCL_MESSAGE_POINTINDICES_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include "std_msgs/Header.h"

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

    typedef ::std_msgs::Header_<ContainerAllocator>  _header_type;
    ::std_msgs::Header_<ContainerAllocator>  header;

    typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _indices_type;
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  indices;

    public:
      typedef boost::shared_ptr< ::pcl::PointIndices_<ContainerAllocator> > Ptr;
      typedef boost::shared_ptr< ::pcl::PointIndices_<ContainerAllocator>  const> ConstPtr;
  }; // struct PointIndices
  typedef  ::pcl::PointIndices_<std::allocator<void> > PointIndices;

  typedef boost::shared_ptr< ::pcl::PointIndices> PointIndicesPtr;
  typedef boost::shared_ptr< ::pcl::PointIndices const> PointIndicesConstPtr;

  template<typename ContainerAllocator>
  std::ostream& stream_with_indentation (std::ostream& s, const std::string& indent, 
                                         const ::pcl::PointIndices_<ContainerAllocator> & v)
  {
    s << indent << "header: " << std::endl;
    stream_with_indentation(s, indent + "  ", v.header);
    s << indent << "indices[]" << std::endl;
    for (size_t i = 0; i < v.indices.size (); ++i)
    {
      s << indent << "  indices[" << i << "]: ";
      s << indent << "  " << v.indices[i] << std::endl;
    }
    return (s);
  }

  template<typename ContainerAllocator>
  std::ostream& operator<<(std::ostream& s, const  ::pcl::PointIndices_<ContainerAllocator> & v)
  {
    stream_with_indentation (s, "", v);
    return (s);
  }

} // namespace pcl

#endif // PCL_MESSAGE_POINTINDICES_H


#ifndef PCL_MESSAGE_VERTICES_H
#define PCL_MESSAGE_VERTICES_H
#include <string>
#include <vector>
#include <ostream>

namespace pcl
{
  template <class ContainerAllocator>
  struct Vertices_
  {
    typedef Vertices_<ContainerAllocator> Type;

    Vertices_()
    : vertices()
    {
    }

    Vertices_(const ContainerAllocator& _alloc)
    : vertices(_alloc)
    {
    }

    typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _vertices_type;
    std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  vertices;

  public:
    typedef boost::shared_ptr< ::pcl::Vertices_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::pcl::Vertices_<ContainerAllocator>  const> ConstPtr;
  }; // struct Vertices
  typedef  ::pcl::Vertices_<std::allocator<void> > Vertices;

  typedef boost::shared_ptr< ::pcl::Vertices> VerticesPtr;
  typedef boost::shared_ptr< ::pcl::Vertices const> VerticesConstPtr;

} // namespace pcl

#endif // PCL_MESSAGE_VERTICES_H


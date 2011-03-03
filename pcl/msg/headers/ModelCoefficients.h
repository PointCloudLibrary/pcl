#ifndef PCL_MESSAGE_MODELCOEFFICIENTS_H
#define PCL_MESSAGE_MODELCOEFFICIENTS_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include "Header.h"

namespace pcl
{
  template <class ContainerAllocator>
  struct ModelCoefficients_
  {
    typedef ModelCoefficients_<ContainerAllocator> Type;

    ModelCoefficients_()
    : header()
    , values()
    {
    }

    ModelCoefficients_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , values(_alloc)
    {
    }

    typedef ::roslib::Header_<ContainerAllocator>  _header_type;
    ::roslib::Header_<ContainerAllocator>  header;

    typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _values_type;
    std::vector<float, typename ContainerAllocator::template rebind<float>::other >  values;

  public:
    typedef boost::shared_ptr< ::pcl::ModelCoefficients_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::pcl::ModelCoefficients_<ContainerAllocator>  const> ConstPtr;
  }; // struct ModelCoefficients
  typedef  ::pcl::ModelCoefficients_<std::allocator<void> > ModelCoefficients;

  typedef boost::shared_ptr< ::pcl::ModelCoefficients> ModelCoefficientsPtr;
  typedef boost::shared_ptr< ::pcl::ModelCoefficients const> ModelCoefficientsConstPtr;

} // namespace pcl

#endif // PCL_MESSAGE_MODELCOEFFICIENTS_H


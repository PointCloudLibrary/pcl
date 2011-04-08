#ifndef PCL_MESSAGE_MODELCOEFFICIENTS_H
#define PCL_MESSAGE_MODELCOEFFICIENTS_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include "std_msgs/Header.h"

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

    typedef ::std_msgs::Header_<ContainerAllocator>  _header_type;
    ::std_msgs::Header_<ContainerAllocator>  header;

    typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _values_type;
    std::vector<float, typename ContainerAllocator::template rebind<float>::other >  values;

  public:
    typedef boost::shared_ptr< ::pcl::ModelCoefficients_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::pcl::ModelCoefficients_<ContainerAllocator>  const> ConstPtr;
  }; // struct ModelCoefficients
  typedef  ::pcl::ModelCoefficients_<std::allocator<void> > ModelCoefficients;

  typedef boost::shared_ptr< ::pcl::ModelCoefficients> ModelCoefficientsPtr;
  typedef boost::shared_ptr< ::pcl::ModelCoefficients const> ModelCoefficientsConstPtr;

  template<typename ContainerAllocator>
  std::ostream& stream_with_indentation (std::ostream& s, const std::string& indent, 
                                         const ::pcl::ModelCoefficients_<ContainerAllocator> & v)
  {
    s << indent << "header: " << std::endl;
    stream_with_indentation (s, indent + "  ", v.header);
    s << indent << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size (); ++i)
    {
      s << indent << "  values[" << i << "]: ";
      s << indent << "  " << v.values[i] << std::endl;
    }
    return (s);
  }

  template<typename ContainerAllocator>
  std::ostream& operator<<(std::ostream& s, const  ::pcl::ModelCoefficients_<ContainerAllocator> & v)
  {
    stream_with_indentation (s, "", v);
    return (s);
  }

} // namespace pcl

#endif // PCL_MESSAGE_MODELCOEFFICIENTS_H


#ifndef PCL_ROSLIB_MESSAGE_HEADER_H
#define PCL_ROSLIB_MESSAGE_HEADER_H
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/win32_macros.h>

namespace std_msgs
{
  template <class ContainerAllocator>
  struct Header_ 
  {
    typedef Header_<ContainerAllocator> Type;

    Header_()
    : seq(0)
    , stamp()
    , frame_id()
    {
    }

    Header_(const ContainerAllocator& _alloc)
    : seq(0)
    , stamp()
    , frame_id(_alloc)
    {
    }

    pcl::uint32_t seq;
    pcl::uint64_t stamp;

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _frame_id_type;
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  frame_id;

    typedef boost::shared_ptr<Header_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr<Header_<ContainerAllocator>  const> ConstPtr;
  }; // struct Header
  typedef Header_<std::allocator<void> > Header;

  typedef boost::shared_ptr<Header> HeaderPtr;
  typedef boost::shared_ptr<Header const> HeaderConstPtr;

  template<typename ContainerAllocator>
  std::ostream& stream_with_indentation (std::ostream& s, const std::string& indent, 
                                         const Header_<ContainerAllocator> & v)
  {
    s << indent << "seq: ";
    s << indent << "  " << v.seq << std::endl;
    s << indent << "stamp: ";
    s << indent << "  " << v.stamp << std::endl;
    s << indent << "frame_id: ";
    s << indent << "  " << v.frame_id << std::endl;
    return (s);
  }

  template<typename ContainerAllocator>
  std::ostream& operator << (std::ostream& out, const Header_<ContainerAllocator> & h)
  {
    out << "seq: " << h.seq;
    out << " stamp: " << h.stamp;
    out << " frame_id: " << h.frame_id << std::endl;
    return (out);
  }

} // namespace std_msgs

#endif // PCL_ROSLIB_MESSAGE_HEADER_H


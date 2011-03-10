#ifndef PCL_ROSLIB_MESSAGE_HEADER_H
#define PCL_ROSLIB_MESSAGE_HEADER_H
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#ifndef _WIN32
# include <stdint.h>
#else
#  if (_MSC_VER >= 1600)
#    include <stdint.h>
  // Visual Studio pre-2010 doesn't have stdint.h
#  else
    typedef signed char int8_t;
    typedef short int16_t;
    typedef int int32_t;

    typedef unsigned char uint8_t;
    typedef unsigned short uint16_t;
    typedef unsigned int uint32_t;

    typedef signed char int_least8_t;
    typedef short int_least16_t;
    typedef int int_least32_t;

    typedef unsigned char uint_least8_t;
    typedef unsigned short uint_least16_t;
    typedef unsigned int uint_least32_t;

    typedef char int_fast8_t;
    typedef int int_fast16_t;
    typedef int int_fast32_t;

    typedef unsigned char uint_fast8_t;
    typedef unsigned int uint_fast16_t;
    typedef unsigned int uint_fast32_t;

    typedef _Longlong int64_t;
    typedef _ULonglong uint64_t;
#  endif
#endif

namespace roslib
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

    uint32_t seq;
    uint64_t stamp;

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _frame_id_type;
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  frame_id;

    typedef boost::shared_ptr<Header_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr<Header_<ContainerAllocator>  const> ConstPtr;
  }; // struct Header
  typedef Header_<std::allocator<void> > Header;

  typedef boost::shared_ptr<Header> HeaderPtr;
  typedef boost::shared_ptr<Header const> HeaderConstPtr;
} // namespace pcl

#endif // PCL_ROSLIB_MESSAGE_HEADER_H


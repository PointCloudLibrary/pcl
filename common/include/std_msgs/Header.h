#ifndef PCL_ROSLIB_MESSAGE_HEADER_H
#define PCL_ROSLIB_MESSAGE_HEADER_H
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/pcl_macros.h>
#include <ostream>

namespace std_msgs
{
  struct Header
  {
    Header (): seq (0), stamp (), frame_id ()
    {}

    pcl::uint32_t seq;
    pcl::uint64_t stamp;

    std::string frame_id;

    typedef boost::shared_ptr<Header> Ptr;
    typedef boost::shared_ptr<Header const> ConstPtr;
  }; // struct Header

  typedef boost::shared_ptr<Header> HeaderPtr;
  typedef boost::shared_ptr<Header const> HeaderConstPtr;

  inline std::ostream& operator << (std::ostream& out, const Header &h)
  {
    out << "seq: " << h.seq;
    out << " stamp: " << h.stamp;
    out << " frame_id: " << h.frame_id << std::endl;
    return (out);
  }

} // namespace std_msgs

#endif // PCL_ROSLIB_MESSAGE_HEADER_H


#pragma once

#ifdef USE_ROS
   #error USE_ROS setup requires PCL to compile against ROS message headers, which is now deprecated
#endif 

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/pcl_macros.h>
#include <ostream>

namespace pcl
{
  struct PCLHeader
  {
    PCLHeader (): seq (0), stamp ()
    {}

    /** \brief Sequence number */
    pcl::uint32_t seq;
    /** \brief A timestamp associated with the time when the data was acquired
      *
      * The value represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch).
      */
    pcl::uint64_t stamp;
    /** \brief Coordinate frame ID */
    std::string frame_id;

    typedef boost::shared_ptr<PCLHeader> Ptr;
    typedef boost::shared_ptr<PCLHeader const> ConstPtr;
  }; // struct PCLHeader

  typedef boost::shared_ptr<PCLHeader> HeaderPtr;
  typedef boost::shared_ptr<PCLHeader const> HeaderConstPtr;

  inline std::ostream& operator << (std::ostream& out, const PCLHeader &h)
  {
    out << "seq: " << h.seq;
    out << " stamp: " << h.stamp;
    out << " frame_id: " << h.frame_id << std::endl;
    return (out);
  }

  inline bool operator== (const PCLHeader &lhs, const PCLHeader &rhs)
  {
    return (&lhs == &rhs) ||
      (lhs.seq == rhs.seq && lhs.stamp == rhs.stamp && lhs.frame_id == rhs.frame_id);
  }

} // namespace pcl

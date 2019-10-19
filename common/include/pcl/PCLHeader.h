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
    std::uint32_t seq;
    /** \brief A timestamp associated with the time when the data was acquired
      *
      * The value represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch).
      */
    std::uint64_t stamp;
    /** \brief Coordinate frame ID */
    std::string frame_id;

    using Ptr = boost::shared_ptr<PCLHeader>;
    using ConstPtr = boost::shared_ptr<const PCLHeader>;
  }; // struct PCLHeader

  using HeaderPtr = boost::shared_ptr<PCLHeader>;
  using HeaderConstPtr = boost::shared_ptr<const PCLHeader>;

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

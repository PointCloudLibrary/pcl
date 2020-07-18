#pragma once

#include <pcl/memory.h>  // for shared_ptr

#include <string>   // for string
#include <ostream>  // for ostream

namespace pcl
{
  struct PCLHeader
  {
    /** \brief Sequence number */
    std::uint32_t seq = 0;
    /** \brief A timestamp associated with the time when the data was acquired
      *
      * The value represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch).
      */
    std::uint64_t stamp = 0;
    /** \brief Coordinate frame ID */
    std::string frame_id;

    using Ptr = shared_ptr<PCLHeader>;
    using ConstPtr = shared_ptr<const PCLHeader>;
  }; // struct PCLHeader

  using HeaderPtr = PCLHeader::Ptr;
  using HeaderConstPtr = PCLHeader::ConstPtr;

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


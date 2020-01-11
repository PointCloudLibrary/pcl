#pragma once

#include <string>
#include <vector>
#include <ostream>
#include <boost/shared_ptr.hpp>
#include <pcl/pcl_macros.h>

namespace pcl
{
  /** \brief Describes a set of vertices in a polygon mesh, by basically
    * storing an array of indices.
    */
  struct Vertices
  {
    Vertices ()
    {}

    std::vector<std::uint32_t> vertices;

  public:
    using Ptr = shared_ptr<Vertices>;
    using ConstPtr = shared_ptr<const Vertices>;
  }; // struct Vertices


  using VerticesPtr = Vertices::Ptr;
  using VerticesConstPtr = Vertices::ConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::Vertices & v)
  {
    s << "vertices[]" << std::endl;
    for (std::size_t i = 0; i < v.vertices.size (); ++i)
    {
      s << "  vertices[" << i << "]: ";
      s << "  " << v.vertices[i] << std::endl;
    }
    return (s);
  }
} // namespace pcl

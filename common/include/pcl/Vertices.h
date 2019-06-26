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

    std::vector<uint32_t> vertices;

  public:
    using Ptr = boost::shared_ptr<Vertices>;
    using ConstPtr = boost::shared_ptr<const Vertices>;
  }; // struct Vertices


  using VerticesPtr = boost::shared_ptr<Vertices>;
  using VerticesConstPtr = boost::shared_ptr<const Vertices>;

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::Vertices & v)
  {
    s << "vertices[]" << std::endl;
    for (size_t i = 0; i < v.vertices.size (); ++i)
    {
      s << "  vertices[" << i << "]: ";
      s << "  " << v.vertices[i] << std::endl;
    }
    return (s);
  }
} // namespace pcl

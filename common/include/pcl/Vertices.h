#ifndef PCL_MESSAGE_VERTICES_H
#define PCL_MESSAGE_VERTICES_H
#include <string>
#include <vector>
#include <ostream>
#include <pcl/win32_macros.h>

namespace pcl
{
  struct Vertices
  {
    Vertices () : vertices ()
    {}

    std::vector<int> vertices;

  public:
    typedef boost::shared_ptr< ::pcl::Vertices > Ptr;
    typedef boost::shared_ptr< ::pcl::Vertices const> ConstPtr;
  }; // struct Vertices

  typedef boost::shared_ptr< ::pcl::Vertices> VerticesPtr;
  typedef boost::shared_ptr< ::pcl::Vertices const> VerticesConstPtr;

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

#endif // PCL_MESSAGE_VERTICES_H


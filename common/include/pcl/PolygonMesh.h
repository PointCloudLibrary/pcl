#pragma once

#include <algorithm>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/Vertices.h>

namespace pcl
{
  struct PolygonMesh
  {
    PolygonMesh ()
    {}

    ::pcl::PCLHeader  header;

    ::pcl::PCLPointCloud2 cloud;

    std::vector< ::pcl::Vertices>  polygons;

    /** \brief Inplace concatenate two pcl::PolygonMesh
      * \param[in,out] mesh1 the first input and output mesh
      * \param[in] mesh2 the second input mesh
      * \return true if successful, false otherwise (unexpected error)
      */
    static bool
    concatenate (pcl::PolygonMesh &mesh1, const pcl::PolygonMesh &mesh2)
    {
      const auto point_offset = mesh1.cloud.width * mesh1.cloud.height;
      
      bool success = pcl::PCLPointCloud2::concatenate(mesh1.cloud, mesh2.cloud);
      if (success == false) {
        return false;
      }
      // Make the resultant polygon mesh take the newest stamp
      mesh1.header.stamp = std::max(mesh1.header.stamp, mesh2.header.stamp);

      std::transform(mesh2.polygons.begin (),
                     mesh2.polygons.end (),
                     std::back_inserter (mesh1.polygons),
                     [point_offset](auto polygon)
                     {
                        std::transform(polygon.vertices.begin (),
                                       polygon.vertices.end (),
                                       polygon.vertices.begin (),
                                       [point_offset](auto& point_idx)
                                       {
                                         return point_idx + point_offset;
                                       });
                        return polygon;
                      });

      return true;
    }

    /** \brief Concatenate two pcl::PCLPointCloud2
      * \param[in] mesh1 the first input mesh
      * \param[in] mesh2 the second input mesh
      * \param[out] mesh_out the resultant output mesh
      * \return true if successful, false otherwise (unexpected error)
      */
    static bool
    concatenate (const PolygonMesh &mesh1,
                 const PolygonMesh &mesh2,
                 PolygonMesh &mesh_out)
    {
      mesh_out = mesh1;
      return concatenate(mesh_out, mesh2);
    }

    /** \brief Add another polygon mesh to the current mesh.
      * \param[in] rhs the mesh to add to the current mesh
      * \return the new mesh as a concatenation of the current mesh and the new given mesh
      */
    inline PolygonMesh&
    operator += (const PolygonMesh& rhs)
    {
      concatenate((*this), rhs);
      return (*this);
    }

    /** \brief Add a polygon mesh to another mesh.
      * \param[in] rhs the mesh to add to the current mesh
      * \return the new mesh as a concatenation of the current mesh and the new given mesh
      */
    inline const PolygonMesh
    operator + (const PolygonMesh& rhs)
    {
      return (PolygonMesh (*this) += rhs);
    }

  public:
    using Ptr = shared_ptr< ::pcl::PolygonMesh>;
    using ConstPtr = shared_ptr<const ::pcl::PolygonMesh>;
  }; // struct PolygonMesh

  using PolygonMeshPtr = PolygonMesh::Ptr;
  using PolygonMeshConstPtr = PolygonMesh::ConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::PolygonMesh &v)
  {
    s << "header: " << std::endl;
    s << v.header;
    s << "cloud: " << std::endl;
    s << v.cloud;
    s << "polygons[]" << std::endl;
    for (std::size_t i = 0; i < v.polygons.size (); ++i)
    {
      s << "  polygons[" << i << "]: " << std::endl;
      s << v.polygons[i];
    }
    return (s);
  }

} // namespace pcl

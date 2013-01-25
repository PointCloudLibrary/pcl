/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_GEOMETRY_MESH_CONVERSION_H
#define PCL_GEOMETRY_MESH_CONVERSION_H

#include <pcl/PolygonMesh.h>
#include <pcl/ros/conversions.h>

namespace pcl
{
  namespace geometry
  {
    /** \brief Conversions for the half-edge mesh.
      * \author Martin Saelzle
      * \ingroup geometry
      */
    class MeshConversion
    {
      public:

        /** \brief Constructor. */
        MeshConversion () {}

        /** \brief Convert from a half-edge mesh to a face-vertex mesh.
          * \param[in] half_edge_mesh The input mesh.
          * \param[out] face_vertex_mesh The output mesh.
          */
        template <class HalfEdgeMeshT> void
        toFaceVertexMesh (const HalfEdgeMeshT& half_edge_mesh, pcl::PolygonMesh& face_vertex_mesh)
        {
          typedef HalfEdgeMeshT HalfEdgeMesh;
          typedef typename HalfEdgeMeshT::VertexAroundFaceCirculator VAFC;
          typedef typename HalfEdgeMesh::FaceIndex FaceIndex;

          pcl::Vertices polygon;
          pcl::toROSMsg (half_edge_mesh.getVertexDataCloud (), face_vertex_mesh.cloud);

          face_vertex_mesh.polygons.reserve (half_edge_mesh.sizeFaces ());
          for (size_t i=0; i<half_edge_mesh.sizeFaces (); ++i)
          {
            VAFC       circ     = half_edge_mesh.getVertexAroundFaceCirculator (FaceIndex (i));
            const VAFC circ_end = circ;
            polygon.vertices.clear ();
            do
            {
              polygon.vertices.push_back (circ.getTargetIndex ().get ());
            } while (++circ != circ_end);
            face_vertex_mesh.polygons.push_back (polygon);
          }
        }
    };
  } // End namespace geometry
} // End namespace pcl

#endif // PCL_GEOMETRY_MESH_CONVERSION_H

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

#include <pcl/apps/in_hand_scanner/mesh_processing.h>

#include <cmath>

#include <pcl/apps/in_hand_scanner/utils.h>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::MeshProcessing::MeshProcessing ()
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::MeshProcessing::processBoundary (Mesh& mesh, const std::vector <HalfEdgeIndices>& boundary_collection, const bool cleanup) const
{
  typedef std::vector <Mesh::HalfEdgeIndices> BoundaryCollection;

  Mesh::VertexIndex vi_a, vi_b, vi_c, vi_d;
  Eigen::Vector3f ab, bc, ac, n_adb, n_plane; // Edges and normals
  Mesh::FaceIndex opposite_face;

  for (BoundaryCollection::const_iterator it_bc=boundary_collection.begin (); it_bc!=boundary_collection.end (); ++it_bc)
  {
    const Mesh::HalfEdgeIndices& boundary = *it_bc;
    if (boundary.size () == 3)
    {
      opposite_face = mesh.getOppositeFaceIndex (boundary [0]);

      if (mesh.getOppositeFaceIndex (boundary [1]) == opposite_face  &&
          mesh.getOppositeFaceIndex (boundary [2]) == opposite_face)
      {
        // Isolated face.
        mesh.deleteFace (opposite_face);
      }
      else
      {
        // Close triangular hole.
        mesh.addFace (mesh.getTerminatingVertexIndex (boundary [0]),
                      mesh.getTerminatingVertexIndex (boundary [1]),
                      mesh.getTerminatingVertexIndex (boundary [2]));
      }
    }
    else // size != 3
    {
      // Add triangles where the angle between the edges is below a threshold. In the example this would leave only triangles 1-2-3 and triangles 4-5-6 (threshold = 60 degrees). Triangle 1-2-3 should not be added because vertex 2 is not convex (as vertex 5).

      // Example: The boundary is on the top. Vertex 7 is connected to vertex 0.
      //           2                   //
      //          / \                  //
      // ... 0 - 1   3 - 4   6 - 7 ... //
      //                  \ /          //
      //                   5           //

      for (int i=0; i<boundary.size (); ++i)
      {
        // The vertices on the boundary
        vi_a = mesh.getOriginatingVertexIndex (boundary [i]);
        vi_b = mesh.getTerminatingVertexIndex (boundary [i]);
        vi_c = mesh.getTerminatingVertexIndex (boundary [(i+1) % boundary.size ()]);

        const Eigen::Vector4f& v_a = mesh.getVertexDataCloud () [vi_a.get ()].getVector4fMap ();
        const Eigen::Vector4f& v_b = mesh.getVertexDataCloud () [vi_b.get ()].getVector4fMap ();
        const Eigen::Vector4f& v_c = mesh.getVertexDataCloud () [vi_c.get ()].getVector4fMap ();

        ab = (v_b - v_a).head <3> ();
        bc = (v_c - v_b).head <3> ();
        ac = (v_c - v_a).head <3> ();

        const float angle = std::acos (pcl::ihs::clamp (-ab.dot (bc) / ab.norm () / bc.norm (), -1.f, 1.f));

        if (angle < 1.047197551196598f) // 60 * pi / 180
        {
          // Third vertex belonging to the face of edge ab
          vi_d = mesh.getTerminatingVertexIndex (
                   mesh.getNextHalfEdgeIndex (
                     mesh.getOppositeHalfEdgeIndex (boundary [i])));
          const Eigen::Vector4f& v_d = mesh.getVertexDataCloud () [vi_d.get ()].getVector4fMap ();

          // n_adb is the normal of triangle a-d-b.
          // The plane goes through edge a-b and is perpendicular to the plane through a-d-b.
          n_adb   = (v_d - v_a).head <3> ().cross (ab)/*.normalized ()*/;
          n_plane = n_adb.cross (ab/*.nomalized ()*/);

          if (n_plane.dot (ac) > 0.f)
          {
            mesh.addFace (vi_a, vi_b, vi_c);
          }
        }
      }
    }
  }

  if (cleanup)
    mesh.cleanUp ();
}

////////////////////////////////////////////////////////////////////////////////

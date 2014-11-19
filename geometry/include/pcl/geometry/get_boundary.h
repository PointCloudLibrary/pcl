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

#ifndef PCL_GEOMETRY_GET_BOUNDARY_H
#define PCL_GEOMETRY_GET_BOUNDARY_H

#include <vector>

namespace pcl
{
  namespace geometry
  {
    /** \brief Get a collection of boundary half-edges for the input mesh.
      * \param[in] mesh The input mesh.
      * \param[out] boundary_he_collection Collection of boundary half-edges. Each element in the vector is one connected boundary. The whole boundary is the union of all elements.
      * \param [in] expected_size If you already know the size of the longest boundary you can tell this here. Defaults to 3 (minimum possible boundary).
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshT> void
    getBoundBoundaryHalfEdges (const MeshT&                                   mesh,
                               std::vector <typename MeshT::HalfEdgeIndices>& boundary_he_collection,
                               const size_t                                   expected_size = 3)
    {
      typedef MeshT                                            Mesh;
      typedef typename Mesh::HalfEdgeIndex                     HalfEdgeIndex;
      typedef typename Mesh::HalfEdgeIndices                   HalfEdgeIndices;
      typedef typename Mesh::InnerHalfEdgeAroundFaceCirculator IHEAFC;

      boundary_he_collection.clear ();

      HalfEdgeIndices boundary_he; boundary_he.reserve (expected_size);
      std::vector <bool> visited (mesh.sizeEdges (), false);
      IHEAFC circ, circ_end;

      for (HalfEdgeIndex i (0); i<HalfEdgeIndex (mesh.sizeHalfEdges ()); ++i)
      {
        if (mesh.isBoundary (i) && !visited [pcl::geometry::toEdgeIndex (i).get ()])
        {
          boundary_he.clear ();

          circ     = mesh.getInnerHalfEdgeAroundFaceCirculator (i);
          circ_end = circ;
          do
          {
            visited [pcl::geometry::toEdgeIndex (circ.getTargetIndex ()).get ()] = true;
            boundary_he.push_back (circ.getTargetIndex ());
          } while (++circ != circ_end);

          boundary_he_collection.push_back (boundary_he);
        }
      }
    }

  } // End namespace geometry
} // End namespace pcl

#endif // PCL_GEOMETRY_GET_BOUNDARY_H

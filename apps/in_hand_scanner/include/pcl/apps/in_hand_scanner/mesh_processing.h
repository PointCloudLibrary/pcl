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

#ifndef PCL_APPS_IN_HAND_SCANNER_MESH_PROCESSING_H
#define PCL_APPS_IN_HAND_SCANNER_MESH_PROCESSING_H

#include <pcl/apps/in_hand_scanner/common_types.h>

namespace pcl
{
  namespace ihs
  {
    /** \brief Contains methods that take advantage of the connectivity information in the mesh.
      * \author Martin Saelzle
      * \ingroup apps
      */
    class MeshProcessing
    {
      public:

        typedef pcl::ihs::Mesh        Mesh;
        typedef Mesh::HalfEdgeIndices HalfEdgeIndices;

        // Currently works only on the manifold mesh.
        BOOST_STATIC_ASSERT (Mesh::IsManifold::value);

        /** \brief Constructor. */
        MeshProcessing ();

        /** \brief Inserts triangles into jagged boundaries, removes isolated triangles and closes triangular holes.
          * \param[in,out] mesh The mesh which should be processed.
          * \param[in] boundary_collection Collection of boundary half-edges.
          * \param[in] cleanup Calls mesh.cleanup () if true.
          */
        void
        processBoundary (Mesh& mesh, const std::vector <HalfEdgeIndices>& boundary_collection, const bool cleanup=true) const;
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_MESH_PROCESSING_H

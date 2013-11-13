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

#ifndef PCL_GEOMETRY_MESH_TRAITS_H
#define PCL_GEOMETRY_MESH_TRAITS_H

#include <pcl/geometry/boost.h>

namespace pcl
{ 
  namespace geometry
  {
    /** \brief No data is associated with the vertices / half-edges / edges / faces. */
    struct NoData
    {
#if defined(_LIBCPP_VERSION) && _LIBCPP_VERSION <= 1101
      operator unsigned char() const
      {
        return 0;
      }
#endif
    };

    /** \brief The mesh traits are used to set up compile time settings for the mesh.
      * \tparam VertexDataT   Data stored for each vertex. Defaults to pcl::NoData.
      * \tparam HalfEdgeDataT Data stored for each half-edge. Defaults to pcl::NoData.
      * \tparam EdgeDataT     Data stored for each edge. Defaults to pcl::NoData.
      * \tparam FaceDataT     Data stored for each face. Defaults to pcl::NoData.
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class VertexDataT   = pcl::geometry::NoData,
              class HalfEdgeDataT = pcl::geometry::NoData,
              class EdgeDataT     = pcl::geometry::NoData,
              class FaceDataT     = pcl::geometry::NoData>
    struct DefaultMeshTraits
    {
      typedef VertexDataT   VertexData;
      typedef HalfEdgeDataT HalfEdgeData;
      typedef EdgeDataT     EdgeData;
      typedef FaceDataT     FaceData;

      /** \brief Specifies wether the mesh is manifold or not (only non-manifold vertices can be represented). */
      typedef boost::false_type IsManifold;
    };
  } // End namespace geometry
} // End namespace pcl

#endif // PCL_GEOMETRY_MESH_TRAITS_H

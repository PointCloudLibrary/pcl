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

#ifndef PCL_IN_HAND_SCANNER_COMMON_FUNCTIONS_HPP
#define PCL_IN_HAND_SCANNER_COMMON_FUNCTIONS_HPP

#include <pcl/geometry/impl/mesh_base.hpp>
#include <algorithm>

// Keep these functions in here until they are properly tested
namespace pcl
{

  /** \brief Compute the 3D (X-Y-Z) centroid of the mesh and return it as a 3D vector.
    * \param[in] mesh the input mesh
    * \param[out] centroid the output centroid
    * \return number of valid points used to determine the centroid.
    * \note if return value is 0, the centroid is not changed, thus not valid.
    * \note no check for NaN or Inf is performed!
    * \ingroup common
    */
  template <class VertexDataT, class FaceDataT, class HalfEdgeDataT, class ScalarT> inline unsigned int
  compute3DCentroid (const MeshBase <VertexDataT, FaceDataT, HalfEdgeDataT>& mesh,
                     Eigen::Matrix<ScalarT, 4, 1>&                           centroid)
  {
    typedef MeshBase <VertexDataT, FaceDataT, HalfEdgeDataT> Mesh;
    typedef typename Mesh::VertexConstIterator               VertexConstIterator;
    typedef Eigen::Matrix<ScalarT, 4, 1>                     Vec4;

    ScalarT x = 0.;
    ScalarT y = 0.;
    ScalarT z = 0.;

    for (VertexConstIterator it = mesh.beginVertexes (); it!=mesh.endVertexes (); ++it)
    {
      x += it->x;
      y += it->y;
      z += it->z;
    }

    ScalarT n = static_cast <ScalarT> (mesh.sizeVertexes ());

    if (mesh.sizeVertexes ()) centroid = Vec4 (x/n, y/n, z/n, 1.);

    return (mesh.sizeVertexes ());
  }
} // End namespace pcl

#endif // PCL_IN_HAND_SCANNER_COMMON_FUNCTIONS_HPP

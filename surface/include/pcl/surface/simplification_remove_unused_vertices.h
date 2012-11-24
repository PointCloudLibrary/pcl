/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_SURFACE_SIMPLIFICATION_REMOVE_UNUSED_VERTICES_H_
#define PCL_SURFACE_SIMPLIFICATION_REMOVE_UNUSED_VERTICES_H_

#include <pcl/surface/boost.h>
#include <pcl/PolygonMesh.h>
#include <pcl/pcl_macros.h>

namespace pcl
{
  namespace surface
  {
    class PCL_EXPORTS SimplificationRemoveUnusedVertices
    {
      public:
        typedef boost::shared_ptr<SimplificationRemoveUnusedVertices> Ptr;
        typedef boost::shared_ptr<const SimplificationRemoveUnusedVertices> ConstPtr;

        /** \brief Constructor. */
        SimplificationRemoveUnusedVertices () {};
        /** \brief Destructor. */
        ~SimplificationRemoveUnusedVertices () {};

        /** \brief Simply a polygonal mesh.
          * \param[in] input the input mesh
          * \param[out] output the output mesh
          */
        inline void
        simplify (const pcl::PolygonMesh& input, pcl::PolygonMesh& output)
        {
          std::vector<int> indices;
          simplify (input, output, indices);
        }

        /** \brief Perform simplification (remove unused vertices).
          * \param[in] input the input mesh
          * \param[out] output the output mesh
          * \param[out] indices the resultant vector of indices
          */
        void
        simplify (const pcl::PolygonMesh& input, pcl::PolygonMesh& output, std::vector<int>& indices);

    };
  }
}

#endif /* PCL_SURFACE_SIMPLIFICATION_REMOVE_UNUSED_VERTICES_H_ */

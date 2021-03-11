/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#pragma once

#include <pcl/point_types.h>
#include <pcl/surface/processing.h>

namespace pcl
{

  /** \brief The ear clipping triangulation algorithm.
    * The code is inspired by Flavien Brebion implementation, which is
    * in n^3 and does not handle holes.
    * \author Nicolas Burrus
    * \ingroup surface
    */
  class PCL_EXPORTS EarClipping : public MeshProcessing
  {
    public:
      using Ptr = shared_ptr<EarClipping>;
      using ConstPtr = shared_ptr<const EarClipping>;

      using MeshProcessing::input_mesh_;
      using MeshProcessing::initCompute;
      /** \brief Empty constructor */
      EarClipping () = default;

    protected:
      /** \brief a Pointer to the point cloud data. */
      pcl::PointCloud<pcl::PointXYZ>::Ptr points_;

      /** \brief This method should get called before starting the actual computation. */
      bool
      initCompute () override;

      /** \brief The actual surface reconstruction method. 
        * \param[out] output the output polygonal mesh 
        */
      void
      performProcessing (pcl::PolygonMesh &output) override;

      /** \brief Triangulate one polygon. 
        * \param[in] vertices the set of vertices
        * \param[out] output the resultant polygonal mesh
        */
      void
      triangulate (const Vertices& vertices, PolygonMesh& output);

      /** \brief Compute the signed area of a polygon. 
        * \param[in] vertices the vertices representing the polygon 
        */
      float
      area (const Indices& vertices);

      /** \brief Compute the signed area of a polygon.
        * \param[in] vertices the vertices representing the polygon
        */
      template <typename T = pcl::index_t, std::enable_if_t<!std::is_same<T, std::uint32_t>::value, pcl::index_t> = 0>
      PCL_DEPRECATED(1, 14, "This method creates a useless copy of the vertices vector. Use  area method which accepts Indices instead")
      float
      area (const std::vector<std::uint32_t>& vertices)
      {
        return area(Indices (vertices.cbegin(), vertices.cend()));
      }

      /** \brief Check if the triangle (u,v,w) is an ear. 
        * \param[in] u the first triangle vertex 
        * \param[in] v the second triangle vertex 
        * \param[in] w the third triangle vertex 
        * \param[in] vertices a set of input vertices
        */
      bool
      isEar (int u, int v, int w, const Indices& vertices);

      /** \brief Check if the triangle (u,v,w) is an ear. 
        * \param[in] u the first triangle vertex 
        * \param[in] v the second triangle vertex 
        * \param[in] w the third triangle vertex 
        * \param[in] vertices a set of input vertices
        */
      template <typename T = pcl::index_t, std::enable_if_t<!std::is_same<T, std::uint32_t>::value, pcl::index_t> = 0>
      PCL_DEPRECATED(1, 14, "This method creates a useless copy of the vertices vector. Use  isEar method which accepts Indices instead")
      bool
      isEar (int u, int v, int w, const std::vector<std::uint32_t>& vertices)
      {
        return isEar(u, v, w, Indices (vertices.cbegin(), vertices.cend()));
      }

      /** \brief Check if p is inside the triangle (u,v,w). 
        * \param[in] u the first triangle vertex 
        * \param[in] v the second triangle vertex 
        * \param[in] w the third triangle vertex 
        * \param[in] p the point to check
        */
      bool
      isInsideTriangle (const Eigen::Vector3f& u,
                        const Eigen::Vector3f& v,
                        const Eigen::Vector3f& w,
                        const Eigen::Vector3f& p);

      /** \brief Compute the cross product between 2D vectors.
       * \param[in] p1 the first 2D vector
       * \param[in] p2 the first 2D vector
       */
      float
      crossProduct (const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) const
      {
        return p1[0]*p2[1] - p1[1]*p2[0];
      }

  };

}

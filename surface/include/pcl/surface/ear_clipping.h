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

#ifndef PCL_SURFACE_EAR_CLIPPING_H_
#define PCL_SURFACE_EAR_CLIPPING_H_

#include <pcl/point_types.h>
#include <pcl/surface/mesh_processing.h>

namespace pcl
{

  /** \brief The ear clipping triangulation algorithm.
    * The code is inspired by Flavien Brebion implementation, which is
    * in n^3 and does not handle holes.
    * \author Nicolas Burrus
    * \ingroup surface
    */
  class EarClipping : public MeshProcessing
  {
    public:
      using MeshProcessing::input_mesh_;
      using MeshProcessing::initCompute;
      /** \brief Empty constructor */
      EarClipping () : MeshProcessing (), points_ ()
        { };

    protected:
      PointCloud<PointXYZ>::Ptr points_;

      bool
      initCompute ();

      void
      performReconstruction (pcl::PolygonMesh &output);

      /** \brief Triangulate one polygon. */
      void
      triangulate (const Vertices& vertices, PolygonMesh& output);

      /** \brief Compute the signed area of a polygon. */
      float
      area (const std::vector<uint32_t>& vertices);

      /** \brief Check if the triangle (u,v,w) is an ear. */
      bool
      isEar (int u, int v, int w, const std::vector<uint32_t>& vertices);

      /** \brief Check if p is inside the triangle (u,v,w). */
      bool
      isInsideTriangle (const PointXY& u, const PointXY& v, const PointXY& w,const PointXY& p);

      /** \brief Project a 3D point to 2D. */
      PointXY
      toPointXY (const PointXYZ& p) const;

      /** \brief Compute the cross product between 2D vectors. */
      float
      crossProduct (const PointXY& p1, const PointXY& p2) const;

      /** \brief Subtract two 2D vectors. */
      PointXY
      difference (const PointXY& p1, const PointXY& p2) const;
  };

}

#endif  // #ifndef PCL_SURFACE_EAR_CLIPPING_H_

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
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
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
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
 * @author thomas.moerwald
 *
 */

#ifndef NURBS_TRIANGULATION_H
#define NURBS_TRIANGULATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/openNURBS/opennurbs.h>
#include <pcl/PolygonMesh.h>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief Triangulation functions for NURBS surfaces and curves */
    class Triangulation
    {

    private:
      static void
      createIndices (std::vector<pcl::Vertices> &vertices, unsigned vidx, unsigned segX, unsigned segY);

      static void
      createVertices (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float x0, float y0, float z0, float width,
                      float height, unsigned segX, unsigned segY);

    public:
      /** \brief Converts an openNURBS NurbsSurface to a pcl::PolygonMesh by sampling the NURBS according to the resolution specified.
       *  \param[in] nurbs The openNURBS surface.
       *  \param[out] mesh The pcl::PolygonMesh
       *  \param[in] resolution mesh resolution (number of vertices along each of the two dimensions of the surface. */
      static void
      convert (const ON_NurbsSurface &nurbs, PolygonMesh &mesh, unsigned resolution);

    };
  }
}

#endif

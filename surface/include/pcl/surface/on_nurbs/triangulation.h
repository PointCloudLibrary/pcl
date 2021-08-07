/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * 
 *
 */

#pragma once

#include <pcl/pcl_exports.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/3rdparty/opennurbs/opennurbs.h>
#include <pcl/PolygonMesh.h>

#include <pcl/surface/on_nurbs/nurbs_data.h>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief Functions for NURBS surface triangulation, trimming and curve sampling. */
    class PCL_EXPORTS Triangulation
    {
      protected:
        /** \brief Create indices for triangulation. */
        static void
        createIndices (std::vector<pcl::Vertices> &vertices, unsigned vidx, unsigned segX, unsigned segY);

        /** \brief Create vertices (cloud) for triangulation. */
        static void
        createVertices (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float x0, float y0, float z0, float width,
                        float height, unsigned segX, unsigned segY);

      public:
        static bool
        isInside(const ON_NurbsCurve &curve, const pcl::PointXYZ &v);

        //      /** \brief Converts an NurbsObject to a pcl::PolygonMesh by sampling all NURBS according to the resolution specified.
        //       *  \param[in] object The NURBS object.
        //       *  \param[out] mesh The pcl::PolygonMesh
        //       *  \param[in] resolution mesh resolution (number of vertices along each of the two dimensions of the surface. */
        //      static void
        //      convertObject2PolygonMesh (const NurbsObject &object, PolygonMesh &mesh, unsigned resolution);

        /** \brief Converts an openNURBS NurbsSurface to a pcl::PolygonMesh by sampling the NURBS according to the resolution specified.
         *  \param[in] nurbs The openNURBS surface.
         *  \param[out] mesh The pcl::PolygonMesh
         *  \param[in] resolution mesh resolution (number of vertices along each of the two dimensions of the surface. */
        static void
        convertSurface2PolygonMesh (const ON_NurbsSurface &nurbs, PolygonMesh &mesh, unsigned resolution);

        /** \brief Converts an openNURBS NurbsSurface to a pcl::PolygonMesh by sampling the NURBS according to the resolution specified.
         *  \param[in] nurbs The nurbs surface.
         *  \param[in] curve The nurbs curve for trimming (direction of curve decides if inside or outside is trimmed)
         *  \param[out] mesh The pcl::PolygonMesh
         *  \param[in] resolution mesh resolution (number of vertices along each of the two dimensions of the surface. */
        static void
        convertTrimmedSurface2PolygonMesh (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve, PolygonMesh &mesh,
                                           unsigned resolution);
        static void
        convertTrimmedSurface2PolygonMesh (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve, PolygonMesh &mesh,
                                           unsigned resolution, vector_vec3d &start, vector_vec3d &end);

        /** \brief Converts an openNURBS NurbsSurface to a point-cloud (vertices) and an vertex-index list
         * by sampling the NURBS according to the resolution specified.
         *  \param[in] nurbs The openNURBS surface.
         *  \param[out] cloud The actual vertices (point-cloud).
         *  \param[out] vertices The vertex-indices for a polygon mesh.
         *  \param[in] resolution mesh resolution (number of vertices along each of the two dimensions of the surface. */
        static void
        convertSurface2Vertices (const ON_NurbsSurface &nurbs, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 std::vector<pcl::Vertices> &vertices, unsigned resolution);

        /** \brief Converts an openNURBS NurbsCurve to a sequence of points 'cloud',
         * by ELEMENT-WISE sampling of the curve according to the resolution specified.
         *  \param[in] nurbs The openNURBS surface.
         *  \param[out] cloud The actual vertices (point-cloud).
         *  \param[in] resolution number of sampling points within one NurbsCurve element. */
        static void
        convertCurve2PointCloud (const ON_NurbsCurve &nurbs, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                 unsigned resolution);

        /** \brief Converts an openNURBS NurbsCurve, defined on an NurbsSurface to a sequence of points 'cloud',
         * by ELEMENT-WISE sampling of the curve according to the resolution specified.
         *  \param[in] nurbs The openNURBS surface.
         *  \param[out] cloud The actual vertices (point-cloud).
         *  \param[in] resolution number of sampling points within one NurbsCurve element. */
        static void
        convertCurve2PointCloud (const ON_NurbsCurve &curve, const ON_NurbsSurface &surf,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, unsigned resolution);

    };
  }
}

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <pcl/pcl_macros.h>
#include <pcl/surface/vtk_smoothing/vtk.h>

namespace pcl
{
  struct PolygonMesh;

  class PCL_EXPORTS VTKUtils
  {
    public:
      /** \brief Convert a PCL PolygonMesh to a VTK vtkPolyData.
        * \param[in] triangles PolygonMesh to be converted to vtkPolyData, stored in the object.
        * \param[out] triangles_out_vtk
        */
      static int
      convertToVTK (const pcl::PolygonMesh &triangles, vtkSmartPointer<vtkPolyData> &triangles_out_vtk);

      /** \brief Convert the vtkPolyData object back to PolygonMesh.
        * \param[in] vtk_polygons
        * \param[out] triangles the PolygonMesh to store the vtkPolyData in.
        */
      static void
      convertToPCL (vtkSmartPointer<vtkPolyData> &vtk_polygons, pcl::PolygonMesh &triangles);

      /** \brief Convert vtkPolyData object to a PCL PolygonMesh
        * \param[in] poly_data Pointer (vtkSmartPointer) to a vtkPolyData object
        * \param[out] mesh PCL Polygon Mesh to fill
        * \return Number of points in the point cloud of mesh.
        */
      static int
      vtk2mesh (const vtkSmartPointer<vtkPolyData>& poly_data, pcl::PolygonMesh& mesh);

      /** \brief Convert a PCL PolygonMesh to a vtkPolyData object
        * \param[in] mesh Reference to PCL Polygon Mesh
        * \param[out] poly_data Pointer (vtkSmartPointer) to a vtkPolyData object
        * \return Number of points in the point cloud of mesh.
        */
      static int
      mesh2vtk (const pcl::PolygonMesh& mesh, vtkSmartPointer<vtkPolyData> &poly_data);
  };
}

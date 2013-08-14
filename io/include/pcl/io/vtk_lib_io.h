/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 * $Id$
 *
 */

#ifndef PCL_IO_VTK_LIB_IO_H_
#define PCL_IO_VTK_LIB_IO_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/pcl_macros.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image_planar.h>

// Ignore warnings in the above headers
#ifdef __GNUC__
#pragma GCC system_header 
#endif
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkOBJReader.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkPNGReader.h>
#include <vtkImageData.h>
#include <vtkPolyDataNormals.h>

namespace pcl
{
  namespace io
  {
    /** \brief Convert vtkPolyData object to a PCL PolygonMesh
      * \param[in] poly_data Pointer (vtkSmartPointer) to a vtkPolyData object
      * \param[out] mesh PCL Polygon Mesh to fill
      * \return Number of points in the point cloud of mesh.
      */
    PCL_EXPORTS int
    vtk2mesh (const vtkSmartPointer<vtkPolyData>& poly_data, 
              pcl::PolygonMesh& mesh);

    /** \brief Convert vtkPolyData object to a PCL TextureMesh
      * \note In addition to the vtk2mesh (const vtkSmartPointer<vtkPolyData>&, pcl::PolygonMesh&)
      * method, it fills the mesh with the uv-coordinates.
      * \param[in] poly_data Pointer (vtkSmartPointer) to a vtkPolyData object
      * \param[out] mesh PCL TextureMesh to fill
      * \return Number of points in the point cloud of mesh.
      */
    PCL_EXPORTS int
    vtk2mesh (const vtkSmartPointer<vtkPolyData>& poly_data,
              pcl::TextureMesh& mesh);


    /** \brief Convert a PCL PolygonMesh to a vtkPolyData object
      * \param[in] mesh Reference to PCL Polygon Mesh
      * \param[out] poly_data Pointer (vtkSmartPointer) to a vtkPolyData object
      * \return Number of points in the point cloud of mesh.
      */
    PCL_EXPORTS int
    mesh2vtk (const pcl::PolygonMesh& mesh, 
              vtkSmartPointer<vtkPolyData>& poly_data);

    /** \brief Load a \ref PolygonMesh object given an input file name, based on the file extension
      * \param[in] file_name the name of the file containing the polygon data
      * \param[out] mesh the object that we want to load the data in 
      * \ingroup io
      */ 
    PCL_EXPORTS int
    loadPolygonFile (const std::string &file_name, 
                     pcl::PolygonMesh& mesh);

    /** \brief Save a \ref PolygonMesh object given an input file name, based on the file extension
      * \param[in] file_name the name of the file to save the data to
      * \param[in] mesh the object that contains the data
      * \ingroup io
      */
    PCL_EXPORTS int
    savePolygonFile (const std::string &file_name, 
                     const pcl::PolygonMesh& mesh);

    /** \brief Load a VTK file into a \ref PolygonMesh object
      * \param[in] file_name the name of the file that contains the data
      * \param[out] mesh the object that we want to load the data in 
      * \ingroup io
      */
    PCL_EXPORTS int
    loadPolygonFileVTK (const std::string &file_name, 
                        pcl::PolygonMesh& mesh);

    /** \brief Load a PLY file into a \ref PolygonMesh object
      * \param[in] file_name the name of the file that contains the data
      * \param[out] mesh the object that we want to load the data in 
      * \ingroup io
      */
    PCL_EXPORTS int
    loadPolygonFilePLY (const std::string &file_name, 
                        pcl::PolygonMesh& mesh);

    /** \brief Load an OBJ file into a \ref PolygonMesh object
      * \param[in] file_name the name of the file that contains the data
      * \param[out] mesh the object that we want to load the data in 
      * \ingroup io
      */
    PCL_EXPORTS int
    loadPolygonFileOBJ (const std::string &file_name, 
                        pcl::PolygonMesh& mesh);

    /** \brief Load an OBJ file into a \ref TextureMesh object.
      * \note In addition to the loadPolygonFileOBJ (const std::string, pcl::PolygonMesh&)
      * method, this method also loads the uv-coordinates from the file. It does not
      * load the material information.
      * \param[in] file_name the name of the file that contains the data
      * \param[out] mesh the object that we want to load the data in
      * \ingroup io
      */
    PCL_EXPORTS int
    loadPolygonFileOBJ (const std::string &file_name,
                        pcl::TextureMesh& mesh);


    /** \brief Load an STL file into a \ref PolygonMesh object
      * \param[in] file_name the name of the file that contains the data
      * \param[out] mesh the object that we want to load the data in 
      * \ingroup io
      */
    PCL_EXPORTS int
    loadPolygonFileSTL (const std::string &file_name, 
                        pcl::PolygonMesh& mesh);

    /** \brief Save a \ref PolygonMesh object into a VTK file
      * \param[in] file_name the name of the file to save the data to
      * \param[in] mesh the object that contains the data
      * \ingroup io
      */
    PCL_EXPORTS int
    savePolygonFileVTK (const std::string &file_name, 
                        const pcl::PolygonMesh& mesh);

    /** \brief Save a \ref PolygonMesh object into a PLY file
      * \param[in] file_name the name of the file to save the data to
      * \param[in] mesh the object that contains the data
      * \ingroup io
      */
    PCL_EXPORTS int
    savePolygonFilePLY (const std::string &file_name, 
                        const pcl::PolygonMesh& mesh);

    /** \brief Save a \ref PolygonMesh object into an STL file
      * \param[in] file_name the name of the file to save the data to
      * \param[in] mesh the object that contains the data
      * \ingroup io
      */
    PCL_EXPORTS int
    savePolygonFileSTL (const std::string &file_name, 
                        const pcl::PolygonMesh& mesh);

    /** \brief Write a \ref RangeImagePlanar object to a PNG file
      * \param[in] file_name the name of the file to save the data to
      * \param[in] range_image the object that contains the data
      * \ingroup io
      */
    PCL_EXPORTS void
    saveRangeImagePlanarFilePNG (const std::string &file_name,
                                 const pcl::RangeImagePlanar& range_image);

    /** \brief Convert a pcl::PointCloud object to a VTK PolyData one.
      * \param[in] cloud the input pcl::PointCloud object
      * \param[out] polydata the resultant VTK PolyData object
      * \ingroup io
      */
    template <typename PointT> void
    pointCloudTovtkPolyData (const pcl::PointCloud<PointT>& cloud, 
                             vtkPolyData* const polydata);

    /** \brief Convert a PCLPointCloud2 object to a VTK PolyData object.
      * \param[in] cloud the input PCLPointCloud2Ptr object
      * \param[out] poly_data the resultant VTK PolyData object
      * \ingroup io
      */
    PCL_EXPORTS void
    pointCloudTovtkPolyData(const pcl::PCLPointCloud2Ptr& cloud, vtkSmartPointer<vtkPolyData>& poly_data);

    /** \brief Convert a pcl::PointCloud object to a VTK StructuredGrid one.
      * \param[in] cloud the input pcl::PointCloud object
      * \param[out] structured_grid the resultant VTK StructuredGrid object
      * \ingroup io
      */
    template <typename PointT> void
    pointCloudTovtkStructuredGrid (const pcl::PointCloud<PointT>& cloud, 
                                   vtkStructuredGrid* const structured_grid);

    /** \brief Convert a VTK PolyData object to a pcl::PointCloud one.
      * \param[in] polydata the input VTK PolyData object
      * \param[out] cloud the resultant pcl::PointCloud object
      * \ingroup io
      */
    template <typename PointT> void
    vtkPolyDataToPointCloud (vtkPolyData* const polydata, 
                             pcl::PointCloud<PointT>& cloud);

    /** \brief Convert a VTK StructuredGrid object to a pcl::PointCloud one.
      * \param[in] structured_grid the input VTK StructuredGrid object
      * \param[out] cloud the resultant pcl::PointCloud object
      * \ingroup io
      */
    template <typename PointT> void
    vtkStructuredGridToPointCloud (vtkStructuredGrid* const structured_grid, 
                                   pcl::PointCloud<PointT>& cloud);

  }
}

#include <pcl/io/impl/vtk_lib_io.hpp>

#endif /* PLC_IO_VTK_LIB_IO_H_ */

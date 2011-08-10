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

#ifndef PLC_IO_VTK_LIB_IO_H_
#define PLC_IO_VTK_LIB_IO_H_

#include <vtkSmartPointer.h>
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
#include <vtkOBJExporter.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkVRMLExporter.h>
#include <boost/filesystem.hpp>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PolygonMesh.h"
#include "pcl/pcl_macros.h"
#include "pcl/ros/conversions.h"

#include "pcl/io/pcd_io.h"

namespace pcl
{
  namespace io
  {

//    /** \brief Saves a PolygonMesh in ascii VTK format.
//      *        (For compability with (old/pcl_native) vtk_io.h!)
//      * \param file_name the name of the file to write to disk
//      * \param triangles the polygonal mesh to save
//      * \param precision the output ASCII precision
//      * \ingroup io
//      */
//    PCL_EXPORTS int
//    saveVTKFile (const std::string &file_name, const pcl::PolygonMesh &triangles, unsigned precision = 5)
//    {
//
//    }


    /// CONVERSION FUNCTIONS //////////////////////////////////////////////////

    /**
     * \brief Convert vtkPolyData object to a PCL PolygonMesh
     * \param poly_data Pointer (vtkSmartPointer) to a vtkPolyData object
     * \param mesh PCL Polygon Mesh to fill
     * \return Number of points in the point cloud of mesh.
     */
    int
    vtk2mesh (const vtkSmartPointer<vtkPolyData>& poly_data, pcl::PolygonMesh& mesh);

    /**
     * \brief Convert a PCL PolygonMesh to a vtkPolyData object
     * \param mesh Reference to PCL Polygon Mesh
     * \param poly_data Pointer (vtkSmartPointer) to a vtkPolyData object
     * \return Number of points in the point cloud of mesh.
     */
    int
    mesh2vtk (const pcl::PolygonMesh& mesh, vtkSmartPointer<vtkPolyData>& poly_data);


    int
    loadPolygonFile (const std::string &file_name, pcl::PolygonMesh& mesh);

    int
    savePolygonFile (const std::string &file_name, const pcl::PolygonMesh& mesh);



    int
    loadPolygonFileVTK (const std::string &file_name, pcl::PolygonMesh& mesh);

    int
    loadPolygonFilePLY (const std::string &file_name, pcl::PolygonMesh& mesh);

    int
    loadPolygonFileOBJ (const std::string &file_name, pcl::PolygonMesh& mesh);

    int
    loadPolygonFileSTL (const std::string &file_name, pcl::PolygonMesh& mesh);



    int
    savePolygonFileVTK (const std::string &file_name, const pcl::PolygonMesh& mesh);

    int
    savePolygonFilePLY (const std::string &file_name, const pcl::PolygonMesh& mesh);

    int
    savePolygonFileOBJ (const std::string &file_name, const pcl::PolygonMesh& mesh);

    int
    savePolygonFileSTL (const std::string &file_name, const pcl::PolygonMesh& mesh);

    int
    savePolygonFileWRL (const std::string &file_name, const pcl::PolygonMesh& mesh);

    inline int
    savePolygonFileVRML (const std::string &file_name, const pcl::PolygonMesh& mesh)
    {
      return savePolygonFileWRL (file_name, mesh);
    }



    /**
     * TODO documentation
     * TODO proper building the library (depending on whether or not vtk was found)
     * TODO writing files
     * TODO more importers/exporters
     * TODO IMPORTANT colors and other scalars
     * TODO DXF files
     *
     */
  }
}

#endif /* PLC_IO_VTK_LIB_IO_H_ */

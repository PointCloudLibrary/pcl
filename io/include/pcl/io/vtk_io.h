/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef PCL_IO_VTK_IO_H_
#define PCL_IO_VTK_IO_H_

#include <pcl/pcl_macros.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>

// Please do not add any functions tha depend on VTK structures to this file!
// Use vtk_io_lib.h instead.

namespace pcl
{
  namespace io
  {
    /** \brief Saves a PolygonMesh in ascii VTK format. 
      * \param[in] file_name the name of the file to write to disk
      * \param[in] triangles the polygonal mesh to save
      * \param[in] precision the output ASCII precision
      * \ingroup io
      */
    PCL_EXPORTS int 
    saveVTKFile (const std::string &file_name, const pcl::PolygonMesh &triangles, unsigned precision = 5);

    /** \brief Saves a PointCloud in ascii VTK format. 
      * \param[in] file_name the name of the file to write to disk
      * \param[in] cloud the point cloud to save
      * \param[in] precision the output ASCII precision
      * \ingroup io
      */
    PCL_EXPORTS int 
    saveVTKFile (const std::string &file_name, const pcl::PCLPointCloud2 &cloud, unsigned precision = 5);
  }
}

#endif  //#ifndef PCL_IO_VTK_IO_H_

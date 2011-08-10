/*
 * Software License Agreement (BSD License)
 *
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

#ifndef VTK_SMOOTHER_H_
#define VTK_SMOOTHER_H_

#include <pcl/pcl_base.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace pcl
{
  namespace surface
  {
    /** \brief VtkSmoother is a wrapper around some subdivision and filter methods from VTK.
      * \author Greg Long(?), Dirk Holz
      * \ingroup surface
      *
      * TODO: inheritance from some PCLBase-like interface for PolygonMesh
      * TODO: wrap more vtk functionality in here
      * TODO: Do we want to wrap any VTK functionality anyway or are we just going to provide conversion as in mesh2vtk and vtk2mesh?
      */
    class VtkSmoother
    {
    public:
      VtkSmoother ();
      ~VtkSmoother ();

      /** \brief Convert (input) mesh into the vtk data structure. */
      int
      convertToVTK (const pcl::PolygonMesh &triangles);

      /** \brief Subdivision using a given filter. TODO: make ENUM for all implemented filters. */
      void
      subdivideMesh (int filter);

      /** \brief Smooth mesh vertices. */
      void
      smoothMeshWindowedSinc (int num_iter, float feature_angle, float pass_band);

      /** \brief Apply Laplacian filter. */
      void
      smoothMeshLaplacian (int num_iter);

      /** \brief Convert (output) mesh back to PCL PolygonMesh. */
      void
      convertToPCL (pcl::PolygonMesh &triangles);

    private:
      vtkSmartPointer<vtkPolyData> vtk_polygons;
    };
  }
}

#endif /* VTK_SMOOTHER_H_ */


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

#ifndef VTK_MESH_SMOOTHING_LAPLACIAN_H_
#define VTK_MESH_SMOOTHING_LAPLACIAN_H_

#include "pcl/surface/mesh_processing.h"
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>


namespace pcl
{
  class MeshSmoothingLaplacianVTK : public MeshProcessing
  {
    public:
      /** \brief Empty constructor */
      MeshSmoothingLaplacianVTK ();

      /** \brief Set the number of iterations for the smoothing filter.
       * \param[in] num_iter the number of iterations
       */
      inline void
      setNumIter (int num_iter)
      {
        num_iter_ = num_iter;
      };

      /** \brief Get the number of iterations. */
      inline int
      getNumIter ()
      {
        return num_iter_;
      };

    protected:
      void
      performReconstruction (pcl::PolygonMesh &output);

    private:
      vtkSmartPointer<vtkPolyData> vtk_polygons_;
      int num_iter_;
  };
}
#endif /* VTK_MESH_SMOOTHING_LAPLACIAN_H_ */

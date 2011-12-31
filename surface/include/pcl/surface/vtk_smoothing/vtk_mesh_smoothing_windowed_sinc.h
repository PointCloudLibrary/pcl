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

#ifndef VTK_MESH_SMOOTHING_WINDOWED_SINC_H_
#define VTK_MESH_SMOOTHING_WINDOWED_SINC_H_

#include <pcl/surface/processing.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>


namespace pcl
{
  class PCL_EXPORTS MeshSmoothingWindowedSincVTK : public MeshProcessing
  {
    public:
      /** \brief Empty constructor */
      MeshSmoothingWindowedSincVTK ();

      /** \brief Set the number of iterations for the smoothing filter.
        * \param[in] num_iter the number of iterations
        */
      inline void
      setNumIter (int num_iter)
      {
        num_iter_ = num_iter;
      };

      /** \brief Set the feature angle for sharp edge identification.
        * \param[in] feature_angle the feature angle.
        */
      inline void
      setFeatureAngle (float feature_angle)
      {
        feature_angle_ = feature_angle;
      };

      /** \brief Set the pass band value for windowed sinc filtering.
        * \param[in] pass_band value for the pass band.
        */
      inline void
      setPassBand (float pass_band)
      {
        pass_band_ = pass_band;
      };

      /** \brief Get the number of iterations. */
      inline int
      getNumIter ()
      {
        return num_iter_;
      };

      /** \brief Get the feature angle. */
      inline float
      getFeatureAngle ()
      {
        return feature_angle_;
      };

      /** \brief Get the pass band value. */
      inline float
      getPassBand ()
      {
        return pass_band_;
      };


    protected:
      void
      performProcessing (pcl::PolygonMesh &output);

    private:
      vtkSmartPointer<vtkPolyData> vtk_polygons_;
      int num_iter_;
      float feature_angle_;
      float pass_band_;
  };
}
#endif /* VTK_MESH_SMOOTHING_WINDOWED_SINC_H_ */

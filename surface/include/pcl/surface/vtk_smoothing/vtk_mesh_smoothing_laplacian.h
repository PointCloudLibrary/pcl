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

#include <pcl/surface/processing.h>
#include <pcl/surface/vtk_smoothing/vtk.h>

namespace pcl
{
  /** \brief PCL mesh smoothing based on the vtkSmoothPolyDataFilter algorithm from the VTK library.
    * Please check out the original documentation for more details on the inner workings of the algorithm
    * Warning: This wrapper does two fairly computationally expensive conversions from the PCL PolygonMesh
    * data structure to the vtkPolyData data structure and back.
    */
  class PCL_EXPORTS MeshSmoothingLaplacianVTK : public MeshProcessing
  {
    public:
      /** \brief Empty constructor that sets the values of the algorithm parameters to the VTK defaults */
      MeshSmoothingLaplacianVTK ()
        : num_iter_ (20)
        , convergence_ (0.0f)
        , relaxation_factor_ (0.01f)
        , feature_edge_smoothing_ (false)
        , feature_angle_ (45.f)
        , edge_angle_ (15.f)
        , boundary_smoothing_ (true)
      {};

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
      getNumIter () const
      {
        return num_iter_;
      };

      /** \brief Specify a convergence criterion for the iteration process. Smaller numbers result in more smoothing iterations.
       * \param[in] convergence convergence criterion for the Laplacian smoothing
       */
      inline void
      setConvergence (float convergence)
      {
        convergence_ = convergence;
      };

      /** \brief Get the convergence criterion. */
      inline float
      getConvergence () const
      {
        return convergence_;
      };

      /** \brief Specify the relaxation factor for Laplacian smoothing. As in all iterative methods,
       * the stability of the process is sensitive to this parameter.
       * In general, small relaxation factors and large numbers of iterations are more stable than larger relaxation
       * factors and smaller numbers of iterations.
       * \param[in] relaxation_factor the relaxation factor of the Laplacian smoothing algorithm
       */
      inline void
      setRelaxationFactor (float relaxation_factor)
      {
        relaxation_factor_ = relaxation_factor;
      };

      /** \brief Get the relaxation factor of the Laplacian smoothing */
      inline float
      getRelaxationFactor () const
      {
        return relaxation_factor_;
      };

      /** \brief Turn on/off smoothing along sharp interior edges.
       * \param[in] feature_edge_smoothing whether to enable/disable smoothing along sharp interior edges
       */
      inline void
      setFeatureEdgeSmoothing (bool feature_edge_smoothing)
      {
        feature_edge_smoothing_ = feature_edge_smoothing;
      };

      /** \brief Get the status of the feature edge smoothing */
      inline bool
      getFeatureEdgeSmoothing () const
      {
        return feature_edge_smoothing_;
      };

      /** \brief Specify the feature angle for sharp edge identification.
       * \param[in] feature_angle the angle threshold for considering an edge to be sharp
       */
      inline void
      setFeatureAngle (float feature_angle)
      {
        feature_angle_ = feature_angle;
      };

      /** \brief Get the angle threshold for considering an edge to be sharp */
      inline float
      getFeatureAngle () const
      {
        return feature_angle_;
      };

      /** \brief Specify the edge angle to control smoothing along edges (either interior or boundary).
       * \param[in] edge_angle the angle to control smoothing along edges
       */
      inline void
      setEdgeAngle (float edge_angle)
      {
        edge_angle_ = edge_angle;
      };

      /** \brief Get the edge angle to control smoothing along edges */
      inline float
      getEdgeAngle () const
      {
        return edge_angle_;
      };

      /** \brief Turn on/off the smoothing of vertices on the boundary of the mesh.
       * \param[in] boundary_smoothing decision whether boundary smoothing is on or off
       */
      inline void
      setBoundarySmoothing (bool boundary_smoothing)
      {
        boundary_smoothing_ = boundary_smoothing;
      };

      /** \brief Get the status of the boundary smoothing */
      inline bool
      getBoundarySmoothing () const
      {
        return boundary_smoothing_;
      }

    protected:
      void
      performProcessing (pcl::PolygonMesh &output) override;

    private:
      vtkSmartPointer<vtkPolyData> vtk_polygons_;

      /// Parameters
      int num_iter_;
      float convergence_;
      float relaxation_factor_;
      bool feature_edge_smoothing_;
      float feature_angle_;
      float edge_angle_;
      bool boundary_smoothing_;
  };
}

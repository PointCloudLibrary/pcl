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
#include <pcl/surface/vtk_smoothing/vtk.h>

namespace pcl
{
  /** \brief PCL mesh smoothing based on the vtkWindowedSincPolyDataFilter algorithm from the VTK library.
    * Please check out the original documentation for more details on the inner workings of the algorithm
    * Warning: This wrapper does two fairly computationally expensive conversions from the PCL PolygonMesh
    * data structure to the vtkPolyData data structure and back.
    */
  class PCL_EXPORTS MeshSmoothingWindowedSincVTK : public MeshProcessing
  {
    public:
      /** \brief Empty constructor that sets the values of the algorithm parameters to the VTK defaults */
      MeshSmoothingWindowedSincVTK ()
        : MeshProcessing (),
          num_iter_ (20),
          pass_band_ (0.1f),
          feature_edge_smoothing_ (false),
          feature_angle_ (45.f),
          edge_angle_ (15.f),
          boundary_smoothing_ (true),
          normalize_coordinates_ (false)
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
      getNumIter ()
      {
        return num_iter_;
      };

      /** \brief Set the pass band value for windowed sinc filtering.
        * \param[in] pass_band value for the pass band.
        */
      inline void
      setPassBand (float pass_band)
      {
        pass_band_ = pass_band;
      };

      /** \brief Get the pass band value. */
      inline float
      getPassBand ()
      {
        return pass_band_;
      };

      /** \brief Turn on/off coordinate normalization. The positions can be translated and scaled such that they fit
       * within a [-1, 1] prior to the smoothing computation. The default is off. The numerical stability of the
       * solution can be improved by turning normalization on. If normalization is on, the coordinates will be rescaled
       * to the original coordinate system after smoothing has completed.
       * \param[in] normalize_coordinates decision whether to normalize coordinates or not
       */
      inline void
      setNormalizeCoordinates (bool normalize_coordinates)
      {
        normalize_coordinates_ = normalize_coordinates;
      }

      /** \brief Get whether the coordinate normalization is active or not */
      inline bool
      getNormalizeCoordinates ()
      {
        return normalize_coordinates_;
      }

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
      getFeatureEdgeSmoothing ()
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
      getFeatureAngle ()
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
      getEdgeAngle ()
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
      getBoundarySmoothing ()
      {
        return boundary_smoothing_;
      }


    protected:
      void
      performProcessing (pcl::PolygonMesh &output);

    private:
      vtkSmartPointer<vtkPolyData> vtk_polygons_;
      int num_iter_;
      float pass_band_;
      bool feature_edge_smoothing_;
      float feature_angle_;
      float edge_angle_;
      bool boundary_smoothing_;
      bool normalize_coordinates_;
  };
}
#endif /* VTK_MESH_SMOOTHING_WINDOWED_SINC_H_ */

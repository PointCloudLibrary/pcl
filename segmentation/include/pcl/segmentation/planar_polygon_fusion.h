/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#ifndef PCL_SEGMENTATION_PLANAR_POLYGON_FUSION_H_
#define PCL_SEGMENTATION_PLANAR_POLYGON_FUSION_H_

#include <Eigen/Core>
#include <vector>
#include <pcl/geometry/planar_polygon.h>

namespace pcl
{
  /** \brief PlanarPolygonFusion takes a list of 2D planar polygons and
    * attempts to reduce them to a minimum set that best represents the scene,
    * based on various given comparators.
    */
  template <typename PointT>
  class PlanarPolygonFusion
  {
    public:
      /** \brief Constructor */
      PlanarPolygonFusion () : states_ () {}
     
      /** \brief Destructor */
      virtual ~PlanarPolygonFusion () {}

      /** \brief Reset the state (clean the list of planar models). */
      void 
      reset ()
      {
        states_.clear ();
      }
      
      /** \brief Set the list of 2D planar polygons to refine.
        * \param[in] input the list of 2D planar polygons to refine
        */
      void
      setInputPolygons (const std::vector<PlanarPolygon<PointT> > &input)
      {
        int statesize = states_.size ();
        states_.reserve (statesize + input.size ());
        for (size_t i = 0; i < input.size (); ++i)
        {
          states_[statesize].plane_coefficients = input[i].coefficients;
          states_[statesize].covariance = Eigen::Matrix4f::Identity ();
          states_[statesize].contour.resize (input[i].contour.size ());
          for (size_t j = 0; j < input[i].contour.size (); ++j)
          {
            states_[statesize].contour[j].first = input[i].contour[j];
            states_[statesize].contour[j].second = false;
          }
        }
      }

      /** \brief Set the list of 2D planar polygons to refine together with their covariance matrices
        * \param[in] input the list of 2D planar polygons to refine
        */
      void
      setInputPolygons (const std::vector<std::pair<PlanarPolygon<PointT>, Eigen::Matrix4f> > &input)
      {
        int statesize = states_.size ();
        states_.reserve (statesize + input.size ());
        for (size_t i = 0; i < input.size (); ++i)
        {
          states_[statesize].plane_coefficients = input[i].first.coefficients;
          states_[statesize].covariance = input[i].second;
          states_[statesize].contour.resize (input[i].first.contour.size ());
          for (size_t j = 0; j < input[i].first.contour.size (); ++j)
          {
            states_[statesize].contour[j].first = input[i].first.contour[j];
            states_[statesize].contour[j].second = false;
          }
        }
      }

      /** \brief Refine the given input polygons based on the given comparators.
        * \param[out] output the resultant set of merged polygons 
        */ 
      void
      refine (std::vector<PlanarPolygon<PointT> > &output);

    protected:
      /** \brief This class represents the state of a polygonal plane internally. */
      struct PlaneState
      {
        /** \brief Plane parameters: normal.x, normal.y, normal.z, distance */
        Eigen::Vector4f plane_coefficients;
        
        /** \brief The polygon vertices with labels that tell whether the next line segment is a real plane 
          * boundary or just unobserved. */
        std::vector<std::pair<PointT, bool> > contour;
        
        /** \brief The covariance matrix (how certain are we about this model) for this plane. */
        Eigen::Matrix4f covariance;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };
     
      /** \brief Internal list of planar states. */
      std::vector<PlaneState> states_;
  };
}

#endif // PCL_SEGMENTATION_PLANAR_POLYGON_FUSION_H_

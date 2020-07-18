/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2016-, Open Perception, Inc.
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
*
*/

#pragma once

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>


namespace pcl
{

  /** \brief FLARELocalReferenceFrameEstimation implements the Fast LocAl Reference framE algorithm
    * for local reference frame estimation as described here:
    *
    *  - A. Petrelli, L. Di Stefano,
    *    "A repeatable and efficient canonical reference for surface matching",
    *    3DimPVT, 2012
    *
    * FLARE algorithm is deployed in ReLOC algorithm proposed in:
    * 
    * Petrelli A., Di Stefano L., "Pairwise registration by local orientation cues", Computer Graphics Forum, 2015.
    *
    * \author Alioscia Petrelli
    * \ingroup features
    */
  template<typename PointInT, typename PointNT, typename PointOutT = ReferenceFrame, typename SignedDistanceT = float>
  class FLARELocalReferenceFrameEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    protected:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;
      using Feature<PointInT, PointOutT>::fake_surface_;
      using Feature<PointInT, PointOutT>::getClassName;

      using typename Feature<PointInT, PointOutT>::PointCloudIn;
      using typename Feature<PointInT, PointOutT>::PointCloudOut;

      using typename Feature<PointInT, PointOutT>::PointCloudInConstPtr;

      using typename Feature<PointInT, PointOutT>::KdTreePtr;

      using PointCloudSignedDistance = pcl::PointCloud<SignedDistanceT>;
      using PointCloudSignedDistancePtr = typename PointCloudSignedDistance::Ptr;

      using Ptr = shared_ptr<FLARELocalReferenceFrameEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const FLARELocalReferenceFrameEstimation<PointInT, PointNT, PointOutT> >;

    public:
      /** \brief Constructor. */
      FLARELocalReferenceFrameEstimation () :
        tangent_radius_ (0.0f),
        margin_thresh_ (0.85f),
        min_neighbors_for_normal_axis_ (6),
        min_neighbors_for_tangent_axis_ (6),
        sampled_surface_ (), 
        sampled_tree_ (),
        fake_sampled_surface_ (false)
      {
        feature_name_ = "FLARELocalReferenceFrameEstimation";
      }

      //Getters/Setters

      /** \brief Set the maximum distance of the points used to estimate the x_axis of the FLARE Reference Frame for a given point.
        *
        * \param[in] radius The search radius for x axis.
        */
      inline void
      setTangentRadius (float radius)
      {
        tangent_radius_ = radius;
      }

      /** \brief Get the maximum distance of the points used to estimate the x_axis of the FLARE Reference Frame for a given point.
        *
        * \return The search radius for x axis.
        */
      inline float
      getTangentRadius () const
      {
        return (tangent_radius_);
      }

      /** \brief Set the percentage of the search tangent radius after which a point is considered part of the support.
        *
        * \param[in] margin_thresh the percentage of the search tangent radius after which a point is considered part of the support.
        */
      inline void
      setMarginThresh (float margin_thresh)
      {
        margin_thresh_ = margin_thresh;
      }

      /** \brief Get the percentage of the search tangent radius after which a point is considered part of the support.
        *
        * \return The percentage of the search tangent radius after which a point is considered part of the support.
        */
      inline float
      getMarginThresh () const
      {
        return (margin_thresh_);
      }


      /** \brief Set min number of neighbours required for the computation of Z axis.
        *
        * \param[in] min_neighbors_for_normal_axis min number of neighbours required for the computation of Z axis.
        */
      inline void
      setMinNeighboursForNormalAxis (int min_neighbors_for_normal_axis)
      {
        min_neighbors_for_normal_axis_ = min_neighbors_for_normal_axis;
      }

      /** \brief Get min number of neighbours required for the computation of Z axis.
        *
        * \return min number of neighbours required for the computation of Z axis.
        */
      inline int
      getMinNeighboursForNormalAxis () const
      {
        return (min_neighbors_for_normal_axis_);
      }


      /** \brief Set min number of neighbours required for the computation of X axis.
        *
        * \param[in] min_neighbors_for_tangent_axis min number of neighbours required for the computation of X axis.
        */
      inline void
      setMinNeighboursForTangentAxis (int min_neighbors_for_tangent_axis)
      {
        min_neighbors_for_tangent_axis_ = min_neighbors_for_tangent_axis;
      }

      /** \brief Get min number of neighbours required for the computation of X axis.
        *
        * \return min number of neighbours required for the computation of X axis.
        */
      inline int
      getMinNeighboursForTangentAxis () const
      {
        return (min_neighbors_for_tangent_axis_);
      }


      /** \brief Provide a pointer to the dataset used for the estimation of X axis.
        * As the estimation of x axis is negligibly affected by surface downsampling,
        * this method lets to consider a downsampled version of surface_ in the estimation of x axis.  
        * This is optional, if this is not set, it will only use the data in the
        * surface_ cloud to estimate the x axis. 
        * \param[in] cloud a pointer to a PointCloud 
        */
      inline void 
      setSearchSampledSurface(const PointCloudInConstPtr &cloud)
      {
        sampled_surface_ = cloud;
        fake_sampled_surface_ = false;
      }

      /** \brief Get a pointer to the sampled_surface_ cloud dataset. */
      inline const PointCloudInConstPtr& 
      getSearchSampledSurface() const
      {
        return (sampled_surface_);
      }

      /** \brief Provide a pointer to the search object linked to sampled_surface.
        * \param[in] tree a pointer to the spatial search object linked to sampled_surface.
        */
      inline void 
      setSearchMethodForSampledSurface (const KdTreePtr &tree) { sampled_tree_ = tree; }

      /** \brief Get a pointer to the search method used for the extimation of x axis. */
      inline const KdTreePtr&  
      getSearchMethodForSampledSurface () const
      {
        return (sampled_tree_);
      }

      /** \brief Get the signed distances of the highest points from the fitted planes. */
      inline const std::vector<SignedDistanceT> & 
      getSignedDistancesFromHighestPoints () const 
      {
        return (signed_distances_from_highest_points_);
      }

    protected:
      /** \brief This method should get called before starting the actual computation. */
      bool
      initCompute () override;

      /** \brief This method should get called after the actual computation is ended. */
      bool
      deinitCompute () override;

      /** \brief Estimate the LRF descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] index the index of the point in input_
        * \param[out] lrf the resultant local reference frame
        * \return signed distance of the highest point from the fitted plane. Max if the lrf is not computable.
        */
      SignedDistanceT
      computePointLRF (const int index, Eigen::Matrix3f &lrf);

      /** \brief Abstract feature estimation method.
      * \param[out] output the resultant features
      */
      void
      computeFeature (PointCloudOut &output) override;


    private:
      /** \brief Radius used to find tangent axis. */
      float tangent_radius_;

      /** \brief Threshold that define if a support point is near the margins. */
      float margin_thresh_; 

      /** \brief Min number of neighbours required for the computation of Z axis. Otherwise, feature point normal is used. */
      int min_neighbors_for_normal_axis_;

      /** \brief Min number of neighbours required for the computation of X axis. Otherwise, a random X axis is set */
      int min_neighbors_for_tangent_axis_;

      /** \brief An input point cloud describing the surface that is to be used
        * for nearest neighbor searches for the estimation of X axis.
        */
      PointCloudInConstPtr sampled_surface_;

      /** \brief A pointer to the spatial search object used for the estimation of X axis. */
      KdTreePtr sampled_tree_;

      /** \brief Class for normal estimation. */
      NormalEstimation<PointInT, PointNT> normal_estimation_;

      /** \brief Signed distances of the highest points from the fitted planes.*/
      std::vector<SignedDistanceT> signed_distances_from_highest_points_;

      /** \brief If no sampled_surface_ is given, we use surface_ as the sampled surface. */
      bool fake_sampled_surface_;

  };

}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/flare.hpp>
#endif

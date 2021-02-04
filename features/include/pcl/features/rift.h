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

#pragma once

#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief RIFTEstimation estimates the Rotation Invariant Feature Transform descriptors for a given point cloud 
    * dataset containing points and intensity.  For more information about the RIFT descriptor, see:
    *
    *  Svetlana Lazebnik, Cordelia Schmid, and Jean Ponce. 
    *  A sparse texture representation using local affine regions. 
    *  In IEEE Transactions on Pattern Analysis and Machine Intelligence, volume 27, pages 1265-1278, August 2005.
    *
    * \author Michael Dixon
    * \ingroup features
    */

  template <typename PointInT, typename GradientT, typename PointOutT>
  class RIFTEstimation: public Feature<PointInT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;

      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::indices_;

      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::search_radius_;
      
      using PointCloudIn = pcl::PointCloud<PointInT>;
      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;

      using PointCloudGradient = pcl::PointCloud<GradientT>;
      using PointCloudGradientPtr = typename PointCloudGradient::Ptr;
      using PointCloudGradientConstPtr = typename PointCloudGradient::ConstPtr;

      using Ptr = shared_ptr<RIFTEstimation<PointInT, GradientT, PointOutT> >;
      using ConstPtr = shared_ptr<const RIFTEstimation<PointInT, GradientT, PointOutT> >;


      /** \brief Empty constructor. */
      RIFTEstimation () : gradient_ (), nr_distance_bins_ (4), nr_gradient_bins_ (8)
      {
        feature_name_ = "RIFTEstimation";
      };

      /** \brief Provide a pointer to the input gradient data
        * \param[in] gradient a pointer to the input gradient data
        */
      inline void 
      setInputGradient (const PointCloudGradientConstPtr &gradient) { gradient_ = gradient; };

      /** \brief Returns a shared pointer to the input gradient data */
      inline PointCloudGradientConstPtr 
      getInputGradient () const { return (gradient_); };

      /** \brief Set the number of bins to use in the distance dimension of the RIFT descriptor
        * \param[in] nr_distance_bins the number of bins to use in the distance dimension of the RIFT descriptor
        */
      inline void 
      setNrDistanceBins (int nr_distance_bins) { nr_distance_bins_ = nr_distance_bins; };

      /** \brief Returns the number of bins in the distance dimension of the RIFT descriptor. */
      inline int 
      getNrDistanceBins () const { return (nr_distance_bins_); };

      /** \brief Set the number of bins to use in the gradient orientation dimension of the RIFT descriptor
        * \param[in] nr_gradient_bins the number of bins to use in the gradient orientation dimension of the RIFT descriptor
        */
      inline void 
      setNrGradientBins (int nr_gradient_bins) { nr_gradient_bins_ = nr_gradient_bins; };

      /** \brief Returns the number of bins in the gradient orientation dimension of the RIFT descriptor. */
      inline int 
      getNrGradientBins () const { return (nr_gradient_bins_); };

      /** \brief Estimate the Rotation Invariant Feature Transform (RIFT) descriptor for a given point based on its 
        * spatial neighborhood of 3D points and the corresponding intensity gradient vector field
        * \param[in] cloud the dataset containing the Cartesian coordinates of the points
        * \param[in] gradient the dataset containing the intensity gradient at each point in \a cloud
        * \param[in] p_idx the index of the query point in \a cloud (i.e. the center of the neighborhood)
        * \param[in] radius the radius of the RIFT feature
        * \param[in] indices the indices of the points that comprise \a p_idx's neighborhood in \a cloud
        * \param[in] squared_distances the squared distances from the query point to each point in the neighborhood
        * \param[out] rift_descriptor the resultant RIFT descriptor
        */
      void 
      computeRIFT (const PointCloudIn &cloud, const PointCloudGradient &gradient, int p_idx, float radius,
                   const pcl::Indices &indices, const std::vector<float> &squared_distances, 
                   Eigen::MatrixXf &rift_descriptor);

    protected:

      /** \brief Estimate the Rotation Invariant Feature Transform (RIFT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface (), the gradient in 
        * setInputGradient (), and the spatial locator in setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains the RIFT feature estimates
        */
      void 
      computeFeature (PointCloudOut &output) override;

      /** \brief The intensity gradient of the input point cloud data*/
      PointCloudGradientConstPtr gradient_;

      /** \brief The number of distance bins in the descriptor. */
      int nr_distance_bins_;

      /** \brief The number of gradient orientation bins in the descriptor. */
      int nr_gradient_bins_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/rift.hpp>
#endif

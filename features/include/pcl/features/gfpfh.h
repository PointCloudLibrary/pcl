/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: gfpfh.h 1423 2011-06-21 09:51:32Z bouffa $
 *
 */

#pragma once

#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief @b GFPFHEstimation estimates the Global Fast Point Feature Histogram (GFPFH) descriptor for a given point
    * cloud dataset containing points and labels.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> R.B. Rusu, A. Holzbach, M. Beetz.
    *      Detecting and Segmenting Objects for Mobile Manipulation.
    *      In the S3DV Workshop of the 12th International Conference on Computer Vision (ICCV),
    *      2009.
    * </li>
    * </ul>
    *
    * \author Radu B. Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointLT, typename PointOutT>
  class GFPFHEstimation : public FeatureFromLabels<PointInT, PointLT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<GFPFHEstimation<PointInT, PointLT, PointOutT> >;
      using ConstPtr = shared_ptr<const GFPFHEstimation<PointInT, PointLT, PointOutT> >;
      using FeatureFromLabels<PointInT, PointLT, PointOutT>::feature_name_;
      using FeatureFromLabels<PointInT, PointLT, PointOutT>::getClassName;
      using FeatureFromLabels<PointInT, PointLT, PointOutT>::indices_;
      using FeatureFromLabels<PointInT, PointLT, PointOutT>::k_;
      using FeatureFromLabels<PointInT, PointLT, PointOutT>::search_parameter_;
      using FeatureFromLabels<PointInT, PointLT, PointOutT>::surface_;

      using FeatureFromLabels<PointInT, PointLT, PointOutT>::input_;
      using FeatureFromLabels<PointInT, PointLT, PointOutT>::labels_;

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;
      using PointCloudIn = typename Feature<PointInT, PointOutT>::PointCloudIn;

      /** \brief Empty constructor. */
      GFPFHEstimation () : 
        octree_leaf_size_ (0.01),
        number_of_classes_ (16),
        descriptor_size_ (PointOutT::descriptorSize ())
      {
        feature_name_ = "GFPFHEstimation";
      }

      /** \brief Set the size of the octree leaves.
        */
      inline void
      setOctreeLeafSize (double size) { octree_leaf_size_ = size; }

      /** \brief Get the sphere radius used for determining the neighbors. */
      inline double
      getOctreeLeafSize () { return (octree_leaf_size_); }

      /** \brief Return the empty label value. */
      inline std::uint32_t
      emptyLabel () const { return 0; }

      /** \brief Return the number of different classes. */
      inline std::uint32_t
      getNumberOfClasses () const { return number_of_classes_; }

      /** \brief Set the number of different classes.
       * \param n number of different classes.
       */
      inline void
      setNumberOfClasses (std::uint32_t n) { number_of_classes_ = n; }

      /** \brief Return the size of the descriptor. */
      inline int
      descriptorSize () const { return descriptor_size_; }

      /** \brief Overloaded computed method from pcl::Feature.
        * \param[out] output the resultant point cloud model dataset containing the estimated features
        */
      void
      compute (PointCloudOut &output);

    protected:

      /** \brief Estimate the Point Feature Histograms (PFH) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the PFH feature estimates
        */
      void 
      computeFeature (PointCloudOut &output) override;

      /** \brief Return the dominant label of a set of points. */
      std::uint32_t
      getDominantLabel (const pcl::Indices& indices);

      /** \brief Compute the fixed-length histograms of transitions. */
      void computeTransitionHistograms (const std::vector< std::vector<int> >& label_histograms,
                                        std::vector< std::vector<int> >& transition_histograms);

      /** \brief Compute the distance of each transition histogram to the mean. */
      void
      computeDistancesToMean (const std::vector< std::vector<int> >& transition_histograms,
                              std::vector<float>& distances);

      /** \brief Return the Intersection Kernel distance between two histograms. */
      float
      computeHIKDistance (const std::vector<int>& histogram,
                          const std::vector<float>& mean_histogram);

      /** \brief Compute the binned histogram of distance values. */
      void
      computeDistanceHistogram (const std::vector<float>& distances,
                                std::vector<float>& histogram);

      /** \brief Compute the mean histogram of the given set of histograms. */
      void
      computeMeanHistogram (const std::vector< std::vector<int> >& histograms,
                            std::vector<float>& mean_histogram);

    private:
      /** \brief Size of octree leaves. */
      double octree_leaf_size_;

      /** \brief Number of possible classes/labels. */
      std::uint32_t number_of_classes_;

      /** \brief Dimension of the descriptors. */
      int descriptor_size_;
   };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/gfpfh.hpp>
#endif

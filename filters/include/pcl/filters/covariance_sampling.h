/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.

 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */


#ifndef PCL_FILTERS_COVARIANCE_SAMPLING_H_
#define PCL_FILTERS_COVARIANCE_SAMPLING_H_

#include <pcl/filters/filter_indices.h>

namespace pcl
{
  template <typename PointT, typename PointNT>
  class CovarianceSampling : public FilterIndices<PointT>
  {
      using FilterIndices<PointT>::filter_name_;
      using FilterIndices<PointT>::getClassName;
      using FilterIndices<PointT>::indices_;
      using FilterIndices<PointT>::input_;
      using FilterIndices<PointT>::initCompute;

      typedef typename FilterIndices<PointT>::PointCloud Cloud;
      typedef typename Cloud::Ptr CloudPtr;
      typedef typename Cloud::ConstPtr CloudConstPtr;
      typedef typename PointCloud<PointNT>::Ptr NormalsPtr;

    public:
      /** \brief Empty constructor. */
      CovarianceSampling ()
      { filter_name_ = "NormalSpaceSampling"; }

      /** \brief Set number of indices to be sampled.
        * \param[in] sample the number of sample indices
        */
      inline void
      setNumberOfSamples (unsigned int samples)
      { num_samples_ = samples; }

      /** \brief Get the value of the internal \a num_samples_ parameter. */
      inline unsigned int
      getNumberOfSamples () const
      { return (num_samples_); }

      /** \brief Set the normals computed on the input point cloud
        * \param[in] normals the normals computed for the input cloud
        */
      inline void
      setNormals (const NormalsPtr &normals)
      { input_normals_ = normals; }

      /** \brief Get the normals computed on the input point cloud */
      inline NormalsPtr
      getNormals () const
      { return (input_normals_); }

      /** \brief Compute the condition number of the input point cloud. The condition number is the ratio between the
        * largest and smallest eigenvalues of the 6x6 covariance matrix of the cloud. The closer this number is to 1.0,
        * the more stable the cloud is for ICP registration.
        * \return the condition number
        */
      double
      computeConditionNumber ();

    protected:
      /** \brief Number of indices that will be returned. */
      unsigned int num_samples_;

      /** \brief The normals computed at each point in the input cloud */
      NormalsPtr input_normals_;

      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > scaled_points_;

      bool
      initCompute ();

      /** \brief Sample of point indices into a separate PointCloud
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (Cloud &output);

      /** \brief Sample of point indices
        * \param[out] indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices);

      static bool
      sort_dot_list_function (std::pair<int, double> a,
                              std::pair<int, double> b)
      { return (a.second > b.second); }
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/covariance_sampling.hpp>
#endif


#endif /* PCL_FILTERS_COVARIANCE_SAMPLING_H_ */

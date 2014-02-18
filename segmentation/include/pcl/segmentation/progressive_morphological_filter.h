/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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
 */

#ifndef PCL_PROGRESSIVE_MORPHOLOGICAL_FILTER_H_
#define PCL_PROGRESSIVE_MORPHOLOGICAL_FILTER_H_

#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
  /** \brief
    * Implements the Progressive Morphological Filter for segmentation of ground points.
    * Description can be found in the article
    * "A Progressive Morphological Filter for Removing Nonground Measurements from
    * Airborne LIDAR Data"
    * by K. Zhang, S. Chen, D. Whitman, M. Shyu, J. Yan, and C. Zhang.
    */
  template <typename PointT>
  class PCL_EXPORTS ProgressiveMorphologicalFilter : public pcl::PCLBase<PointT>
  {
    public:

      typedef pcl::search::Search <PointT> KdTree;
      typedef typename KdTree::Ptr KdTreePtr;
      typedef pcl::PointCloud <PointT> PointCloud;

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      ProgressiveMorphologicalFilter ();

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding KNN. In other words it frees memory.
        */
      virtual
      ~ProgressiveMorphologicalFilter ();

      /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
      int
      getMaxWindowSize ();

      /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid. */
      void
      setMaxWindowSize (int max_window_size);

      /** \brief Returns smoothness threshold. */
      float
      getSlope () const;

      /** \brief Allows to set smoothness threshold used for testing the points.
        * \param[in] theta new threshold value for the angle between normals
        */
      void
      setSlope (float slope);

      float getMaxDistance () const;
      void setMaxDistance (float max_distance);

      float getInitialDistance () const;
      void setInitialDistance (float initial_distance);

      std::vector<int>
      firstIteration (const typename PointCloud::ConstPtr &source, float c, float b, int k, float s, float dh_0, float dh_max, float dh, float w, float w_max, bool exponential=true);

      std::vector<int>
      pmfIteration (const PointCloud source, float c, float b, int k, float s, float dh_0, float dh_max, float dh, float w, float w_max, std::vector<int> indices, bool exponential=true);

      /** \brief This method launches the segmentation algorithm and returns the clusters that were
        * obtained during the segmentation.
        * \param[out] clusters clusters that were obtained. Each cluster is an array of point indices.
        */
      virtual void
      extract (std::vector<int>& ground);

    protected:

      /** \brief Stores the maximum number of points that a cluster needs to contain in order to be considered valid. */
      int max_window_size_;

      /** \brief Thershold used for testing the smoothness between points. */
      float slope_;

      float max_distance_;

      float initial_distance_;

      /** \brief After the segmentation it will contain the segments. */
      std::vector <int> ground_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/progressive_morphological_filter.hpp>
#endif

#endif


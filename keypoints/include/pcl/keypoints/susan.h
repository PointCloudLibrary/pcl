/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 */

#pragma once

#include <pcl/keypoints/keypoint.h>
#include <pcl/common/intensity.h>

namespace pcl
{
  /** \brief SUSANKeypoint implements a RGB-D extension of the SUSAN detector including normal 
    * directions variation in top of intensity variation. 
    * It is different from Harris in that it exploits normals directly so it is faster.  
    * Original paper "SUSAN — A New Approach to Low Level Image Processing", Smith,
    * Stephen M. and Brady, J. Michael 
    *
    * \author Nizar Sallem 
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal, typename IntensityT= pcl::common::IntensityFieldAccessor<PointInT> >
  class SUSANKeypoint : public Keypoint<PointInT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT> >;
      using ConstPtr = shared_ptr<const SUSANKeypoint<PointInT, PointOutT, NormalT, Intensity> >;

      using PointCloudIn = typename Keypoint<PointInT, PointOutT>::PointCloudIn;
      using PointCloudOut = typename Keypoint<PointInT, PointOutT>::PointCloudOut;
      using KdTree = typename Keypoint<PointInT, PointOutT>::KdTree;
      using PointCloudInConstPtr = typename PointCloudIn::ConstPtr;

      using PointCloudN = pcl::PointCloud<NormalT>;
      using PointCloudNPtr = typename PointCloudN::Ptr;
      using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::surface_;
      using Keypoint<PointInT, PointOutT>::tree_;
      using Keypoint<PointInT, PointOutT>::k_;
      using Keypoint<PointInT, PointOutT>::search_radius_;
      using Keypoint<PointInT, PointOutT>::search_parameter_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;
      using Keypoint<PointInT, PointOutT>::initCompute;

      /** \brief Constructor
        * \param[in] radius the radius for normal estimation as well as for non maxima suppression
        * \param[in] distance_threshold to test if the nucleus is far enough from the centroid
        * \param[in] angular_threshold to test if normals are parallel
        * \param[in] intensity_threshold to test if points are of same color
        */
      SUSANKeypoint (float radius = 0.01f, 
                     float distance_threshold = 0.001f, 
                     float angular_threshold = 0.0001f, 
                     float intensity_threshold = 7.0f)
        : distance_threshold_ (distance_threshold)
        , angular_threshold_ (angular_threshold)
        , intensity_threshold_ (intensity_threshold)
        , normals_ (new pcl::PointCloud<NormalT>)
        , threads_ (0)
        , label_idx_ (-1)
      {
        name_ = "SUSANKeypoint";
        search_radius_ = radius;
        geometric_validation_ = false;
        tolerance_ = 2 * distance_threshold_;
      }
      
      /** \brief Empty destructor */
      ~SUSANKeypoint () {}

      /** \brief set the radius for normal estimation and non maxima supression.
        * \param[in] radius
        */
      void 
      setRadius (float radius);

      void 
      setDistanceThreshold (float distance_threshold);

      /** \brief set the angular_threshold value for detecting corners. Normals are considered as 
        * parallel if 1 - angular_threshold <= (Ni.Nj) <= 1
        * \param[in] angular_threshold
        */
      void 
      setAngularThreshold (float angular_threshold);

      /** \brief set the intensity_threshold value for detecting corners. 
        * \param[in] intensity_threshold
        */
      void 
      setIntensityThreshold (float intensity_threshold);

      /**
        * \brief set normals if precalculated normals are available.
        * \param normals
        */
      void 
      setNormals (const PointCloudNConstPtr &normals);

      void
      setSearchSurface (const PointCloudInConstPtr &cloud) override;

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      void
      setNumberOfThreads (unsigned int nr_threads);

      /** \brief Apply non maxima suppression to the responses to keep strongest corners.
        * \note in SUSAN points with less response or stronger corners
        */
      void 
      setNonMaxSupression (bool nonmax);
    
      /** \brief Filetr false positive using geometric criteria. 
        * The nucleus and the centroid should at least distance_threshold_ from each other AND all the 
        * points belonging to the USAN must be within the segment [nucleus centroid].
        * \param[in] validate 
        */
      void
      setGeometricValidation (bool validate);
    
    protected:
      bool
      initCompute () override;

      void 
      detectKeypoints (PointCloudOut &output) override;
      /** \brief return true if a point lies within the line between the nucleus and the centroid
        * \param[in] nucleus coordinate of the nucleus
        * \param[in] centroid of the SUSAN
        * \param[in] nc to centroid vector (used to speed up since it is constant for a given
        * neighborhood)
        * \param[in] point the query point to test against
        * \return true if the point lies within [nucleus centroid]
        */
      bool
      isWithinNucleusCentroid (const Eigen::Vector3f& nucleus,
                               const Eigen::Vector3f& centroid,
                               const Eigen::Vector3f& nc,
                               const PointInT& point) const;
    private:
      float distance_threshold_;
      float angular_threshold_;
      float intensity_threshold_;
      float tolerance_;
      PointCloudNConstPtr normals_;
      unsigned int threads_;
      bool geometric_validation_;
      bool nonmax_;
      /// intensity field accessor
      IntensityT intensity_;
      /** \brief Set to a value different than -1 if the output cloud has a "label" field and we have 
        * to save the keypoints indices. 
        */
      int label_idx_;
      /** \brief The list of fields present in the output point cloud data. */
      std::vector<pcl::PCLPointField> out_fields_;
      pcl::common::IntensityFieldAccessor<PointOutT> intensity_out_;
  };
}

#include <pcl/keypoints/impl/susan.hpp>

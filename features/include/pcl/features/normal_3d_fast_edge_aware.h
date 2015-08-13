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
 */

#ifndef PCL_NORMAL_3D_FAST_EDGE_AWARE_H_
#define PCL_NORMAL_3D_FAST_EDGE_AWARE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief A fast and accurate 3d normal estimation algorithm for organized point clouds that pays attention to 3d discontinuities both in depth and on surfaces.
    * The method builds upon an efficient edge detection algorithm for depth edges (jump edges) and surface discontinuities (i.e. the border between
    * two surfaces). That algorithm uses specialized integral images on local slope for discovering surface edges. The already computed data structures
    * for edge detection are then re-used to compute average 3d surface slope vectors along x and y direction around a query point. The cross product of both
    * slope vectors yields the surface normal. Due to the use and re-use of efficient data structures and algorithms this normal computation method is very fast
    * and accurate while computing correct normal directions close to non-differentiable surface edges (i.e. no "round-edge" effect at surface borders). For example,
    * using this algorithm normals of two touching surfaces of a cube will receive correct perpendicular surface normals even close to the edge while other normal
    * estimation methods rather tend to amalgamate different normal direction of the two surfaces into one average direction.
    *
    * If your are using this method for normal computation, please cite:
    * Richard Bormann, Joshua Hampp, Martin Haegele, Markus Vincze. Fast and Accurate Normal Estimation by Efficient 3d Edge Detection.
    * In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems, Sept 28 - Oct 03, 2015, Congress Center Hamburg, Hamburg, Germany.
    *
    * \author Richard Bormann
    * \ingroup features
    */
  template <typename PointInT, typename PointOutT>
  class FastEdgeAwareNormalEstimation : public Feature<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<LinearLeastSquaresNormalEstimation<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const LinearLeastSquaresNormalEstimation<PointInT, PointOutT> > ConstPtr;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;
      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::k_;

      /** \brief Constructor */
      FastEdgeAwareNormalEstimation () :
          use_depth_dependent_smoothing_(false),
          max_depth_change_factor_(1.0f),
          normal_smoothing_size_(9.0f)
      {
          feature_name_ = "LinearLeastSquaresNormalEstimation";
          tree_.reset ();
          k_ = 1;
      };

      /** \brief Destructor */
      virtual ~FastEdgeAwareNormalEstimation ();

      /** \brief Computes the normal at the specified position. 
        * \param[in] pos_x x position (pixel)
        * \param[in] pos_y y position (pixel)
        * \param[out] normal the output estimated normal 
        */
      void
      computePointNormal (const int pos_x, const int pos_y, PointOutT &normal);

      /** \brief Set the normal smoothing size
        * \param[in] normal_smoothing_size factor which influences the size of the area used to smooth normals 
        * (depth dependent if useDepthDependentSmoothing is true)
        */
      void
      setNormalSmoothingSize (float normal_smoothing_size)
      {
        normal_smoothing_size_ = normal_smoothing_size;
      }

      /** \brief Set whether to use depth depending smoothing or not
        * \param[in] use_depth_dependent_smoothing decides whether the smoothing is depth dependent
        */
      void
      setDepthDependentSmoothing (bool use_depth_dependent_smoothing)
      {
        use_depth_dependent_smoothing_ = use_depth_dependent_smoothing;
      }

      /** \brief The depth change threshold for computing object borders
        * \param[in] max_depth_change_factor the depth change threshold for computing object borders based on 
        * depth changes
        */
      void 
      setMaxDepthChangeFactor (float max_depth_change_factor)
      {
        max_depth_change_factor_ = max_depth_change_factor;
      }

      /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      virtual inline void 
      setInputCloud (const typename PointCloudIn::ConstPtr &cloud) 
      { 
        input_ = cloud; 
      }

    protected:
      /** \brief Computes the normal for the complete cloud. 
        * \param[out] output the resultant normals
        */
      void 
      computeFeature (PointCloudOut &output);

    private:

      /** the threshold used to detect depth discontinuities */
      //float distance_threshold_;

      /** \brief Smooth data based on depth (true/false). */
      bool use_depth_dependent_smoothing_;

      /** \brief Threshold for detecting depth discontinuities */
      float max_depth_change_factor_;

      /** \brief */
      float normal_smoothing_size_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/normal_3d_fast_edge_aware.hpp>
#endif

#endif  // PCL_NORMAL_3D_FAST_EDGE_AWARE_H_


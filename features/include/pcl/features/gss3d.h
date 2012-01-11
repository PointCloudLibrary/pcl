/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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


#ifndef PCL_FEATURES_GSS3D_H_
#define PCL_FEATURES_GSS3D_H_

#include <pcl/features/feature.h>
#include <boost/multi_array.hpp>

namespace pcl
{
  /** \brief Class for computing scale-dependent 3D shape descriptors, as proposed in:
    * John Novatnack and Ko Nishino
    * Scale-Dependent/Invariant Local 3D Shape Descriptors for Fully Automatic Registration of Multiple Sets of Range Images
    * Proceedings of Tenth European Conference on Computer Vision ECCV'08
    * October 2008
    *
    * and
    *
    * John Novatnack and Ko Nishino
    * Scale-Dependent 3D Geometric Features
    * Proceedings of IEEE Eleventh International Conference on Computer Vision ICCV'07
    * October 2007
    *
    * \author Alexandru-Eugen Ichim
    */

  /// NOT FUNCTIONAL -> CURRENTLY WORK IN PROGRESS
  template <typename PointInT, typename PointNT, typename PointOutT>
  class GSS3DEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;
      typedef typename Feature<PointInT, PointOutT>::PointCloudInPtr  PointCloudInPtr;
      typedef typename FeatureFromNormals<PointInT, PointNT, PointOutT>::PointCloudN PointCloudN;
      typedef typename FeatureFromNormals<PointInT, PointNT, PointOutT>::PointCloudNPtr PointCloudNPtr;

      /** \brief Empty constructor. 
        * \param[in] scales
        * \param[in] window_size
        */
      GSS3DEstimation (const std::vector<int> &scales, int window_size)
      {
        scales_ = scales;
        window_size_ = window_size;
        feature_name_ = "GSS3DEstimation";

        // Slight hack in order to pass the check for the presence of a search method in Feature::initCompute ()
        Feature<PointInT, PointOutT>::tree_.reset (new pcl::search::KdTree <PointInT> ());
        Feature<PointInT, PointOutT>::search_radius_ = 1.0f;
      }

      /** \brief A. */
      std::vector<PointCloudNPtr> normal_maps_;

      typedef boost::multi_array<float, 3> Array3D;
      /** \brief A. */
      boost::shared_ptr<Array3D> d_horiz_normal_maps_, d_vert_normal_maps_,
                                 dd_horiz_normal_maps_, dd_vert_normal_maps_,
                                 laplacians_, grams_;
      /** \brief A. */
      std::vector<PointCloudInPtr> edges_, corners_;


    protected:
      /** \brief A. */
      void
      computeFeature (PointCloudOut &output);

      /** \brief A. */
      void
      calculateGeometricScaleSpace ();

      /** \brief A. */
      void
      computeHorizontalDerivatives ();

      /** \brief A. */
      void
      computeDerivatives ();


      /** \brief A. */
      void
      extractCorners ();

      /** \brief A. */
      void
      extractEdges ();

      /** \brief A. */
      // uses the Bresenham algorithm for rasterizing lines
      float
      computeGeodesicDistance (size_t x0, size_t y0,
                               size_t x1, size_t y1);

      /// TODO add a hash map to avoid repeating computations

      // paramters
      /** \brief A. */
      std::vector<int> scales_;
      /** \brief A. */
      int window_size_;

    private:
      /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
        * \param[out] output the output point cloud 
        */
      void 
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output) {}
  };
}


#endif /* PCL_GSS3D_H_ */

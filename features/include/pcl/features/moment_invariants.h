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

#ifndef PCL_MOMENT_INVARIANTS_H_
#define PCL_MOMENT_INVARIANTS_H_

#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief MomentInvariantsEstimation estimates the 3 moment invariants (j1, j2, j3) at each 3D point.
    *
    * \note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \ref NormalEstimationOMP for an example on how to extend this to parallel implementations.
    * \author Radu B. Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointOutT>
  class MomentInvariantsEstimation: public Feature<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<MomentInvariantsEstimation<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const MomentInvariantsEstimation<PointInT, PointOutT> > ConstPtr;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::input_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Empty constructor. */
      MomentInvariantsEstimation () : xyz_centroid_ (), temp_pt_ ()
      {
        feature_name_ = "MomentInvariantsEstimation";
      };

      /** \brief Compute the 3 moment invariants (j1, j2, j3) for a given set of points, using their indices.
        * \param[in] cloud the input point cloud
        * \param[in] indices the point cloud indices that need to be used
        * \param[out] j1 the resultant first moment invariant
        * \param[out] j2 the resultant second moment invariant
        * \param[out] j3 the resultant third moment invariant
        */
      void 
      computePointMomentInvariants (const pcl::PointCloud<PointInT> &cloud, 
                                    const std::vector<int> &indices, 
                                    float &j1, float &j2, float &j3);

      /** \brief Compute the 3 moment invariants (j1, j2, j3) for a given set of points, using their indices.
        * \param[in] cloud the input point cloud
        * \param[out] j1 the resultant first moment invariant
        * \param[out] j2 the resultant second moment invariant
        * \param[out] j3 the resultant third moment invariant
        */
      void 
      computePointMomentInvariants (const pcl::PointCloud<PointInT> &cloud, 
                                    float &j1, float &j2, float &j3);

    protected:

      /** \brief Estimate moment invariants for all points given in <setInputCloud (), setIndices ()> using the surface
        * in setSearchSurface () and the spatial locator in setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains the moment invariants
        */
      void 
      computeFeature (PointCloudOut &output);

    private:
      /** \brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch. */
      Eigen::Vector4f xyz_centroid_;

      /** \brief Internal data vector. */
      Eigen::Vector4f temp_pt_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/moment_invariants.hpp>
#endif

#endif  //#ifndef PCL_MOMENT_INVARIANTS_H_

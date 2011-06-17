/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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

#ifndef PCL_FEATURES_IMPL_PPF_H_
#define PCL_FEATURES_IMPL_PPF_H_

#include "pcl/features/ppf.h"
#include <pcl/features/pfh.h>

template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PPFEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  /// initialize output container
  output.clear ();
  output.resize (input_->points.size () * input_->points.size ());
  PCL_INFO ("PPFEstimation input size: %u\nOutput PPF size: %u\n",
           input_->points.size (), output.size ());

  /// compute point pair features for every pair of points in the cloud
  for (size_t i = 0; i < input_->points.size (); ++i)
    for (size_t j = 0 ; j < input_->points.size (); ++j)
    {
      PointOutT p;
      if (i != j)
      {
        if (pcl::computePairFeatures (input_->points[i].getVector4fMap (),
                                      normals_->points[i].getNormalVector4fMap (),
                                      input_->points[j].getVector4fMap (),
                                      normals_->points[j].getNormalVector4fMap (),
                                      p.f1, p.f2, p.f3, p.f4))
        {
          /// calculate alpha_m angle
          Eigen::Vector3f model_reference_point = input_->points[i].getVector3fMap (),
              model_reference_normal = normals_->points[i].getNormalVector3fMap (),
              model_point = input_->points[j].getVector3fMap ();
          Eigen::AngleAxisf rotation_mg (acos (model_reference_normal.dot (Eigen::Vector3f::UnitX ())),
                                         model_reference_normal.cross (Eigen::Vector3f::UnitX ()).normalized ());
          Eigen::Affine3f transform_mg = Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg;
          p.alpha_m = acos (Eigen::Vector3f::UnitY ().dot ((transform_mg * model_point).normalized ()));
        }
        else
        {
          PCL_ERROR ("Computing pair feature vector between points %zu and %zu went wrong.\n", i, j);
          p.f1 = p.f2 = p.f3 = p.f4 = p.alpha_m = 0.0;
        }
      }
      /// do not calculate the feature for identity pairs (i, i) as they are not used
      /// in the following computations
      else
        p.f1 = p.f2 = p.f3 = p.f4 = p.alpha_m = 0.0;

      output.points[i*input_->points.size () + j] = p;
    }
}

#define PCL_INSTANTIATE_PPFEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::PPFEstimation<T,NT,OutT>;


#endif // PCL_FEATURES_IMPL_PPF_H_

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
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
 */

#ifndef PCL_FEATURES_IMPL_PPF_H_
#define PCL_FEATURES_IMPL_PPF_H_

#include <pcl/features/ppf.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfh_tools.h> // for computePairFeatures

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT>
pcl::PPFEstimation<PointInT, PointNT, PointOutT>::PPFEstimation ()
    : FeatureFromNormals <PointInT, PointNT, PointOutT> ()
{
  feature_name_ = "PPFEstimation";
  // Slight hack in order to pass the check for the presence of a search method in Feature::initCompute ()
  Feature<PointInT, PointOutT>::tree_.reset (new pcl::search::KdTree <PointInT> ());
  Feature<PointInT, PointOutT>::search_radius_ = 1.0f;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PPFEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Initialize output container - overwrite the sizes done by Feature::initCompute ()
  output.resize (indices_->size () * input_->size ());
  output.height = 1;
  output.width = output.size ();
  output.is_dense = true;

  // Compute point pair features for every pair of points in the cloud
  for (std::size_t index_i = 0; index_i < indices_->size (); ++index_i)
  {
    std::size_t i = (*indices_)[index_i];
    for (std::size_t j = 0 ; j < input_->size (); ++j)
    {
      PointOutT p;
      if (i != j)
      {
        if (//pcl::computePPFPairFeature
            pcl::computePairFeatures ((*input_)[i].getVector4fMap (),
                                      (*normals_)[i].getNormalVector4fMap (),
                                      (*input_)[j].getVector4fMap (),
                                      (*normals_)[j].getNormalVector4fMap (),
                                      p.f1, p.f2, p.f3, p.f4))
        {
          // Calculate alpha_m angle
          Eigen::Vector3f model_reference_point = (*input_)[i].getVector3fMap (),
                          model_reference_normal = (*normals_)[i].getNormalVector3fMap (),
                          model_point = (*input_)[j].getVector3fMap ();
          float rotation_angle = std::acos (model_reference_normal.dot (Eigen::Vector3f::UnitX ()));
          bool parallel_to_x = (model_reference_normal.y() == 0.0f && model_reference_normal.z() == 0.0f);
          Eigen::Vector3f rotation_axis = (parallel_to_x)?(Eigen::Vector3f::UnitY ()):(model_reference_normal.cross (Eigen::Vector3f::UnitX ()). normalized());
          Eigen::AngleAxisf rotation_mg (rotation_angle, rotation_axis);
          Eigen::Affine3f transform_mg (Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg);

          Eigen::Vector3f model_point_transformed = transform_mg * model_point;
          float angle = std::atan2 ( -model_point_transformed(2), model_point_transformed(1));
          if (std::sin (angle) * model_point_transformed(2) < 0.0f)
            angle *= (-1);
          p.alpha_m = -angle;
        }
        else
        {
          PCL_ERROR ("[pcl::%s::computeFeature] Computing pair feature vector between points %u and %u went wrong.\n", getClassName ().c_str (), i, j);
          p.f1 = p.f2 = p.f3 = p.f4 = p.alpha_m = std::numeric_limits<float>::quiet_NaN ();
          output.is_dense = false;
        }
      }
      // Do not calculate the feature for identity pairs (i, i) as they are not used
      // in the following computations
      else
      {
        p.f1 = p.f2 = p.f3 = p.f4 = p.alpha_m = std::numeric_limits<float>::quiet_NaN ();
        output.is_dense = false;
      }

      output[index_i*input_->size () + j] = p;
    }
  }
}

#define PCL_INSTANTIATE_PPFEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::PPFEstimation<T,NT,OutT>;


#endif // PCL_FEATURES_IMPL_PPF_H_

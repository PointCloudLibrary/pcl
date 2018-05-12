/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#ifndef PCL_FEATURES_IMPL_PPFRGB_H_
#define PCL_FEATURES_IMPL_PPFRGB_H_

#include <pcl/features/ppfrgb.h>
#include <pcl/features/pfhrgb.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT>
pcl::PPFRGBEstimation<PointInT, PointNT, PointOutT>::PPFRGBEstimation ()
: FeatureFromNormals <PointInT, PointNT, PointOutT> ()
{
  feature_name_ = "PPFRGBEstimation";
  // Slight hack in order to pass the check for the presence of a search method in Feature::initCompute ()
  Feature<PointInT, PointOutT>::tree_.reset (new pcl::search::KdTree <PointInT> ());
  Feature<PointInT, PointOutT>::search_radius_ = 1.0f;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PPFRGBEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Initialize output container - overwrite the sizes done by Feature::initCompute ()
  output.points.resize (indices_->size () * input_->points.size ());
  output.height = 1;
  output.width = static_cast<uint32_t> (output.points.size ());

  // Compute point pair features for every pair of points in the cloud
  for (size_t index_i = 0; index_i < indices_->size (); ++index_i)
  {
    size_t i = (*indices_)[index_i];
    for (size_t j = 0 ; j < input_->points.size (); ++j)
    {
      PointOutT p;
      if (i != j)
      {
        if (pcl::computeRGBPairFeatures
            (input_->points[i].getVector4fMap (), normals_->points[i].getNormalVector4fMap (), input_->points[i].getRGBVector4i (),
             input_->points[j].getVector4fMap (), normals_->points[j].getNormalVector4fMap (), input_->points[j].getRGBVector4i (),
             p.f1, p.f2, p.f3, p.f4, p.r_ratio, p.g_ratio, p.b_ratio))
        {
          // Calculate alpha_m angle
          Eigen::Vector3f model_reference_point = input_->points[i].getVector3fMap (),
              model_reference_normal = normals_->points[i].getNormalVector3fMap (),
              model_point = input_->points[j].getVector3fMap ();
          Eigen::AngleAxisf rotation_mg (acosf (model_reference_normal.dot (Eigen::Vector3f::UnitX ())),
                                         model_reference_normal.cross (Eigen::Vector3f::UnitX ()).normalized ());
          Eigen::Affine3f transform_mg = Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg;

          Eigen::Vector3f model_point_transformed = transform_mg * model_point;
          float angle = atan2f ( -model_point_transformed(2), model_point_transformed(1));
          if (sin (angle) * model_point_transformed(2) < 0.0f)
            angle *= (-1);
          p.alpha_m = -angle;
        }
        else
        {
          PCL_ERROR ("[pcl::%s::computeFeature] Computing pair feature vector between points %lu and %lu went wrong.\n", getClassName ().c_str (), i, j);
           p.f1 = p.f2 = p.f3 = p.f4 = p.alpha_m = p.r_ratio = p.g_ratio = p.b_ratio = 0.f;
        }
      }
      // Do not calculate the feature for identity pairs (i, i) as they are not used
      // in the following computations
      else
         p.f1 = p.f2 = p.f3 = p.f4 = p.alpha_m = p.r_ratio = p.g_ratio = p.b_ratio = 0.f;

      output.points[index_i*input_->points.size () + j] = p;
    }
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT>
pcl::PPFRGBRegionEstimation<PointInT, PointNT, PointOutT>::PPFRGBRegionEstimation ()
: FeatureFromNormals <PointInT, PointNT, PointOutT> ()
{
  feature_name_ = "PPFRGBEstimation";
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PPFRGBRegionEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  PCL_INFO ("before computing output size: %u\n", output.size ());
  output.resize (indices_->size ());
  for (int index_i = 0; index_i < static_cast<int> (indices_->size ()); ++index_i)
  {
    int i = (*indices_)[index_i];
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    tree_->radiusSearch (i, static_cast<float> (search_radius_), nn_indices, nn_distances);

    PointOutT average_feature_nn;
    average_feature_nn.alpha_m = 0;
    average_feature_nn.f1 = average_feature_nn.f2 = average_feature_nn.f3 = average_feature_nn.f4 =
        average_feature_nn.r_ratio = average_feature_nn.g_ratio = average_feature_nn.b_ratio = 0.0f;

    for (std::vector<int>::iterator nn_it = nn_indices.begin (); nn_it != nn_indices.end (); ++nn_it)
    {
      int j = *nn_it;
      if (i != j)
      {
        float f1, f2, f3, f4, r_ratio, g_ratio, b_ratio;
        if (pcl::computeRGBPairFeatures
            (input_->points[i].getVector4fMap (), normals_->points[i].getNormalVector4fMap (), input_->points[i].getRGBVector4i (),
             input_->points[j].getVector4fMap (), normals_->points[j].getNormalVector4fMap (), input_->points[j].getRGBVector4i (),
             f1, f2, f3, f4, r_ratio, g_ratio, b_ratio))
        {
          average_feature_nn.f1 += f1;
          average_feature_nn.f2 += f2;
          average_feature_nn.f3 += f3;
          average_feature_nn.f4 += f4;
          average_feature_nn.r_ratio += r_ratio;
          average_feature_nn.g_ratio += g_ratio;
          average_feature_nn.b_ratio += b_ratio;
        }
        else
        {
          PCL_ERROR ("[pcl::%s::computeFeature] Computing pair feature vector between points %lu and %lu went wrong.\n", getClassName ().c_str (), i, j);
        }
      }
    }

    float normalization_factor = static_cast<float> (nn_indices.size ());
    average_feature_nn.f1 /= normalization_factor;
    average_feature_nn.f2 /= normalization_factor;
    average_feature_nn.f3 /= normalization_factor;
    average_feature_nn.f4 /= normalization_factor;
    average_feature_nn.r_ratio /= normalization_factor;
    average_feature_nn.g_ratio /= normalization_factor;
    average_feature_nn.b_ratio /= normalization_factor;
    output.points[index_i] = average_feature_nn;
  }
  PCL_INFO ("Output size: %u\n", output.points.size ());
}


#define PCL_INSTANTIATE_PPFRGBEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::PPFRGBEstimation<T,NT,OutT>;
#define PCL_INSTANTIATE_PPFRGBRegionEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::PPFRGBRegionEstimation<T,NT,OutT>;

#endif // PCL_FEATURES_IMPL_PPFRGB_H_

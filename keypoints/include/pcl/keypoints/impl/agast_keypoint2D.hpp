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
 */

#ifndef PCL_KEYPOINTS_AGAST_KEYPOINT_2D_IMPL_H_
#define PCL_KEYPOINTS_AGAST_KEYPOINT_2D_IMPL_H_

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT> bool
pcl::AgastKeypoint2D<PointInT, IntensityT>::initCompute ()
{
  if (!pcl::Keypoint<PointInT, pcl::PointXY>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed.!\n", name_.c_str ());
    return (false);
  }

  if (!input_->isOrganized ())
  {    
    PCL_ERROR ("[pcl::%s::initCompute] %s doesn't support non organized clouds!\n", name_.c_str ());
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT> void
pcl::AgastKeypoint2D<PointInT, IntensityT>::detectKeypoints (PointCloudOut &output)
{
  // image size
  const size_t width = input_->width;
  const size_t height = input_->height;

  // destination for intensity data; will be forwarded to AGAST
  std::vector<unsigned char> image_data (width*height);

  for (size_t row_index = 0; row_index < height; ++row_index)
  {
    for (size_t col_index = 0; col_index < width; ++col_index)
    {
      image_data[row_index*width + col_index] = static_cast<unsigned char> (intensity_ ((*input_) (col_index, row_index)));
    }
  }

  if (apply_non_max_suppression_)
  {
    pcl::PointCloud<pcl::PointXY> tmp_cloud;

    AgastHelper7_12 agast_helper (width, height, threshold_);
    agast_helper.detectKeypoints (image_data, tmp_cloud);

    agast_helper.applyNonMaxSuppression (image_data, tmp_cloud, output);
  }
  else
  {
    AgastHelper7_12 agast_helper (width, height, threshold_);
    agast_helper.detectKeypoints (image_data, output);
  }

  // we don not change the denseness
  output.is_dense = input_->is_dense;
}


#define AgastKeypoint2D(T,I) template class PCL_EXPORTS pcl::AgastKeypoint2D<T,I>;
#endif 

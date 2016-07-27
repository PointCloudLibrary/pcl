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
 *
 */

#ifndef PCL_HARRIS_KEYPOINT_2D_IMPL_H_
#define PCL_HARRIS_KEYPOINT_2D_IMPL_H_

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::setMethod (ResponseMethod method)
{
  method_ = method;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::setThreshold (float threshold)
{
  threshold_= threshold;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::setRefine (bool do_refine)
{
  refine_ = do_refine;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::setNonMaxSupression (bool nonmax)
{
  nonmax_ = nonmax;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::setWindowWidth (int window_width)
{
  window_width_= window_width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::setWindowHeight (int window_height)
{
  window_height_= window_height;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::setSkippedPixels (int skipped_pixels)
{
  skipped_pixels_= skipped_pixels;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::setMinimalDistance (int min_distance)
{
  min_distance_ = min_distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::computeSecondMomentMatrix (std::size_t index, float* coefficients) const
{
  static const int width = static_cast<int> (input_->width);
  static const int height = static_cast<int> (input_->height);
  
  int x = static_cast<int> (index % input_->width);
  int y = static_cast<int> (index / input_->width);
  // indices        0   1   2
  // coefficients: ixix  ixiy  iyiy
  memset (coefficients, 0, sizeof (float) * 3);

  int endx = std::min (width, x + half_window_width_);
  int endy = std::min (height, y + half_window_height_);
  for (int xx = std::max (0, x - half_window_width_); xx < endx; ++xx)
    for (int yy = std::max (0, y - half_window_height_); yy < endy; ++yy)
    {
      const float& ix = derivatives_rows_ (xx,yy);
      const float& iy = derivatives_cols_ (xx,yy);
      coefficients[0]+= ix * ix;
      coefficients[1]+= ix * iy;
      coefficients[2]+= iy * iy;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> bool
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::initCompute ()
{
  if (!pcl::Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed.!\n", name_.c_str ());
    return (false);
  }

  if (!input_->isOrganized ())
  {    
    PCL_ERROR ("[pcl::%s::initCompute] %s doesn't support non organized clouds!\n", name_.c_str ());
    return (false);
  }
  
  if (indices_->size () != input_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] %s doesn't support setting indices!\n", name_.c_str ());
    return (false);
  }
  
  if ((window_height_%2) == 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] Window height must be odd!\n", name_.c_str ());
    return (false);
  }

  if ((window_width_%2) == 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] Window width must be odd!\n", name_.c_str ());
    return (false);
  }

  if (window_height_ < 3 || window_width_ < 3)
  {
    PCL_ERROR ("[pcl::%s::initCompute] Window size must be >= 3x3!\n", name_.c_str ());
    return (false);
  }
  
  half_window_width_ = window_width_ / 2;
  half_window_height_ = window_height_ / 2;

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::detectKeypoints (PointCloudOut &output)
{
  derivatives_cols_.resize (input_->width, input_->height);
  derivatives_rows_.resize (input_->width, input_->height);
  //Compute cloud intensities first derivatives along columns and rows
  //!!! nsallem 20120220 : we don't test here for density so if one term in nan the result is nan
  int w = static_cast<int> (input_->width) - 1;
  int h = static_cast<int> (input_->height) - 1;
  // j = 0 --> j-1 out of range ; use 0
  // i = 0 --> i-1 out of range ; use 0
  derivatives_cols_(0,0) = (intensity_ ((*input_) (0,1)) - intensity_ ((*input_) (0,0))) * 0.5;
  derivatives_rows_(0,0) = (intensity_ ((*input_) (1,0)) - intensity_ ((*input_) (0,0))) * 0.5;

  for(int i = 1; i < w; ++i)
	{
		derivatives_cols_(i,0) = (intensity_ ((*input_) (i,1)) - intensity_ ((*input_) (i,0))) * 0.5;
	}

  derivatives_rows_(w,0) = (intensity_ ((*input_) (w,0)) - intensity_ ((*input_) (w-1,0))) * 0.5;
  derivatives_cols_(w,0) = (intensity_ ((*input_) (w,1)) - intensity_ ((*input_) (w,0))) * 0.5;

  for(int j = 1; j < h; ++j)
  {
    // i = 0 --> i-1 out of range ; use 0
		derivatives_rows_(0,j) = (intensity_ ((*input_) (1,j)) - intensity_ ((*input_) (0,j))) * 0.5;
    for(int i = 1; i < w; ++i)
    {
      // derivative with respect to rows
      derivatives_rows_(i,j) = (intensity_ ((*input_) (i+1,j)) - intensity_ ((*input_) (i-1,j))) * 0.5;

      // derivative with respect to cols
      derivatives_cols_(i,j) = (intensity_ ((*input_) (i,j+1)) - intensity_ ((*input_) (i,j-1))) * 0.5;
    }
    // i = w --> w+1 out of range ; use w
    derivatives_rows_(w,j) = (intensity_ ((*input_) (w,j)) - intensity_ ((*input_) (w-1,j))) * 0.5;
  }

  // j = h --> j+1 out of range use h
  derivatives_cols_(0,h) = (intensity_ ((*input_) (0,h)) - intensity_ ((*input_) (0,h-1))) * 0.5;
  derivatives_rows_(0,h) = (intensity_ ((*input_) (1,h)) - intensity_ ((*input_) (0,h))) * 0.5;

  for(int i = 1; i < w; ++i)
	{
    derivatives_cols_(i,h) = (intensity_ ((*input_) (i,h)) - intensity_ ((*input_) (i,h-1))) * 0.5;
	}
  derivatives_rows_(w,h) = (intensity_ ((*input_) (w,h)) - intensity_ ((*input_) (w-1,h))) * 0.5;
  derivatives_cols_(w,h) = (intensity_ ((*input_) (w,h)) - intensity_ ((*input_) (w,h-1))) * 0.5;

  switch (method_)
  {
    case HARRIS:
      responseHarris(*response_);
      break;
    case NOBLE:
      responseNoble(*response_);
      break;
    case LOWE:
      responseLowe(*response_);
      break;
    case TOMASI:
      responseTomasi(*response_);
      break;
  }
  
  if (!nonmax_)
  {
    output = *response_;
    for (size_t i = 0; i < response_->size (); ++i)
      keypoints_indices_->indices.push_back (i);
  }
  else
  {    
    std::sort (indices_->begin (), indices_->end (), 
               boost::bind (&HarrisKeypoint2D::greaterIntensityAtIndices, this, _1, _2));
    float threshold = threshold_ * response_->points[indices_->front ()].intensity;
    output.clear ();
    output.reserve (response_->size());
    std::vector<bool> occupency_map (response_->size (), false);    
    int width (response_->width);
    int height (response_->height);
    const int occupency_map_size (occupency_map.size ());

#ifdef _OPENMP
#pragma omp parallel for shared (output, occupency_map) firstprivate (width, height) num_threads(threads_)
#endif
    for (int i = 0; i < occupency_map_size; ++i)
    {
      int idx = indices_->at (i);
      const PointOutT& point_out = response_->points [idx];
      if (occupency_map[idx] || point_out.intensity < threshold || !isFinite (point_out))
        continue;
        
#ifdef _OPENMP
#pragma omp critical
#endif
      {
        output.push_back (point_out);
        keypoints_indices_->indices.push_back (idx);
      }
      
			int u_end = std::min (width, idx % width + min_distance_);
			int v_end = std::min (height, idx / width + min_distance_);
      for(int u = std::max (0, idx % width - min_distance_); u < u_end; ++u)
        for(int v = std::max (0, idx / width - min_distance_); v < v_end; ++v)
          occupency_map[v*input_->width+u] = true;
    }

    // if (refine_)
    //   refineCorners (output);

    output.height = 1;
    output.width = static_cast<uint32_t> (output.size());
  }

  // we don not change the denseness
  output.is_dense = input_->is_dense;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::responseHarris (PointCloudOut &output) const
{
  PCL_ALIGN (16) float covar [3];
  output.clear ();
  output.resize (input_->size ());
  const int output_size (output.size ());

#ifdef _OPENMP
#pragma omp parallel for shared (output) private (covar) num_threads(threads_)
#endif
  for (int index = 0; index < output_size; ++index)
  {
    PointOutT& out_point = output.points [index];
    const PointInT &in_point = (*input_).points [index];
    out_point.intensity = 0;
    out_point.x = in_point.x;
    out_point.y = in_point.y;
    out_point.z = in_point.z;
    if (isFinite (in_point))
    {
      computeSecondMomentMatrix (index, covar);
      float trace = covar [0] + covar [2];
      if (trace != 0.f)
      {
        float det = covar[0] * covar[2] - covar[1] * covar[1];
        out_point.intensity = 0.04f + det - 0.04f * trace * trace;
      }
    }
  }

  output.height = input_->height;
  output.width = input_->width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::responseNoble (PointCloudOut &output) const
{
  PCL_ALIGN (16) float covar [3];
  output.clear ();
  output.resize (input_->size ());
  const int output_size (output.size ());

#ifdef _OPENMP
#pragma omp parallel for shared (output) private (covar) num_threads(threads_)
#endif
  for (int index = 0; index < output_size; ++index)
  {
    PointOutT &out_point = output.points [index];
    const PointInT &in_point = input_->points [index];
    out_point.x = in_point.x;
    out_point.y = in_point.y;
    out_point.z = in_point.z;
    out_point.intensity = 0;
    if (isFinite (in_point))
    {    
      computeSecondMomentMatrix (index, covar);
      float trace = covar [0] + covar [2];
      if (trace != 0)
      {
        float det = covar[0] * covar[2] - covar[1] * covar[1];
        out_point.intensity = det / trace;
      }
    }    
  }
  
  output.height = input_->height;
  output.width = input_->width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::responseLowe (PointCloudOut &output) const
{
  PCL_ALIGN (16) float covar [3];
  output.clear ();
  output.resize (input_->size ());
  const int output_size (output.size ());

#ifdef _OPENMP
#pragma omp parallel for shared (output) private (covar) num_threads(threads_)
#endif
  for (int index = 0; index < output_size; ++index)      
  {
    PointOutT &out_point = output.points [index];
    const PointInT &in_point = input_->points [index];
    out_point.x = in_point.x;
    out_point.y = in_point.y;
    out_point.z = in_point.z;
    out_point.intensity = 0;
    if (isFinite (in_point))
    {    
      computeSecondMomentMatrix (index, covar);
      float trace = covar [0] + covar [2];
      if (trace != 0)
      {
        float det = covar[0] * covar[2] - covar[1] * covar[1];
        out_point.intensity = det / (trace * trace);
      }
    }
  }

  output.height = input_->height;
  output.width = input_->width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::responseTomasi (PointCloudOut &output) const
{
  PCL_ALIGN (16) float covar [3];
  output.clear ();
  output.resize (input_->size ());
  const int output_size (output.size ());

#ifdef _OPENMP
#pragma omp parallel for shared (output) private (covar) num_threads(threads_)
#endif
  for (int index = 0; index < output_size; ++index)
  {
    PointOutT &out_point = output.points [index];
    const PointInT &in_point = input_->points [index];
    out_point.x = in_point.x;
    out_point.y = in_point.y;
    out_point.z = in_point.z;
    out_point.intensity = 0;
    if (isFinite (in_point))
    {      
      computeSecondMomentMatrix (index, covar);
      // min egenvalue
      out_point.intensity = ((covar[0] + covar[2] - sqrt((covar[0] - covar[2])*(covar[0] - covar[2]) + 4 * covar[1] * covar[1])) /2.0f);
    }    
  }
  
  output.height = input_->height;
  output.width = input_->width;
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// template <typename PointInT, typename PointOutT, typename IntensityT> void
// pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>::refineCorners (PointCloudOut &corners) const
// {
//   std::vector<int> nn_indices;
//   std::vector<float> nn_dists;

//   Eigen::Matrix2f nnT;
//   Eigen::Matrix2f NNT;
//   Eigen::Matrix2f NNTInv;
//   Eigen::Vector2f NNTp;
//   float diff;
//   const unsigned max_iterations = 10;
// #ifdef _OPENMP
//   #pragma omp parallel for shared (corners) private (nnT, NNT, NNTInv, NNTp, diff, nn_indices, nn_dists) num_threads(threads_)
// #endif
//   for (int cIdx = 0; cIdx < static_cast<int> (corners.size ()); ++cIdx)
//   {
//     unsigned iterations = 0;
//     do {
//       NNT.setZero();
//       NNTp.setZero();
//       PointInT corner;
//       corner.x = corners[cIdx].x;
//       corner.y = corners[cIdx].y;
//       corner.z = corners[cIdx].z;
//       tree_->radiusSearch (corner, search_radius_, nn_indices, nn_dists);
//       for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
//       {
//         if (!pcl_isfinite (normals_->points[*iIt].normal_x))
//           continue;

//         nnT = normals_->points[*iIt].getNormalVector3fMap () * normals_->points[*iIt].getNormalVector3fMap ().transpose();
//         NNT += nnT;
//         NNTp += nnT * surface_->points[*iIt].getVector3fMap ();
//       }
//       if (invert3x3SymMatrix (NNT, NNTInv) != 0)
//         corners[cIdx].getVector3fMap () = NNTInv * NNTp;

//       diff = (corners[cIdx].getVector3fMap () - corner.getVector3fMap()).squaredNorm ();
//     } while (diff > 1e-6 && ++iterations < max_iterations);
//   }
// }

#define PCL_INSTANTIATE_HarrisKeypoint2D(T,U,I) template class PCL_EXPORTS pcl::HarrisKeypoint2D<T,U,I>;
#endif // #ifndef PCL_HARRIS_KEYPOINT_2D_IMPL_H_

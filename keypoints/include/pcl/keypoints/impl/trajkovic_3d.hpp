/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
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

#ifndef PCL_TRAJKOVIC_KEYPOINT_3D_IMPL_H_
#define PCL_TRAJKOVIC_KEYPOINT_3D_IMPL_H_

#include <pcl/features/integral_image_normal.h>


namespace pcl
{

template <typename PointInT, typename PointOutT, typename NormalT> bool
TrajkovicKeypoint3D<PointInT, PointOutT, NormalT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
    return (false);

  keypoints_indices_.reset (new pcl::PointIndices);
  keypoints_indices_->indices.reserve (input_->size ());

  if (indices_->size () != input_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] %s doesn't support setting indices!\n", name_.c_str ());
    return (false);
  }

  if ((window_size_%2) == 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] Window size must be odd!\n", name_.c_str ());
    return (false);
  }

  if (window_size_ < 3)
  {
    PCL_ERROR ("[pcl::%s::initCompute] Window size must be >= 3x3!\n", name_.c_str ());
    return (false);
  }

  half_window_size_ = window_size_ / 2;

  if (!normals_)
  {
    NormalsPtr normals (new Normals ());
    pcl::IntegralImageNormalEstimation<PointInT, NormalT> normal_estimation;
    normal_estimation.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointInT, NormalT>::SIMPLE_3D_GRADIENT);
    normal_estimation.setInputCloud (input_);
    normal_estimation.setNormalSmoothingSize (5.0);
    normal_estimation.compute (*normals);
    normals_ = normals;
  }

  if (normals_->size () != input_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] normals given, but the number of normals does not match the number of input points!\n", name_.c_str ());
    return (false);
  }

  return (true);
}


template <typename PointInT, typename PointOutT, typename NormalT> void
TrajkovicKeypoint3D<PointInT, PointOutT, NormalT>::detectKeypoints (PointCloudOut &output)
{
  response_.reset (new pcl::PointCloud<float> (input_->width, input_->height));
  const Normals &normals = *normals_;
  const PointCloudIn &input = *input_;
  pcl::PointCloud<float>& response = *response_;
  const int w = static_cast<int> (input_->width) - half_window_size_;
  const int h = static_cast<int> (input_->height) - half_window_size_;

  if (method_ == FOUR_CORNERS)
  {
#if OPENMP_LEGACY_CONST_DATA_SHARING_RULE
#pragma omp parallel for \
  default(none) \
  shared(input, normals, response) \
  num_threads(threads_)
#else
#pragma omp parallel for \
  default(none) \
  shared(h, input, normals, response, w) \
  num_threads(threads_)
#endif
    for(int j = half_window_size_; j < h; ++j)
    {
      for(int i = half_window_size_; i < w; ++i)
      {
        if (!isFinite (input (i,j))) continue;
        const NormalT &center = normals (i,j);
        if (!isFinite (center)) continue;

        int count = 0;
        const NormalT &up = getNormalOrNull (i, j-half_window_size_, count);
        const NormalT &down = getNormalOrNull (i, j+half_window_size_, count);
        const NormalT &left = getNormalOrNull (i-half_window_size_, j, count);
        const NormalT &right = getNormalOrNull (i+half_window_size_, j, count);
        // Get rid of isolated points
        if (!count) continue;

        float sn1 = squaredNormalsDiff (up, center);
        float sn2 = squaredNormalsDiff (down, center);
        float r1 = sn1 + sn2;
        float r2 = squaredNormalsDiff (right, center) + squaredNormalsDiff (left, center);

        float d = std::min (r1, r2);
        if (d < first_threshold_) continue;

        sn1 = std::sqrt (sn1);
        sn2 = std::sqrt (sn2);
        float b1 = normalsDiff (right, up) * sn1;
        b1+= normalsDiff (left, down) * sn2;
        float b2 = normalsDiff (right, down) * sn2;
        b2+= normalsDiff (left, up) * sn1;
        float B = std::min (b1, b2);
        float A = r2 - r1 - 2*B;

        response (i,j) = ((B < 0) && ((B + A) > 0)) ? r1 - ((B*B)/A) : d;
      }
    }
  }
  else
  {
#if OPENMP_LEGACY_CONST_DATA_SHARING_RULE
#pragma omp parallel for \
  default(none) \
  shared(input, normals, response) \
  num_threads(threads_)
#else
#pragma omp parallel for \
  default(none) \
  shared(h, input, normals, response, w) \
  num_threads(threads_)
#endif
    for(int j = half_window_size_; j < h; ++j)
    {
      for(int i = half_window_size_; i < w; ++i)
      {
        if (!isFinite (input (i,j))) continue;
        const NormalT &center = normals (i,j);
        if (!isFinite (center)) continue;

        int count = 0;
        const NormalT &up = getNormalOrNull (i, j-half_window_size_, count);
        const NormalT &down = getNormalOrNull (i, j+half_window_size_, count);
        const NormalT &left = getNormalOrNull (i-half_window_size_, j, count);
        const NormalT &right = getNormalOrNull (i+half_window_size_, j, count);
        const NormalT &upleft = getNormalOrNull (i-half_window_size_, j-half_window_size_, count);
        const NormalT &upright = getNormalOrNull (i+half_window_size_, j-half_window_size_, count);
        const NormalT &downleft = getNormalOrNull (i-half_window_size_, j+half_window_size_, count);
        const NormalT &downright = getNormalOrNull (i+half_window_size_, j+half_window_size_, count);
        // Get rid of isolated points
        if (!count) continue;

        std::vector<float> r (4,0);

        r[0] = squaredNormalsDiff (up, center);
        r[0]+= squaredNormalsDiff (down, center);

        r[1] = squaredNormalsDiff (upright, center);
        r[1]+= squaredNormalsDiff (downleft, center);

        r[2] = squaredNormalsDiff (right, center);
        r[2]+= squaredNormalsDiff (left, center);

        r[3] = squaredNormalsDiff (downright, center);
        r[3]+= squaredNormalsDiff (upleft, center);

        float d = *(std::min_element (r.begin (), r.end ()));

        if (d < first_threshold_) continue;

        std::vector<float> B (4,0);
        std::vector<float> A (4,0);
        std::vector<float> sumAB (4,0);
        B[0] = normalsDiff (upright, up) * normalsDiff (up, center);
        B[0]+= normalsDiff (downleft, down) * normalsDiff (down, center);
        B[1] = normalsDiff (right, upright) * normalsDiff (upright, center);
        B[1]+= normalsDiff (left, downleft) * normalsDiff (downleft, center);
        B[2] = normalsDiff (downright, right) * normalsDiff (downright, center);
        B[2]+= normalsDiff (upleft, left) * normalsDiff (upleft, center);
        B[3] = normalsDiff (down, downright) * normalsDiff (downright, center);
        B[3]+= normalsDiff (up, upleft) * normalsDiff (upleft, center);
        A[0] = r[1] - r[0] - B[0] - B[0];
        A[1] = r[2] - r[1] - B[1] - B[1];
        A[2] = r[3] - r[2] - B[2] - B[2];
        A[3] = r[0] - r[3] - B[3] - B[3];
        sumAB[0] = A[0] + B[0];
        sumAB[1] = A[1] + B[1];
        sumAB[2] = A[2] + B[2];
        sumAB[3] = A[3] + B[3];
        if ((*std::max_element (B.begin (), B.end ()) < 0) &&
            (*std::min_element (sumAB.begin (), sumAB.end ()) > 0))
        {
          std::vector<float> D (4,0);
          D[0] = B[0] * B[0] / A[0];
          D[1] = B[1] * B[1] / A[1];
          D[2] = B[2] * B[2] / A[2];
          D[3] = B[3] * B[3] / A[3];
          response (i,j) = *(std::min (D.begin (), D.end ()));
        }
        else
          response (i,j) = d;
      }
    }
  }
  // Non maximas suppression
  pcl::Indices indices = *indices_;
  std::sort (indices.begin (), indices.end (), [this] (int p1, int p2) { return greaterCornernessAtIndices (p1, p2); });

  output.clear ();
  output.reserve (input_->size ());

  std::vector<bool> occupency_map (indices.size (), false);
  const int width (input_->width);
  const int height (input_->height);

#if OPENMP_LEGACY_CONST_DATA_SHARING_RULE
#pragma omp parallel for \
  default(none) \
  shared(indices, occupency_map, output) \
  num_threads(threads_)
#else
#pragma omp parallel for \
  default(none) \
  shared(height, indices, occupency_map, output, width) \
  num_threads(threads_)
#endif
  for (int i = 0; i < static_cast<int>(indices.size ()); ++i)
  {
    int idx = indices[static_cast<std::size_t>(i)];
    if (((*response_)[idx] < second_threshold_) || occupency_map[idx])
      continue;

    PointOutT p;
    p.getVector3fMap () = (*input_)[idx].getVector3fMap ();
    p.intensity = response_->points [idx];

#pragma omp critical
    {
      output.push_back (p);
      keypoints_indices_->indices.push_back (idx);
    }

    const int x = idx % width;
    const int y = idx / width;
    const int u_end = std::min (width, x + half_window_size_);
    const int v_end = std::min (height, y + half_window_size_);
    for(int v = std::max (0, y - half_window_size_); v < v_end; ++v)
      for(int u = std::max (0, x - half_window_size_); u < u_end; ++u)
        occupency_map[v*width + u] = true;
  }

  output.height = 1;
  output.width = output.size();
  // we don not change the denseness
  output.is_dense = true;
}

} // namespace pcl

#define PCL_INSTANTIATE_TrajkovicKeypoint3D(T,U,N) template class PCL_EXPORTS pcl::TrajkovicKeypoint3D<T,U,N>;

#endif


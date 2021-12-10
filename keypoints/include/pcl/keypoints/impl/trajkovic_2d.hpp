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


#ifndef PCL_TRAJKOVIC_KEYPOINT_2D_IMPL_H_
#define PCL_TRAJKOVIC_KEYPOINT_2D_IMPL_H_


namespace pcl
{

template <typename PointInT, typename PointOutT, typename IntensityT> bool
TrajkovicKeypoint2D<PointInT, PointOutT, IntensityT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
    return (false);

  keypoints_indices_.reset (new pcl::PointIndices);
  keypoints_indices_->indices.reserve (input_->size ());

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

  return (true);
}


template <typename PointInT, typename PointOutT, typename IntensityT> void
TrajkovicKeypoint2D<PointInT, PointOutT, IntensityT>::detectKeypoints (PointCloudOut &output)
{
  response_.reset (new pcl::PointCloud<float> (input_->width, input_->height));
  const int w = static_cast<int> (input_->width) - half_window_size_;
  const int h = static_cast<int> (input_->height) - half_window_size_;

  if (method_ == pcl::TrajkovicKeypoint2D<PointInT, PointOutT, IntensityT>::FOUR_CORNERS)
  {
#if OPENMP_LEGACY_CONST_DATA_SHARING_RULE
#pragma omp parallel for \
  default(none) \
  num_threads(threads_)
#else
#pragma omp parallel for \
  default(none) \
  shared(h, w) \
  num_threads(threads_)
#endif
    for(int j = half_window_size_; j < h; ++j)
    {
      for(int i = half_window_size_; i < w; ++i)
      {
        float center = intensity_ ((*input_) (i,j));
        float up = intensity_ ((*input_) (i, j-half_window_size_));
        float down = intensity_ ((*input_) (i, j+half_window_size_));
        float left = intensity_ ((*input_) (i-half_window_size_, j));
        float right = intensity_ ((*input_) (i+half_window_size_, j));

        float up_center = up - center;
        float r1 = up_center * up_center;
        float down_center = down - center;
        r1+= down_center * down_center;

        float right_center = right - center;
        float r2 = right_center * right_center;
        float left_center = left - center;
        r2+= left_center * left_center;

        float d = std::min (r1, r2);

        if (d < first_threshold_)
          continue;

        float b1 = (right - up) * up_center;
        b1+= (left - down) * down_center;
        float b2 = (right - down) * down_center;
        b2+= (left - up) * up_center;
        float B = std::min (b1, b2);
        float A = r2 - r1 - 2*B;

        (*response_) (i,j) = ((B < 0) && ((B + A) > 0)) ? r1 - ((B*B)/A) : d;
      }
    }
  }
  else
  {
#if OPENMP_LEGACY_CONST_DATA_SHARING_RULE
#pragma omp parallel for \
  default(none) \
  num_threads(threads_)
#else
#pragma omp parallel for \
  default(none) \
  shared(h, w) \
  num_threads(threads_)
#endif
    for(int j = half_window_size_; j < h; ++j)
    {
      for(int i = half_window_size_; i < w; ++i)
      {
        float center = intensity_ ((*input_) (i,j));
        float up = intensity_ ((*input_) (i, j-half_window_size_));
        float down = intensity_ ((*input_) (i, j+half_window_size_));
        float left = intensity_ ((*input_) (i-half_window_size_, j));
        float right = intensity_ ((*input_) (i+half_window_size_, j));
        float upleft = intensity_ ((*input_) (i-half_window_size_, j-half_window_size_));
        float upright = intensity_ ((*input_) (i+half_window_size_, j-half_window_size_));
        float downleft = intensity_ ((*input_) (i-half_window_size_, j+half_window_size_));
        float downright = intensity_ ((*input_) (i+half_window_size_, j+half_window_size_));
        std::vector<float> r (4,0);

        float up_center = up - center;
        r[0] = up_center * up_center;
        float down_center = down - center;
        r[0]+= down_center * down_center;

        float upright_center = upright - center;
        r[1] = upright_center * upright_center;
        float downleft_center = downleft - center;
        r[1]+= downleft_center * downleft_center;

        float right_center = right - center;
        r[2] = right_center * right_center;
        float left_center = left - center;
        r[2]+= left_center * left_center;

        float downright_center = downright - center;
        r[3] = downright_center * downright_center;
        float upleft_center = upleft - center;
        r[3]+= upleft_center * upleft_center;

        float d = *(std::min_element (r.begin (), r.end ()));

        if (d < first_threshold_)
          continue;

        std::vector<float> B (4,0);
        std::vector<float> A (4,0);
        std::vector<float> sumAB (4,0);
        B[0] = (upright - up) * up_center;
        B[0]+= (downleft - down) * down_center;
        B[1] = (right - upright) * upright_center;
        B[1]+= (left - downleft) * downleft_center;
        B[2] = (downright - right) * downright_center;
        B[2]+= (upleft - left) * upleft_center;
        B[3] = (down - downright) * downright_center;
        B[3]+= (up - upleft) * upleft_center;
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
          (*response_) (i,j) = *(std::min (D.begin (), D.end ()));
        }
        else
          (*response_) (i,j) = d;
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
  for (std::size_t i = 0; i < indices.size (); ++i)
  {
    int idx = indices[i];
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
  output.is_dense = input_->is_dense;
}

} // namespace pcl

#define PCL_INSTANTIATE_TrajkovicKeypoint2D(T,U,I) template class PCL_EXPORTS pcl::TrajkovicKeypoint2D<T,U,I>;

#endif


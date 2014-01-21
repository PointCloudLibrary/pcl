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

#ifndef PCL_SUSAN_KEYPOINT_2D_IMPL_H_
#define PCL_SUSAN_KEYPOINT_2D_IMPL_H_

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::SusanKeypoint2D<PointInT, PointOutT, IntensityT>::lineToCentroid (int x1, int y1, std::vector<Eigen::Vector2i>& points)
{
  static int x0 = half_window_size_;
  static int y0 = half_window_size_;
  points.reserve (window_size_ * window_size_);
  int dx = abs (x1-x0);
  int dy = abs (y1-y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true)
  {
    if ((x0 > 0) && (y0 > 0) && (x0 < window_size_) && (y0 < window_size_))
      points.push_back (Eigen::Vector2i (x0, y0));
    if ((x0 == x1) && (y0 == y1))
      break;
    int e2 = 2 * err;
    if (e2 > -dy)
    {
      err-= dy;
      x0 += sx;
    }

    if ((x0 == x1) && (y0 == y1))
    {
      if ((x0 > 0) && (y0 > 0) && (x0 < window_size_) && (y0 < window_size_))
        points.push_back (Eigen::Vector2i (x0, y0));
      break;
    }

    if (e2 < dx)
    {
      err+= dx;
      y0+= sy;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> bool
pcl::SusanKeypoint2D<PointInT, PointOutT, IntensityT>::initCompute ()
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
    PCL_ERROR ("[pcl::%s::initCompute] Window size must be at least 3x3!\n", name_.c_str ());
    return (false);
  }

  half_window_size_ = window_size_ / 2;
  geometric_threshold_ = 0.5 * window_size_ * window_size_;
  return (true);
}
/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::SusanKeypoint2D<PointInT, PointOutT, IntensityT>::detectKeypoints (PointCloudOut &output)
{
  response_.reset (new pcl::PointCloud<float> (input_->width, input_->height));
  pcl::PointCloud<float> &response = *response_;

  int w = static_cast<int> (input_->width) - half_window_size_;
  int h = static_cast<int> (input_->height) - half_window_size_;

  for(int j = half_window_size_; j < h; ++j)
  {
    for(int i = half_window_size_; i < w; ++i)
    {
      float center = intensity_ ((*input_) (i,j));
      Eigen::Vector2f nucleus (i, j);
      Eigen::Vector2f centroid = Eigen::Vector2f::Zero ();
      Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> usan (window_size_, window_size_);
      usan.setZero ();

      float area = 0;
      for (int y = 0, jj = j - half_window_size_; y < window_size_; ++jj, ++y)
        for (int x = 0, ii = i - half_window_size_; x < window_size_; ++ii, ++x)
        {
          float intensity = intensity_ ((*input_) (ii,jj));
          float c = fabs (center - intensity);
          if (c <= threshold_)
          {
            usan (x, y) = true;
            ++area;
            centroid[0] += static_cast<float> (ii);
            centroid[1] += static_cast<float> (jj);
          }
        }

      if (area >= geometric_threshold_)
        continue;

      response (i, j) = 1.f / (geometric_threshold_ - area);
      if (!test_distance_)
        continue;

      centroid /= area;
      Eigen::Vector2f nc = centroid - nucleus;
      if (nc.norm () <= distance_threshold_)
      {
        response (i,j) = 0;
        continue;
      }

      if (!test_contiguity_)
        continue;

      // All the points on the line from the nucleus to the centroid must be part of the USAN
      std::vector<Eigen::Vector2i> nucleus_center;
      float shifted_x = half_window_size_ + centroid[0] - nucleus[0];
      float shifted_y = half_window_size_ + centroid[1] - nucleus[1];
      lineToCentroid (shifted_x, shifted_y, nucleus_center);
      bool all_usan_on_line = true;
      for (std::vector<Eigen::Vector2i>::const_iterator p = nucleus_center.begin ();
           p != nucleus_center.end ();
           ++p)
      {
        if (usan ((*p)[0], (*p)[1]) == false)
        {
          all_usan_on_line = false;
          break;
        }
      }

      if (!all_usan_on_line)
      {
        response (i,j) = 0;
        continue;
      }
    }
  }

  // Non maximas suppression
  std::vector<int> indices = *indices_;
  std::sort (indices.begin (), indices.end (),
             boost::bind (&SusanKeypoint2D::greaterCornernessAtIndices, this, _1, _2));

  output.clear ();
  output.reserve (input_->size ());

  std::vector<bool> occupency_map (indices.size (), false);
  const int width (input_->width);
  const int height (input_->height);
  const int occupency_map_size (indices.size ());

#ifdef _OPENMP
#pragma omp parallel for shared (output, keypoints_indices_) num_threads (threads_)
#endif
  for (int i = 0; i < indices.size (); ++i)
  {
    int idx = indices[i];
    if ((response_->points[idx] == 0) || occupency_map[idx])
      continue;

    PointOutT p;
    p.getVector3fMap () = input_->points[idx].getVector3fMap ();
    p.intensity = response_->points [idx];

#ifdef _OPENMP
#pragma omp critical
#endif
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
  output.width = static_cast<uint32_t> (output.size());
  // we don not change the denseness
  output.is_dense = input_->is_dense;
}

#define PCL_INSTANTIATE_SusanKeypoint2D(T,U,I) template class PCL_EXPORTS pcl::SusanKeypoint2D<T,U,I>;
#endif

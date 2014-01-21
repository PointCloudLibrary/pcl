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

#ifndef PCL_SUSAN_KEYPOINT_3D_IMPL_HPP_
#define PCL_SUSAN_KEYPOINT_3D_IMPL_HPP_

#include <pcl/keypoints/susan.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> inline const NormalT&
pcl::SusanKeypoint3D<PointInT, PointOutT, NormalT>::getNormalOrNull (std::size_t pos, int& counter) const
{
  static const NormalT null;
  if (!isFinite (normals_->points[pos])) return (null);
  ++counter;
  return (normals_->points[pos]);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> inline const NormalT&
pcl::SusanKeypoint3D<PointInT, PointOutT, NormalT>::getNormalOrNull (int u, int v, int& counter) const
{
  static const NormalT null;
  const NormalT& n = (*normals_) (u, v);
  if (!isFinite (n)) return (null);
  ++counter;
  return (n);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> inline float
pcl::SusanKeypoint3D<PointInT, PointOutT, NormalT>::normalsDiff (const NormalT& a, const NormalT& b) const
{
  return (a.normal_x*b.normal_x + a.normal_y*b.normal_y + a.normal_z*b.normal_z);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::SusanKeypoint3D<PointInT, PointOutT, NormalT>::lineToCentroid (int x1, int y1, std::vector<Eigen::Vector2i>& points)
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
template <typename PointInT, typename PointOutT, typename NormalT> bool
pcl::SusanKeypoint3D<PointInT, PointOutT, NormalT>::initCompute ()
{
  if (!input_->isOrganized ())
  {
    if (!Keypoint<PointInT, PointOutT>::initCompute ())
    {
      PCL_ERROR ("[pcl::%s::initCompute] init failed!\n", name_.c_str ());
      return (false);
    }
    
    if (!normals_)
    {
      NormalsPtr normals (new Normals ());
      pcl::NormalEstimation<PointInT, NormalT> normal_estimation;
      normal_estimation.setInputCloud (surface_);
      normal_estimation.setRadiusSearch (search_radius_);
      normal_estimation.compute (*normals);
      normals_ = normals;
    }
  }
  else
  {
    if (!PCLBase<PointInT>::initCompute ())
      return (false);
    
    keypoints_indices_.reset (new pcl::PointIndices);
    keypoints_indices_->indices.reserve (input_->size ());

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

    if (!surface_)
      surface_ = input_;

    if (!normals_)
    {
      NormalsPtr normals (new Normals ());
      IntegralImageNormalEstimation<PointInT, NormalT> normal_estimation;
      normal_estimation.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointInT, NormalT>::SIMPLE_3D_GRADIENT);
      normal_estimation.setInputCloud (surface_);
      normal_estimation.setNormalSmoothingSize (5.0);
      normal_estimation.compute (*normals);
      normals_ = normals;
    }

    if (test_distance_ && (distance_threshold_ < 0.01))
    {
      PCL_WARN ("[pcl::%s::initCompute] Distance threshold metric is in pixels. Please consider higher value 0.01 for instance.\n", name_.c_str ());
    }
  }
  
  if (normals_->size () != surface_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] normals given, but the number of normals %d does not match the number of input points %d!\n", name_.c_str (), normals_->size (), input_->size ());
    return (false);
  }

  threshold_ = cos (angular_threshold_);
  
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::SusanKeypoint3D<PointInT, PointOutT, NormalT>::detectKeypoints (PointCloudOut &output)
{
  if (input_->isOrganized ())
    detectKeypointsOrganized (output);
  else
    detectKeypointsNonOrganized (output);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::SusanKeypoint3D<PointInT, PointOutT, NormalT>::detectKeypointsNonOrganized (PointCloudOut &output)
{
  typename PointCloudOut::Ptr response (new PointCloudOut);
  response->resize (input_->size ());

  const int input_size (input_->size ());
#if defined (HAVE_OPENMP) && (defined (_WIN32) || ((__GNUC__ > 4) && (__GNUC_MINOR__ > 2)))
#pragma omp parallel for num_threads (threads_)
#endif
  for (int point_index = 0; point_index < input_size; ++point_index)
  {
    const PointInT& point = input_->points [point_index];
    const NormalT& normal = normals_->points [point_index];
    response->points [point_index].getVector3fMap () = point.getVector3fMap ();
    response->points [point_index].intensity = 0;
    if (!isFinite (point))
      continue;
    if (!isFinite (normal))
      continue;
    const Eigen::Vector3f &nucleus = point.getVector3fMap ();
    const Eigen::Vector3f &nucleus_normal = normals_->points [point_index].getNormalVector3fMap ();
    int counter = 0;
    std::vector<int> nn_indices;
    std::vector<float> nn_dists;
    if (searchForNeighbors (point_index, search_radius_, nn_indices, nn_dists) <= 1)
      continue;

    float area = 0;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero ();
    std::vector<bool> usan (nn_indices.size (), false);
    std::vector<bool>::iterator usan_it = usan.begin ();
    for (std::vector<int>::const_iterator index = nn_indices.begin (); index != nn_indices.end (); ++index, ++usan_it)
    {
      if (point_index == *index)
        continue;

      const NormalT &n = getNormalOrNull (*index, counter);
      if (normalsDiff (normal, n) >= threshold_)
      {
        *usan_it = true;
        area+= 1;
        centroid += input_->points[*index].getVector3fMap ();
      }
    }

    if (area > 0)
    {
      centroid /= area;
      float geometric_threshold = 0.25 * counter;
      if (area < geometric_threshold)
      {
        response->points [point_index].intensity = 1.f / (geometric_threshold - area);
        // Remove false positives
        // If the centroid is not far enough then it is not a corner
        if (test_distance_)
        {
          Eigen::Vector3f nc = centroid - nucleus;
          float nc_sq_norm = nc.squaredNorm ();
          float nc_norm = sqrt (nc_sq_norm);
          if (nc_norm <= distance_threshold_)
          {
            response->points [point_index].intensity = 0;
            continue;
          }

          if (test_contiguity_)
          {
            std::vector<bool>::const_iterator const_usan_it = usan.begin ();
            // All the points on the line from the nucleus to the centroid must be part of the USAN
            for (std::vector<int>::const_iterator index = nn_indices.begin ();
                 index != nn_indices.end ();
                 ++index, ++usan_it)
            {
              // skip if me
              if (point_index == *index)
                continue;
              // skip if not in USAN
              if (!(*const_usan_it))
                continue;
              Eigen::Vector3f np = input_->points[*index].getVector3fMap () - nucleus;
              // check if the point is between the nucleus and the centroid
              float r = np.dot (nc) / nc_sq_norm;
              if ((r > 0) && (r < 1))
                continue;
              else
              {
                response->points [point_index].intensity = 0;
                break;
              }
            }
          }
        }
      }
    }
  }

  if (!nonmax_)
  {
    output = *response;
    output.is_dense = response->is_dense;
    for (size_t i = 0; i < response->size (); ++i)
      keypoints_indices_->indices.push_back (i);
  }
  else
  {
    output.points.clear ();
    output.points.reserve (response->size ());

#if defined (HAVE_OPENMP) && (defined (_WIN32) || ((__GNUC__ > 4) && (__GNUC_MINOR__ > 2)))
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for (int idx = 0; idx < input_size; ++idx)
    {
      const PointOutT& point = response->points [idx];
      const NormalT& normal = normals_->points [idx];
      const float &intensity = response->points[idx].intensity;
      if (!isFinite (point) || !isFinite (normal) || (intensity == 0))
        continue;
      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      searchForNeighbors (idx, search_radius_, nn_indices, nn_dists);
      bool is_maxima = true;
      for (std::vector<int>::const_iterator neighbor = nn_indices.begin (); neighbor != nn_indices.end (); ++neighbor)
      {
        if (intensity < response->points[*neighbor].intensity)
        {
          is_maxima = false;
          break;
        }
      }
      if (is_maxima)
#if defined (HAVE_OPENMP) && (defined (_WIN32) || ((__GNUC__ > 4) && (__GNUC_MINOR__ > 2)))
#pragma omp critical
#endif
      {
        output.points.push_back (response->points[idx]);
        keypoints_indices_->indices.push_back (idx);
      }
    }

    output.height = 1;
    output.width = static_cast<uint32_t> (output.points.size ());
    output.is_dense = true;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::SusanKeypoint3D<PointInT, PointOutT, NormalT>::detectKeypointsOrganized (PointCloudOut &output)
{
  pcl::PointCloud<float>::Ptr response_ (new pcl::PointCloud<float> (input_->width, input_->height));
  pcl::PointCloud<float> &response = *response_;
  
  int w = static_cast<int> (input_->width) - half_window_size_;
  int h = static_cast<int> (input_->height) - half_window_size_;
  const float geometric_threshold = 0.25 * window_size_ * window_size_;
#if defined (HAVE_OPENMP) && (defined (_WIN32) || ((__GNUC__ > 4) && (__GNUC_MINOR__ > 2)))
#pragma omp parallel for num_threads (threads_)
#endif
  for(int j = half_window_size_; j < h; ++j)
  {
    for(int i = half_window_size_; i < w; ++i)
    {
      (*response_) (i, j) = 0;
      if (!pcl::isFinite ((*input_) (i,j)))
        continue;
      
      const NormalT& normal = (*normals_) (i,j);
      if (!pcl::isFinite (normal))
        continue;
      
      Eigen::Vector2f nucleus (i, j);
      Eigen::Vector2f centroid = Eigen::Vector2f::Zero ();
      Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> usan (window_size_, window_size_);
      usan.setZero ();
      
      int area = 0;
      int counter = 0;
      for (int y = 0, jj = j - half_window_size_; y < window_size_; ++jj, ++y)
        for (int x = 0, ii = i - half_window_size_; x < window_size_; ++ii, ++x)
        {
          const NormalT& n = getNormalOrNull (ii,jj,counter);
          if (normalsDiff (normal, n) >= threshold_)
          { 
            usan (x, y) = true;
            ++area;
            centroid[0] += static_cast<float> (ii);
            centroid[1] += static_cast<float> (jj);
          }
        }

      if (counter == 0)
        continue;

      if (area >= geometric_threshold)
        continue;
      
      response (i, j) = 1.f / (geometric_threshold - area);
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

  if (!nonmax_)
  {
    output = PointCloudOut (input_->width, input_->height);
    output.is_dense = response_->is_dense;
    keypoints_indices_->indices.resize (input_->size ());
#ifdef _OPENMP
#pragma omp parallel for num_threads (threads_)
#endif
    for (size_t i = 0; i < response_->size (); ++i)
    {
      keypoints_indices_->indices[i] = i;
      output.points[i].getVector3fMap () = input_->points[i].getVector3fMap ();
      output.points[i].intensity = response_->points[i];
    }
  }
  else
  {
    // Non maximas suppression
    std::vector<int> indices = *indices_;
    std::sort (indices.begin (), indices.end (),
               boost::bind (&SusanKeypoint3D::greaterCornernessAtIndices, this, _1, _2, response_));

    output.clear ();
    output.reserve (input_->size ());

    std::vector<bool> occupency_map (indices.size (), false);
    const int width (input_->width);
    const int height (input_->height);
    const int occupency_map_size (indices.size ());
    int nb_occupied = 0;
  
#ifdef _OPENMP
#pragma omp parallel for shared (output, keypoints_indices_) num_threads (threads_)
#endif
    for (int i = 0; i < indices.size () && nb_occupied < input_->size (); ++i)
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
        {
          occupency_map[v*width + u] = true;
          ++nb_occupied;
        }
    }

    output.height = 1;
    output.width = static_cast<uint32_t> (output.size());
    // we don not change the denseness
    output.is_dense = input_->is_dense;
  }
}

#define PCL_INSTANTIATE_SUSAN_3D (T,U,N) template class PCL_EXPORTS pcl::SusanKeypoint3D<T,U,N>;
#endif // #ifndef PCL_SUSAN_KEYPOINT_3D_IMPL_H_

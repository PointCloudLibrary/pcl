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

#ifndef PCL_SEARCH_IMPL_ORGANIZED_NEIGHBOR_SEARCH_H_
#define PCL_SEARCH_IMPL_ORGANIZED_NEIGHBOR_SEARCH_H_

#include <pcl/search/organized.h>
#include <pcl/common/eigen.h>
#include <pcl/common/time.h>
#include <Eigen/Eigenvalues>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighbor<PointT>::radiusSearch (const               PointT &query,
                                                      const double        radius,
                                                      std::vector<int>    &k_indices,
                                                      std::vector<float>  &k_sqr_distances,
                                                      unsigned int        max_nn) const
{
  // NAN test
  assert (isFinite (query) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

  // search window
  unsigned left, right, top, bottom;
  //unsigned x, y, idx;
  float squared_distance;
  double squared_radius;

  k_indices.clear ();
  k_sqr_distances.clear ();

  squared_radius = radius * radius;

  this->getProjectedRadiusSearchBox (query, static_cast<float> (squared_radius), left, right, top, bottom);

  // iterate over search box
  if (max_nn == 0 || max_nn >= static_cast<unsigned int> (input_->points.size ()))
    max_nn = static_cast<unsigned int> (input_->points.size ());

  k_indices.reserve (max_nn);
  k_sqr_distances.reserve (max_nn);

  unsigned yEnd  = (bottom + 1) * input_->width + right + 1;
  register unsigned idx  = top * input_->width + left;
  unsigned skip = input_->width - right + left - 1;
  unsigned xEnd = idx - left + right + 1;

  for (; xEnd != yEnd; idx += skip, xEnd += input_->width)
  {
    for (; idx < xEnd; ++idx)
    {
      if (!mask_[idx] || !isFinite (input_->points[idx]))
        continue;

      float dist_x = input_->points[idx].x - query.x;
      float dist_y = input_->points[idx].y - query.y;
      float dist_z = input_->points[idx].z - query.z;
      squared_distance = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
      //squared_distance = (input_->points[idx].getVector3fMap () - query.getVector3fMap ()).squaredNorm ();
      if (squared_distance <= squared_radius)
      {
        k_indices.push_back (idx);
        k_sqr_distances.push_back (squared_distance);
        // already done ?
        if (k_indices.size () == max_nn)
        {
          if (sorted_results_)
            this->sortResults (k_indices, k_sqr_distances);
          return (max_nn);
        }
      }
    }
  }
  if (sorted_results_)
    this->sortResults (k_indices, k_sqr_distances);  
  return (static_cast<int> (k_indices.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighbor<PointT>::nearestKSearch (const PointT &query,
                                                        int k,
                                                        std::vector<int> &k_indices,
                                                        std::vector<float> &k_sqr_distances) const
{
  assert (isFinite (query) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  if (k < 1)
  {
    k_indices.clear ();
    k_sqr_distances.clear ();
    return (0);
  }

  Eigen::Vector3f queryvec (query.x, query.y, query.z);
  // project query point on the image plane
  //Eigen::Vector3f q = KR_ * query.getVector3fMap () + projection_matrix_.block <3, 1> (0, 3);
  Eigen::Vector3f q (KR_ * queryvec + projection_matrix_.block <3, 1> (0, 3));
  int xBegin = int(q [0] / q [2] + 0.5f);
  int yBegin = int(q [1] / q [2] + 0.5f);
  int xEnd   = xBegin + 1; // end is the pixel that is not used anymore, like in iterators
  int yEnd   = yBegin + 1;

  // the search window. This is supposed to shrink within the iterations
  unsigned left = 0;
  unsigned right = input_->width - 1;
  unsigned top = 0;
  unsigned bottom = input_->height - 1;

  std::priority_queue <Entry> results;
  //std::vector<Entry> k_results;
  //k_results.reserve (k);
  // add point laying on the projection of the query point.
  if (xBegin >= 0 && 
      xBegin < static_cast<int> (input_->width) && 
      yBegin >= 0 && 
      yBegin < static_cast<int> (input_->height))
    testPoint (query, k, results, yBegin * input_->width + xBegin);
  else // point lys
  {
    // find the box that touches the image border -> dont waste time evaluating boxes that are completely outside the image!
    int dist = std::numeric_limits<int>::max ();

    if (xBegin < 0)
      dist = -xBegin;
    else if (xBegin >= static_cast<int> (input_->width))
      dist = xBegin - static_cast<int> (input_->width);

    if (yBegin < 0)
      dist = std::min (dist, -yBegin);
    else if (yBegin >= static_cast<int> (input_->height))
      dist = std::min (dist, yBegin - static_cast<int> (input_->height));

    xBegin -= dist;
    xEnd   += dist;

    yBegin -= dist;
    yEnd   += dist;
  }

  
  // stop used as isChanged as well as stop.
  bool stop = false;
  do
  {
    // increment box size
    --xBegin;
    ++xEnd;
    --yBegin;
    ++yEnd;

    // the range in x-direction which intersects with the image width
    int xFrom = xBegin;
    int xTo   = xEnd;
    clipRange (xFrom, xTo, 0, input_->width);
    
    // if x-extend is not 0
    if (xTo > xFrom)
    {
      // if upper line of the rectangle is visible and x-extend is not 0
      if (yBegin >= 0 && yBegin < static_cast<int> (input_->height))
      {
        int idx   = yBegin * input_->width + xFrom;
        int idxTo = idx + xTo - xFrom;
        for (; idx < idxTo; ++idx)
          stop = testPoint (query, k, results, idx) || stop;
      }
      

      // the row yEnd does NOT belong to the box -> last row = yEnd - 1
      // if lower line of the rectangle is visible
      if (yEnd > 0 && yEnd <= static_cast<int> (input_->height))
      {
        int idx   = (yEnd - 1) * input_->width + xFrom;
        int idxTo = idx + xTo - xFrom;

        for (; idx < idxTo; ++idx)
          stop = testPoint (query, k, results, idx) || stop;
      }
      
      // skip first row and last row (already handled above)
      int yFrom = yBegin + 1;
      int yTo   = yEnd - 1;
      clipRange (yFrom, yTo, 0, input_->height);
      
      // if we have lines in between that are also visible
      if (yFrom < yTo)
      {
        if (xBegin >= 0 && xBegin < static_cast<int> (input_->width))
        {
          int idx   = yFrom * input_->width + xBegin;
          int idxTo = yTo * input_->width + xBegin;

          for (; idx < idxTo; idx += input_->width)
            stop = testPoint (query, k, results, idx) || stop;
        }
        
        if (xEnd > 0 && xEnd <= static_cast<int> (input_->width))
        {
          int idx   = yFrom * input_->width + xEnd - 1;
          int idxTo = yTo * input_->width + xEnd - 1;

          for (; idx < idxTo; idx += input_->width)
            stop = testPoint (query, k, results, idx) || stop;
        }
        
      }
      // stop here means that the k-nearest neighbor changed -> recalculate bounding box of ellipse.
      if (stop)
        getProjectedRadiusSearchBox (query, results.top ().distance, left, right, top, bottom);
      
    }
    // now we use it as stop flag -> if bounding box is completely within the already examined search box were done!
    stop = (static_cast<int> (left)   >= xBegin && static_cast<int> (left)   < xEnd && 
            static_cast<int> (right)  >= xBegin && static_cast<int> (right)  < xEnd &&
            static_cast<int> (top)    >= yBegin && static_cast<int> (top)    < yEnd && 
            static_cast<int> (bottom) >= yBegin && static_cast<int> (bottom) < yEnd);
    
  } while (!stop);

  
  k_indices.resize (results.size ());
  k_sqr_distances.resize (results.size ());
  size_t idx = results.size () - 1;
  while (!results.empty ())
  {
    k_indices [idx] = results.top ().index;
    k_sqr_distances [idx] = results.top ().distance;
    results.pop ();
    --idx;
  }
  
  return (static_cast<int> (k_indices.size ()));
}

////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighbor<PointT>::getProjectedRadiusSearchBox (const PointT& point,
                                                                     float squared_radius,
                                                                     unsigned &minX,
                                                                     unsigned &maxX,
                                                                     unsigned &minY,
                                                                     unsigned &maxY) const
{
  Eigen::Vector3f queryvec (point.x, point.y, point.z);
  //Eigen::Vector3f q = KR_ * point.getVector3fMap () + projection_matrix_.block <3, 1> (0, 3);
  Eigen::Vector3f q (KR_ * queryvec + projection_matrix_.block <3, 1> (0, 3));

  float a = squared_radius * KR_KRT_.coeff (8) - q [2] * q [2];
  float b = squared_radius * KR_KRT_.coeff (7) - q [1] * q [2];
  float c = squared_radius * KR_KRT_.coeff (4) - q [1] * q [1];
  int min, max;
  // a and c are multiplied by two already => - 4ac -> - ac
  float det = b * b - a * c;
  if (det < 0)
  {
    minY = 0;
    maxY = input_->height - 1;
  }
  else
  {
    float y1 = static_cast<float> ((b - sqrt (det)) / a);
    float y2 = static_cast<float> ((b + sqrt (det)) / a);

    min = std::min (static_cast<int> (floor (y1)), static_cast<int> (floor (y2)));
    max = std::max (static_cast<int> (ceil (y1)), static_cast<int> (ceil (y2)));
    minY = static_cast<unsigned> (std::min (static_cast<int> (input_->height) - 1, std::max (0, min)));
    maxY = static_cast<unsigned> (std::max (std::min (static_cast<int> (input_->height) - 1, max), 0));
  }

  b = squared_radius * KR_KRT_.coeff (6) - q [0] * q [2];
  c = squared_radius * KR_KRT_.coeff (0) - q [0] * q [0];

  det = b * b - a * c;
  if (det < 0)
  {
    minX = 0;
    maxX = input_->width - 1;
  }
  else
  {
    float x1 = static_cast<float> ((b - sqrt (det)) / a);
    float x2 = static_cast<float> ((b + sqrt (det)) / a);

    min = std::min (static_cast<int> (floor (x1)), static_cast<int> (floor (x2)));
    max = std::max (static_cast<int> (ceil (x1)), static_cast<int> (ceil (x2)));
    minX = static_cast<unsigned> (std::min (static_cast<int> (input_->width)- 1, std::max (0, min)));
    maxX = static_cast<unsigned> (std::max (std::min (static_cast<int> (input_->width) - 1, max), 0));
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighbor<PointT>::computeCameraMatrix (Eigen::Matrix3f& camera_matrix) const
{
  pcl::getCameraMatrixFromProjectionMatrix (projection_matrix_, camera_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighbor<PointT>::estimateProjectionMatrix ()
{
  // internally we calculate with double but store the result into float matrices.
  projection_matrix_.setZero ();
  if (input_->height == 1 || input_->width == 1)
  {
    PCL_ERROR ("[pcl::%s::estimateProjectionMatrix] Input dataset is not organized!\n", this->getName ().c_str ());
    return;
  }
  
  const unsigned ySkip = (std::max) (input_->height >> pyramid_level_, unsigned (1));
  const unsigned xSkip = (std::max) (input_->width >> pyramid_level_, unsigned (1));

  std::vector<int> indices;
  indices.reserve (input_->size () >> (pyramid_level_ << 1));
  
  for (unsigned yIdx = 0, idx = 0; yIdx < input_->height; yIdx += ySkip, idx += input_->width * (ySkip - 1))
  {
    for (unsigned xIdx = 0; xIdx < input_->width; xIdx += xSkip, idx += xSkip)
    {
      if (!mask_ [idx])
        continue;

      indices.push_back (idx);
    }
  }

  double residual_sqr = pcl::estimateProjectionMatrix<PointT> (input_, projection_matrix_, indices);
  
  if (fabs (residual_sqr) > eps_ * float (indices.size ()))
  {
    PCL_ERROR ("[pcl::%s::radiusSearch] Input dataset is not from a projective device!\nResidual (MSE) %f, using %d valid points\n", this->getName ().c_str (), residual_sqr / double (indices.size()), indices.size ());
    return;
  }

  // get left 3x3 sub matrix, which contains K * R, with K = camera matrix = [[fx s cx] [0 fy cy] [0 0 1]]
  // and R being the rotation matrix
  KR_ = projection_matrix_.topLeftCorner <3, 3> ();

  // precalculate KR * KR^T needed by calculations during nn-search
  KR_KRT_ = KR_ * KR_.transpose ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> bool
pcl::search::OrganizedNeighbor<PointT>::projectPoint (const PointT& point, pcl::PointXY& q) const
{
  Eigen::Vector3f projected = KR_ * point.getVector3fMap () + projection_matrix_.block <3, 1> (0, 3);
  q.x = projected [0] / projected [2];
  q.y = projected [1] / projected [2];
  return (projected[2] != 0);
}
#define PCL_INSTANTIATE_OrganizedNeighbor(T) template class PCL_EXPORTS pcl::search::OrganizedNeighbor<T>;

#endif

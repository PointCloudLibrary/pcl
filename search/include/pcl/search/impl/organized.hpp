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

      squared_distance = (input_->points[idx].getVector3fMap () - query.getVector3fMap ()).squaredNorm ();
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

  // project query point on the image plane
  Eigen::Vector3f q = KR_ * query.getVector3fMap () + projection_matrix_.block <3, 1> (0, 3);
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
  Eigen::Vector3f q = KR_ * point.getVector3fMap () + projection_matrix_.block <3, 1> (0, 3);

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
    float y1 = (b - sqrt (det)) / a;
    float y2 = (b + sqrt (det)) / a;

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
    float x1 = (b - sqrt (det)) / a;
    float x2 = (b + sqrt (det)) / a;

    min = std::min (static_cast<int> (floor (x1)), static_cast<int> (floor (x2)));
    max = std::max (static_cast<int> (ceil (x1)), static_cast<int> (ceil (x2)));
    minX = static_cast<unsigned> (std::min (static_cast<int> (input_->width)- 1, std::max (0, min)));
    maxX = static_cast<unsigned> (std::max (std::min (static_cast<int> (input_->width) - 1, max), 0));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> template <typename MatrixType> void
pcl::search::OrganizedNeighbor<PointT>::makeSymmetric (MatrixType& matrix, bool use_upper_triangular) const
{
  if (use_upper_triangular && (MatrixType::Flags & Eigen::RowMajorBit) )
  {
    matrix.coeffRef (4) = matrix.coeff (1);
    matrix.coeffRef (8) = matrix.coeff (2);
    matrix.coeffRef (9) = matrix.coeff (6);
    matrix.coeffRef (12) = matrix.coeff (3);
    matrix.coeffRef (13) = matrix.coeff (7);
    matrix.coeffRef (14) = matrix.coeff (11);
  }
  else
  {
    matrix.coeffRef (1) = matrix.coeff (4);
    matrix.coeffRef (2) = matrix.coeff (8);
    matrix.coeffRef (6) = matrix.coeff (9);
    matrix.coeffRef (3) = matrix.coeff (12);
    matrix.coeffRef (7) = matrix.coeff (13);
    matrix.coeffRef (11) = matrix.coeff (14);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighbor<PointT>::computeCameraMatrix (Eigen::Matrix3f& camera_matrix) const
{
  Eigen::Matrix3f cam = KR_KRT_ / KR_KRT_.coeff (8);

  memset (&(camera_matrix.coeffRef (0)), 0, sizeof (Eigen::Matrix3f::Scalar) * 9);
  camera_matrix.coeffRef (8) = 1.0;
  
  if (camera_matrix.Flags & Eigen::RowMajorBit)
  {
    camera_matrix.coeffRef (2) = cam.coeff (2);
    camera_matrix.coeffRef (5) = cam.coeff (5);
    camera_matrix.coeffRef (4) = sqrt (cam.coeff (4) - cam.coeff (5) * cam.coeff (5));
    camera_matrix.coeffRef (1) = (cam.coeff (1) - cam.coeff (2) * cam.coeff (5)) / camera_matrix.coeff (4);
    camera_matrix.coeffRef (0) = sqrt (cam.coeff (0) - camera_matrix.coeff (1) * camera_matrix.coeff (1) - cam.coeff (2) * cam.coeff (2));
  }
  else
  {
    camera_matrix.coeffRef (6) = cam.coeff (2);
    camera_matrix.coeffRef (7) = cam.coeff (5);
    camera_matrix.coeffRef (4) = sqrt (cam.coeff (4) - cam.coeff (5) * cam.coeff (5));
    camera_matrix.coeffRef (3) = (cam.coeff (1) - cam.coeff (2) * cam.coeff (5)) / camera_matrix.coeff (4);
    camera_matrix.coeffRef (0) = sqrt (cam.coeff (0) - camera_matrix.coeff (3) * camera_matrix.coeff (3) - cam.coeff (2) * cam.coeff (2));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighbor<PointT>::estimateProjectionMatrix ()
{
  // internally we calculate with double but store the result into float matrices.
  typedef double Scalar;
  projection_matrix_.setZero ();
  if (input_->height == 1 || input_->width == 1)
  {
    PCL_ERROR ("[pcl::%s::estimateProjectionMatrix] Input dataset is not organized!\n", this->getName ().c_str ());
    return;
  }
  
  const unsigned ySkip = (input_->height >> pyramid_level_);
  const unsigned xSkip = (input_->width >> pyramid_level_);
  Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> A = Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>::Zero ();
  Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> B = Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>::Zero ();
  Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> C = Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>::Zero ();
  Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> D = Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>::Zero ();

  for (unsigned yIdx = 0, idx = 0; yIdx < input_->height; yIdx += ySkip, idx += input_->width * (ySkip - 1))
  {
    for (unsigned xIdx = 0; xIdx < input_->width; xIdx += xSkip, idx += xSkip)
    {
      if (!mask_ [idx])
        continue;

      const PointT& point = input_->points[idx];
      if (pcl_isfinite (point.x))
      {
        Scalar xx = point.x * point.x;
        Scalar xy = point.x * point.y;
        Scalar xz = point.x * point.z;
        Scalar yy = point.y * point.y;
        Scalar yz = point.y * point.z;
        Scalar zz = point.z * point.z;
        Scalar xx_yy = xIdx * xIdx + yIdx * yIdx;

        A.coeffRef (0) += xx;
        A.coeffRef (1) += xy;
        A.coeffRef (2) += xz;
        A.coeffRef (3) += point.x;

        A.coeffRef (5) += yy;
        A.coeffRef (6) += yz;
        A.coeffRef (7) += point.y;

        A.coeffRef (10) += zz;
        A.coeffRef (11) += point.z;
        A.coeffRef (15) += 1.0;

        B.coeffRef (0) -= xx * xIdx;
        B.coeffRef (1) -= xy * xIdx;
        B.coeffRef (2) -= xz * xIdx;
        B.coeffRef (3) -= point.x * static_cast<double>(xIdx);

        B.coeffRef (5) -= yy * xIdx;
        B.coeffRef (6) -= yz * xIdx;
        B.coeffRef (7) -= point.y * static_cast<double>(xIdx);

        B.coeffRef (10) -= zz * xIdx;
        B.coeffRef (11) -= point.z * static_cast<double>(xIdx);

        B.coeffRef (15) -= xIdx;

        C.coeffRef (0) -= xx * yIdx;
        C.coeffRef (1) -= xy * yIdx;
        C.coeffRef (2) -= xz * yIdx;
        C.coeffRef (3) -= point.x * static_cast<double>(yIdx);

        C.coeffRef (5) -= yy * yIdx;
        C.coeffRef (6) -= yz * yIdx;
        C.coeffRef (7) -= point.y * static_cast<double>(yIdx);

        C.coeffRef (10) -= zz * yIdx;
        C.coeffRef (11) -= point.z * static_cast<double>(yIdx);

        C.coeffRef (15) -= yIdx;

        D.coeffRef (0) += xx * xx_yy;
        D.coeffRef (1) += xy * xx_yy;
        D.coeffRef (2) += xz * xx_yy;
        D.coeffRef (3) += point.x * xx_yy;

        D.coeffRef (5) += yy * xx_yy;
        D.coeffRef (6) += yz * xx_yy;
        D.coeffRef (7) += point.y * xx_yy;

        D.coeffRef (10) += zz * xx_yy;
        D.coeffRef (11) += point.z * xx_yy;

        D.coeffRef (15) += xx_yy;
      }
    }
  }

  makeSymmetric(A);
  makeSymmetric(B);
  makeSymmetric(C);
  makeSymmetric(D);

  Eigen::Matrix<Scalar, 12, 12, Eigen::RowMajor> X = Eigen::Matrix<Scalar, 12, 12, Eigen::RowMajor>::Zero ();
  X.topLeftCorner<4,4> () = A;
  X.block<4,4> (0, 8) = B;
  X.block<4,4> (8, 0) = B;
  X.block<4,4> (4, 4) = A;
  X.block<4,4> (4, 8) = C;
  X.block<4,4> (8, 4) = C;
  X.block<4,4> (8, 8) = D;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 12, 12, Eigen::RowMajor> > ei_symm(X);
  Eigen::Matrix<Scalar, 12, 12, Eigen::RowMajor> eigen_vectors = ei_symm.eigenvectors();

  // check whether the residual MSE is low. If its high, the cloud was not captured from a projective device.
  Eigen::Matrix<Scalar, 1, 1> residual_sqr = eigen_vectors.col (0).transpose () * X *  eigen_vectors.col (0);
  if ( fabs (residual_sqr.coeff (0)) > eps_ * A.coeff (15))
  {
    PCL_ERROR ("[pcl::%s::radiusSearch] Input dataset is not from a projective device!\nResidual (MSE) %f, using %d valid points\n", this->getName ().c_str (), residual_sqr.coeff (0) / A.coeff (15), static_cast<int> (A.coeff (15)));
    return;
  }

  projection_matrix_.coeffRef (0) = static_cast <float> (eigen_vectors.coeff (0));
  projection_matrix_.coeffRef (1) = static_cast <float> (eigen_vectors.coeff (12));
  projection_matrix_.coeffRef (2) = static_cast <float> (eigen_vectors.coeff (24));
  projection_matrix_.coeffRef (3) = static_cast <float> (eigen_vectors.coeff (36));
  projection_matrix_.coeffRef (4) = static_cast <float> (eigen_vectors.coeff (48));
  projection_matrix_.coeffRef (5) = static_cast <float> (eigen_vectors.coeff (60));
  projection_matrix_.coeffRef (6) = static_cast <float> (eigen_vectors.coeff (72));
  projection_matrix_.coeffRef (7) = static_cast <float> (eigen_vectors.coeff (84));
  projection_matrix_.coeffRef (8) = static_cast <float> (eigen_vectors.coeff (96));
  projection_matrix_.coeffRef (9) = static_cast <float> (eigen_vectors.coeff (108));
  projection_matrix_.coeffRef (10) = static_cast <float> (eigen_vectors.coeff (120));
  projection_matrix_.coeffRef (11) = static_cast <float> (eigen_vectors.coeff (132));

  if (projection_matrix_.coeff (0) < 0)
    projection_matrix_ *= -1.0;

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

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_SURFACE_IMPL_GRID_PROJECTION_H_
#define PCL_SURFACE_IMPL_GRID_PROJECTION_H_

#include <pcl/surface/grid_projection.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::GridProjection<PointNT>::GridProjection () :
  cell_hash_map_ (), leaf_size_ (0.001), gaussian_scale_ (),
  data_size_ (0), max_binary_search_level_ (10), k_ (50), padding_size_ (3), data_ () 
{}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::GridProjection<PointNT>::GridProjection (double resolution) :
  cell_hash_map_ (), leaf_size_ (resolution), gaussian_scale_ (),
  data_size_ (0), max_binary_search_level_ (10), k_ (50), padding_size_ (3), data_ () 
{}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::GridProjection<PointNT>::~GridProjection ()
{
  vector_at_data_point_.clear ();
  surface_.clear ();
  cell_hash_map_.clear ();
  occupied_cell_list_.clear ();
  data_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::scaleInputDataPoint (double scale_factor)
{
  for (auto& point: *data_) {
    point.getVector3fMap() /= static_cast<float> (scale_factor);
  }
  max_p_ /= static_cast<float> (scale_factor);
  min_p_ /= static_cast<float> (scale_factor);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::getBoundingBox ()
{
  pcl::getMinMax3D (*data_, min_p_, max_p_);

  Eigen::Vector4f bounding_box_size = max_p_ - min_p_;
  double scale_factor = (std::max)((std::max)(bounding_box_size.x (),
                                              bounding_box_size.y ()),
                                              bounding_box_size.z ());
  if (scale_factor > 1)
    scaleInputDataPoint (scale_factor);

  // Round the max_p_, min_p_ so that they are aligned with the cells vertices
  int upper_right_index[3];
  int lower_left_index[3];
  for (std::size_t i = 0; i < 3; ++i)
  {
    upper_right_index[i] = static_cast<int> (max_p_(i) / leaf_size_ + 5);
    lower_left_index[i] = static_cast<int> (min_p_(i) / leaf_size_ - 5);
    max_p_(i) = static_cast<float> (upper_right_index[i] * leaf_size_);
    min_p_(i) = static_cast<float> (lower_left_index[i] * leaf_size_);
  }
  bounding_box_size = max_p_ - min_p_;
  PCL_DEBUG ("[pcl::GridProjection::getBoundingBox] Size of Bounding Box is [%f, %f, %f]\n",
      bounding_box_size.x (), bounding_box_size.y (), bounding_box_size.z ());
  double max_size =
    (std::max) ((std::max)(bounding_box_size.x (), bounding_box_size.y ()),
                bounding_box_size.z ());

  data_size_ = static_cast<int> (max_size / leaf_size_);
  PCL_DEBUG ("[pcl::GridProjection::getBoundingBox] Lower left point is [%f, %f, %f]\n",
      min_p_.x (), min_p_.y (), min_p_.z ());
  PCL_DEBUG ("[pcl::GridProjection::getBoundingBox] Upper left point is [%f, %f, %f]\n",
      max_p_.x (), max_p_.y (), max_p_.z ());
  PCL_DEBUG ("[pcl::GridProjection::getBoundingBox] Padding size: %d\n", padding_size_);
  PCL_DEBUG ("[pcl::GridProjection::getBoundingBox] Leaf size: %f\n", leaf_size_);
  occupied_cell_list_.resize (data_size_ * data_size_ * data_size_);
  gaussian_scale_ = pow ((padding_size_+1) * leaf_size_ / 2.0, 2.0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::getVertexFromCellCenter (
    const Eigen::Vector4f &cell_center,
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &pts) const
{
  assert (pts.size () == 8);

  float sz = static_cast<float> (leaf_size_) / 2.0f;

  pts[0] = cell_center + Eigen::Vector4f (-sz,  sz, -sz, 0);
  pts[1] = cell_center + Eigen::Vector4f (-sz, -sz, -sz, 0);
  pts[2] = cell_center + Eigen::Vector4f (sz,  -sz, -sz, 0);
  pts[3] = cell_center + Eigen::Vector4f (sz,   sz, -sz, 0);
  pts[4] = cell_center + Eigen::Vector4f (-sz,  sz,  sz, 0);
  pts[5] = cell_center + Eigen::Vector4f (-sz, -sz,  sz, 0);
  pts[6] = cell_center + Eigen::Vector4f (sz,  -sz,  sz, 0);
  pts[7] = cell_center + Eigen::Vector4f (sz,   sz,  sz, 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::getDataPtsUnion (const Eigen::Vector3i &index,
                                               pcl::Indices &pt_union_indices)
{
  for (int i = index[0] - padding_size_; i <= index[0] + padding_size_; ++i)
  {
    for (int j = index[1] - padding_size_; j <= index[1] + padding_size_; ++j)
    {
      for (int k = index[2] - padding_size_; k <= index[2] + padding_size_; ++k)
      {
        Eigen::Vector3i cell_index_3d (i, j, k);
        int cell_index_1d = getIndexIn1D (cell_index_3d);
        if (cell_hash_map_.find (cell_index_1d) != cell_hash_map_.end ())
        {
          // Get the indices of the input points within the cell(i,j,k), which
          // is stored in the hash map
          pt_union_indices.insert (pt_union_indices.end (),
                                   cell_hash_map_.at (cell_index_1d).data_indices.begin (),
                                   cell_hash_map_.at (cell_index_1d).data_indices.end ());
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::createSurfaceForCell (const Eigen::Vector3i &index,
                                                    pcl::Indices &pt_union_indices)
{
  // 8 vertices of the cell
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > vertices (8);

  // 4 end points that shared by 3 edges connecting the upper left front points
  Eigen::Vector4f pts[4];
  Eigen::Vector3f vector_at_pts[4];

  // Given the index of cell, caluate the coordinates of the eight vertices of the cell
  // index the index of the cell in (x,y,z) 3d format
  Eigen::Vector4f cell_center = Eigen::Vector4f::Zero ();
  getCellCenterFromIndex (index, cell_center);
  getVertexFromCellCenter (cell_center, vertices);

  // Get the indices of the cells which stores the 4 end points.
  Eigen::Vector3i indices[4];
  indices[0] = Eigen::Vector3i (index[0], index[1], index[2] - 1);
  indices[1] = Eigen::Vector3i (index[0], index[1], index[2]);
  indices[2] = Eigen::Vector3i (index[0], index[1] - 1, index[2]);
  indices[3] = Eigen::Vector3i (index[0] + 1, index[1], index[2]);

  // Get the coordinate of the 4 end points, and the corresponding vectors
  for (std::size_t i = 0; i < 4; ++i)
  {
    pts[i] = vertices[I_SHIFT_PT[i]];
    unsigned int index_1d = getIndexIn1D (indices[i]);
    if (cell_hash_map_.find (index_1d) == cell_hash_map_.end () ||
        occupied_cell_list_[index_1d] == 0)
      return;
    vector_at_pts[i] = cell_hash_map_.at (index_1d).vect_at_grid_pt;
  }

  // Go through the 3 edges, test whether they are intersected by the surface
  for (std::size_t i = 0; i < 3; ++i)
  {
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > end_pts (2);
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vect_at_end_pts (2);
    for (std::size_t j = 0; j < 2; ++j)
    {
      end_pts[j] = pts[I_SHIFT_EDGE[i][j]];
      vect_at_end_pts[j] = vector_at_pts[I_SHIFT_EDGE[i][j]];
    }

    if (isIntersected (end_pts, vect_at_end_pts, pt_union_indices))
    {
      // Indices of cells that contains points which will be connected to
      // create a polygon
      Eigen::Vector3i polygon[4];
      Eigen::Vector4f polygon_pts[4];
      int polygon_indices_1d[4];
      bool is_all_in_hash_map = true;
      switch (i)
      {
        case 0:
          polygon[0] = Eigen::Vector3i (index[0] - 1, index[1] + 1, index[2]);
          polygon[1] = Eigen::Vector3i (index[0] - 1, index[1], index[2]);
          polygon[2] = Eigen::Vector3i (index[0], index[1], index[2]);
          polygon[3] = Eigen::Vector3i (index[0], index[1] + 1, index[2]);
          break;
        case 1:
          polygon[0] = Eigen::Vector3i (index[0], index[1] + 1, index[2] + 1);
          polygon[1] = Eigen::Vector3i (index[0], index[1] + 1, index[2]);
          polygon[2] = Eigen::Vector3i (index[0], index[1], index[2]);
          polygon[3] = Eigen::Vector3i (index[0], index[1], index[2] + 1);
          break;
        case 2:
          polygon[0] = Eigen::Vector3i (index[0] - 1, index[1], index[2] + 1);
          polygon[1] = Eigen::Vector3i (index[0] - 1, index[1], index[2]);
          polygon[2] = Eigen::Vector3i (index[0], index[1], index[2]);
          polygon[3] = Eigen::Vector3i (index[0], index[1], index[2] + 1);
          break;
        default:
          break;
      }
      for (std::size_t k = 0; k < 4; k++)
      {
        polygon_indices_1d[k] = getIndexIn1D (polygon[k]);
        if (!occupied_cell_list_[polygon_indices_1d[k]])
        {
          is_all_in_hash_map = false;
          break;
        }
      }
      if (is_all_in_hash_map)
      {
        for (std::size_t k = 0; k < 4; k++)
        {
          polygon_pts[k] = cell_hash_map_.at (polygon_indices_1d[k]).pt_on_surface;
          surface_.push_back (polygon_pts[k]);
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::getProjection (const Eigen::Vector4f &p,
                                             pcl::Indices &pt_union_indices, Eigen::Vector4f &projection)
{
  const double projection_distance = leaf_size_ * 3;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > end_pt (2);
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > end_pt_vect (2);
  end_pt[0] = p;
  getVectorAtPoint (end_pt[0], pt_union_indices, end_pt_vect[0]);
  end_pt_vect[0].normalize();

  double dSecond = getD2AtPoint (end_pt[0], end_pt_vect[0], pt_union_indices);

  // Find another point which is projection_distance away from the p, do a
  // binary search between these two points, to find the projected point on the
  // surface
  if (dSecond > 0)
    end_pt[1] = end_pt[0] + Eigen::Vector4f (
        end_pt_vect[0][0] * static_cast<float> (projection_distance),
        end_pt_vect[0][1] * static_cast<float> (projection_distance),
        end_pt_vect[0][2] * static_cast<float> (projection_distance), 
        0.0f);
  else
    end_pt[1] = end_pt[0] - Eigen::Vector4f (
        end_pt_vect[0][0] * static_cast<float> (projection_distance),
        end_pt_vect[0][1] * static_cast<float> (projection_distance),
        end_pt_vect[0][2] * static_cast<float> (projection_distance), 
        0.0f);
  getVectorAtPoint (end_pt[1], pt_union_indices, end_pt_vect[1]);
  if (end_pt_vect[1].dot (end_pt_vect[0]) < 0)
  {
    Eigen::Vector4f mid_pt = end_pt[0] + (end_pt[1] - end_pt[0]) * 0.5;
    findIntersection (0, end_pt, end_pt_vect, mid_pt, pt_union_indices, projection);
  }
  else
    projection = p;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::getProjectionWithPlaneFit (const Eigen::Vector4f &p,
                                                         pcl::Indices (&pt_union_indices),
                                                         Eigen::Vector4f &projection)
{
  // Compute the plane coefficients
  Eigen::Vector4f model_coefficients;
  /// @remark iterative weighted least squares or sac might give better results
  Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;

  computeMeanAndCovarianceMatrix (*data_, pt_union_indices, covariance_matrix, xyz_centroid);

  // Get the plane normal
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);

  // The normalization is not necessary, since the eigenvectors from libeigen are already normalized
  model_coefficients[0] = eigen_vector [0];
  model_coefficients[1] = eigen_vector [1];
  model_coefficients[2] = eigen_vector [2];
  model_coefficients[3] = 0;
  // Hessian form (D = nc . p_plane (centroid here) + p)
  model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

  // Projected point
  Eigen::Vector3f point (p.x (), p.y (), p.z ());     //= Eigen::Vector3f::MapAligned (&output[cp].x, 3);
  float distance = point.dot (model_coefficients.head <3> ()) + model_coefficients[3];
  point -= distance * model_coefficients.head < 3 > ();

  projection = Eigen::Vector4f (point[0], point[1], point[2], 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::getVectorAtPoint (const Eigen::Vector4f &p,
                                                pcl::Indices &pt_union_indices,
                                                Eigen::Vector3f &vo)
{
  std::vector <double> pt_union_dist (pt_union_indices.size ());
  std::vector <double> pt_union_weight (pt_union_indices.size ());
  Eigen::Vector3f out_vector (0, 0, 0);
  double sum = 0.0;
  double mag = 0.0;

  for (std::size_t i = 0; i < pt_union_indices.size (); ++i)
  {
    Eigen::Vector4f pp ((*data_)[pt_union_indices[i]].x, (*data_)[pt_union_indices[i]].y, (*data_)[pt_union_indices[i]].z, 0);
    pt_union_dist[i] = (pp - p).squaredNorm ();
    pt_union_weight[i] = pow (M_E, -pow (pt_union_dist[i], 2.0) / gaussian_scale_);
    mag += pow (M_E, -pow (sqrt (pt_union_dist[i]), 2.0) / gaussian_scale_);
    sum += pt_union_weight[i];
  }

  pcl::VectorAverage3f vector_average;

  Eigen::Vector3f v (
      (*data_)[pt_union_indices[0]].normal[0],
      (*data_)[pt_union_indices[0]].normal[1],
      (*data_)[pt_union_indices[0]].normal[2]);

  for (std::size_t i = 0; i < pt_union_weight.size (); ++i)
  {
    pt_union_weight[i] /= sum;
    Eigen::Vector3f vec ((*data_)[pt_union_indices[i]].normal[0],
                  (*data_)[pt_union_indices[i]].normal[1],
                  (*data_)[pt_union_indices[i]].normal[2]);
    if (vec.dot (v) < 0)
      vec = -vec;
    vector_average.add (vec, static_cast<float> (pt_union_weight[i]));
  }
  out_vector = vector_average.getMean ();
  // vector_average.getEigenVector1(out_vector);

  out_vector.normalize ();
  double d1 = getD1AtPoint (p, out_vector, pt_union_indices);
  out_vector *= static_cast<float> (sum);
  vo = ((d1 > 0) ? -1 : 1) * out_vector;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::getVectorAtPointKNN (const Eigen::Vector4f &p,
                                                   pcl::Indices &k_indices,
                                                   std::vector <float> &k_squared_distances,
                                                   Eigen::Vector3f &vo)
{
  Eigen::Vector3f out_vector (0, 0, 0);
  std::vector <float> k_weight;
  k_weight.resize (k_);
  float sum = 0.0;
  for (int i = 0; i < k_; i++)
  {
    //k_weight[i] = pow (M_E, -pow (k_squared_distances[i], 2.0) / gaussian_scale_);
    k_weight[i] = std::pow (static_cast<float>(M_E), static_cast<float>(-std::pow (static_cast<float>(k_squared_distances[i]), 2.0f) / gaussian_scale_));
    sum += k_weight[i];
  }
  pcl::VectorAverage3f vector_average;
  for (int i = 0; i < k_; i++)
  {
    k_weight[i] /= sum;
    Eigen::Vector3f vec ((*data_)[k_indices[i]].normal[0],
                         (*data_)[k_indices[i]].normal[1],
                         (*data_)[k_indices[i]].normal[2]);
    vector_average.add (vec, k_weight[i]);
  }
  vector_average.getEigenVector1 (out_vector);
  out_vector.normalize ();
  double d1 = getD1AtPoint (p, out_vector, k_indices);
  out_vector *= sum;
  vo = ((d1 > 0) ? -1 : 1) * out_vector;

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> double
pcl::GridProjection<PointNT>::getMagAtPoint (const Eigen::Vector4f &p,
                                             const pcl::Indices &pt_union_indices)
{
  std::vector <double> pt_union_dist (pt_union_indices.size ());
  double sum = 0.0;
  for (std::size_t i = 0; i < pt_union_indices.size (); ++i)
  {
    Eigen::Vector4f pp ((*data_)[pt_union_indices[i]].x, (*data_)[pt_union_indices[i]].y, (*data_)[pt_union_indices[i]].z, 0);
    pt_union_dist[i] = (pp - p).norm ();
    sum += pow (M_E, -pow (pt_union_dist[i], 2.0) / gaussian_scale_);
  }
  return (sum);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> double
pcl::GridProjection<PointNT>::getD1AtPoint (const Eigen::Vector4f &p, const Eigen::Vector3f &vec,
                                            const pcl::Indices &pt_union_indices)
{
  double sz = 0.01 * leaf_size_;
  Eigen::Vector3f v = vec * static_cast<float> (sz);

  double forward  = getMagAtPoint (p + Eigen::Vector4f (v[0], v[1], v[2], 0), pt_union_indices);
  double backward = getMagAtPoint (p - Eigen::Vector4f (v[0], v[1], v[2], 0), pt_union_indices);
  return ((forward - backward) / (0.02 * leaf_size_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> double
pcl::GridProjection<PointNT>::getD2AtPoint (const Eigen::Vector4f &p, const Eigen::Vector3f &vec,
                                            const pcl::Indices &pt_union_indices)
{
  double sz = 0.01 * leaf_size_;
  Eigen::Vector3f v = vec * static_cast<float> (sz);

  double forward = getD1AtPoint (p + Eigen::Vector4f (v[0], v[1], v[2], 0), vec, pt_union_indices);
  double backward = getD1AtPoint (p - Eigen::Vector4f (v[0], v[1], v[2], 0), vec, pt_union_indices);
  return ((forward - backward) / (0.02 * leaf_size_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> bool
pcl::GridProjection<PointNT>::isIntersected (const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &end_pts,
                                             std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &vect_at_end_pts,
                                             pcl::Indices &pt_union_indices)
{
  assert (end_pts.size () == 2);
  assert (vect_at_end_pts.size () == 2);

  double length[2];
  for (std::size_t i = 0; i < 2; ++i)
  {
    length[i] = vect_at_end_pts[i].norm ();
    vect_at_end_pts[i].normalize ();
  }
  double dot_prod = vect_at_end_pts[0].dot (vect_at_end_pts[1]);
  if (dot_prod < 0)
  {
    double ratio = length[0] / (length[0] + length[1]);
    Eigen::Vector4f start_pt = 
      end_pts[0] + (end_pts[1] - end_pts[0]) * static_cast<float> (ratio);
    Eigen::Vector4f intersection_pt = Eigen::Vector4f::Zero ();
    findIntersection (0, end_pts, vect_at_end_pts, start_pt, pt_union_indices, intersection_pt);

    Eigen::Vector3f vec;
    getVectorAtPoint (intersection_pt, pt_union_indices, vec);
    vec.normalize ();

    double d2 = getD2AtPoint (intersection_pt, vec, pt_union_indices);
    if (d2 < 0)
      return (true);
  }
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::findIntersection (int level,
                                                const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &end_pts,
                                                const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &vect_at_end_pts,
                                                const Eigen::Vector4f &start_pt,
                                                pcl::Indices &pt_union_indices,
                                                Eigen::Vector4f &intersection)
{
  assert (end_pts.size () == 2);
  assert (vect_at_end_pts.size () == 2);

  Eigen::Vector3f vec;
  getVectorAtPoint (start_pt, pt_union_indices, vec);
  double d1 = getD1AtPoint (start_pt, vec, pt_union_indices);
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > new_end_pts (2);
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > new_vect_at_end_pts (2);
  if ((std::abs (d1) < 10e-3) || (level == max_binary_search_level_))
  {
    intersection = start_pt;
    return;
  }
  vec.normalize ();
  if (vec.dot (vect_at_end_pts[0]) < 0)
  {
    Eigen::Vector4f new_start_pt = end_pts[0] + (start_pt - end_pts[0]) * 0.5;
    new_end_pts[0] = end_pts[0];
    new_end_pts[1] = start_pt;
    new_vect_at_end_pts[0] = vect_at_end_pts[0];
    new_vect_at_end_pts[1] = vec;
    findIntersection (level + 1, new_end_pts, new_vect_at_end_pts, new_start_pt, pt_union_indices, intersection);
    return;
  }
  if (vec.dot (vect_at_end_pts[1]) < 0)
  {
    Eigen::Vector4f new_start_pt = start_pt + (end_pts[1] - start_pt) * 0.5;
    new_end_pts[0] = start_pt;
    new_end_pts[1] = end_pts[1];
    new_vect_at_end_pts[0] = vec;
    new_vect_at_end_pts[1] = vect_at_end_pts[1];
    findIntersection (level + 1, new_end_pts, new_vect_at_end_pts, new_start_pt, pt_union_indices, intersection);
    return;
  }
  intersection = start_pt;
  return;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::fillPad (const Eigen::Vector3i &index)
{
  for (int i = index[0] - padding_size_; i < index[0] + padding_size_; ++i)
  {
    for (int j = index[1] - padding_size_; j < index[1] + padding_size_; ++j)
    {
      for (int k = index[2] - padding_size_; k < index[2] + padding_size_; ++k)
      {
        Eigen::Vector3i cell_index_3d (i, j, k);
        unsigned int cell_index_1d = getIndexIn1D (cell_index_3d);
        if (cell_hash_map_.find (cell_index_1d) == cell_hash_map_.end ())
        {
          cell_hash_map_[cell_index_1d].data_indices.resize (0);
          getCellCenterFromIndex (cell_index_3d, cell_hash_map_[cell_index_1d].pt_on_surface);
        }
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::storeVectAndSurfacePoint (int index_1d,
                                                        const Eigen::Vector3i &,
                                                        pcl::Indices &pt_union_indices,
                                                        const Leaf &cell_data)
{
  // Get point on grid
  Eigen::Vector4f grid_pt (
      cell_data.pt_on_surface.x () - static_cast<float> (leaf_size_) / 2.0f,
      cell_data.pt_on_surface.y () + static_cast<float> (leaf_size_) / 2.0f,
      cell_data.pt_on_surface.z () + static_cast<float> (leaf_size_) / 2.0f, 0.0f);

  // Save the vector and the point on the surface
  getVectorAtPoint (grid_pt, pt_union_indices, cell_hash_map_[index_1d].vect_at_grid_pt);
  getProjection (cell_data.pt_on_surface, pt_union_indices, cell_hash_map_[index_1d].pt_on_surface);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::storeVectAndSurfacePointKNN (int index_1d, const Eigen::Vector3i &,
                                                           const Leaf &cell_data)
{
  Eigen::Vector4f cell_center = cell_data.pt_on_surface;
  Eigen::Vector4f grid_pt (
      cell_center.x () - static_cast<float> (leaf_size_) / 2.0f,
      cell_center.y () + static_cast<float> (leaf_size_) / 2.0f,
      cell_center.z () + static_cast<float> (leaf_size_) / 2.0f, 0.0f);

  pcl::Indices k_indices;
  k_indices.resize (k_);
  std::vector <float> k_squared_distances;
  k_squared_distances.resize (k_);

  PointNT pt; pt.x = grid_pt.x (); pt.y = grid_pt.y (); pt.z = grid_pt.z ();
  tree_->nearestKSearch (pt, k_, k_indices, k_squared_distances);

  getVectorAtPointKNN (grid_pt, k_indices, k_squared_distances, cell_hash_map_[index_1d].vect_at_grid_pt);
  getProjectionWithPlaneFit (cell_center, k_indices, cell_hash_map_[index_1d].pt_on_surface);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> bool
pcl::GridProjection<PointNT>::reconstructPolygons (std::vector<pcl::Vertices> &polygons)
{
  data_.reset (new pcl::PointCloud<PointNT> (*input_));
  getBoundingBox ();

  // Store the point cloud data into the voxel grid, and at the same time
  // create a hash map to store the information for each cell
  cell_hash_map_.max_load_factor (2.0);
  cell_hash_map_.rehash (data_->size () / static_cast<long unsigned int> (cell_hash_map_.max_load_factor ()));

  // Go over all points and insert them into the right leaf
  for (pcl::index_t cp = 0; cp < static_cast<pcl::index_t> (data_->size ()); ++cp)
  {
    // Check if the point is invalid
    if (!std::isfinite ((*data_)[cp].x) ||
        !std::isfinite ((*data_)[cp].y) ||
        !std::isfinite ((*data_)[cp].z))
      continue;

    Eigen::Vector3i index_3d;
    getCellIndex ((*data_)[cp].getVector4fMap (), index_3d);
    int index_1d = getIndexIn1D (index_3d);
    if (cell_hash_map_.find (index_1d) == cell_hash_map_.end ())
    {
      Leaf cell_data;
      cell_data.data_indices.push_back (cp);
      getCellCenterFromIndex (index_3d, cell_data.pt_on_surface);
      cell_hash_map_[index_1d] = cell_data;
      occupied_cell_list_[index_1d] = 1;
    }
    else
    {
      Leaf cell_data = cell_hash_map_.at (index_1d);
      cell_data.data_indices.push_back (cp);
      cell_hash_map_[index_1d] = cell_data;
    }
  }

  Eigen::Vector3i index;
  int numOfFilledPad = 0;

  for (int i = 0; i < data_size_; ++i)
  {
    for (int j = 0; j < data_size_; ++j)
    {
      for (int k = 0; k < data_size_; ++k)
      {
        index[0] = i;
        index[1] = j;
        index[2] = k;
        if (occupied_cell_list_[getIndexIn1D (index)])
        {
          fillPad (index);
          numOfFilledPad++;
        }
      }
    }
  }

  // Update the hashtable and store the vector and point
  for (const auto &entry : cell_hash_map_)
  {
    getIndexIn3D (entry.first, index);
    pcl::Indices pt_union_indices;
    getDataPtsUnion (index, pt_union_indices);

    // Needs at least 10 points (?)
    // NOTE: set as parameter later
    if (pt_union_indices.size () > 10)
    {
      storeVectAndSurfacePoint (entry.first, index, pt_union_indices, entry.second);
      //storeVectAndSurfacePointKNN(entry.first, index, entry.second);
      occupied_cell_list_[entry.first] = 1;
    }
  }

  // Go through the hash table another time to extract surface
  for (const auto &entry : cell_hash_map_)
  {
    getIndexIn3D (entry.first, index);
    pcl::Indices pt_union_indices;
    getDataPtsUnion (index, pt_union_indices);

    // Needs at least 10 points (?)
    // NOTE: set as parameter later
    if (pt_union_indices.size () > 10)
      createSurfaceForCell (index, pt_union_indices);
  }

  polygons.resize (surface_.size () / 4);
  // Copy the data from surface_ to polygons
  for (int i = 0; i < static_cast<int> (polygons.size ()); ++i)
  {
    pcl::Vertices v;
    v.vertices.resize (4);
    for (int j = 0; j < 4; ++j)
      v.vertices[j] = i*4+j;
     polygons[i] = v;
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::performReconstruction (pcl::PolygonMesh &output)
{
  if (!reconstructPolygons (output.polygons))
    return;

  // The mesh surface is held in surface_. Copy it to the output format
  output.header = input_->header;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = surface_.size ();
  cloud.height = 1;
  cloud.is_dense = true;

  cloud.resize (surface_.size ());
  // Copy the data from surface_ to cloud
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    cloud[i].x = surface_[i].x ();
    cloud[i].y = surface_[i].y ();
    cloud[i].z = surface_[i].z ();
  }
  pcl::toPCLPointCloud2 (cloud, output.cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::GridProjection<PointNT>::performReconstruction (pcl::PointCloud<PointNT> &points,
                                                     std::vector<pcl::Vertices> &polygons)
{
  if (!reconstructPolygons (polygons))
    return;

  // The mesh surface is held in surface_. Copy it to the output format
  points.header = input_->header;
  points.width = surface_.size ();
  points.height = 1;
  points.is_dense = true;

  points.resize (surface_.size ());
  // Copy the data from surface_ to cloud
  for (std::size_t i = 0; i < points.size (); ++i)
  {
    points[i].x = surface_[i].x ();
    points[i].y = surface_[i].y ();
    points[i].z = surface_[i].z ();
  }
}

#define PCL_INSTANTIATE_GridProjection(T) template class PCL_EXPORTS pcl::GridProjection<T>;

#endif    // PCL_SURFACE_IMPL_GRID_PROJECTION_H_


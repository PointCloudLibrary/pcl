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
 */

#ifndef PCL_VOXEL_GRID_COVARIANCE_IMPL_H_
#define PCL_VOXEL_GRID_COVARIANCE_IMPL_H_

#include <pcl/common/common.h>
#include <pcl/common/point_tests.h> // for isXYZFinite
#include <pcl/filters/voxel_grid_covariance.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues> // for SelfAdjointEigenSolver
#include <boost/mpl/size.hpp> // for size
#include <boost/random/mersenne_twister.hpp> // for mt19937
#include <boost/random/normal_distribution.hpp> // for normal_distribution
#include <boost/random/variate_generator.hpp> // for variate_generator

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::VoxelGridCovariance<PointT>::applyFilter (PointCloud &output)
{
  voxel_centroids_leaf_indices_.clear ();

  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height = 1;                          // downsampling breaks the organized structure
  output.is_dense = true;                     // we filter out invalid points
  output.clear ();

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    getMinMax3D<PointT> (input_, filter_field_name_, static_cast<float> (filter_limit_min_), static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
  else
    getMinMax3D<PointT> (*input_, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
  std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
  std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

  if((dx*dy*dz) > std::numeric_limits<std::int32_t>::max())
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.\n", getClassName().c_str());
    output.clear();
    return;
  }

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int> (std::floor (min_p[0] * inverse_leaf_size_[0]));
  max_b_[0] = static_cast<int> (std::floor (max_p[0] * inverse_leaf_size_[0]));
  min_b_[1] = static_cast<int> (std::floor (min_p[1] * inverse_leaf_size_[1]));
  max_b_[1] = static_cast<int> (std::floor (max_p[1] * inverse_leaf_size_[1]));
  min_b_[2] = static_cast<int> (std::floor (min_p[2] * inverse_leaf_size_[2]));
  max_b_[2] = static_cast<int> (std::floor (max_p[2] * inverse_leaf_size_[2]));

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  // Clear the leaves
  leaves_.clear ();

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

  int centroid_size = 4;

  if (downsample_all_data_)
    centroid_size = boost::mpl::size<FieldList>::value;

  // ---[ RGB special case
  std::vector<pcl::PCLPointField> fields;
  int rgba_index = -1;
  rgba_index = pcl::getFieldIndex<PointT> ("rgb", fields);
  if (rgba_index == -1)
    rgba_index = pcl::getFieldIndex<PointT> ("rgba", fields);
  if (rgba_index >= 0)
  {
    rgba_index = fields[rgba_index].offset;
    centroid_size += 4;
  }

  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex<PointT> (filter_field_name_, fields);
    if (distance_idx == -1)
      PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);

    // First pass: go over all points and insert them into the right leaf
    for (const auto& point: *input_)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!isXYZFinite (point))
          continue;

      // Get the distance value
      const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*> (&point);
      float distance_value = 0;
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (filter_limit_negative_)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
          continue;
      }

      // Compute the centroid leaf index
      const Eigen::Vector4i ijk =
          Eigen::floor(point.getArray4fMap() * inverse_leaf_size_.array())
              .template cast<int>();
      // divb_mul_[3] = 0 by assignment
      int idx = (ijk - min_b_).dot(divb_mul_);

      Leaf& leaf = leaves_[idx];
      if (leaf.nr_points == 0)
      {
        leaf.centroid.resize (centroid_size);
        leaf.centroid.setZero ();
      }

      Eigen::Vector3d pt3d = point.getVector3fMap().template cast<double>();
      // Accumulate point sum for centroid calculation
      leaf.mean_ += pt3d;
      // Accumulate x*xT for single pass covariance calculation
      leaf.cov_ += pt3d * pt3d.transpose ();

      // Do we need to process all the fields?
      if (!downsample_all_data_)
      {
        leaf.centroid.template head<3> () += point.getVector3fMap();
      }
      else
      {
        // Copy all the fields
        Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
        pcl::for_each_type<FieldList> (NdCopyPointEigenFunctor<PointT> (point, centroid));
        // ---[ RGB special case
        if (rgba_index >= 0)
        {
          // Fill r/g/b data, assuming that the order is BGRA
          const pcl::RGB& rgb = *reinterpret_cast<const RGB*> (reinterpret_cast<const char*> (&point) + rgba_index);
          centroid[centroid_size - 4] = rgb.a;
          centroid[centroid_size - 3] = rgb.r;
          centroid[centroid_size - 2] = rgb.g;
          centroid[centroid_size - 1] = rgb.b;
        }
        leaf.centroid += centroid;
      }
      ++leaf.nr_points;
    }
  }
  // No distance filtering, process all data
  else
  {
    // First pass: go over all points and insert them into the right leaf
    for (const auto& point: *input_)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!isXYZFinite (point))
          continue;

      // Compute the centroid leaf index
      const Eigen::Vector4i ijk =
          Eigen::floor(point.getArray4fMap() * inverse_leaf_size_.array())
              .template cast<int>();
      // divb_mul_[3] = 0 by assignment
      int idx = (ijk - min_b_).dot(divb_mul_);

      Leaf& leaf = leaves_[idx];
      if (leaf.nr_points == 0)
      {
        leaf.centroid.resize (centroid_size);
        leaf.centroid.setZero ();
      }

      Eigen::Vector3d pt3d = point.getVector3fMap().template cast<double>();
      // Accumulate point sum for centroid calculation
      leaf.mean_ += pt3d;
      // Accumulate x*xT for single pass covariance calculation
      leaf.cov_ += pt3d * pt3d.transpose ();

      // Do we need to process all the fields?
      if (!downsample_all_data_)
      {
        leaf.centroid.template head<3> () += point.getVector3fMap();
      }
      else
      {
        // Copy all the fields
        Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
        pcl::for_each_type<FieldList> (NdCopyPointEigenFunctor<PointT> (point, centroid));
        // ---[ RGB special case
        if (rgba_index >= 0)
        {
          // Fill r/g/b data, assuming that the order is BGRA
          const pcl::RGB& rgb = *reinterpret_cast<const RGB*> (reinterpret_cast<const char*> (&point) + rgba_index);
          centroid[centroid_size - 4] = rgb.a;
          centroid[centroid_size - 3] = rgb.r;
          centroid[centroid_size - 2] = rgb.g;
          centroid[centroid_size - 1] = rgb.b;
        }
        leaf.centroid += centroid;
      }
      ++leaf.nr_points;
    }
  }

  // Second pass: go over all leaves and compute centroids and covariance matrices
  output.reserve (leaves_.size ());
  if (searchable_)
    voxel_centroids_leaf_indices_.reserve (leaves_.size ());
  int cp = 0;
  if (save_leaf_layout_)
    leaf_layout_.resize (div_b_[0] * div_b_[1] * div_b_[2], -1);

  // Eigen values and vectors calculated to prevent near singluar matrices
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
  Eigen::Matrix3d eigen_val;
  Eigen::Vector3d pt_sum;

  // Eigen values less than a threshold of max eigen value are inflated to a set fraction of the max eigen value.
  double min_covar_eigvalue;

  for (typename std::map<std::size_t, Leaf>::iterator it = leaves_.begin (); it != leaves_.end (); ++it)
  {

    // Normalize the centroid
    Leaf& leaf = it->second;

    // Normalize the centroid
    leaf.centroid /= static_cast<float> (leaf.nr_points);
    // Point sum used for single pass covariance calculation
    pt_sum = leaf.mean_;
    // Normalize mean
    leaf.mean_ /= leaf.nr_points;

    // If the voxel contains sufficient points, its covariance is calculated and is added to the voxel centroids and output clouds.
    // Points with less than the minimum points will have a can not be accuratly approximated using a normal distribution.
    if (leaf.nr_points >= min_points_per_voxel_)
    {
      if (save_leaf_layout_)
        leaf_layout_[it->first] = cp++;

      output.push_back (PointT ());

      // Do we need to process all the fields?
      if (!downsample_all_data_)
      {
        output.back ().x = leaf.centroid[0];
        output.back ().y = leaf.centroid[1];
        output.back ().z = leaf.centroid[2];
      }
      else
      {
        pcl::for_each_type<FieldList> (pcl::NdCopyEigenPointFunctor<PointT> (leaf.centroid, output.back ()));
        // ---[ RGB special case
        if (rgba_index >= 0)
        {
          pcl::RGB& rgb = *reinterpret_cast<RGB*> (reinterpret_cast<char*> (&output.back ()) + rgba_index);
          rgb.a = leaf.centroid[centroid_size - 4];
          rgb.r = leaf.centroid[centroid_size - 3];
          rgb.g = leaf.centroid[centroid_size - 2];
          rgb.b = leaf.centroid[centroid_size - 1];
        }
      }

      // Stores the voxel indice for fast access searching
      if (searchable_)
        voxel_centroids_leaf_indices_.push_back (static_cast<int> (it->first));

      // Single pass covariance calculation
      leaf.cov_ = (leaf.cov_ - pt_sum * leaf.mean_.transpose()) / (leaf.nr_points - 1.0);

      //Normalize Eigen Val such that max no more than 100x min.
      eigensolver.compute (leaf.cov_);
      eigen_val = eigensolver.eigenvalues ().asDiagonal ();
      leaf.evecs_ = eigensolver.eigenvectors ();

      if (eigen_val (0, 0) < -Eigen::NumTraits<double>::dummy_precision () || eigen_val (1, 1) < -Eigen::NumTraits<double>::dummy_precision () || eigen_val (2, 2) <= 0)
      {
        PCL_WARN ("[VoxelGridCovariance::applyFilter] Invalid eigen value! (%g, %g, %g)\n", eigen_val (0, 0), eigen_val (1, 1), eigen_val (2, 2));
        leaf.nr_points = -1;
        continue;
      }

      // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]

      min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val (2, 2);
      if (eigen_val (0, 0) < min_covar_eigvalue)
      {
        eigen_val (0, 0) = min_covar_eigvalue;

        if (eigen_val (1, 1) < min_covar_eigvalue)
        {
          eigen_val (1, 1) = min_covar_eigvalue;
        }

        leaf.cov_ = leaf.evecs_ * eigen_val * leaf.evecs_.inverse ();
      }
      leaf.evals_ = eigen_val.diagonal ();

      leaf.icov_ = leaf.cov_.inverse ();
      if (leaf.icov_.maxCoeff () == std::numeric_limits<float>::infinity ( )
          || leaf.icov_.minCoeff () == -std::numeric_limits<float>::infinity ( ) )
      {
        leaf.nr_points = -1;
      }

    }
  }

  output.width = output.size ();
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint (const Eigen::Matrix<int, 3, Eigen::Dynamic>& relative_coordinates, const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  neighbors.clear ();

  // Find displacement coordinates
  Eigen::Vector4i ijk = Eigen::floor(reference_point.getArray4fMap() * inverse_leaf_size_).template cast<int>();
  ijk[3] = 0;
  const Eigen::Array4i diff2min = min_b_ - ijk;
  const Eigen::Array4i diff2max = max_b_ - ijk;
  neighbors.reserve (relative_coordinates.cols ());

  // Check each neighbor to see if it is occupied and contains sufficient points
  for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)
  {
    const Eigen::Vector4i displacement = (Eigen::Vector4i () << relative_coordinates.col (ni), 0).finished ();
    // Checking if the specified cell is in the grid
    if ((diff2min <= displacement.array ()).all () && (diff2max >= displacement.array ()).all ())
    {
      const auto leaf_iter = leaves_.find (((ijk + displacement - min_b_).dot (divb_mul_)));
      if (leaf_iter != leaves_.end () && leaf_iter->second.nr_points >= min_points_per_voxel_)
      {
        LeafConstPtr leaf = &(leaf_iter->second);
        neighbors.push_back (leaf);
      }
    }
  }

  return static_cast<int> (neighbors.size());
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint (const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  Eigen::MatrixXi relative_coordinates = pcl::getAllNeighborCellIndices();
  return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::VoxelGridCovariance<PointT>::getVoxelAtPoint(const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  return getNeighborhoodAtPoint(Eigen::Matrix<int, 3, Eigen::Dynamic>::Zero(3,1), reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::VoxelGridCovariance<PointT>::getFaceNeighborsAtPoint(const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  Eigen::Matrix<int, 3, Eigen::Dynamic> relative_coordinates(3, 7);
  relative_coordinates.setZero();
  relative_coordinates(0, 1) = 1;
  relative_coordinates(0, 2) = -1;
  relative_coordinates(1, 3) = 1;
  relative_coordinates(1, 4) = -1;
  relative_coordinates(2, 5) = 1;
  relative_coordinates(2, 6) = -1;

  return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::VoxelGridCovariance<PointT>::getAllNeighborsAtPoint(const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  Eigen::Matrix<int, 3, Eigen::Dynamic> relative_coordinates(3, 27);
  relative_coordinates.col(0).setZero();
  relative_coordinates.rightCols(26) = pcl::getAllNeighborCellIndices();

  return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::VoxelGridCovariance<PointT>::getDisplayCloud (pcl::PointCloud<PointXYZ>& cell_cloud)
{
  cell_cloud.clear ();

  int pnt_per_cell = 1000;
  boost::mt19937 rng;
  boost::normal_distribution<> nd (0.0, 1.0);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

  Eigen::LLT<Eigen::Matrix3d> llt_of_cov;
  Eigen::Matrix3d cholesky_decomp;
  Eigen::Vector3d cell_mean;
  Eigen::Vector3d rand_point;
  Eigen::Vector3d dist_point;

  cell_cloud.reserve (pnt_per_cell * std::count_if (leaves_.begin (), leaves_.end (),
      [this] (auto& l) { return (l.second.nr_points >= min_points_per_voxel_); }));

  // Generate points for each occupied voxel with sufficient points.
  for (typename std::map<std::size_t, Leaf>::iterator it = leaves_.begin (); it != leaves_.end (); ++it)
  {
    Leaf& leaf = it->second;

    if (leaf.nr_points >= min_points_per_voxel_)
    {
      cell_mean = leaf.mean_;
      llt_of_cov.compute (leaf.cov_);
      cholesky_decomp = llt_of_cov.matrixL ();

      // Random points generated by sampling the normal distribution given by voxel mean and covariance matrix
      for (int i = 0; i < pnt_per_cell; i++)
      {
        rand_point = Eigen::Vector3d (var_nor (), var_nor (), var_nor ());
        dist_point = cell_mean + cholesky_decomp * rand_point;
        cell_cloud.push_back (PointXYZ (static_cast<float> (dist_point (0)), static_cast<float> (dist_point (1)), static_cast<float> (dist_point (2))));
      }
    }
  }
}

#define PCL_INSTANTIATE_VoxelGridCovariance(T) template class PCL_EXPORTS pcl::VoxelGridCovariance<T>;

#endif    // PCL_VOXEL_GRID_COVARIANCE_IMPL_H_

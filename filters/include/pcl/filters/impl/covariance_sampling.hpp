/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.

 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_FILTERS_IMPL_COVARIANCE_SAMPLING_H_
#define PCL_FILTERS_IMPL_COVARIANCE_SAMPLING_H_

#include <pcl/filters/covariance_sampling.h>
#include <list>
#include <Eigen/Eigenvalues> // for SelfAdjointEigenSolver

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT> bool
pcl::CovarianceSampling<PointT, PointNT>::initCompute ()
{
  if (!FilterIndices<PointT>::initCompute ())
    return false;

  if (num_samples_ > indices_->size ())
  {
    PCL_ERROR ("[pcl::CovarianceSampling::initCompute] The number of samples you asked for (%d) is larger than the number of input indices (%lu)\n",
               num_samples_, indices_->size ());
    return false;
  }

  // Prepare the point cloud by centering at the origin and then scaling the points such that the average distance from
  // the origin is 1.0 => rotations and translations will have the same magnitude
  Eigen::Vector3f centroid (0.f, 0.f, 0.f);
  for (std::size_t p_i = 0; p_i < indices_->size (); ++p_i)
    centroid += (*input_)[(*indices_)[p_i]].getVector3fMap ();
  centroid /= float (indices_->size ());

  scaled_points_.resize (indices_->size ());
  double average_norm = 0.0;
  for (std::size_t p_i = 0; p_i < indices_->size (); ++p_i)
  {
    scaled_points_[p_i] = (*input_)[(*indices_)[p_i]].getVector3fMap () - centroid;
    average_norm += scaled_points_[p_i].norm ();
  }

  average_norm /= double (scaled_points_.size ());
  for (std::size_t p_i = 0; p_i < scaled_points_.size (); ++p_i)
    scaled_points_[p_i] /= float (average_norm);

  return (true);
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT> double
pcl::CovarianceSampling<PointT, PointNT>::computeConditionNumber ()
{
  Eigen::Matrix<double, 6, 6> covariance_matrix;
  if (!computeCovarianceMatrix (covariance_matrix))
    return (-1.);

  return computeConditionNumber (covariance_matrix);
}


///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT> double
pcl::CovarianceSampling<PointT, PointNT>::computeConditionNumber (const Eigen::Matrix<double, 6, 6> &covariance_matrix)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > solver (covariance_matrix, Eigen::EigenvaluesOnly);
  const double max_ev = solver.eigenvalues (). maxCoeff ();
  const double min_ev = solver.eigenvalues (). minCoeff ();
  return (max_ev / min_ev);
}


///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT> bool
pcl::CovarianceSampling<PointT, PointNT>::computeCovarianceMatrix (Eigen::Matrix<double, 6, 6> &covariance_matrix)
{
  if (!initCompute ())
    return false;

  //--- Part A from the paper
  // Set up matrix F
  Eigen::Matrix<double, 6, Eigen::Dynamic> f_mat = Eigen::Matrix<double, 6, Eigen::Dynamic> (6, indices_->size ());
  for (std::size_t p_i = 0; p_i < scaled_points_.size (); ++p_i)
  {
    f_mat.block<3, 1> (0, p_i) = scaled_points_[p_i].cross (
                                     (*input_normals_)[(*indices_)[p_i]].getNormalVector3fMap ()).template cast<double> ();
    f_mat.block<3, 1> (3, p_i) = (*input_normals_)[(*indices_)[p_i]].getNormalVector3fMap ().template cast<double> ();
  }

  // Compute the covariance matrix C and its 6 eigenvectors (initially complex, move them to a double matrix)
  covariance_matrix = f_mat * f_mat.transpose ();
  return true;
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT> void
pcl::CovarianceSampling<PointT, PointNT>::applyFilter (Indices &sampled_indices)
{
  Eigen::Matrix<double, 6, 6> c_mat;
  // Invokes initCompute()
  if (!computeCovarianceMatrix (c_mat))
    return;

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > solver (c_mat);
  const Eigen::Matrix<double, 6, 6> x = solver.eigenvectors ();

  //--- Part B from the paper
  /// TODO figure out how to fill the candidate_indices - see subsequent paper paragraphs
  std::vector<std::size_t> candidate_indices;
  candidate_indices.resize (indices_->size ());
  for (std::size_t p_i = 0; p_i < candidate_indices.size (); ++p_i)
    candidate_indices[p_i] = p_i;

  // Compute the v 6-vectors
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > v;
  v.resize (candidate_indices.size ());
  for (std::size_t p_i = 0; p_i < candidate_indices.size (); ++p_i)
  {
    v[p_i].block<3, 1> (0, 0) = scaled_points_[p_i].cross (
                                  (*input_normals_)[(*indices_)[candidate_indices[p_i]]].getNormalVector3fMap ()).template cast<double> ();
    v[p_i].block<3, 1> (3, 0) = (*input_normals_)[(*indices_)[candidate_indices[p_i]]].getNormalVector3fMap ().template cast<double> ();
  }


  // Set up the lists to be sorted
  std::vector<std::list<std::pair<int, double> > > L;
  L.resize (6);

  for (std::size_t i = 0; i < 6; ++i)
  {
    for (std::size_t p_i = 0; p_i < candidate_indices.size (); ++p_i)
      L[i].push_back (std::make_pair (p_i, std::abs (v[p_i].dot (x.block<6, 1> (0, i)))));

    // Sort in decreasing order
    L[i].sort (sort_dot_list_function);
  }

  // Initialize the 6 t's
  std::vector<double> t (6, 0.0);

  sampled_indices.resize (num_samples_);
  std::vector<bool> point_sampled (candidate_indices.size (), false);
  // Now select the actual points
  for (std::size_t sample_i = 0; sample_i < num_samples_; ++sample_i)
  {
    // Find the most unconstrained dimension, i.e., the minimum t
    std::size_t min_t_i = 0;
    for (std::size_t i = 0; i < 6; ++i)
    {
      if (t[min_t_i] > t[i])
        min_t_i = i;
    }

    // Add the point from the top of the list corresponding to the dimension to the set of samples
    while (point_sampled [L[min_t_i].front ().first])
      L[min_t_i].pop_front ();

    sampled_indices[sample_i] = L[min_t_i].front ().first;
    point_sampled[L[min_t_i].front ().first] = true;
    L[min_t_i].pop_front ();

    // Update the running totals
    for (std::size_t i = 0; i < 6; ++i)
    {
      double val = v[sampled_indices[sample_i]].dot (x.block<6, 1> (0, i));
      t[i] += val * val;
    }
  }

  // Remap the sampled_indices to the input_ cloud
  for (auto &sampled_index : sampled_indices)
    sampled_index = (*indices_)[candidate_indices[sampled_index]];
}


///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT> void
pcl::CovarianceSampling<PointT, PointNT>::applyFilter (Cloud &output)
{
  Indices sampled_indices;
  applyFilter (sampled_indices);

  output.resize (sampled_indices.size ());
  output.header = input_->header;
  output.height = 1;
  output.width = output.size ();
  output.is_dense = true;
  for (std::size_t i = 0; i < sampled_indices.size (); ++i)
    output[i] = (*input_)[sampled_indices[i]];
}


#define PCL_INSTANTIATE_CovarianceSampling(T,NT) template class PCL_EXPORTS pcl::CovarianceSampling<T,NT>;

#endif /* PCL_FILTERS_IMPL_COVARIANCE_SAMPLING_H_ */

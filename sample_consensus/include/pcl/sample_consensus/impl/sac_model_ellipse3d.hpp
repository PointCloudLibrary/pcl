/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/sample_consensus/sac_model_ellipse3d.h>
#include <pcl/common/concatenate.h>
#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelEllipse3D<PointT>::isSampleGood (
    const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }

  const Eigen::Vector3f p0 = (*input_)[samples[0]].getVector3fMap();
  const Eigen::Vector3f p1 = (*input_)[samples[1]].getVector3fMap();
  const Eigen::Vector3f p2 = (*input_)[samples[2]].getVector3fMap();

  if ((p1 - p0).cross(p1 - p2).squaredNorm() < Eigen::NumTraits<float>::dummy_precision ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::isSampleGood] Sample points too similar or collinear!\n");
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelEllipse3D<PointT>::computeModelCoefficients (const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
    return (false);
  }

  model_coefficients.resize (model_size_);

  Eigen::Matrix<float, 6, 3> pts;
  for (int i = 0; i < 6; ++i)
    pts.row(i) = (*input_)[samples[i]].getVector3fMap();

  return internal::computeModelCoefficientsEllipse3D (pts, model_coefficients);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelEllipse3D<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }
  
  Eigen::ArrayXf x (indices_->size ()), y (indices_->size ()), z (indices_->size ());
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    const auto& pt = (*input_)[(*indices_)[i]];
    x[i] = pt.x; y[i] = pt.y; z[i] = pt.z;
  }

  internal::getDistancesToModelEllipse3D (model_coefficients, x, y, z, distances);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelEllipse3D<PointT>::selectWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold,
    Indices &inliers)
{
  if (!isModelValid (model_coefficients)) return;
  std::vector<double> distances;
  getDistancesToModel (model_coefficients, distances);

  inliers.clear();
  error_sqr_dists_.clear();
  inliers.reserve (indices_->size ());
  error_sqr_dists_.reserve (indices_->size ());

  const auto squared_threshold = threshold * threshold;
  for (std::size_t i = 0; i < distances.size (); ++i)
  {
    if (distances [i] * distances [i] < squared_threshold)
    {
      inliers.push_back ((*indices_) [i]);
      error_sqr_dists_.push_back (distances [i] * distances [i]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelEllipse3D<PointT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  if (!isModelValid (model_coefficients)) return (0);
  std::vector<double> distances;
  getDistancesToModel (model_coefficients, distances);

  std::size_t nr_p = 0;
  const auto squared_threshold = threshold * threshold;
  for (const auto& d : distances)
    if (d * d < squared_threshold)
      nr_p++;
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelEllipse3D<PointT>::optimizeModelCoefficients (
      const Indices &inliers,
      const Eigen::VectorXf &model_coefficients,
      Eigen::VectorXf &optimized_coefficients) const
{
  optimized_coefficients = model_coefficients;
  if (!isModelValid (model_coefficients)) return;
  if (inliers.size () <= sample_size_) return;

  Eigen::ArrayXf x (inliers.size ()), y (inliers.size ()), z (inliers.size ());
  for (std::size_t i = 0; i < inliers.size (); ++i)
  {
    const auto& pt = (*input_)[inliers[i]];
    x[i] = pt.x; y[i] = pt.y; z[i] = pt.z;
  }

  internal::optimizeModelCoefficientsEllipse3D (x, y, z, model_coefficients, optimized_coefficients);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelEllipse3D<PointT>::projectPoints (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients,
      PointCloud &projected_points, bool copy_data_fields) const
{
  if (!isModelValid (model_coefficients)) return;

  projected_points.header   = input_->header;
  projected_points.is_dense = input_->is_dense;

  if (copy_data_fields)
  {
    projected_points.resize (input_->size ());
    projected_points.width    = input_->width;
    projected_points.height   = input_->height;
    pcl::copyPointCloud (*input_, projected_points);
  }
  else
  {
    projected_points.resize (inliers.size ());
    projected_points.width    = inliers.size ();
    projected_points.height   = 1;
    pcl::copyPointCloud (*input_, inliers, projected_points);
  }

  const Eigen::Vector3f c (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
  const Eigen::Vector3f n_axis (model_coefficients[5], model_coefficients[6], model_coefficients[7]);
  const Eigen::Vector3f x_axis (model_coefficients[8], model_coefficients[9], model_coefficients[10]);
  const Eigen::Vector3f y_axis = n_axis.cross(x_axis).normalized();
  const float par_a(model_coefficients[3]);
  const float par_b(model_coefficients[4]);

  const Eigen::Matrix3f Rot = (Eigen::Matrix3f(3,3) << x_axis(0), y_axis(0), n_axis(0), x_axis(1), y_axis(1), n_axis(1), x_axis(2), y_axis(2), n_axis(2)).finished();
  const Eigen::Matrix3f Rot_T = Rot.transpose();
  const Eigen::VectorXf params = (Eigen::VectorXf(5) << par_a, par_b, 0.0f, 0.0f, 0.0f).finished();

  for (std::size_t i = 0; i < inliers.size (); ++i)
  {
    Eigen::Vector3f p;
    if (copy_data_fields)
      p = (*input_)[inliers[i]].getVector3fMap ();
    else
      p = projected_points[i].getVector3fMap ();

    float th_opt;
    Eigen::Vector2f p_th_;
    internal::dVec2Ellipse (params, (Rot_T * (p - c)) (0), (Rot_T * (p - c)) (1), th_opt);

    internal::getEllipsePoint (params, th_opt, p_th_ (0), p_th_ (1));
    projected_points[i].getVector3fMap() = c + Rot * Eigen::Vector3f(p_th_ (0), p_th_ (1), 0.0f);
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelEllipse3D<PointT>::doSamplesVerifyModel (
      const std::set<index_t> &indices,
      const Eigen::VectorXf &model_coefficients,
      const double threshold) const
{
  if (!isModelValid (model_coefficients)) return (false);

  Eigen::ArrayXf x (indices.size ()), y (indices.size ()), z (indices.size ());
  std::size_t i = 0;
  for (const auto& idx : indices)
  {
    const auto& pt = (*input_)[idx];
    x[i] = pt.x; y[i] = pt.y; z[i] = pt.z;
    ++i;
  }

  std::vector<double> dists;
  internal::getDistancesToModelEllipse3D (model_coefficients, x, y, z, dists);

  const auto squared_threshold = threshold * threshold;
  for (const auto& d : dists)
    if (d * d > squared_threshold)
      return (false);
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelEllipse3D<PointT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
  if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients)) return (false);

  if (radius_min_ != std::numeric_limits<double>::lowest() && (model_coefficients[3] < radius_min_ || model_coefficients[4] < radius_min_)) return (false);
  if (radius_max_ != std::numeric_limits<double>::max() && (model_coefficients[3] > radius_max_ || model_coefficients[4] > radius_max_)) return (false);

  return (true);
}

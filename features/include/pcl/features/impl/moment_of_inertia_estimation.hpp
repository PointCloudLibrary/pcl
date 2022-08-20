/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Sergey Ushakov
 * Email  : sergey.s.ushakov@mail.ru
 *
 */

#ifndef PCL_MOMENT_OF_INERTIA_ESTIMATION_HPP_
#define PCL_MOMENT_OF_INERTIA_ESTIMATION_HPP_

#include <Eigen/Eigenvalues> // for EigenSolver

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/feature.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::MomentOfInertiaEstimation<PointT>::MomentOfInertiaEstimation () :
  is_valid_ (false),
  step_ (10.0f),
  point_mass_ (0.0001f),
  normalize_ (true),
  mean_value_ (0.0f, 0.0f, 0.0f),
  major_axis_ (0.0f, 0.0f, 0.0f),
  middle_axis_ (0.0f, 0.0f, 0.0f),
  minor_axis_ (0.0f, 0.0f, 0.0f),
  major_value_ (0.0f),
  middle_value_ (0.0f),
  minor_value_ (0.0f),
  aabb_min_point_ (),
  aabb_max_point_ (),
  obb_min_point_ (),
  obb_max_point_ (),
  obb_position_ (0.0f, 0.0f, 0.0f)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::MomentOfInertiaEstimation<PointT>::~MomentOfInertiaEstimation ()
{
  moment_of_inertia_.clear ();
  eccentricity_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::setAngleStep (const float step)
{
  if (step <= 0.0f)
    return;

  step_ = step;

  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::MomentOfInertiaEstimation<PointT>::getAngleStep () const
{
  return (step_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::setNormalizePointMassFlag (bool need_to_normalize)
{
  normalize_ = need_to_normalize;

  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MomentOfInertiaEstimation<PointT>::getNormalizePointMassFlag () const
{
  return (normalize_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::setPointMass (const float point_mass)
{
  if (point_mass <= 0.0f)
    return;

  point_mass_ = point_mass;

  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::MomentOfInertiaEstimation<PointT>::getPointMass () const
{
  return (point_mass_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::compute ()
{
  moment_of_inertia_.clear ();
  eccentricity_.clear ();

  if (!initCompute ())
  {
    deinitCompute ();
    return;
  }

  if (normalize_)
  {
    if (!indices_->empty ())
      point_mass_ = 1.0f / static_cast <float> (indices_->size () * indices_->size ());
    else
      point_mass_ = 1.0f;
  }

  computeMeanValue ();

  Eigen::Matrix <float, 3, 3> covariance_matrix;
  covariance_matrix.setZero ();
  computeCovarianceMatrix (covariance_matrix);

  computeEigenVectors (covariance_matrix, major_axis_, middle_axis_, minor_axis_, major_value_, middle_value_, minor_value_);

  float theta = 0.0f;
  while (theta <= 90.0f)
  {
    float phi = 0.0f;
    Eigen::Vector3f rotated_vector;
    rotateVector (major_axis_, middle_axis_, theta, rotated_vector);
    while (phi <= 360.0f)
    {
      Eigen::Vector3f current_axis;
      rotateVector (rotated_vector, minor_axis_, phi, current_axis);
      current_axis.normalize ();

      //compute moment of inertia for the current axis
      float current_moment_of_inertia = calculateMomentOfInertia (current_axis, mean_value_);
      moment_of_inertia_.push_back (current_moment_of_inertia);

      //compute eccentricity for the current plane
      typename pcl::PointCloud<PointT>::Ptr projected_cloud (new pcl::PointCloud<PointT> ());
      getProjectedCloud (current_axis, mean_value_, projected_cloud);
      Eigen::Matrix <float, 3, 3> covariance_matrix;
      covariance_matrix.setZero ();
      computeCovarianceMatrix (projected_cloud, covariance_matrix);
      projected_cloud.reset ();
      float current_eccentricity = computeEccentricity (covariance_matrix, current_axis);
      eccentricity_.push_back (current_eccentricity);

      phi += step_;
    }
    theta += step_;
  }

  computeOBB ();

  is_valid_ = true;

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MomentOfInertiaEstimation<PointT>::getAABB (PointT& min_point, PointT& max_point) const
{
  min_point = aabb_min_point_;
  max_point = aabb_max_point_;

  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MomentOfInertiaEstimation<PointT>::getOBB (PointT& min_point, PointT& max_point, PointT& position, Eigen::Matrix3f& rotational_matrix) const
{
  min_point = obb_min_point_;
  max_point = obb_max_point_;
  position.x = obb_position_ (0);
  position.y = obb_position_ (1);
  position.z = obb_position_ (2);
  rotational_matrix = obb_rotational_matrix_;

  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::computeOBB ()
{
  obb_min_point_.x = std::numeric_limits <float>::max ();
  obb_min_point_.y = std::numeric_limits <float>::max ();
  obb_min_point_.z = std::numeric_limits <float>::max ();

  obb_max_point_.x = std::numeric_limits <float>::min ();
  obb_max_point_.y = std::numeric_limits <float>::min ();
  obb_max_point_.z = std::numeric_limits <float>::min ();

  auto number_of_points = static_cast <unsigned int> (indices_->size ());
  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    float x = ((*input_)[(*indices_)[i_point]].x - mean_value_ (0)) * major_axis_ (0) +
              ((*input_)[(*indices_)[i_point]].y - mean_value_ (1)) * major_axis_ (1) +
              ((*input_)[(*indices_)[i_point]].z - mean_value_ (2)) * major_axis_ (2);
    float y = ((*input_)[(*indices_)[i_point]].x - mean_value_ (0)) * middle_axis_ (0) +
              ((*input_)[(*indices_)[i_point]].y - mean_value_ (1)) * middle_axis_ (1) +
              ((*input_)[(*indices_)[i_point]].z - mean_value_ (2)) * middle_axis_ (2);
    float z = ((*input_)[(*indices_)[i_point]].x - mean_value_ (0)) * minor_axis_ (0) +
              ((*input_)[(*indices_)[i_point]].y - mean_value_ (1)) * minor_axis_ (1) +
              ((*input_)[(*indices_)[i_point]].z - mean_value_ (2)) * minor_axis_ (2);

    if (x <= obb_min_point_.x) obb_min_point_.x = x;
    if (y <= obb_min_point_.y) obb_min_point_.y = y;
    if (z <= obb_min_point_.z) obb_min_point_.z = z;

    if (x >= obb_max_point_.x) obb_max_point_.x = x;
    if (y >= obb_max_point_.y) obb_max_point_.y = y;
    if (z >= obb_max_point_.z) obb_max_point_.z = z;
  }

  obb_rotational_matrix_ << major_axis_ (0), middle_axis_ (0), minor_axis_ (0),
                            major_axis_ (1), middle_axis_ (1), minor_axis_ (1),
                            major_axis_ (2), middle_axis_ (2), minor_axis_ (2);

  Eigen::Vector3f shift (
    (obb_max_point_.x + obb_min_point_.x) / 2.0f,
    (obb_max_point_.y + obb_min_point_.y) / 2.0f,
    (obb_max_point_.z + obb_min_point_.z) / 2.0f);

  obb_min_point_.x -= shift (0);
  obb_min_point_.y -= shift (1);
  obb_min_point_.z -= shift (2);

  obb_max_point_.x -= shift (0);
  obb_max_point_.y -= shift (1);
  obb_max_point_.z -= shift (2);

  obb_position_ = mean_value_ + obb_rotational_matrix_ * shift;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MomentOfInertiaEstimation<PointT>::getEigenValues (float& major, float& middle, float& minor) const
{
  major = major_value_;
  middle = middle_value_;
  minor = minor_value_;

  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MomentOfInertiaEstimation<PointT>::getEigenVectors (Eigen::Vector3f& major, Eigen::Vector3f& middle, Eigen::Vector3f& minor) const
{
  major = major_axis_;
  middle = middle_axis_;
  minor = minor_axis_;

  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MomentOfInertiaEstimation<PointT>::getMomentOfInertia (std::vector <float>& moment_of_inertia) const
{
  moment_of_inertia.resize (moment_of_inertia_.size (), 0.0f);
  std::copy (moment_of_inertia_.cbegin (), moment_of_inertia_.cend (), moment_of_inertia.begin ());

  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MomentOfInertiaEstimation<PointT>::getEccentricity (std::vector <float>& eccentricity) const
{
  eccentricity.resize (eccentricity_.size (), 0.0f);
  std::copy (eccentricity_.cbegin (), eccentricity_.cend (), eccentricity.begin ());

  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::computeMeanValue ()
{
  mean_value_ (0) = 0.0f;
  mean_value_ (1) = 0.0f;
  mean_value_ (2) = 0.0f;

  aabb_min_point_.x = std::numeric_limits <float>::max ();
  aabb_min_point_.y = std::numeric_limits <float>::max ();
  aabb_min_point_.z = std::numeric_limits <float>::max ();

  aabb_max_point_.x = -std::numeric_limits <float>::max ();
  aabb_max_point_.y = -std::numeric_limits <float>::max ();
  aabb_max_point_.z = -std::numeric_limits <float>::max ();

  auto number_of_points = static_cast <unsigned int> (indices_->size ());
  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    mean_value_ (0) += (*input_)[(*indices_)[i_point]].x;
    mean_value_ (1) += (*input_)[(*indices_)[i_point]].y;
    mean_value_ (2) += (*input_)[(*indices_)[i_point]].z;

    if ((*input_)[(*indices_)[i_point]].x <= aabb_min_point_.x) aabb_min_point_.x = (*input_)[(*indices_)[i_point]].x;
    if ((*input_)[(*indices_)[i_point]].y <= aabb_min_point_.y) aabb_min_point_.y = (*input_)[(*indices_)[i_point]].y;
    if ((*input_)[(*indices_)[i_point]].z <= aabb_min_point_.z) aabb_min_point_.z = (*input_)[(*indices_)[i_point]].z;

    if ((*input_)[(*indices_)[i_point]].x >= aabb_max_point_.x) aabb_max_point_.x = (*input_)[(*indices_)[i_point]].x;
    if ((*input_)[(*indices_)[i_point]].y >= aabb_max_point_.y) aabb_max_point_.y = (*input_)[(*indices_)[i_point]].y;
    if ((*input_)[(*indices_)[i_point]].z >= aabb_max_point_.z) aabb_max_point_.z = (*input_)[(*indices_)[i_point]].z;
  }

  if (number_of_points == 0)
    number_of_points = 1;

  mean_value_ (0) /= number_of_points;
  mean_value_ (1) /= number_of_points;
  mean_value_ (2) /= number_of_points;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::computeCovarianceMatrix (Eigen::Matrix <float, 3, 3>& covariance_matrix) const
{
  covariance_matrix.setZero ();

  auto number_of_points = static_cast <unsigned int> (indices_->size ());
  float factor = 1.0f / static_cast <float> ((number_of_points - 1 > 0)?(number_of_points - 1):1);
  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    Eigen::Vector3f current_point (0.0f, 0.0f, 0.0f);
    current_point (0) = (*input_)[(*indices_)[i_point]].x - mean_value_ (0);
    current_point (1) = (*input_)[(*indices_)[i_point]].y - mean_value_ (1);
    current_point (2) = (*input_)[(*indices_)[i_point]].z - mean_value_ (2);

    covariance_matrix += current_point * current_point.transpose ();
  }

  covariance_matrix *= factor;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::computeCovarianceMatrix (PointCloudConstPtr cloud, Eigen::Matrix <float, 3, 3>& covariance_matrix) const
{
  covariance_matrix.setZero ();

  const auto number_of_points = cloud->size ();
  float factor = 1.0f / static_cast <float> ((number_of_points - 1 > 0)?(number_of_points - 1):1);
  Eigen::Vector3f current_point;
  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    current_point (0) = (*cloud)[i_point].x - mean_value_ (0);
    current_point (1) = (*cloud)[i_point].y - mean_value_ (1);
    current_point (2) = (*cloud)[i_point].z - mean_value_ (2);

    covariance_matrix += current_point * current_point.transpose ();
  }

  covariance_matrix *= factor;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::computeEigenVectors (const Eigen::Matrix <float, 3, 3>& covariance_matrix,
  Eigen::Vector3f& major_axis, Eigen::Vector3f& middle_axis, Eigen::Vector3f& minor_axis, float& major_value,
  float& middle_value, float& minor_value)
{
  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> > eigen_solver;
  eigen_solver.compute (covariance_matrix);

  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvectorsType eigen_vectors;
  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvalueType eigen_values;
  eigen_vectors = eigen_solver.eigenvectors ();
  eigen_values = eigen_solver.eigenvalues ();

  unsigned int temp = 0;
  unsigned int major_index = 0;
  unsigned int middle_index = 1;
  unsigned int minor_index = 2;

  if (eigen_values.real () (major_index) < eigen_values.real () (middle_index))
  {
    temp = major_index;
    major_index = middle_index;
    middle_index = temp;
  }

  if (eigen_values.real () (major_index) < eigen_values.real () (minor_index))
  {
    temp = major_index;
    major_index = minor_index;
    minor_index = temp;
  }

  if (eigen_values.real () (middle_index) < eigen_values.real () (minor_index))
  {
    temp = minor_index;
    minor_index = middle_index;
    middle_index = temp;
  }

  major_value = eigen_values.real () (major_index);
  middle_value = eigen_values.real () (middle_index);
  minor_value = eigen_values.real () (minor_index);

  major_axis = eigen_vectors.col (major_index).real ();
  middle_axis = eigen_vectors.col (middle_index).real ();
  minor_axis = eigen_vectors.col (minor_index).real ();

  major_axis.normalize ();
  middle_axis.normalize ();
  minor_axis.normalize ();

  float det = major_axis.dot (middle_axis.cross (minor_axis));
  if (det <= 0.0f)
  {
    major_axis (0) = -major_axis (0);
    major_axis (1) = -major_axis (1);
    major_axis (2) = -major_axis (2);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::rotateVector (const Eigen::Vector3f& vector, const Eigen::Vector3f& axis, const float angle, Eigen::Vector3f& rotated_vector) const
{
  Eigen::Matrix <float, 3, 3> rotation_matrix;
  const float x = axis (0);
  const float y = axis (1);
  const float z = axis (2);
  const float rad = M_PI / 180.0f;
  const float cosine = std::cos (angle * rad);
  const float sine = std::sin (angle * rad);
  rotation_matrix << cosine + (1 - cosine) * x * x,      (1 - cosine) * x * y - sine * z,    (1 - cosine) * x * z + sine * y,
                     (1 - cosine) * y * x + sine * z,    cosine + (1 - cosine) * y * y,      (1 - cosine) * y * z - sine * x,
                     (1 - cosine) * z * x - sine * y,    (1 - cosine) * z * y + sine * x,    cosine + (1 - cosine) * z * z;

  rotated_vector = rotation_matrix * vector;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::MomentOfInertiaEstimation<PointT>::calculateMomentOfInertia (const Eigen::Vector3f& current_axis, const Eigen::Vector3f& mean_value) const
{
  float moment_of_inertia = 0.0f;
  auto number_of_points = static_cast <unsigned int> (indices_->size ());
  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    Eigen::Vector3f vector;
    vector (0) = mean_value (0) - (*input_)[(*indices_)[i_point]].x;
    vector (1) = mean_value (1) - (*input_)[(*indices_)[i_point]].y;
    vector (2) = mean_value (2) - (*input_)[(*indices_)[i_point]].z;

    Eigen::Vector3f product = vector.cross (current_axis);

    float distance = product (0) * product (0) + product (1) * product (1) + product (2) * product (2);

    moment_of_inertia += distance;
  }

  return (point_mass_ * moment_of_inertia);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::getProjectedCloud (const Eigen::Vector3f& normal_vector, const Eigen::Vector3f& point, typename pcl::PointCloud <PointT>::Ptr projected_cloud) const
{
  const float D = - normal_vector.dot (point);

  auto number_of_points = static_cast <unsigned int> (indices_->size ());
  projected_cloud->points.resize (number_of_points, PointT ());

  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    const unsigned int index = (*indices_)[i_point];
    float K = - (D + normal_vector (0) * (*input_)[index].x + normal_vector (1) * (*input_)[index].y + normal_vector (2) * (*input_)[index].z);
    PointT projected_point;
    projected_point.x = (*input_)[index].x + K * normal_vector (0);
    projected_point.y = (*input_)[index].y + K * normal_vector (1);
    projected_point.z = (*input_)[index].z + K * normal_vector (2);
    (*projected_cloud)[i_point] = projected_point;
  }
  projected_cloud->width = number_of_points;
  projected_cloud->height = 1;
  projected_cloud->header = input_->header;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::MomentOfInertiaEstimation<PointT>::computeEccentricity (const Eigen::Matrix <float, 3, 3>& covariance_matrix, const Eigen::Vector3f& normal_vector)
{
  Eigen::Vector3f major_axis (0.0f, 0.0f, 0.0f);
  Eigen::Vector3f middle_axis (0.0f, 0.0f, 0.0f);
  Eigen::Vector3f minor_axis (0.0f, 0.0f, 0.0f);
  float major_value = 0.0f;
  float middle_value = 0.0f;
  float minor_value = 0.0f;
  computeEigenVectors (covariance_matrix, major_axis, middle_axis, minor_axis, major_value, middle_value, minor_value);

  float major = std::abs (major_axis.dot (normal_vector));
  float middle = std::abs (middle_axis.dot (normal_vector));
  float minor = std::abs (minor_axis.dot (normal_vector));

  float eccentricity = 0.0f;

  if (major >= middle && major >= minor && middle_value != 0.0f)
    eccentricity = std::pow (1.0f - (minor_value * minor_value) / (middle_value * middle_value), 0.5f);

  if (middle >= major && middle >= minor && major_value != 0.0f)
    eccentricity = std::pow (1.0f - (minor_value * minor_value) / (major_value * major_value), 0.5f);

  if (minor >= major && minor >= middle && major_value != 0.0f)
    eccentricity = std::pow (1.0f - (middle_value * middle_value) / (major_value * major_value), 0.5f);

  return (eccentricity);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MomentOfInertiaEstimation<PointT>::getMassCenter (Eigen::Vector3f& mass_center) const
{
  mass_center = mean_value_;

  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::setInputCloud (const PointCloudConstPtr& cloud)
{
  pcl::PCLBase<PointT>::setInputCloud (cloud);
  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::setIndices (const IndicesPtr& indices)
{
  pcl::PCLBase<PointT>::setIndices (indices);
  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::setIndices (const IndicesConstPtr& indices)
{
  pcl::PCLBase<PointT>::setIndices (indices);
  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::setIndices (const PointIndicesConstPtr& indices)
{
  pcl::PCLBase<PointT>::setIndices (indices);
  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MomentOfInertiaEstimation<PointT>::setIndices (std::size_t row_start, std::size_t col_start, std::size_t nb_rows, std::size_t nb_cols)
{
  pcl::PCLBase<PointT>::setIndices (row_start, col_start, nb_rows, nb_cols);
  is_valid_ = false;
}

#endif    // PCL_MOMENT_OF_INERTIA_ESTIMATION_HPP_

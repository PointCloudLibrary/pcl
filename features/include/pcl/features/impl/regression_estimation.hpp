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
 * Author : Alessio Stella
 * Email  : alessio.stella.g@gmail.com
 *
 */

#ifndef PCL_REGRESSION_ESTIMATION_HPP_
#define PCL_REGRESSION_ESTIMATION_HPP_

#include <Eigen/Eigenvalues> // for EigenSolver

#include <pcl/features/regression_estimation.h>
#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::RegressionEstimation<PointT>::RegressionEstimation()
:
  essential_(false),
  is_valid_(false),
  step_ (10.0f),
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
pcl::RegressionEstimation<PointT>::~RegressionEstimation()
{
  ;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::RegressionEstimation<PointT>::setEssential(const bool essential)
{

  essential_ = essential;

  is_valid_ = false;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool
pcl::RegressionEstimation<PointT>::getEssential() const
{
  return (essential_);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::RegressionEstimation<PointT>::PlaneFittingCloud(double& a, double& b, double& c)
{

  mean_value_(0) = mean_value_(1) = mean_value_(2) = 0.0f;

  if (!essential_) {
    aabb_min_point_.x = aabb_min_point_.y = aabb_min_point_.z =
        std::numeric_limits<float>::max();


    aabb_max_point_.x = aabb_max_point_.y = aabb_max_point_.z =
        -std::numeric_limits<float>::max();

  }

  a = b = c = 0.0;
  double SX, SY, SXsq, SYsq, SXY, SZX, SZY, SZ, xsq, ysq;
  SX = SY = SXsq = SYsq = SXY = SZX = SZY = SZ = xsq = ysq = 0;
  auto number_of_points = static_cast<unsigned int>(indices_->size());
  double size = (double)number_of_points; 


  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    auto point= ((*input_)[(*indices_)[i_point]]);
    xsq = point.x * point.x;
    ysq = point.y * point.y;
    SX += point.x;
    SY += point.y;
    SZ += point.z;
    SXsq += xsq;
    SYsq += ysq;
    SXY += point.x * point.y;
    SZX += point.x * point.z;
    SZY += point.y * point.z;

    if (!essential_)
    {
      if (point.x <= aabb_min_point_.x)
        aabb_min_point_.x = point.x;
      if (point.y <= aabb_min_point_.y)
        aabb_min_point_.y = point.y;
      if (point.z <= aabb_min_point_.z)
        aabb_min_point_.z = point.z;

      if (point.x >= aabb_max_point_.x)
        aabb_max_point_.x = point.x;
      if (point.y >= aabb_max_point_.y)
        aabb_max_point_.y = point.y;
      if (point.z >= aabb_max_point_.z)
        aabb_max_point_.z = point.z;
    }
  }

  if (size <= 0.0)
    size = 1;

  mean_value_(0) = SX / size;
  mean_value_(1) = SY / size;
  mean_value_(2) = SZ / size;

  double det = size * SXsq * SYsq - SXsq * SY * SY - size * SXY * SXY +
               2 * SXY * SX * SY - SX * SX * SYsq;

  if (det == 0.0)
      det = 1.0;

  a = (SZX * (size * SYsq - SY * SY) + SZY * (-size * SXY + SX * SY) +
       SZ * (SXY * SY - SYsq * SX)) /
      det;
  b = (SZX * (-size * SXY + SX * SY) + SZY * (size * SXsq - SX * SX) +
       SZ * (-SXsq * SY + SXY * SX)) /
      det;
  c = (SZX * (SXY * SY - SYsq * SX) + SZY * (-SXsq * SY + SXY * SX) +
       SZ * (SXsq * SYsq - SXY * SXY)) /
      det;

  return;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename PointT>
void
pcl::RegressionEstimation<PointT>::compute()
{

  if (!initCompute()) {
    deinitCompute();
    return;
  }

  double a, b, c;
  PlaneFittingCloud(a, b, c);

  double sqrt_ = sqrt(a * a + b * b + 1);
  minor_axis_(0) = -a / sqrt_;
  minor_axis_(1) = -b / sqrt_;
  minor_axis_(2) = 1.0 / sqrt_;

  // here fix the other two in an arbitrary but temporary way (on the plane orthogonal
  // to minor_axis_)
  major_axis_(0) = 0;
  major_axis_(1) = -1.0 / sqrt(b * b + 1);
  major_axis_(2) = -b / sqrt(b * b + 1);

  // cross product of the previous two: middle=minor X major  (( versy= versz X versx ))
  middle_axis_(0) = minor_axis_(1) * major_axis_(2) - minor_axis_(2) * major_axis_(1);
  middle_axis_(1) = minor_axis_(2) * major_axis_(0) - minor_axis_(0) * major_axis_(2);
  middle_axis_(2) = minor_axis_(0) * major_axis_(1) - minor_axis_(1) * major_axis_(0);

  // gonna now apply the regression in two dimensions (on the points projected on
  // the regressed plane), to find the regression line which will be the direction of
  // the major axis

  auto number_of_points = static_cast<unsigned int>(indices_->size());

  pcl::PointCloud<PointXY> demean2D;
  demean2D.width = number_of_points;
  demean2D.height = 1;
  demean2D.resize(number_of_points);

  double x1, y1, z1;


  if (!essential_) {
    x_m.resize(number_of_points);
    y_m.resize(number_of_points);
    z_m.resize(number_of_points);
  }

  for (unsigned int i = 0; i < number_of_points; ++i) {
    auto p = ((*input_)[(*indices_)[i]]);
//     vector applied on the mass center
    if (essential_) {
      x1 = p.x - mean_value_(0);
      y1 = p.y - mean_value_(1);
      z1 = p.z - mean_value_(2);
    }
   else {
      x_m[i] = (x1 = p.x - mean_value_(0));
      y_m[i] = (y1 = p.y - mean_value_(1));
      z_m[i] = (z1 = p.z - mean_value_(2));
    }

    // major component
    demean2D[i].x= x1 * major_axis_(0) + y1 * major_axis_(1) + z1 * major_axis_(2);

    // middle component
    demean2D[i].y= x1 * middle_axis_(0) + y1 * middle_axis_(1) + z1 * middle_axis_(2);
  }

  //2D PCA :
  // Compute the product cloud_demean2D * cloud_demean2D^T
  Eigen::Matrix2f alpha = Eigen::Matrix2f::Zero();
  for(unsigned int i=0;i<number_of_points;++i)
    {
    alpha(0,0) += demean2D[i].x * demean2D[i].x;
    alpha(1,0) += demean2D[i].x * demean2D[i].y;
    alpha(1,1) += demean2D[i].y * demean2D[i].y;
    }
  //std::cout << "alpha" << alpha << std::endl;
  if (static_cast<float>(number_of_points)) {
    alpha(0, 1) = alpha(1, 0) / (static_cast<float>(number_of_points));
    alpha(0, 0) /= (static_cast<float>(number_of_points));
    alpha(1, 0) /= (static_cast<float>(number_of_points));
    alpha(1, 1) /= (static_cast<float>(number_of_points));
  }
  else
    alpha(0, 1) = alpha(1, 0);

  // Compute eigen vectors and values
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> evd(alpha);
  //Eigen::Vector2f eigenvalues_;
  Eigen::Matrix2f eigenvectors_;


  //for (int i = 0; i < 2; ++i) {
    //eigenvalues_[i] = evd.eigenvalues()[ i];
   // eigenvectors_.col(i) = evd.eigenvectors().col( i);
 // }

  eigenvectors_ = evd.eigenvectors();

  double MiddleVectorX = major_axis_(0) * eigenvectors_.col(0)(0) +  middle_axis_(0) * eigenvectors_.col(0)(1);
  double MiddleVectorY = major_axis_(1) * eigenvectors_.col(0)(0) +  middle_axis_(1) * eigenvectors_.col(0)(1);
  double MiddleVectorZ = major_axis_(2) * eigenvectors_.col(0)(0) +  middle_axis_(2) * eigenvectors_.col(0)(1);

  middle_axis_(0) = MiddleVectorX;
  middle_axis_(1) = MiddleVectorY;
  middle_axis_(2) = MiddleVectorZ;

  // cross product of the other two: x = y X z   major = middle X minor
  major_axis_(0) = middle_axis_(1) * minor_axis_(2) - middle_axis_(2) * minor_axis_(1);
  major_axis_(1) = middle_axis_(2) * minor_axis_(0) - middle_axis_(0) * minor_axis_(2);
  major_axis_(2) = middle_axis_(0) * minor_axis_(1) - middle_axis_(1) * minor_axis_(0);

  // arbitrary
  // to be done: take an estimate from the two eigen values
  major_value_ = 2.0;
  middle_value_ = 1.5;
  minor_value_ = 1.0;

  if (!essential_)
    computeOBB();

  is_valid_ = true;

  deinitCompute();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename PointT>
void
pcl::RegressionEstimation<PointT>::computeByPCA()
{

  if (!initCompute()) {
    deinitCompute();
    return;
  }

  auto number_of_points = static_cast<unsigned int>(indices_->size());

  
  mean_value_(0) = mean_value_(1) = mean_value_(2) = 0.0f;

  if (!essential_) {
    aabb_min_point_.x = aabb_min_point_.y = aabb_min_point_.z =
        std::numeric_limits<float>::max();

    aabb_max_point_.x = aabb_max_point_.y = aabb_max_point_.z =
        -std::numeric_limits<float>::max();
  }

  
  double SX, SY, SZ;
  SX = SY = SZ= 0;

  double size = (double)number_of_points;

  for (unsigned int i_point = 0; i_point < number_of_points; i_point++) {
    auto point = ((*input_)[(*indices_)[i_point]]);

    SX += point.x;
    SY += point.y;
    SZ += point.z;
    

    if (!essential_) {
      if (point.x <= aabb_min_point_.x)
        aabb_min_point_.x = point.x;
      if (point.y <= aabb_min_point_.y)
        aabb_min_point_.y = point.y;
      if (point.z <= aabb_min_point_.z)
        aabb_min_point_.z = point.z;

      if (point.x >= aabb_max_point_.x)
        aabb_max_point_.x = point.x;
      if (point.y >= aabb_max_point_.y)
        aabb_max_point_.y = point.y;
      if (point.z >= aabb_max_point_.z)
        aabb_max_point_.z = point.z;
    }
  }

  if (size <= 0.0)
    size = 1;

  mean_value_(0) = SX / size;
  mean_value_(1) = SY / size;
  mean_value_(2) = SZ / size;

  pcl::PointCloud<PointXYZ> demean;
  demean.width = number_of_points;
  demean.height = 1;
  demean.resize(number_of_points);


  if (!essential_) {
    x_m.resize(number_of_points);
    y_m.resize(number_of_points);
    z_m.resize(number_of_points);
  }

  for (unsigned int i = 0; i < number_of_points; ++i) {
    auto p = ((*input_)[(*indices_)[i]]);
    //     vector applied on the mass center
    if (essential_) {
      demean[i].x =  p.x - mean_value_(0);
      demean[i].y =  p.y - mean_value_(1);
      demean[i].z =  p.z - mean_value_(2);
    }
    else {
      demean[i].x = x_m[i] =  p.x - mean_value_(0);
      demean[i].y = y_m[i] =  p.y - mean_value_(1);
      demean[i].z = z_m[i] =  p.z - mean_value_(2);
    }

  }

  // PCA :
  //  Compute the product cloud_demean * cloud_demean^T
  Eigen::Matrix3f alpha = Eigen::Matrix3f::Zero();
  for (unsigned int i = 0; i < number_of_points; ++i) {
    alpha(0, 0) += demean[i].x * demean[i].x;
    alpha(1, 1) += demean[i].y * demean[i].y;
    alpha(2, 2) += demean[i].z * demean[i].z;
    alpha(1, 0) += demean[i].x * demean[i].y;
    alpha(2, 0) += demean[i].x * demean[i].z;
    alpha(2, 1) += demean[i].y * demean[i].z;
  }

  if (static_cast<float>(number_of_points)) {
    alpha(0, 0) /= (static_cast<float>(number_of_points));
    alpha(1, 1) /= (static_cast<float>(number_of_points));
    alpha(2, 2) /= (static_cast<float>(number_of_points));
    alpha(1, 0) /= (static_cast<float>(number_of_points));
    alpha(2, 0) /= (static_cast<float>(number_of_points));
    alpha(2, 1) /= (static_cast<float>(number_of_points));
  }

    alpha(0, 1) = alpha(1, 0);
    alpha(0, 2) = alpha(2, 0);
    alpha(0, 3) = alpha(3, 0);


  // Compute eigen vectors and values
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd(alpha);
  // Eigen::Vector3f eigenvalues_;
  Eigen::Matrix3f eigenvectors_;

  eigenvectors_ = evd.eigenvectors();

  minor_axis_ = eigenvectors_.col(0);
  middle_axis_ = eigenvectors_.col(1);

  // cross product of the other two: x = y X z   major = middle X minor
  major_axis_(0) = middle_axis_(1) * minor_axis_(2) - middle_axis_(2) * minor_axis_(1);
  major_axis_(1) = middle_axis_(2) * minor_axis_(0) - middle_axis_(0) * minor_axis_(2);
  major_axis_(2) = middle_axis_(0) * minor_axis_(1) - middle_axis_(1) * minor_axis_(0);

  // arbitrary
  // to be done: take an estimate from the two eigen values
  major_value_ = 2.0;
  middle_value_ = 1.5;
  minor_value_ = 1.0;

  if (!essential_)
    computeOBB();

  is_valid_ = true;

  deinitCompute();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegressionEstimation<PointT>::getAABB(PointT& min_point, PointT& max_point) const
{
  min_point = aabb_min_point_;
  max_point = aabb_max_point_;

  if (essential_)
    return false;

  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegressionEstimation<PointT>::getOBB(PointT& min_point,
                                          PointT& max_point,
                                          PointT& position,
                                          Eigen::Matrix3f& rotational_matrix) const
{
  min_point = obb_min_point_;
  max_point = obb_max_point_;
  position.x = obb_position_ (0);
  position.y = obb_position_ (1);
  position.z = obb_position_ (2);
  rotational_matrix = obb_rotational_matrix_;

  if (essential_)
    return false;
  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::RegressionEstimation<PointT>::computeOBB()
{
  obb_min_point_.x = std::numeric_limits<float>::max();
  obb_min_point_.y = std::numeric_limits<float>::max();
  obb_min_point_.z = std::numeric_limits<float>::max();

  obb_max_point_.x = std::numeric_limits<float>::min();
  obb_max_point_.y = std::numeric_limits<float>::min();
  obb_max_point_.z = std::numeric_limits<float>::min();


  unsigned int siz = std::min(std::min(x_m.size(), y_m.size()), z_m.size());//this is a redundant check

  //Please note that here I don't need to use the indexes
  //because the indexes were already used before to store x_m y_m z_m values (inside method compute() lines 351-353)
  for (unsigned int i = 0; i < siz; ++i) {
    float x = x_m[i] * major_axis_(0) +
              y_m[i] * major_axis_(1) +
              z_m[i] * major_axis_(2);
    float y = x_m[i] * middle_axis_(0) +
              y_m[i] * middle_axis_(1) +
              z_m[i] * middle_axis_(2);
    float z = x_m[i] * minor_axis_(0) +
              y_m[i] * minor_axis_(1) +
              z_m[i] * minor_axis_(2);

    if (x <= obb_min_point_.x)
      obb_min_point_.x = x;
    if (y <= obb_min_point_.y)
      obb_min_point_.y = y;
    if (z <= obb_min_point_.z)
      obb_min_point_.z = z;

    if (x >= obb_max_point_.x)
      obb_max_point_.x = x;
    if (y >= obb_max_point_.y)
      obb_max_point_.y = y;
    if (z >= obb_max_point_.z)
      obb_max_point_.z = z;
  }

  obb_rotational_matrix_ << major_axis_(0), middle_axis_(0), minor_axis_(0),
      major_axis_(1), middle_axis_(1), minor_axis_(1), major_axis_(2), middle_axis_(2),
      minor_axis_(2);

  Eigen::Vector3f shift((obb_max_point_.x + obb_min_point_.x) / 2.0f,
                        (obb_max_point_.y + obb_min_point_.y) / 2.0f,
                        (obb_max_point_.z + obb_min_point_.z) / 2.0f);

  obb_min_point_.x -= shift(0);
  obb_min_point_.y -= shift(1);
  obb_min_point_.z -= shift(2);

  obb_max_point_.x -= shift(0);
  obb_max_point_.y -= shift(1);
  obb_max_point_.z -= shift(2);

  obb_position_ = mean_value_ + obb_rotational_matrix_ * shift;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegressionEstimation<PointT>::getRegressionVectors(Eigen::Vector3f& major,
                                                   Eigen::Vector3f& middle,
                                                   Eigen::Vector3f& minor) const
{
  major = major_axis_;
  middle = middle_axis_;
  minor = minor_axis_;

  return (is_valid_);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RegressionEstimation<PointT>::getMassCenter(Eigen::Vector3f& mass_center) const
{
  mass_center = mean_value_;

  return (is_valid_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegressionEstimation<PointT>::setInputCloud(const PointCloudConstPtr& cloud)
{
  pcl::PCLBase<PointT>::setInputCloud (cloud);
  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegressionEstimation<PointT>::setIndices(const IndicesPtr& indices)
{
  pcl::PCLBase<PointT>::setIndices (indices);
  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegressionEstimation<PointT>::setIndices(const IndicesConstPtr& indices)
{
  pcl::PCLBase<PointT>::setIndices (indices);
  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegressionEstimation<PointT>::setIndices(const PointIndicesConstPtr& indices)
{
  pcl::PCLBase<PointT>::setIndices (indices);
  is_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RegressionEstimation<PointT>::setIndices(std::size_t row_start,
                                              std::size_t col_start,
                                              std::size_t nb_rows,
                                              std::size_t nb_cols)
{
  pcl::PCLBase<PointT>::setIndices (row_start, col_start, nb_rows, nb_cols);
  is_valid_ = false;
}

#endif    // PCL_REGRESSION_ESTIMATION_HPP_

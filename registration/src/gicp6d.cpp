/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/registration/gicp6d.h>
#include <pcl/memory.h>                 // for pcl::make_shared
#include <pcl/point_types_conversion.h> // for PointXYZRGBtoXYZLAB

namespace pcl {

GeneralizedIterativeClosestPoint6D::GeneralizedIterativeClosestPoint6D(float lab_weight)
: cloud_lab_(new pcl::PointCloud<PointXYZLAB>)
, target_lab_(new pcl::PointCloud<PointXYZLAB>)
, lab_weight_(lab_weight)
{
  // set rescale mask (leave x,y,z unchanged, scale L,a,b by lab_weight)
  float alpha[6] = {1.0, 1.0, 1.0, lab_weight_, lab_weight_, lab_weight_};
  point_rep_.setRescaleValues(alpha);
}

void
GeneralizedIterativeClosestPoint6D::setInputSource(
    const PointCloudSourceConstPtr& cloud)
{
  // call corresponding base class method
  GeneralizedIterativeClosestPoint<PointSource, PointTarget>::setInputSource(cloud);

  // in addition, convert colors of the cloud to CIELAB
  cloud_lab_->resize(cloud->size());
  for (std::size_t point_idx = 0; point_idx < cloud->size(); ++point_idx) {
    PointXYZRGBtoXYZLAB((*cloud)[point_idx], (*cloud_lab_)[point_idx]);
  }
}

void
GeneralizedIterativeClosestPoint6D::setInputTarget(
    const PointCloudTargetConstPtr& target)
{
  // call corresponding base class method
  GeneralizedIterativeClosestPoint<PointSource, PointTarget>::setInputTarget(target);

  // in addition, convert colors of the cloud to CIELAB...
  target_lab_->resize(target->size());
  for (std::size_t point_idx = 0; point_idx < target->size(); ++point_idx) {
    PointXYZRGBtoXYZLAB((*target)[point_idx], (*target_lab_)[point_idx]);
  }

  // ...and build 6d-tree
  target_tree_lab_.setInputCloud(target_lab_);
  target_tree_lab_.setPointRepresentation(
      pcl::make_shared<MyPointRepresentation>(point_rep_));
}

bool
GeneralizedIterativeClosestPoint6D::searchForNeighbors(const PointXYZLAB& query,
                                                       pcl::Indices& index,
                                                       std::vector<float>& distance)
{
  int k = target_tree_lab_.nearestKSearch(query, 1, index, distance);

  // check if neighbor was found
  return (k != 0);
}

// taken from the original GICP class and modified slightly to make use of color values
void
GeneralizedIterativeClosestPoint6D::computeTransformation(PointCloudSource& output,
                                                          const Eigen::Matrix4f& guess)
{
  using namespace pcl;

  IterativeClosestPoint<PointSource, PointTarget>::initComputeReciprocal();

  // Difference between consecutive transforms
  double delta = 0;
  // Get the size of the target
  const std::size_t N = indices_->size();

  // Set the mahalanobis matrices to identity
  mahalanobis_.resize(N, Eigen::Matrix3d::Identity());

  // Compute target cloud covariance matrices
  if ((!target_covariances_) || (target_covariances_->empty())) {
    target_covariances_.reset(new MatricesVector);
    computeCovariances<PointTarget>(target_, tree_, *target_covariances_);
  }
  // Compute input cloud covariance matrices
  if ((!input_covariances_) || (input_covariances_->empty())) {
    input_covariances_.reset(new MatricesVector);
    computeCovariances<PointSource>(input_, tree_reciprocal_, *input_covariances_);
  }

  base_transformation_ = guess;
  nr_iterations_ = 0;
  converged_ = false;
  double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;
  pcl::Indices nn_indices(1);
  std::vector<float> nn_dists(1);

  while (!converged_) {
    std::size_t cnt = 0;
    pcl::Indices source_indices(indices_->size());
    pcl::Indices target_indices(indices_->size());

    // guess corresponds to base_t and transformation_ to t
    Eigen::Matrix4d transform_R = Eigen::Matrix4d::Zero();
    for (std::size_t i = 0; i < 4; i++)
      for (std::size_t j = 0; j < 4; j++)
        for (std::size_t k = 0; k < 4; k++)
          transform_R(i, j) += double(transformation_(i, k)) * double(guess(k, j));

    Eigen::Matrix3d R = transform_R.topLeftCorner<3, 3>();

    for (std::size_t i = 0; i < N; i++) {
      // MODIFICATION: take point from the CIELAB cloud instead
      PointXYZLAB query = (*cloud_lab_)[i];
      query.getVector4fMap() = guess * query.getVector4fMap();
      query.getVector4fMap() = transformation_ * query.getVector4fMap();

      if (!searchForNeighbors(query, nn_indices, nn_dists)) {
        PCL_ERROR("[pcl::%s::computeTransformation] Unable to find a nearest neighbor "
                  "in the target dataset for point %d in the source!\n",
                  getClassName().c_str(),
                  (*indices_)[i]);
        return;
      }

      // Check if the distance to the nearest neighbor is smaller than the user imposed
      // threshold
      if (nn_dists[0] < dist_threshold) {
        Eigen::Matrix3d& C1 = (*input_covariances_)[i];
        Eigen::Matrix3d& C2 = (*target_covariances_)[nn_indices[0]];
        Eigen::Matrix3d& M = mahalanobis_[i];
        // M = R*C1
        M = R * C1;
        // temp = M*R' + C2 = R*C1*R' + C2
        Eigen::Matrix3d temp = M * R.transpose();
        temp += C2;
        // M = temp^-1
        M = temp.inverse();
        source_indices[cnt] = static_cast<int>(i);
        target_indices[cnt] = nn_indices[0];
        cnt++;
      }
    }
    // Resize to the actual number of valid correspondences
    source_indices.resize(cnt);
    target_indices.resize(cnt);
    /* optimize transformation using the current assignment and Mahalanobis metrics*/
    previous_transformation_ = transformation_;
    // optimization right here
    try {
      rigid_transformation_estimation_(
          output, source_indices, *target_, target_indices, transformation_);
      /* compute the delta from this iteration */
      delta = 0.;
      for (int k = 0; k < 4; k++) {
        for (int l = 0; l < 4; l++) {
          double ratio = 1;
          if (k < 3 && l < 3) // rotation part of the transform
            ratio = 1. / rotation_epsilon_;
          else
            ratio = 1. / transformation_epsilon_;
          double c_delta =
              ratio * std::abs(previous_transformation_(k, l) - transformation_(k, l));
          if (c_delta > delta)
            delta = c_delta;
        }
      }
    } catch (PCLException& e) {
      PCL_DEBUG("[pcl::%s::computeTransformation] Optimization issue %s\n",
                getClassName().c_str(),
                e.what());
      break;
    }

    nr_iterations_++;
    // Check for convergence
    if (nr_iterations_ >= max_iterations_ || delta < 1) {
      converged_ = true;
      previous_transformation_ = transformation_;
      PCL_DEBUG("[pcl::%s::computeTransformation] Convergence reached. Number of "
                "iterations: %d out of %d. Transformation difference: %f\n",
                getClassName().c_str(),
                nr_iterations_,
                max_iterations_,
                (transformation_ - previous_transformation_).array().abs().sum());
    }
    else
      PCL_DEBUG("[pcl::%s::computeTransformation] Convergence failed\n",
                getClassName().c_str());
  }
  // for some reason the static equivalent method raises an error
  // final_transformation_.block<3,3> (0,0) = (transformation_.block<3,3> (0,0)) *
  // (guess.block<3,3> (0,0)); final_transformation_.block <3, 1> (0, 3) =
  // transformation_.block <3, 1> (0, 3) + guess.rightCols<1>.block <3, 1> (0, 3);
  final_transformation_.topLeftCorner(3, 3) =
      previous_transformation_.topLeftCorner(3, 3) * guess.topLeftCorner(3, 3);
  final_transformation_(0, 3) = previous_transformation_(0, 3) + guess(0, 3);
  final_transformation_(1, 3) = previous_transformation_(1, 3) + guess(1, 3);
  final_transformation_(2, 3) = previous_transformation_(2, 3) + guess(2, 3);

  // Transform the point cloud
  pcl::transformPointCloud(*input_, output, final_transformation_);
}

} // namespace pcl

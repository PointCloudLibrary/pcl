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
 */

#pragma once

#include <pcl/common/eigen.h> // for computeRoots, eigen33
#include <pcl/common/vector_average.h>

#include <Eigen/Eigenvalues> // for SelfAdjointEigenSolver

namespace pcl
{
  template <typename real, int dimension>
  VectorAverage<real, dimension>::VectorAverage ()
  {
    reset();
  }

  template <typename real, int dimension>
  inline void VectorAverage<real, dimension>::reset()
  {
    noOfSamples_ = 0;
    accumulatedWeight_ = 0.0;
    mean_.fill(0);
    covariance_.fill(0);
  }

  template <typename real, int dimension>
  inline void VectorAverage<real, dimension>::add(const Eigen::Matrix<real, dimension, 1>& sample, real weight) {
    if (weight == 0.0f)
      return;

    ++noOfSamples_;
    accumulatedWeight_ += weight;
    real alpha = weight/accumulatedWeight_;

    Eigen::Matrix<real, dimension, 1> diff = sample - mean_;
    covariance_ = (covariance_ + (diff * diff.transpose())*alpha)*(1.0f-alpha);

    mean_ += (diff)*alpha;

    //if (std::isnan(covariance_(0,0)))
    //{
      //std::cout << PVARN(weight);
      //exit(0);
    //}
  }

  template <typename real, int dimension>
  inline void VectorAverage<real, dimension>::doPCA(Eigen::Matrix<real, dimension, 1>& eigen_values, Eigen::Matrix<real, dimension, 1>& eigen_vector1,
                                                    Eigen::Matrix<real, dimension, 1>& eigen_vector2, Eigen::Matrix<real, dimension, 1>& eigen_vector3) const
  {
    // The following step is necessary for cases where the values in the covariance matrix are small
    // In this case float accuracy is nor enough to calculate the eigenvalues and eigenvectors.
    //Eigen::Matrix<double, dimension, dimension> tmp_covariance = covariance_.template cast<double>();
    //Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, dimension, dimension> > ei_symm(tmp_covariance);
    //eigen_values = ei_symm.eigenvalues().template cast<real>();
    //Eigen::Matrix<real, dimension, dimension> eigen_vectors = ei_symm.eigenvectors().template cast<real>();

    //std::cout << "My covariance is \n"<<covariance_<<"\n";
    //std::cout << "My mean is \n"<<mean_<<"\n";
    //std::cout << "My Eigenvectors \n"<<eigen_vectors<<"\n";

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<real, dimension, dimension> > ei_symm(covariance_);
    eigen_values = ei_symm.eigenvalues();
    Eigen::Matrix<real, dimension, dimension> eigen_vectors = ei_symm.eigenvectors();

    eigen_vector1 = eigen_vectors.col(0);
    eigen_vector2 = eigen_vectors.col(1);
    eigen_vector3 = eigen_vectors.col(2);
  }

  template <typename real, int dimension>
  inline void VectorAverage<real, dimension>::doPCA(Eigen::Matrix<real, dimension, 1>& eigen_values) const
  {
    // The following step is necessary for cases where the values in the covariance matrix are small
    // In this case float accuracy is nor enough to calculate the eigenvalues and eigenvectors.
    //Eigen::Matrix<double, dimension, dimension> tmp_covariance = covariance_.template cast<double>();
    //Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, dimension, dimension> > ei_symm(tmp_covariance, false);
    //eigen_values = ei_symm.eigenvalues().template cast<real>();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<real, dimension, dimension> > ei_symm(covariance_, false);
    eigen_values = ei_symm.eigenvalues();
  }

  template <typename real, int dimension>
  inline void VectorAverage<real, dimension>::getEigenVector1(Eigen::Matrix<real, dimension, 1>& eigen_vector1) const
  {
    // The following step is necessary for cases where the values in the covariance matrix are small
    // In this case float accuracy is nor enough to calculate the eigenvalues and eigenvectors.
    //Eigen::Matrix<double, dimension, dimension> tmp_covariance = covariance_.template cast<double>();
    //Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, dimension, dimension> > ei_symm(tmp_covariance);
    //eigen_values = ei_symm.eigenvalues().template cast<real>();
    //Eigen::Matrix<real, dimension, dimension> eigen_vectors = ei_symm.eigenvectors().template cast<real>();

    //std::cout << "My covariance is \n"<<covariance_<<"\n";
    //std::cout << "My mean is \n"<<mean_<<"\n";
    //std::cout << "My Eigenvectors \n"<<eigen_vectors<<"\n";

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<real, dimension, dimension> > ei_symm(covariance_);
    Eigen::Matrix<real, dimension, dimension> eigen_vectors = ei_symm.eigenvectors();
    eigen_vector1 = eigen_vectors.col(0);
  }


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Special cases for real=float & dimension=3 -> Partial specialization does not work with class templates. :( //
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////
  // float //
  ///////////
  template <>
  inline void VectorAverage<float, 3>::doPCA(Eigen::Matrix<float, 3, 1>& eigen_values, Eigen::Matrix<float, 3, 1>& eigen_vector1,
                                            Eigen::Matrix<float, 3, 1>& eigen_vector2, Eigen::Matrix<float, 3, 1>& eigen_vector3) const
  {
    //std::cout << "Using specialized 3x3 version of doPCA!\n";
    Eigen::Matrix<float, 3, 3> eigen_vectors;
    eigen33(covariance_, eigen_vectors, eigen_values);
    eigen_vector1 = eigen_vectors.col(0);
    eigen_vector2 = eigen_vectors.col(1);
    eigen_vector3 = eigen_vectors.col(2);
  }
  template <>
  inline void VectorAverage<float, 3>::doPCA(Eigen::Matrix<float, 3, 1>& eigen_values) const
  {
    //std::cout << "Using specialized 3x3 version of doPCA!\n";
    computeRoots (covariance_, eigen_values);
  }
  template <>
  inline void VectorAverage<float, 3>::getEigenVector1(Eigen::Matrix<float, 3, 1>& eigen_vector1) const
  {
    //std::cout << "Using specialized 3x3 version of doPCA!\n";
    Eigen::Vector3f::Scalar eigen_value;
    Eigen::Vector3f eigen_vector;
    eigen33(covariance_, eigen_value, eigen_vector);
    eigen_vector1 = eigen_vector;
  }

  ////////////
  // double //
  ////////////
  template <>
  inline void VectorAverage<double, 3>::doPCA(Eigen::Matrix<double, 3, 1>& eigen_values, Eigen::Matrix<double, 3, 1>& eigen_vector1,
                                            Eigen::Matrix<double, 3, 1>& eigen_vector2, Eigen::Matrix<double, 3, 1>& eigen_vector3) const
  {
    //std::cout << "Using specialized 3x3 version of doPCA!\n";
    Eigen::Matrix<double, 3, 3> eigen_vectors;
    eigen33(covariance_, eigen_vectors, eigen_values);
    eigen_vector1 = eigen_vectors.col(0);
    eigen_vector2 = eigen_vectors.col(1);
    eigen_vector3 = eigen_vectors.col(2);
  }
  template <>
  inline void VectorAverage<double, 3>::doPCA(Eigen::Matrix<double, 3, 1>& eigen_values) const
  {
    //std::cout << "Using specialized 3x3 version of doPCA!\n";
    computeRoots (covariance_, eigen_values);
  }
  template <>
  inline void VectorAverage<double, 3>::getEigenVector1(Eigen::Matrix<double, 3, 1>& eigen_vector1) const
  {
    //std::cout << "Using specialized 3x3 version of doPCA!\n";
    Eigen::Vector3d::Scalar eigen_value;
    Eigen::Vector3d eigen_vector;
    eigen33(covariance_, eigen_value, eigen_vector);
    eigen_vector1 = eigen_vector;
  }
}  // namespace pcl

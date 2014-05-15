/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: test_feature.cpp 3564 2011-12-16 06:11:13Z rusu $
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include "boost.h"

using namespace pcl;
using namespace std;

boost::variate_generator< boost::mt19937, boost::uniform_real<double> > rand_double(boost::mt19937 (), boost::uniform_real<double> (0, 1));
boost::variate_generator< boost::mt19937, boost::uniform_int<unsigned> > rand_uint(boost::mt19937 (), boost::uniform_int<unsigned> (0, 100));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, InverseGeneral3x3f)
{
  typedef float Scalar;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix = RMatrix::Zero ();
  RMatrix r_inverse = RMatrix::Zero ();
  CMatrix c_matrix = CMatrix::Zero ();
  CMatrix c_inverse = CMatrix::Zero ();
  Eigen::Matrix<Scalar, 3, 3> result = Eigen::Matrix<Scalar, 3, 3>::Zero ();
  Eigen::Matrix<Scalar, 3, 3> error = Eigen::Matrix<Scalar, 3, 3>::Zero ();
  Scalar determinant;
  const Scalar epsilon = 1e-5f;
  const unsigned iterations = 1000000;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    for (unsigned elIdx = 0; elIdx < 9; ++elIdx)
      r_matrix.coeffRef (elIdx) = Scalar(rand_double ());

    c_matrix = r_matrix;

    // test row-major -> row-major
    determinant = invert3x3Matrix (r_matrix, r_inverse);
    if (fabs (determinant) > epsilon)
    {
      float eps = std::max (epsilon, epsilon / fabs(determinant));

      result = r_inverse * r_matrix;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = r_matrix * r_inverse;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }

    // test row-major -> col-major
    determinant = invert3x3Matrix (c_matrix, c_inverse);
    if (fabs (determinant) > epsilon)
    {
      Scalar eps = std::max (epsilon, epsilon / fabs(determinant));

      result = c_inverse * c_matrix;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = c_matrix * c_inverse;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, InverseGeneral3x3d)
{
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix = RMatrix::Zero ();
  RMatrix r_inverse = RMatrix::Zero ();
  CMatrix c_matrix = CMatrix::Zero ();
  CMatrix c_inverse = CMatrix::Zero ();
  Eigen::Matrix<Scalar, 3, 3> result = Eigen::Matrix<Scalar, 3, 3>::Zero ();
  Eigen::Matrix<Scalar, 3, 3> error = Eigen::Matrix<Scalar, 3, 3>::Zero ();
  Scalar determinant;
  const Scalar epsilon = 1e-13;
  const unsigned iterations = 1000000;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    for (unsigned elIdx = 0; elIdx < 9; ++elIdx)
    {
      r_matrix.coeffRef (elIdx) = Scalar(rand_double ());
    }
    c_matrix = r_matrix;
    // test row-major -> row-major
    determinant = invert3x3Matrix (r_matrix, r_inverse);
    if (fabs (determinant) > epsilon)
    {
      Scalar eps = std::max (epsilon, epsilon / fabs (determinant));

      result = r_inverse * r_matrix;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = r_matrix * r_inverse;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }

    // test row-major -> col-major
    determinant = invert3x3Matrix (c_matrix, c_inverse);
    if (fabs (determinant) > epsilon)
    {
      Scalar eps = std::max (epsilon, epsilon / fabs(determinant));

      result = c_inverse * c_matrix;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = c_matrix * c_inverse;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, InverseSymmetric3x3f)
{
  typedef float Scalar;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix = RMatrix::Zero ();
  RMatrix r_inverse = RMatrix::Zero ();
  CMatrix c_matrix = CMatrix::Zero ();
  CMatrix c_inverse = CMatrix::Zero ();
  Eigen::Matrix<Scalar, 3, 3> result = Eigen::Matrix<Scalar, 3, 3>::Zero ();
  Eigen::Matrix<Scalar, 3, 3> error = Eigen::Matrix<Scalar, 3, 3>::Zero ();
  Scalar determinant;
  const Scalar epsilon = 1e-5f;
  const unsigned iterations = 1000000;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    for (unsigned elIdx = 0; elIdx < 9; ++elIdx)
      r_matrix.coeffRef (elIdx) = Scalar(rand_double ());

    r_matrix.coeffRef (3) = r_matrix.coeffRef (1);
    r_matrix.coeffRef (6) = r_matrix.coeffRef (2);
    r_matrix.coeffRef (7) = r_matrix.coeffRef (5);
    c_matrix = r_matrix;
    //c_matrix.coeffRef (3) = c_matrix.coeffRef (1);
    //c_matrix.coeffRef (6) = c_matrix.coeffRef (2);
    //c_matrix.coeffRef (7) = c_matrix.coeffRef (5);

    // test row-major -> row-major
    determinant = invert3x3SymMatrix (r_matrix, r_inverse);
    if (fabs (determinant) > epsilon)
    {
      float eps = std::max (epsilon, epsilon / fabs(determinant));

      result = r_inverse * r_matrix;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = r_matrix * r_inverse;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }

    // test row-major -> col-major
    determinant = invert3x3SymMatrix (c_matrix, c_inverse);
    if (fabs (determinant) > epsilon)
    {
      Scalar eps = std::max (epsilon, epsilon / fabs(determinant));

      result = c_inverse * c_matrix;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = c_matrix * c_inverse;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, InverseSymmetric3x3d)
{
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix = RMatrix::Zero ();
  RMatrix r_inverse = RMatrix::Zero ();
  CMatrix c_matrix = CMatrix::Zero ();
  CMatrix c_inverse = CMatrix::Zero ();
  Eigen::Matrix<Scalar, 3, 3> result = Eigen::Matrix<Scalar, 3, 3>::Zero ();
  Eigen::Matrix<Scalar, 3, 3> error = Eigen::Matrix<Scalar, 3, 3>::Zero ();
  Scalar determinant;
  const Scalar epsilon = 1e-13;
  const unsigned iterations = 1000000;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    for (unsigned elIdx = 0; elIdx < 9; ++elIdx)
      r_matrix.coeffRef (elIdx) = Scalar(rand_double ());

    r_matrix.coeffRef (3) = r_matrix.coeffRef (1);
    r_matrix.coeffRef (6) = r_matrix.coeffRef (2);
    r_matrix.coeffRef (7) = r_matrix.coeffRef (5);
    c_matrix = r_matrix;
    //c_matrix.coeffRef (3) = c_matrix.coeffRef (1);
    //c_matrix.coeffRef (6) = c_matrix.coeffRef (2);
    //c_matrix.coeffRef (7) = c_matrix.coeffRef (5);

    // test row-major -> row-major
    determinant = invert3x3SymMatrix (r_matrix, r_inverse);
    if (fabs (determinant) > epsilon)
    {
      Scalar eps = std::max (epsilon, epsilon / fabs (determinant));

      result = r_inverse * r_matrix;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = r_matrix * r_inverse;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }

    // test row-major -> col-major
    determinant = invert3x3SymMatrix (c_matrix, c_inverse);
    if (fabs (determinant) > epsilon)
    {
      Scalar eps = std::max (epsilon, epsilon / fabs(determinant));

      result = c_inverse * c_matrix;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = c_matrix * c_inverse;
      error = result - Eigen::Matrix<Scalar, 3, 3>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Inverse2x2f)
{
  typedef float Scalar;
  typedef Eigen::Matrix<Scalar, 2, 2, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 2, 2, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix = RMatrix::Zero ();
  RMatrix r_inverse = RMatrix::Zero ();
  CMatrix c_matrix = CMatrix::Zero ();
  CMatrix c_inverse = CMatrix::Zero ();
  Eigen::Matrix<Scalar, 2, 2> result = Eigen::Matrix<Scalar, 2, 2>::Zero ();
  Eigen::Matrix<Scalar, 2, 2> error = Eigen::Matrix<Scalar, 2, 2>::Zero ();
  Scalar determinant;
  const Scalar epsilon = 1e-6f;
  const unsigned iterations = 1000000;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    for (unsigned elIdx = 0; elIdx < 4; ++elIdx)
      r_matrix.coeffRef (elIdx) = Scalar(rand_double ());

    c_matrix = r_matrix;
    // test row-major -> row-major
    determinant = invert2x2 (r_matrix, r_inverse);
    if (fabs (determinant) > epsilon)
    {
      float eps = std::max (epsilon, epsilon / fabs(determinant));

      result = r_inverse * r_matrix;
      error = result - Eigen::Matrix<Scalar, 2, 2>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = r_matrix * r_inverse;
      error = result - Eigen::Matrix<Scalar, 2, 2>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }

    // test row-major -> col-major
    determinant = invert2x2 (c_matrix, c_inverse);
    if (fabs (determinant) > epsilon)
    {
      Scalar eps = std::max (epsilon, epsilon / fabs(determinant));

      result = c_inverse * c_matrix;
      error = result - Eigen::Matrix<Scalar, 2, 2>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = c_matrix * c_inverse;
      error = result - Eigen::Matrix<Scalar, 2, 2>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Inverse2x2d)
{
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, 2, 2, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 2, 2, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix = RMatrix::Zero ();
  RMatrix r_inverse = RMatrix::Zero ();
  CMatrix c_matrix = CMatrix::Zero ();
  CMatrix c_inverse = CMatrix::Zero ();
  Eigen::Matrix<Scalar, 2, 2> result;
  Eigen::Matrix<Scalar, 2, 2> error;
  Scalar determinant;
  const Scalar epsilon = 1e-15;
  const unsigned iterations = 1000000;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    for (unsigned elIdx = 0; elIdx < 4; ++elIdx)
      r_matrix.coeffRef (elIdx) = Scalar(rand_double ());

    c_matrix = r_matrix;
    // test row-major -> row-major
    determinant = invert2x2 (r_matrix, r_inverse);
    if (fabs (determinant) > epsilon)
    {
      Scalar eps = std::max (epsilon, epsilon / fabs (determinant));

      result = r_inverse * r_matrix;
      error = result - Eigen::Matrix<Scalar, 2, 2>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = r_matrix * r_inverse;
      error = result - Eigen::Matrix<Scalar, 2, 2>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }

    // test row-major -> col-major
    determinant = invert2x2 (c_matrix, c_inverse);
    if (fabs (determinant) > epsilon)
    {
      Scalar eps = std::max (epsilon, epsilon / fabs(determinant));

      result = c_inverse * c_matrix;
      error = result - Eigen::Matrix<Scalar, 2, 2>::Identity ();
      Scalar distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);

      result = c_matrix * c_inverse;
      error = result - Eigen::Matrix<Scalar, 2, 2>::Identity ();
      distance = error.cwiseAbs ().sum ();
      EXPECT_LE (distance, eps);
    }
  }
}

template<class Matrix>
inline void generateSymPosMatrix2x2 (Matrix& matrix)
{
  typedef typename Matrix::Scalar Scalar;

  unsigned test_case = rand_uint () % 10;

  Scalar val1 = Scalar (rand_double ());
  Scalar val2 = Scalar (rand_double ());

  // 10% of test cases include equal eigenvalues
  if (test_case == 0)
    val2 = val1;
  // another 20% includes one zero eigenvalue
  else if (test_case == 1 && val1 != 0)
    val2 = 0.0;
  else if (test_case == 2 && val2 != 0)
    val1 = 0.0;

  Scalar sqrNorm;
  Matrix eigenvectors = Matrix::Identity ();
  Matrix eigenvalues = Matrix::Zero ();

  unsigned test_case2 = rand_uint () % 10;
  if (test_case2 != 0)
  {
    do
    {
      eigenvectors.col (0)[0] = Scalar (rand_double ());
      eigenvectors.col (0)[1] = Scalar (rand_double ());
      sqrNorm = eigenvectors.col (0).squaredNorm ();
    } while (sqrNorm == 0);
    eigenvectors.col (0) /= sqrt (sqrNorm);

    eigenvectors.col (1)[0] = -eigenvectors.col (1)[1];
    eigenvectors.col (1)[1] =  eigenvectors.col (1)[0];
  }
  eigenvalues (0, 0) = val1;
  eigenvalues (1, 1) = val2;
  matrix = eigenvectors * eigenvalues * eigenvectors.adjoint();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, eigen22d)
{
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, 2, 2, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 2, 2, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix;
  RMatrix r_vectors;
  Eigen::Matrix<Scalar, 2, 1> r_eigenvalues;
  Eigen::Matrix<Scalar, 2, 1> c_eigenvalues;
  CMatrix c_matrix;
  CMatrix c_vectors;
  Eigen::Matrix<Scalar, 2, 2> r_result;
  Eigen::Matrix<Scalar, 2, 2> r_error;
  Eigen::Matrix<Scalar, 2, 2> g_result;
  Eigen::Matrix<Scalar, 2, 2> g_error;
  Eigen::Matrix<Scalar, 2, 2> c_result;
  Eigen::Matrix<Scalar, 2, 2> c_error;
  Scalar diff;

  const Scalar epsilon = 1.25e-14;
  const unsigned iterations = 1000000;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    // generate test matrices
    generateSymPosMatrix2x2 (r_matrix);
    c_matrix = r_matrix;

    // calculate the eigenvalue decomposition
    eigen22 (r_matrix, r_vectors, r_eigenvalues);

    // test if U * V * U^T = M
    r_result = r_vectors * r_eigenvalues.asDiagonal () * r_vectors.transpose ();
    r_error = r_result - r_matrix;
    diff = r_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);

    // test if the eigenvalues are orthonormal
    g_result = r_vectors * r_vectors.transpose ();
    g_error = g_result - RMatrix::Identity ();
    diff = g_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);

    // test if column major matrices are also calculated correctly
    eigen22 (c_matrix, c_vectors, c_eigenvalues);
    c_result = c_vectors * c_eigenvalues.asDiagonal () * c_vectors.transpose ();
    c_error = c_result - c_matrix;
    diff = c_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);

    g_result = c_vectors * c_vectors.transpose ();
    g_error = g_result - CMatrix::Identity ();
    diff = g_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, eigen22f)
{
  typedef float Scalar;
  typedef Eigen::Matrix<Scalar, 2, 2, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 2, 2, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix;
  RMatrix r_vectors;
  Eigen::Matrix<Scalar, 2, 1> r_eigenvalues;
  Eigen::Matrix<Scalar, 2, 1> c_eigenvalues;
  CMatrix c_matrix;
  CMatrix c_vectors;
  Eigen::Matrix<Scalar, 2, 2> r_result;
  Eigen::Matrix<Scalar, 2, 2> r_error;
  Eigen::Matrix<Scalar, 2, 2> g_result;
  Eigen::Matrix<Scalar, 2, 2> g_error;
  Eigen::Matrix<Scalar, 2, 2> c_result;
  Eigen::Matrix<Scalar, 2, 2> c_error;
  Scalar diff;

  const Scalar epsilon = 3.1e-5f;
  const unsigned iterations = 1000000;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    // generate test matrices
    generateSymPosMatrix2x2 (r_matrix);
    c_matrix = r_matrix;

    // calculate the eigenvalue decomposition
    eigen22 (r_matrix, r_vectors, r_eigenvalues);

    // test if U * V * U^T = M
    r_result = r_vectors * r_eigenvalues.asDiagonal () * r_vectors.transpose ();
    r_error = r_result - r_matrix;
    diff = r_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);

    // test if the eigenvalues are orthonormal
    g_result = r_vectors * r_vectors.transpose ();
    g_error = g_result - RMatrix::Identity ();
    diff = g_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);

    // test if column major matrices are also calculated correctly
    eigen22 (c_matrix, c_vectors, c_eigenvalues);
    c_result = c_vectors * c_eigenvalues.asDiagonal () * c_vectors.transpose ();
    c_error = c_result - c_matrix;
    diff = c_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);

    g_result = c_vectors * c_vectors.transpose ();
    g_error = g_result - CMatrix::Identity ();
    diff = g_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class Matrix>
inline void generateSymPosMatrix3x3 (Matrix& matrix)
{
  typedef typename Matrix::Scalar Scalar;

  // 3 equal elements != 0
  // 2 equal elements none 0
  // 2 equal elements 1 0
  // 2 x 0 and 1x non 0
  // 1 x 0
  // anything

  unsigned test_case = rand_uint ();

  Scalar val1 = Scalar (rand_double ());
  Scalar val2 = Scalar (rand_double ());
  Scalar val3 = Scalar (rand_double ());

  // 1%: all three values are equal and non-zero
  if (test_case == 0)
  {
    if (val1 ==0)
      val1 = 1.0;
    val2 = val1;
    val3 = val1;
  }
  // 1%: 2 values are equal but none is set explicitely to 0
  else if (test_case == 1)
  {
    val2 = val3;
  }
  // 1%: 2 values equal and the third = 0
  else if (test_case == 2)
  {
    if (val1 == 0)
      val1 = 1.0;
    val2 = val1;
    val3 = 0.0;
  }
  // 1% 2 x 0 and 1x not-0
  else if (test_case == 3)
  {
    if (val1 == 0)
      val1 = 1.0;
    val2 = val3 = 0.0;
  }
  else if (test_case == 4)
  {
    val1 = 0.0;
  }

  Scalar sqrNorm;
  Matrix eigenvectors = Matrix::Zero ();
  Matrix eigenvalues = Matrix::Zero ();

  do
  {
    eigenvectors.col (0)[0] = Scalar (rand_double ());
    eigenvectors.col (0)[1] = Scalar (rand_double ());
    eigenvectors.col (0)[2] = Scalar (rand_double ());
    eigenvectors.col (1)[0] = Scalar (rand_double ());
    eigenvectors.col (1)[1] = Scalar (rand_double ());
    eigenvectors.col (1)[2] = Scalar (rand_double ());
    eigenvectors.col (2) = eigenvectors.col (0).cross (eigenvectors.col (1));

    sqrNorm = eigenvectors.col (2).squaredNorm ();
  } while (sqrNorm == 0);

  eigenvectors.col (0).normalize ();
  eigenvectors.col (2).normalize ();
  eigenvectors.col (1) = eigenvectors.col (2).cross (eigenvectors.col (0));

  eigenvalues (0, 0) = val1;
  eigenvalues (1, 1) = val2;
  eigenvalues (2, 2) = val3;

  matrix = eigenvectors * eigenvalues * eigenvectors.adjoint();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, eigen33d)
{
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix;
  RMatrix r_vectors;
  Eigen::Matrix<Scalar, 3, 1> r_eigenvalues;
  Eigen::Matrix<Scalar, 3, 1> c_eigenvalues;
  CMatrix c_matrix;
  CMatrix c_vectors;
  Eigen::Matrix<Scalar, 3, 3> r_result;
  Eigen::Matrix<Scalar, 3, 3> r_error;
  Eigen::Matrix<Scalar, 3, 3> g_result;
  Eigen::Matrix<Scalar, 3, 3> g_error;
  Eigen::Matrix<Scalar, 3, 3> c_result;
  Eigen::Matrix<Scalar, 3, 3> c_error;
  Scalar diff;

  const Scalar epsilon = 2e-5;
  const unsigned iterations = 1000000;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    // generate test matrices
    generateSymPosMatrix3x3 (r_matrix);
    c_matrix = r_matrix;

    // calculate the eigenvalue decomposition
    eigen33 (r_matrix, r_vectors, r_eigenvalues);

    // test if U * V * U^T = M
    r_result = r_vectors * r_eigenvalues.asDiagonal () * r_vectors.transpose ();
    r_error = r_result - r_matrix;
    diff = r_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);

    // test if the eigenvalues are orthonormal
    g_result = r_vectors * r_vectors.transpose ();
    g_error = g_result - RMatrix::Identity ();
    diff = g_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);

    // test if column major matrices are also calculated correctly
    eigen33 (c_matrix, c_vectors, c_eigenvalues);
    c_result = c_vectors * c_eigenvalues.asDiagonal () * c_vectors.transpose ();
    c_error = c_result - c_matrix;
    diff = c_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);

    g_result = c_vectors * c_vectors.transpose ();
    g_error = g_result - CMatrix::Identity ();
    diff = g_error.cwiseAbs (). sum ();
    EXPECT_LE (diff, epsilon);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// since we use float in this test and some matrices are bad conditioned for the eigenvalue decomposition, we will have
// some errors > 0.2 but less than 1% is > 1e-3 -> we will just check whether the failure rate is below 1%
TEST (PCL, eigen33f)
{
  typedef float Scalar;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor> RMatrix;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::ColMajor> CMatrix;
  RMatrix r_matrix;
  RMatrix r_vectors;
  Eigen::Matrix<Scalar, 3, 1> r_eigenvalues;
  Eigen::Matrix<Scalar, 3, 1> c_eigenvalues;
  CMatrix c_matrix;
  CMatrix c_vectors;
  Eigen::Matrix<Scalar, 3, 3> r_result;
  Eigen::Matrix<Scalar, 3, 3> r_error;
  Eigen::Matrix<Scalar, 3, 3> g_result;
  Eigen::Matrix<Scalar, 3, 3> g_error;
  Eigen::Matrix<Scalar, 3, 3> c_result;
  Eigen::Matrix<Scalar, 3, 3> c_error;
  Scalar diff;

  const Scalar epsilon = 1e-3f;
  const unsigned iterations = 1000000;
  bool r_failed;
  bool c_failed;
  unsigned r_fail_count = 0;
  unsigned c_fail_count = 0;

  // test floating point row-major : row-major
  for (unsigned idx = 0; idx < iterations; ++idx)
  {
    r_failed = c_failed = false;
    // generate test matrices
    generateSymPosMatrix3x3 (r_matrix);
    c_matrix = r_matrix;

    // calculate the eigenvalue decomposition
    eigen33 (r_matrix, r_vectors, r_eigenvalues);

    // test if U * V * U^T = M
    r_result = r_vectors * r_eigenvalues.asDiagonal () * r_vectors.transpose ();
    r_error = r_result - r_matrix;
    diff = r_error.cwiseAbs (). sum ();
    if (diff > epsilon)
      r_failed = true;

    // test if the eigenvalues are orthonormal
    g_result = r_vectors * r_vectors.transpose ();
    g_error = g_result - RMatrix::Identity ();
    diff = g_error.cwiseAbs (). sum ();
    if (diff > epsilon)
      r_failed = true;

    if(r_failed)
      ++r_fail_count;

    // test if column major matrices are also calculated correctly
    eigen33 (c_matrix, c_vectors, c_eigenvalues);
    c_result = c_vectors * c_eigenvalues.asDiagonal () * c_vectors.transpose ();
    c_error = c_result - c_matrix;
    diff = c_error.cwiseAbs (). sum ();
    if (diff > epsilon)
      c_failed = true;

    g_result = c_vectors * c_vectors.transpose ();
    g_error = g_result - CMatrix::Identity ();
    diff = g_error.cwiseAbs (). sum ();
    if (diff > epsilon)
      c_failed = true;

    if(c_failed)
      ++c_fail_count;
  }

  // less than 1% failure rate
  EXPECT_LE (float(r_fail_count) / float(iterations), 0.01);
  EXPECT_LE (float(r_fail_count) / float(iterations), 0.01);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, transformLine)
{
  // This also tests transformPoint and transformVector
  Eigen::VectorXf line;
  Eigen::VectorXd lined, test;
  line.resize(6);
  lined.resize(6);
  test.resize(6);
  double tolerance = 1e-7;

  // Simple translation
  Eigen::Affine3f transformation = Eigen::Affine3f::Identity ();
  Eigen::Affine3d transformationd = Eigen::Affine3d::Identity ();
  transformation.translation() << 1, -2, 0;
  transformationd.translation() << 1, -2, 0;
  line << 1, 2, 3, 0, 1, 0;
  lined << 1, 2, 3, 0, 1, 0;
  test << 2, 0, 3, 0, 1, 0;

  EXPECT_TRUE (pcl::transformLine (line, line, transformation));
  EXPECT_TRUE (pcl::transformLine (lined, lined, transformationd));
  for (int i = 0; i < 6; i++)
  {
    EXPECT_NEAR (line[i], test[i], tolerance);
    EXPECT_NEAR (lined[i], test[i], tolerance);
  }

  // Simple rotation
  transformation = Eigen::Affine3f::Identity ();
  transformationd = Eigen::Affine3d::Identity ();
  transformation.linear() = (Eigen::Matrix3f) Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ());
  transformationd.linear() = (Eigen::Matrix3d) Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());

  line << 1, 2, 3, 0, 1, 0;
  lined << 1, 2, 3, 0, 1, 0;
  test << -2, 1, 3, -1, 0, 0;
  tolerance = 1e-4;

  EXPECT_TRUE (pcl::transformLine (line, line, transformation));
  EXPECT_TRUE (pcl::transformLine (lined, lined, transformationd));
  for (int i = 0; i < 6; i++)
  {
    EXPECT_NEAR (line[i], test[i], tolerance);
    EXPECT_NEAR (lined[i], test[i], tolerance);
  }

  // Random transformation
  transformation = Eigen::Affine3f::Identity ();
  transformationd = Eigen::Affine3d::Identity ();
  transformation.translation() << 25.97, -2.45, 0.48941;
  transformationd.translation() << 25.97, -2.45, 0.48941;
  transformation.linear() = (Eigen::Matrix3f) Eigen::AngleAxisf(M_PI/5, Eigen::Vector3f::UnitX())
  * Eigen::AngleAxisf(M_PI/3, Eigen::Vector3f::UnitY());
  transformationd.linear() = (Eigen::Matrix3d) Eigen::AngleAxisd(M_PI/5, Eigen::Vector3d::UnitX())
  * Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitY());

  line << -1, 9, 3, 1.5, 2.0, 0.2;
  lined << -1, 9, 3, 1.5, 2.0, 0.2;
  test << 28.0681, 3.44044, 7.69363, 0.923205, 2.32281, 0.205528;
  tolerance = 1e-3;

  EXPECT_TRUE (pcl::transformLine (line, line, transformation));
  EXPECT_TRUE (pcl::transformLine (lined, lined, transformationd));
  for (int i = 0; i < 6; i++)
  {
    EXPECT_NEAR (line[i], test[i], tolerance);
    EXPECT_NEAR (lined[i], test[i], tolerance);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, transformPlane)
{
  Eigen::Vector4d test;
  double tolerance = 1e-8;
  pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
  plane->values.resize(4);
  plane->values[0] = 1.0;
  plane->values[1] = 0.0;
  plane->values[2] = 0.0;
  plane->values[3] = -2.0;

  // Simple translation
  Eigen::Affine3f transformation = Eigen::Affine3f::Identity ();
  Eigen::Affine3d transformationd = Eigen::Affine3d::Identity ();
  transformation.translation() << 1, 0, 0;
  transformationd.translation() << 1, 0, 0;
  test << 1, 0, 0, -3;

  pcl::transformPlane (plane, plane, transformation);
  for (int i = 0; i < 4; i++)
  EXPECT_NEAR (plane->values[i], test[i], tolerance);

  plane->values[0] = 1.0;
  plane->values[1] = 0.0;
  plane->values[2] = 0.0;
  plane->values[3] = -2.0;
  pcl::transformPlane (plane, plane, transformationd);
  for (int i = 0; i < 4; i++)
  EXPECT_NEAR (plane->values[i], test[i], tolerance);

  // Simple rotation
  transformation.translation() << 0, 0, 0;
  transformationd.translation() << 0, 0, 0;
  transformation.linear() = (Eigen::Matrix3f) Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ());
  transformationd.linear() = (Eigen::Matrix3d) Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
  test << 0, 1, 0, -2;
  tolerance = 1e-6;

  plane->values[0] = 1.0;
  plane->values[1] = 0.0;
  plane->values[2] = 0.0;
  plane->values[3] = -2.0;
  pcl::transformPlane (plane, plane, transformation);
  for (int i = 0; i < 4; i++)
  EXPECT_NEAR (plane->values[i], test[i], tolerance);

  plane->values[0] = 1.0;
  plane->values[1] = 0.0;
  plane->values[2] = 0.0;
  plane->values[3] = -2.0;
  pcl::transformPlane (plane, plane, transformationd);
  for (int i = 0; i < 4; i++)
  EXPECT_NEAR (plane->values[i], test[i], tolerance);

  // Random transformation
  transformation.translation() << 12.5, -5.4, 0.1;
  transformationd.translation() << 12.5, -5.4, 0.1;
  transformation.linear() = (Eigen::Matrix3f) Eigen::AngleAxisf(M_PI/7, Eigen::Vector3f::UnitY())
  * Eigen::AngleAxisf(M_PI/4, Eigen::Vector3f::UnitZ());
  transformationd.linear() = (Eigen::Matrix3d) Eigen::AngleAxisd(M_PI/7, Eigen::Vector3d::UnitY())
  * Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ());
  test << 5.35315, 2.89914, 0.196848, -49.2788;
  tolerance = 1e-4;

  plane->values[0] = 5.4;
  plane->values[1] = -1.3;
  plane->values[2] = 2.5;
  plane->values[3] = 2.0;
  pcl::transformPlane (plane, plane, transformation);
  for (int i = 0; i < 4; i++)
  EXPECT_NEAR (plane->values[i], test[i], tolerance);

  plane->values[0] = 5.4;
  plane->values[1] = -1.3;
  plane->values[2] = 2.5;
  plane->values[3] = 2.0;
  pcl::transformPlane (plane, plane, transformationd);
  for (int i = 0; i < 4; i++)
  EXPECT_NEAR (plane->values[i], test[i], tolerance);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, checkCoordinateSystem)
{
  Eigen::VectorXd line_x, line_y;
  Eigen::VectorXd line_xd, line_yd;
  line_x.resize(6); line_y.resize(6);
  line_xd.resize(6); line_yd.resize(6);

  Eigen::Vector3d origin, vector_x, vector_y;
  Eigen::Vector3d origind, vector_xd, vector_yd;

  // No translation, no rotation coordinate system
  line_x << 0, 0, 0, 0, 1, 0;// Y
  line_y << 0, 0, 0, 0, 0, 1;// Z
  line_xd << 0, 0, 0, 0, 1, 0;
  line_yd << 0, 0, 0, 0, 0, 1;
  EXPECT_TRUE (pcl::checkCoordinateSystem (line_x, line_y ));
  EXPECT_TRUE (pcl::checkCoordinateSystem (line_xd, line_yd));

  origin << 0, 0, 0;
  vector_x << 0, 1, 0;// Y
  vector_y << 0, 0, 1;// Z
  origind << 0, 0, 0;
  vector_xd << 0, 1, 0;
  vector_yd << 0, 0, 1;
  EXPECT_TRUE (pcl::checkCoordinateSystem (origin, vector_x, vector_y ));
  EXPECT_TRUE (pcl::checkCoordinateSystem (origind, vector_xd, vector_yd));

  // Simple translated coordinate system
  line_x << 10, -1, 0, 1, 0, 0;// X
  line_y << 10, -1, 0, 0, 1, 0;// Y
  line_xd << 10, -1, 0, 1, 0, 0;
  line_yd << 10, -1, 0, 0, 1, 0;
  EXPECT_TRUE (pcl::checkCoordinateSystem (line_x, line_y ));
  EXPECT_TRUE (pcl::checkCoordinateSystem (line_xd, line_yd));

  origin << 10, -1, 0;
  vector_x << 1, 0, 0;
  vector_y << 0, 1, 0;
  origind << 10, -1, 0;
  vector_xd << 1, 0, 0;
  vector_yd << 0, 1, 0;
  EXPECT_TRUE (pcl::checkCoordinateSystem (origin, vector_x, vector_y ));
  EXPECT_TRUE (pcl::checkCoordinateSystem (origind, vector_xd, vector_yd));

  // General coordinate system
  line_x << 1234.56, 0.0, 0.0, 0.00387281179, 0.00572064891, -0.999976099;
  line_y << 1234.56, 0.0, 0.0, 0.6113801, -0.79133445, -0.00202810019;
  line_xd << 1234.56, 0.0, 0.0, 0.00387281179, 0.00572064891, -0.999976099;
  line_yd << 1234.56, 0.0, 0.0, 0.6113801, -0.79133445, -0.00202810019;
  EXPECT_TRUE (pcl::checkCoordinateSystem (line_x, line_y , 1e-7, 5e-3));
  EXPECT_TRUE (pcl::checkCoordinateSystem (line_xd, line_yd, 1e-7, 5e-3));

  origin << 1234.56, 0.0, 0.0;
  vector_x << 0.00387281179, 0.00572064891, -0.999976099;
  vector_y << 0.6113801, -0.79133445, -0.00202810019;
  origind << 1234.56, 0.0, 0.0;
  vector_xd << 0.00387281179, 0.00572064891, -0.999976099;
  vector_yd << 0.6113801, -0.79133445, -0.00202810019;
  EXPECT_TRUE (pcl::checkCoordinateSystem (origin, vector_x, vector_y , 1e-7, 5e-3));
  EXPECT_TRUE (pcl::checkCoordinateSystem (origind, vector_xd, vector_yd, 1e-7, 5e-3));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, transformBetween2CoordinateSystems)
{
  Eigen::Affine3f transformation;
  Eigen::Affine3d transformationd;
  Eigen::VectorXf from_line_x, from_line_y, to_line_x, to_line_y;
  Eigen::VectorXd from_line_xd, from_line_yd, to_line_xd, to_line_yd;
  from_line_x.resize(6); from_line_y.resize(6);
  to_line_x.resize(6); to_line_y.resize(6);
  from_line_xd.resize(6); from_line_yd.resize(6);
  to_line_xd.resize(6); to_line_yd.resize(6);

  Eigen::Matrix4d test;
  double tolerance = 1e-3;

  // Simple translation
  test << 1, 0, 0, 10,
  0, 1, 0, -5,
  0, 0, 1, 1,
  0, 0, 0, 1;

  from_line_x << 0, 0, 0, 1, 0, 0;
  from_line_y << 0, 0, 0, 0, 1, 0;
  to_line_x << 10, -5, 1, 1, 0, 0;
  to_line_y << 10, -5, 1, 0, 1, 0;
  EXPECT_TRUE (pcl::transformBetween2CoordinateSystems (from_line_x, from_line_y, to_line_x, to_line_y, transformation));

  for (int i = 0; i < 3; i++)
  for (int j = 0; j < 3; j++)
  EXPECT_LE ((transformation.matrix())(i,j) - test(i,j), tolerance);

  from_line_xd << 0, 0, 0, 1, 0, 0;
  from_line_yd << 0, 0, 0, 0, 1, 0;
  to_line_xd << 10, -5, 1, 1, 0, 0;
  to_line_yd << 10, -5, 1, 0, 1, 0;
  EXPECT_TRUE (pcl::transformBetween2CoordinateSystems (from_line_xd, from_line_yd, to_line_xd, to_line_yd, transformationd));

  for (int i = 0; i < 3; i++)
  for (int j = 0; j < 3; j++)
  EXPECT_LE ((transformationd.matrix())(i,j) - test(i,j), tolerance);

  // Simple rotation
  test << 0, 0, 1, 0,
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 0, 1;

  from_line_x << 0, 0, 0, 1, 0, 0;
  from_line_y << 0, 0, 0, 0, 1, 0;
  to_line_x << 0, 0, 0, 0, 1, 0;
  to_line_y << 0, 0, 0, 0, 0, 1;
  EXPECT_TRUE (pcl::transformBetween2CoordinateSystems (from_line_x, from_line_y, to_line_x, to_line_y, transformation));

  for (int i = 0; i < 3; i++)
  for (int j = 0; j < 3; j++)
  EXPECT_LE ((transformation.matrix())(i,j) - test(i,j), tolerance);

  from_line_xd << 0, 0, 0, 1, 0, 0;
  from_line_yd << 0, 0, 0, 0, 1, 0;
  to_line_xd << 0, 0, 1, 0, 1, 0;
  to_line_yd << 0, 0, 1, 0, 0, 1;
  EXPECT_TRUE (pcl::transformBetween2CoordinateSystems (from_line_xd, from_line_yd, to_line_xd, to_line_yd, transformationd));

  for (int i = 0; i < 3; i++)
  for (int j = 0; j < 3; j++)
  EXPECT_LE ((transformationd.matrix())(i,j) - test(i,j), tolerance);

  // General case
  test << 0.00397405, 0.00563289, -0.999976, -4.9062,
  0.611348, -0.791359, -0.00213906, -754.746,
  -0.791352, -0.611325, -0.00658855, 976.972,
  0, 0, 0, 1;

  from_line_x << 1234.56, 0.0, 0.0, 0.00387281179, 0.00572064891, -0.999976099;
  from_line_y << 1234.56, 0.0, 0.0, 0.6113801, -0.79133445, -0.00202810019;
  to_line_x << 0, 0, 0, 1, 0, 0;
  to_line_y << 0, 0, 0, 0, 1, 0;
  EXPECT_TRUE (pcl::transformBetween2CoordinateSystems (from_line_x, from_line_y, to_line_x, to_line_y, transformation));

  for (int i = 0; i < 3; i++)
  for (int j = 0; j < 3; j++)
  EXPECT_LE ((transformation.matrix())(i,j) - test(i,j), tolerance);

  from_line_xd << 1234.56, 0.0, 0.0, 0.00387281179, 0.00572064891, -0.999976099;
  from_line_yd << 1234.56, 0.0, 0.0, 0.6113801, -0.79133445, -0.00202810019;
  to_line_xd << 0, 0, 0, 1, 0, 0;
  to_line_yd << 0, 0, 0, 0, 1, 0;
  EXPECT_TRUE (pcl::transformBetween2CoordinateSystems (from_line_xd, from_line_yd, to_line_xd, to_line_yd, transformationd));

  for (int i = 0; i < 3; i++)
  for (int j = 0; j < 3; j++)
  EXPECT_LE ((transformationd.matrix())(i,j) - test(i,j), tolerance);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

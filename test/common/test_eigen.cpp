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
//    c_matrix.coeffRef (3) = c_matrix.coeffRef (1);
//    c_matrix.coeffRef (6) = c_matrix.coeffRef (2);
//    c_matrix.coeffRef (7) = c_matrix.coeffRef (5);

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
//    c_matrix.coeffRef (3) = c_matrix.coeffRef (1);
//    c_matrix.coeffRef (6) = c_matrix.coeffRef (2);
//    c_matrix.coeffRef (7) = c_matrix.coeffRef (5);

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

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

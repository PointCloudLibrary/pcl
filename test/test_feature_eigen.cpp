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
 * $Id: test_feature.cpp 3564 2011-12-16 06:11:13Z rusu $
 *
 */

#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/pcl_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/features/boundary.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/ppf.h>
#include <pcl/features/vfh.h>
#include <pcl/features/gfpfh.h>
#include <pcl/features/rsd.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/features/rift.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/usc.h>
#include <pcl/features/board.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::test;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud;
vector<int> indices;
KdTreePtr tree;
boost::variate_generator< boost::mt19937, boost::uniform_real<double> > rand_double(boost::mt19937 (), boost::uniform_real<double> (0, 1));
boost::variate_generator< boost::mt19937, boost::uniform_int<unsigned> > rand_uint(boost::mt19937 (), boost::uniform_int<unsigned> (0, 100));

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT> void
testIndicesAndSearchSurfaceEigen (const typename PointCloud<PointT>::Ptr & points,
                                  const typename PointCloud<NormalT>::Ptr & normals,
                                  const boost::shared_ptr<vector<int> > & indices, int ndims)
{
  //
  // Test setIndices and setSearchSurface
  //
  PointCloud<Eigen::MatrixXf> full_output, output0, output1, output2;

  // Compute for all points and then subsample the results
  FeatureEstimation est0;
  est0.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est0.setKSearch (10);
  est0.setInputCloud (points);
  est0.setInputNormals (normals);
  est0.computeEigen (full_output);
  output0 = PointCloud<Eigen::MatrixXf> (full_output, *indices);
  //copyPointCloud (full_output, *indices, output0);

  // Compute with all points as "search surface" and the specified sub-cloud as "input"
  typename PointCloud<PointT>::Ptr subpoints (new PointCloud<PointT>);
  copyPointCloud (*points, *indices, *subpoints);
  FeatureEstimation est1;
  est1.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est1.setKSearch (10);
  est1.setInputCloud (subpoints);
  est1.setSearchSurface (points);
  est1.setInputNormals (normals);
  est1.computeEigen (output1);

  // Compute with all points as "input" and the specified indices
  FeatureEstimation est2;
  est2.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est2.setKSearch (10);
  est2.setInputCloud (points);
  est2.setInputNormals (normals);
  est2.setIndices (indices);
  est2.computeEigen (output2);

  // All three of the above cases should produce equivalent results
  ASSERT_EQ (output0.points.rows (), output1.points.rows ());
  ASSERT_EQ (output1.points.rows (), output2.points.rows ());
  for (int i = 0; i < output1.points.rows (); ++i)
  {
    for (int j = 0; j < ndims; ++j)
    {
      ASSERT_EQ (output0.points (i, j), output1.points (i, j));
      ASSERT_EQ (output1.points (i, j), output2.points (i, j));
    }
  }

  //
  // Test the combination of setIndices and setSearchSurface
  //
  PointCloud<Eigen::MatrixXf> output3, output4;

  boost::shared_ptr<vector<int> > indices2 (new vector<int> (0));
  for (size_t i = 0; i < (indices->size ()/2); ++i)
    indices2->push_back (i);

  // Compute with all points as search surface + the specified sub-cloud as "input" but for only a subset of indices
  FeatureEstimation est3;
  est3.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est3.setKSearch (10);
  est3.setSearchSurface (points);
  est3.setInputNormals (normals);
  est3.setInputCloud (subpoints);
  est3.setIndices (indices2);
  est3.computeEigen (output3);

  // Start with features for each point in "subpoints" and then subsample the results
  output4 = PointCloud<Eigen::MatrixXf> (output0, *indices2); // (Re-using "output0" from above)
  //copyPointCloud (output0, *indices2, output4);

  // The two cases above should produce equivalent results
  ASSERT_EQ (output3.points.rows (), output4.points.rows ());
  for (int i = 0; i < output3.points.rows (); ++i)
  {
    for (int j = 0; j < ndims; ++j)
    {
      ASSERT_EQ (output3.points (i, j), output4.points (i, j));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT, typename OutputT> FeatureEstimation
createSHOTDesc (const typename PointCloud<NormalT>::Ptr & normals,
                const int nr_shape_bins = 10,
                const int nr_color_bins = 30,
                const bool describe_shape = true,
                const bool describe_color = false)
{
  FeatureEstimation f (nr_shape_bins);
  f.setInputNormals (normals);
  return (f);
}

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename NormalT, typename OutputT> FeatureEstimation
createSHOTDesc (const typename PointCloud<NormalT>::Ptr & normals,
                const int nr_shape_bins = 10,
                const int nr_color_bins = 30,
                const bool describe_shape = true,
                const bool describe_color = false)
{
  FeatureEstimation f (describe_shape, describe_color, nr_shape_bins,nr_color_bins);
  f.setInputNormals (normals);
  return (f);
}

///////////////////////////////////////////////////////////////////////////////////
template <> UniqueShapeContext<PointXYZ, Eigen::MatrixXf>
createSHOTDesc<UniqueShapeContext<PointXYZ, Eigen::MatrixXf>, PointXYZ, Normal, Eigen::MatrixXf> (
    const PointCloud<Normal>::Ptr & normals,
    const int nr_shape_bins,
    const int nr_color_bins,
    const bool describe_shape,
    const bool describe_color)
    {
  UniqueShapeContext<PointXYZ, Eigen::MatrixXf> usc;
  usc.setAzimuthBins (4);
  usc.setElevationBins (4);
  usc.setRadiusBins (4);
  usc.setMinimalRadius (0.004);
  usc.setPointDensityRadius (0.008);
  usc.setLocalRadius (0.04);
  return (usc);
    }

///////////////////////////////////////////////////////////////////////////////////
template <> ShapeContext3DEstimation<PointXYZ, Normal, Eigen::MatrixXf>
createSHOTDesc<ShapeContext3DEstimation<PointXYZ, Normal, Eigen::MatrixXf>, PointXYZ, Normal, Eigen::MatrixXf> (
    const PointCloud<Normal>::Ptr & normals,
    const int nr_shape_bins,
    const int nr_color_bins,
    const bool describe_shape,
    const bool describe_color)
    {
  ShapeContext3DEstimation<PointXYZ, Normal, Eigen::MatrixXf> sc3d;
  sc3d.setAzimuthBins (4);
  sc3d.setElevationBins (4);
  sc3d.setRadiusBins (4);
  sc3d.setMinimalRadius (0.004);
  sc3d.setPointDensityRadius (0.008);
  sc3d.setInputNormals (normals);
  return (sc3d);
    }

///////////////////////////////////////////////////////////////////////////////////
template <typename FeatureEstimation, typename PointT, typename NormalT> void
testSHOTIndicesAndSearchSurfaceEigen (const typename PointCloud<PointT>::Ptr & points,
                                      const typename PointCloud<NormalT>::Ptr & normals,
                                      const boost::shared_ptr<vector<int> > & indices,
                                      const int nr_shape_bins = 10,
                                      const int nr_color_bins = 30,
                                      const bool describe_shape = true,
                                      const bool describe_color = false)
{
  double radius = 0.04;
  //
  // Test setIndices and setSearchSurface
  //
  PointCloud<Eigen::MatrixXf> full_output, output0, output1, output2;

  // Compute for all points and then subsample the results
  FeatureEstimation est0 = createSHOTDesc<FeatureEstimation, PointT, NormalT, Eigen::MatrixXf>(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est0.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est0.setRadiusSearch (radius);
  est0.setInputCloud (points);
  est0.computeEigen (full_output);

  output0 = PointCloud<Eigen::MatrixXf> (full_output, *indices);

  // Compute with all points as "search surface" and the specified sub-cloud as "input"
  typename PointCloud<PointT>::Ptr subpoints (new PointCloud<PointT>);
  copyPointCloud (*points, *indices, *subpoints);
  FeatureEstimation est1 = createSHOTDesc<FeatureEstimation, PointT, NormalT, Eigen::MatrixXf>(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est1.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est1.setRadiusSearch (radius);
  est1.setInputCloud (subpoints);
  est1.setSearchSurface (points);
  est1.computeEigen (output1);

  //// Compute with all points as "input" and the specified indices
  FeatureEstimation est2 = createSHOTDesc<FeatureEstimation, PointT, NormalT, Eigen::MatrixXf>(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est2.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est2.setRadiusSearch (radius);
  est2.setInputCloud (points);
  est2.setIndices (indices);
  est2.computeEigen (output2);

  // All three of the above cases should produce equivalent results
  ASSERT_EQ (output0.points.rows (), output1.points.rows ());
  ASSERT_EQ (output1.points.rows (), output2.points.rows ());
  for (int i = 0; i < output1.points.rows (); ++i)
  {
    for (int j = 0; j < output0.points.cols (); ++j)
    {
      ASSERT_EQ (output0.points (i, j), output1.points (i, j));
      ASSERT_EQ (output1.points (i, j), output2.points (i, j));
    }
  }

  //
  // Test the combination of setIndices and setSearchSurface
  //
  PointCloud<Eigen::MatrixXf> output3, output4;

  boost::shared_ptr<vector<int> > indices2 (new vector<int> (0));
  for (size_t i = 0; i < (indices->size ()/2); ++i)
    indices2->push_back (i);

  // Compute with all points as search surface + the specified sub-cloud as "input" but for only a subset of indices
  FeatureEstimation est3 = createSHOTDesc<FeatureEstimation, PointT, NormalT, Eigen::MatrixXf>(normals, nr_shape_bins,nr_color_bins,describe_shape,describe_color);
  est3.setSearchMethod (typename search::KdTree<PointT>::Ptr (new search::KdTree<PointT>));
  est3.setRadiusSearch (0.04);
  est3.setSearchSurface (points);
  est3.setInputCloud (subpoints);
  est3.setIndices (indices2);
  est3.computeEigen (output3);

  // Start with features for each point in "subpoints" and then subsample the results
  output4 = PointCloud<Eigen::MatrixXf> (output0, *indices2);

  // The two cases above should produce equivalent results
  ASSERT_EQ (output3.points.rows (), output4.points.rows ());
  for (int i = 0; i < output3.points.rows (); ++i)
    for (int j = 0; j < output3.points.cols (); ++j)
      ASSERT_EQ (output3.points (i, j), output4.points (i, j));
}

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
  const Scalar epsilon = 1e-5;
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
  const Scalar epsilon = 1e-5;
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
  const Scalar epsilon = 1e-6;
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
  Matrix eigenvectors = Matrix::Zero ();
  Matrix eigenvalues = Matrix::Zero ();

  do
  {
    eigenvectors.col (0)[0] = Scalar (rand_double ());
    eigenvectors.col (0)[1] = Scalar (rand_double ());
    sqrNorm = eigenvectors.col (0).squaredNorm ();
  } while (sqrNorm == 0);
  eigenvectors.col (0) /= sqrt (sqrNorm);

  eigenvectors.col (1)[0] = -eigenvectors.col (1)[1];
  eigenvectors.col (1)[1] =  eigenvectors.col (1)[0];

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

  const Scalar epsilon = 1e-14;
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

  const Scalar epsilon = 1e-6;
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

  const Scalar epsilon = 1e-3;
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
TEST (PCL, NormalEstimationEigen)
{
  Eigen::Vector4f plane_parameters;
  float curvature;

  NormalEstimation<PointXYZ, Eigen::MatrixXf> n;

  // computePointNormal (indices, Vector)
  computePointNormal (cloud, indices, plane_parameters, curvature);
  EXPECT_NEAR (fabs (plane_parameters[0]), 0.035592, 1e-4);
  EXPECT_NEAR (fabs (plane_parameters[1]), 0.369596, 1e-4);
  EXPECT_NEAR (fabs (plane_parameters[2]), 0.928511, 1e-4);
  EXPECT_NEAR (fabs (plane_parameters[3]), 0.0622552, 1e-4);
  EXPECT_NEAR (curvature, 0.0693136, 1e-4);

  float nx, ny, nz;
  // computePointNormal (indices)
  n.computePointNormal (cloud, indices, nx, ny, nz, curvature);
  EXPECT_NEAR (fabs (nx), 0.035592, 1e-4);
  EXPECT_NEAR (fabs (ny), 0.369596, 1e-4);
  EXPECT_NEAR (fabs (nz), 0.928511, 1e-4);
  EXPECT_NEAR (curvature, 0.0693136, 1e-4);

  // computePointNormal (Vector)
  computePointNormal (cloud, plane_parameters, curvature);
  EXPECT_NEAR (plane_parameters[0],  0.035592,  1e-4);
  EXPECT_NEAR (plane_parameters[1],  0.369596,  1e-4);
  EXPECT_NEAR (plane_parameters[2],  0.928511,  1e-4);
  EXPECT_NEAR (plane_parameters[3], -0.0622552, 1e-4);
  EXPECT_NEAR (curvature,            0.0693136, 1e-4);

  // flipNormalTowardsViewpoint (Vector)
  flipNormalTowardsViewpoint (cloud.points[0], 0, 0, 0, plane_parameters);
  EXPECT_NEAR (plane_parameters[0], -0.035592,  1e-4);
  EXPECT_NEAR (plane_parameters[1], -0.369596,  1e-4);
  EXPECT_NEAR (plane_parameters[2], -0.928511,  1e-4);
  EXPECT_NEAR (plane_parameters[3],  0.0799743, 1e-4);

  // flipNormalTowardsViewpoint
  flipNormalTowardsViewpoint (cloud.points[0], 0, 0, 0, nx, ny, nz);
  EXPECT_NEAR (nx, -0.035592, 1e-4);
  EXPECT_NEAR (ny, -0.369596, 1e-4);
  EXPECT_NEAR (nz, -0.928511, 1e-4);

  // Object
  PointCloud<Eigen::MatrixXf>::Ptr normals (new PointCloud<Eigen::MatrixXf> ());

  // set parameters
  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();
  n.setInputCloud (cloudptr);
  EXPECT_EQ (n.getInputCloud (), cloudptr);
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  EXPECT_EQ (n.getIndices (), indicesptr);
  n.setSearchMethod (tree);
  EXPECT_EQ (n.getSearchMethod (), tree);
  n.setKSearch (indices.size ());

  // estimate
  n.computeEigen (*normals);
  EXPECT_EQ (normals->points.rows (), indices.size ());

  for (int i = 0; i < normals->points.rows (); ++i)
  {
    EXPECT_NEAR (normals->points.row (i)[0], -0.035592, 1e-4);
    EXPECT_NEAR (normals->points.row (i)[1], -0.369596, 1e-4);
    EXPECT_NEAR (normals->points.row (i)[2], -0.928511, 1e-4);
    EXPECT_NEAR (normals->points.row (i)[3], 0.0693136, 1e-4);
  }

  PointCloud<PointXYZ>::Ptr surfaceptr = cloudptr;
  n.setSearchSurface (surfaceptr);
  EXPECT_EQ (n.getSearchSurface (), surfaceptr);

  // Additional test for searchForNeigbhors
  surfaceptr.reset (new PointCloud<PointXYZ>);
  *surfaceptr = *cloudptr;
  surfaceptr->points.resize (640 * 480);
  surfaceptr->width = 640;
  surfaceptr->height = 480;
  EXPECT_EQ (surfaceptr->points.size (), surfaceptr->width * surfaceptr->height);
  n.setSearchSurface (surfaceptr);
  tree.reset ();
  n.setSearchMethod (tree);

  // estimate
  n.computeEigen (*normals);
  EXPECT_EQ (normals->points.rows (), indices.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, NormalEstimationOpenMPEigen)
{
  NormalEstimationOMP<PointXYZ, Eigen::MatrixXf> n (4); // instantiate 4 threads

  // Object
  PointCloud<Eigen::MatrixXf>::Ptr normals (new PointCloud<Eigen::MatrixXf>);

  // set parameters
  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();
  n.setInputCloud (cloudptr);
  EXPECT_EQ (n.getInputCloud (), cloudptr);
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  EXPECT_EQ (n.getIndices (), indicesptr);
  n.setSearchMethod (tree);
  EXPECT_EQ (n.getSearchMethod (), tree);
  n.setKSearch (indices.size ());

  // estimate
  n.computeEigen (*normals);
  EXPECT_EQ (normals->points.rows (), indices.size ());

  for (int i = 0; i < normals->points.rows (); ++i)
  {
    EXPECT_NEAR (normals->points.row (i)[0], -0.035592, 1e-4);
    EXPECT_NEAR (normals->points.row (i)[1], -0.369596, 1e-4);
    EXPECT_NEAR (normals->points.row (i)[2], -0.928511, 1e-4);
    EXPECT_NEAR (normals->points.row (i)[3], 0.0693136, 1e-4);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, MomentInvariantsEstimationEigen)
{
  float j1, j2, j3;

  MomentInvariantsEstimation<PointXYZ, Eigen::MatrixXf> mi;

  // computePointMomentInvariants (indices))
  mi.computePointMomentInvariants (cloud, indices, j1, j2, j3);
  EXPECT_NEAR (j1, 1.59244, 1e-4);
  EXPECT_NEAR (j2, 0.652063, 1e-4);
  EXPECT_NEAR (j3, 0.053917, 1e-4);

  // computePointMomentInvariants
  mi.computePointMomentInvariants (cloud, indices, j1, j2, j3);
  EXPECT_NEAR (j1, 1.59244, 1e-4);
  EXPECT_NEAR (j2, 0.652063, 1e-4);
  EXPECT_NEAR (j3, 0.053917, 1e-4);

  // Object
  PointCloud<Eigen::MatrixXf>::Ptr moments (new PointCloud<Eigen::MatrixXf>);

  // set parameters
  mi.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  mi.setIndices (indicesptr);
  mi.setSearchMethod (tree);
  mi.setKSearch (indices.size ());

  // estimate
  mi.computeEigen (*moments);
  EXPECT_EQ (moments->points.rows (), indices.size ());

  for (int i = 0; i < moments->points.rows (); ++i)
  {
    EXPECT_NEAR (moments->points (i, 0), 1.59244, 1e-4);
    EXPECT_NEAR (moments->points (i, 1), 0.652063, 1e-4);
    EXPECT_NEAR (moments->points (i, 2), 0.053917, 1e-4);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BOARDLocalReferenceFrameEstimationEigen)
{
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  PointCloud<Eigen::MatrixXf> bunny_LRF;

  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));

  //compute normals
  NormalEstimation<PointXYZ, Normal> ne;

  ne.setRadiusSearch (0.01);
  ne.setViewPoint (1, 1, 10);
  ne.setInputCloud (cloud.makeShared ());
  ne.setSearchMethod (tree);
  ne.setIndices (indicesptr);

  ne.compute (*normals);

  //compute BOARD LRF
  BOARDLocalReferenceFrameEstimation<PointXYZ, Normal, Eigen::MatrixXf> lrf_estimator;

  float meshRes = 0.001;

  lrf_estimator.setFindHoles (true);
  lrf_estimator.setRadiusSearch (15 * meshRes);
  lrf_estimator.setTangentRadius (15 * meshRes);

  lrf_estimator.setInputCloud (cloud.makeShared ());
  lrf_estimator.setInputNormals (normals);
  lrf_estimator.setSearchMethod (tree);
  lrf_estimator.setIndices (indicesptr);

  lrf_estimator.computeEigen (bunny_LRF);

  //TESTS
  EXPECT_EQ (indices.size (), bunny_LRF.size ());

  EXPECT_FALSE (bunny_LRF.is_dense);
  EXPECT_EQ (numeric_limits<float>::max (), bunny_LRF.points (24, 0));
  EXPECT_TRUE (pcl_isnan (bunny_LRF.points (24, 1)));

  //Expected Results
  float point_15_conf = -9.06301;
  Eigen::Vector3f point_15_x (-0.784923f, 0.208529f, 0.583448f);
  Eigen::Vector3f point_15_y (0.334206f, -0.650436f, 0.682085f);
  Eigen::Vector3f point_15_z (0.52173f, 0.730376f, 0.440851f);

  float point_45_conf = -9.55398;
  Eigen::Vector3f point_45_x (0.909111f, 0.30943f, 0.278874f);
  Eigen::Vector3f point_45_y (-0.362239f, 0.917811f, 0.162501f);
  Eigen::Vector3f point_45_z (-0.205671f, -0.248751f, 0.946479f);

  float point_163_conf = -9.04891;
  Eigen::Vector3f point_163_x (-0.443962f, -0.890073f, -0.103285f);
  Eigen::Vector3f point_163_y (0.746929f, -0.30394f, -0.591369f);
  Eigen::Vector3f point_163_z (0.494969f, -0.339693f, 0.799759f);

  float point_253_conf = -9.09443;
  Eigen::Vector3f point_253_x (-0.616855f, 0.757286f, -0.214495f);
  Eigen::Vector3f point_253_y (-0.661937f, -0.646584f, -0.379168f);
  Eigen::Vector3f point_253_z (-0.425827f, -0.0919098f, 0.900124f);

  ////Test Results
  EXPECT_NEAR (point_15_conf, bunny_LRF.points (15,0), 1E-3);
  EXPECT_NEAR_VECTORS (point_15_x, bunny_LRF.points.block<1,3> (15, 1), 1E-3);
  EXPECT_NEAR_VECTORS (point_15_y, bunny_LRF.points.block<1,3> (15, 4), 1E-3);
  EXPECT_NEAR_VECTORS (point_15_z, bunny_LRF.points.block<1,3> (15, 7), 1E-3);

  EXPECT_NEAR (point_45_conf, bunny_LRF.points (45, 0), 1E-3);
  EXPECT_NEAR_VECTORS (point_45_x, bunny_LRF.points.block<1,3> (45, 1), 1E-3);
  EXPECT_NEAR_VECTORS (point_45_y, bunny_LRF.points.block<1,3> (45, 4), 1E-3);
  EXPECT_NEAR_VECTORS (point_45_z, bunny_LRF.points.block<1,3> (45, 7), 1E-3);

  EXPECT_NEAR (point_163_conf, bunny_LRF.points (163, 0), 1E-3);
  EXPECT_NEAR_VECTORS (point_163_x, bunny_LRF.points.block<1,3> (163, 1), 1E-3);
  EXPECT_NEAR_VECTORS (point_163_y, bunny_LRF.points.block<1,3> (163, 4), 1E-3);
  EXPECT_NEAR_VECTORS (point_163_z, bunny_LRF.points.block<1,3> (163, 7), 1E-3);

  EXPECT_NEAR (point_253_conf, bunny_LRF.points (253, 0), 1E-3);
  EXPECT_NEAR_VECTORS (point_253_x, bunny_LRF.points.block<1,3> (253, 1), 1E-3);
  EXPECT_NEAR_VECTORS (point_253_y, bunny_LRF.points.block<1,3> (253, 4), 1E-3);
  EXPECT_NEAR_VECTORS (point_253_z, bunny_LRF.points.block<1,3> (253, 7), 1E-3);  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BoundaryEstimationEigen)
{
  Eigen::Vector4f u = Eigen::Vector4f::Zero ();
  Eigen::Vector4f v = Eigen::Vector4f::Zero ();

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (indices.size ());
  // estimate
  n.compute (*normals);

  BoundaryEstimation<PointXYZ, Normal, Eigen::MatrixXf> b;
  b.setInputNormals (normals);
  EXPECT_EQ (b.getInputNormals (), normals);

  // getCoordinateSystemOnPlane
  for (size_t i = 0; i < normals->points.size (); ++i)
  {
    b.getCoordinateSystemOnPlane (normals->points[i], u, v);
    Vector4fMap n4uv = normals->points[i].getNormalVector4fMap ();
    EXPECT_NEAR (n4uv.dot(u), 0, 1e-4);
    EXPECT_NEAR (n4uv.dot(v), 0, 1e-4);
    EXPECT_NEAR (u.dot(v), 0, 1e-4);
  }

  // isBoundaryPoint (indices)
  bool pt = false;
  pt = b.isBoundaryPoint (cloud, 0, indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, indices.size () / 3, indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, indices.size () / 2, indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, indices.size () - 1, indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, true);

  // isBoundaryPoint (points)
  pt = false;
  pt = b.isBoundaryPoint (cloud, cloud.points[0], indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, cloud.points[indices.size () / 3], indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, cloud.points[indices.size () / 2], indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, false);
  pt = b.isBoundaryPoint (cloud, cloud.points[indices.size () - 1], indices, u, v, M_PI / 2.0);
  EXPECT_EQ (pt, true);

  // Object
  PointCloud<Eigen::MatrixXf>::Ptr bps (new PointCloud<Eigen::MatrixXf> ());

  // set parameters
  b.setInputCloud (cloud.makeShared ());
  b.setIndices (indicesptr);
  b.setSearchMethod (tree);
  b.setKSearch (indices.size ());

  // estimate
  b.computeEigen (*bps);
  EXPECT_EQ (bps->points.rows (), indices.size ());

  pt = bps->points (0, 0);
  EXPECT_EQ (pt, false);
  pt = bps->points (indices.size () / 3, 0);
  EXPECT_EQ (pt, false);
  pt = bps->points (indices.size () / 2, 0);
  EXPECT_EQ (pt, false);
  pt = bps->points (indices.size () - 1, 0);
  EXPECT_EQ (pt, true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PrincipalCurvaturesEstimationEigen)
{
  float pcx, pcy, pcz, pc1, pc2;

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);

  PrincipalCurvaturesEstimation<PointXYZ, Normal, Eigen::MatrixXf> pc;
  pc.setInputNormals (normals);
  EXPECT_EQ (pc.getInputNormals (), normals);

  // computePointPrincipalCurvatures (indices)
  pc.computePointPrincipalCurvatures (*normals, 0, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (fabs (pcx), 0.98509, 1e-4);
  EXPECT_NEAR (fabs (pcy), 0.10714, 1e-4);
  EXPECT_NEAR (fabs (pcz), 0.13462, 1e-4);
  EXPECT_NEAR (pc1, 0.23997423052787781, 1e-4);
  EXPECT_NEAR (pc2, 0.19400238990783691, 1e-4);

  pc.computePointPrincipalCurvatures (*normals, 2, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (pcx, 0.98079, 1e-4);
  EXPECT_NEAR (pcy, -0.04019, 1e-4);
  EXPECT_NEAR (pcz, 0.19086, 1e-4);
  EXPECT_NEAR (pc1, 0.27207490801811218, 1e-4);
  EXPECT_NEAR (pc2, 0.19464978575706482, 1e-4);

  pc.computePointPrincipalCurvatures (*normals, indices.size () - 3, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (pcx, 0.86725, 1e-4);
  EXPECT_NEAR (pcy, -0.37599, 1e-4);
  EXPECT_NEAR (pcz, 0.32635, 1e-4);
  EXPECT_NEAR (pc1, 0.25900053977966309, 1e-4);
  EXPECT_NEAR (pc2, 0.17906945943832397, 1e-4);

  pc.computePointPrincipalCurvatures (*normals, indices.size () - 1, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (pcx, 0.86725, 1e-4);
  EXPECT_NEAR (pcy, -0.375851, 1e-3);
  EXPECT_NEAR (pcz, 0.32636, 1e-4);
  EXPECT_NEAR (pc1, 0.2590005099773407,  1e-4);
  EXPECT_NEAR (pc2, 0.17906956374645233, 1e-4);

  // Object
  PointCloud<Eigen::MatrixXf>::Ptr pcs (new PointCloud<Eigen::MatrixXf>);

  // set parameters
  pc.setInputCloud (cloud.makeShared ());
  pc.setIndices (indicesptr);
  pc.setSearchMethod (tree);
  pc.setKSearch (indices.size ());

  // estimate
  pc.computeEigen (*pcs);
  EXPECT_EQ (pcs->points.rows (), indices.size ());

  // Adjust for small numerical inconsitencies (due to nn_indices not being sorted)
  EXPECT_NEAR (fabs (pcs->points (0, 0)), 0.98509, 1e-4);
  EXPECT_NEAR (fabs (pcs->points (0, 1)), 0.10713, 1e-4);
  EXPECT_NEAR (fabs (pcs->points (0, 2)), 0.13462, 1e-4);
  EXPECT_NEAR (fabs (pcs->points (0, 3)), 0.23997458815574646, 1e-4);
  EXPECT_NEAR (fabs (pcs->points (0, 4)), 0.19400238990783691, 1e-4);

  EXPECT_NEAR (pcs->points (2, 0), 0.98079, 1e-4);
  EXPECT_NEAR (pcs->points (2, 1), -0.04019, 1e-4);
  EXPECT_NEAR (pcs->points (2, 2), 0.19086, 1e-4);
  EXPECT_NEAR (pcs->points (2, 3), 0.27207502722740173, 1e-4);
  EXPECT_NEAR (pcs->points (2, 4), 0.1946497857570648,  1e-4);

  EXPECT_NEAR (pcs->points (indices.size () - 3, 0), 0.86725, 1e-4);
  EXPECT_NEAR (pcs->points (indices.size () - 3, 1), -0.37599, 1e-4);
  EXPECT_NEAR (pcs->points (indices.size () - 3, 2), 0.32636, 1e-4);
  EXPECT_NEAR (pcs->points (indices.size () - 3, 3), 0.2590007483959198,  1e-4);
  EXPECT_NEAR (pcs->points (indices.size () - 3, 4), 0.17906941473484039, 1e-4);

  EXPECT_NEAR (pcs->points (indices.size () - 1, 0), 0.86725, 1e-4);
  EXPECT_NEAR (pcs->points (indices.size () - 1, 1), -0.375851, 1e-3);
  EXPECT_NEAR (pcs->points (indices.size () - 1, 2), 0.32636, 1e-4);
  EXPECT_NEAR (pcs->points (indices.size () - 1, 3), 0.25900065898895264, 1e-4);
  EXPECT_NEAR (pcs->points (indices.size () - 1, 4), 0.17906941473484039, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTShapeEstimationEigen)
{
  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  EXPECT_NEAR (normals->points[103].normal_x, 0.36683175, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_y, -0.44696972, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_z, -0.81587529, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_x, -0.71414840, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_y, -0.06002361, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_z, -0.69741613, 1e-4);

  EXPECT_NEAR (normals->points[140].normal_x, -0.45109111, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_y, -0.19499126, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_z, -0.87091631, 1e-4);

  SHOTEstimation<PointXYZ, Normal, Eigen::MatrixXf> shot;
  shot.setInputNormals (normals);
  EXPECT_EQ (shot.getInputNormals (), normals);
  shot.setRadiusSearch (20 * mr);

  // Object
  PointCloud<Eigen::MatrixXf>::Ptr shots (new PointCloud<Eigen::MatrixXf>);

  // set parameters
  shot.setInputCloud (cloud.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (tree);

  // estimate
  shot.computeEigen (*shots);
  EXPECT_EQ (shots->points.rows (), indices.size ());

  EXPECT_NEAR (shots->points (103, 9 ), 0.0072018504, 1e-4);
  EXPECT_NEAR (shots->points (103, 10), 0.0023103887, 1e-4);
  EXPECT_NEAR (shots->points (103, 11), 0.0024724449, 1e-4);
  EXPECT_NEAR (shots->points (103, 19), 0.0031367359, 1e-4);
  EXPECT_NEAR (shots->points (103, 20), 0.17439659, 1e-4);
  EXPECT_NEAR (shots->points (103, 21), 0.070665278, 1e-4);
  EXPECT_NEAR (shots->points (103, 42), 0.013304681, 1e-4);
  EXPECT_NEAR (shots->points (103, 53), 0.0073520984, 1e-4);
  EXPECT_NEAR (shots->points (103, 54), 0.013584172, 1e-4);
  EXPECT_NEAR (shots->points (103, 55), 0.0050609680, 1e-4);


  // Test results when setIndices and/or setSearchSurface are used

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.points.size (); i+=3)
    test_indices->push_back (i);

  testSHOTIndicesAndSearchSurfaceEigen<SHOTEstimation<PointXYZ, Normal, Eigen::MatrixXf>, PointXYZ, Normal>
  (cloud.makeShared (), normals, test_indices);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GenericSHOTShapeEstimation)
{
  // SHOT length
  const int shapeStep_ = 20;
  //const int dim = 32*(shapeStep_+1);

  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  SHOTEstimation<PointXYZ, Normal, Eigen::MatrixXf> shot (shapeStep_);
  shot.setInputNormals (normals);
  EXPECT_EQ (shot.getInputNormals (), normals);

  shot.setRadiusSearch (20 * mr);

  PointCloud<Eigen::MatrixXf>::Ptr shots (new PointCloud<Eigen::MatrixXf>);

  // set parameters
  shot.setInputCloud (cloud.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (tree);

  // estimate
  shot.computeEigen (*shots);
  EXPECT_EQ (shots->points.rows (), indices.size ());

  EXPECT_NEAR (shots->points (103, 18), 0.0077019366, 1e-5);
  EXPECT_NEAR (shots->points (103, 19), 0.0024708188, 1e-5);
  EXPECT_NEAR (shots->points (103, 21), 0.0079652183, 1e-5);
  EXPECT_NEAR (shots->points (103, 38), 0.0067090928, 1e-5);
  EXPECT_NEAR (shots->points (103, 39), 0.17498907, 1e-5);
  EXPECT_NEAR (shots->points (103, 40), 0.078413926, 1e-5);
  EXPECT_NEAR (shots->points (103, 81), 0.014228539, 1e-5);
  EXPECT_NEAR (shots->points (103, 103), 0.022390056, 1e-5);
  EXPECT_NEAR (shots->points (103, 105), 0.0058866320, 1e-5);
  EXPECT_NEAR (shots->points (103, 123), 0.019105887, 1e-5);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (i);

  testSHOTIndicesAndSearchSurfaceEigen<SHOTEstimation<PointXYZ, Normal, Eigen::MatrixXf>, PointXYZ, Normal>
  (cloud.makeShared (), normals, test_indices, shapeStep_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTShapeAndColorEstimation)
{
  double mr = 0.002;
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  search::KdTree<PointXYZRGBA>::Ptr rgbaTree;
  rgbaTree.reset (new search::KdTree<PointXYZRGBA> (false));

  // Object
  SHOTEstimation<PointXYZRGBA, Normal, Eigen::MatrixXf> shot (true, true);
  shot.setInputNormals (normals);
  EXPECT_EQ (shot.getInputNormals (), normals);

  shot.setRadiusSearch (20 * mr);

  // Create fake point cloud with colors
  PointCloud<PointXYZRGBA> cloudWithColors;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    PointXYZRGBA p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;

    p.rgba = ( (i%255) << 16 ) + ( ( (255 - i ) %255) << 8) + ( ( i*37 ) %255);
    cloudWithColors.push_back (p);
  }

  rgbaTree->setInputCloud (cloudWithColors.makeShared ());
  PointCloud<Eigen::MatrixXf>::Ptr shots (new PointCloud<Eigen::MatrixXf>);

  shot.setInputCloud (cloudWithColors.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (rgbaTree);

  // estimate
  shot.computeEigen (*shots);
  EXPECT_EQ (shots->points.rows (), indices.size ());

  EXPECT_NEAR (shots->points (103, 10), 0.0020453099, 1e-5);
  EXPECT_NEAR (shots->points (103, 11), 0.0021887729, 1e-5);
  EXPECT_NEAR (shots->points (103, 21), 0.062557608, 1e-5);
  EXPECT_NEAR (shots->points (103, 42), 0.011778189, 1e-5);
  EXPECT_NEAR (shots->points (103, 53), 0.0065085669, 1e-5);
  EXPECT_NEAR (shots->points (103, 54), 0.012025614, 1e-5);
  EXPECT_NEAR (shots->points (103, 55), 0.0044803056, 1e-5);
  EXPECT_NEAR (shots->points (103, 64), 0.064429596, 1e-5);
  EXPECT_NEAR (shots->points (103, 65), 0.046486385, 1e-5);
  EXPECT_NEAR (shots->points (103, 86), 0.011518310, 1e-5);

  EXPECT_NEAR (shots->points (103, 357), 0.0020453099, 1e-5);
  EXPECT_NEAR (shots->points (103, 360), 0.0027993850, 1e-5);
  EXPECT_NEAR (shots->points (103, 386), 0.045115642, 1e-5);
  EXPECT_NEAR (shots->points (103, 387), 0.059068538, 1e-5);
  EXPECT_NEAR (shots->points (103, 389), 0.0047547864, 1e-5);
  EXPECT_NEAR (shots->points (103, 453), 0.0051176427, 1e-5);
  EXPECT_NEAR (shots->points (103, 481), 0.0053625242, 1e-5);
  EXPECT_NEAR (shots->points (103, 482), 0.012025614, 1e-5);
  EXPECT_NEAR (shots->points (103, 511), 0.0057367259, 1e-5);
  EXPECT_NEAR (shots->points (103, 512), 0.048357654, 1e-5);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (i);

  testSHOTIndicesAndSearchSurfaceEigen<SHOTEstimation<PointXYZRGBA, Normal, Eigen::MatrixXf>, PointXYZRGBA, Normal>
  (cloudWithColors.makeShared (), normals, test_indices);
}

/*
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTShapeEstimationOpenMP)
{
  // Estimate normals first
  double mr = 0.002;
  NormalEstimationOMP<PointXYZ, Normal> n (omp_get_max_threads ());
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  SHOTEstimationOMP<PointXYZ, Normal, SHOT> shot;
  shot.setInputNormals (normals);
  EXPECT_EQ (shot.getInputNormals (), normals);

  shot.setRadiusSearch ( 20 * mr);

  // Object
  PointCloud<SHOT>::Ptr shots (new PointCloud<SHOT>);

  // set parameters
  shot.setInputCloud (cloud.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (tree);

  // estimate
  shot.computeEigen (*shots);
  EXPECT_EQ (shots->points.size (), indices.size ());

  EXPECT_NEAR (shots->points[103].descriptor[9 ], 0.0072018504, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[10], 0.0023103887, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[11], 0.0024724449, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[19], 0.0031367359, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[20], 0.17439659, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[21], 0.070665278, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[42], 0.013304681, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[53], 0.0073520984, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[54], 0.013584172, 1e-4);
  EXPECT_NEAR (shots->points[103].descriptor[55], 0.0050609680, 1e-4);

   // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (i);

  testSHOTIndicesAndSearchSurfaceEigen<SHOTEstimationOMP<PointXYZ, Normal, Eigen::MatrixXf>, PointXYZ, Normal>
    (cloud.makeShared (), normals, test_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL,SHOTShapeAndColorEstimationOpenMP)
{
  double mr = 0.002;
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  search::KdTree<PointXYZRGBA>::Ptr rgbaTree;

  rgbaTree.reset (new search::KdTree<PointXYZRGBA> (false));

  // Object
  SHOTEstimationOMP<PointXYZRGBA, Normal, SHOT> shot (true, true, -1);
  shot.setInputNormals (normals);

  EXPECT_EQ (shot.getInputNormals (), normals);

  shot.setRadiusSearch ( 20 * mr);

  // Create fake point cloud with colors
  PointCloud<PointXYZRGBA> cloudWithColors;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    PointXYZRGBA p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;

    p.rgba = ( (i%255) << 16 ) + ( ( (255 - i ) %255) << 8) + ( ( i*37 ) %255);
    cloudWithColors.push_back(p);
  }

  rgbaTree->setInputCloud (cloudWithColors.makeShared ());

  PointCloud<SHOT>::Ptr shots (new PointCloud<SHOT> ());

  shot.setInputCloud (cloudWithColors.makeShared ());
  shot.setIndices (indicesptr);
  shot.setSearchMethod (rgbaTree);

  // estimate
  shot.computeEigen (*shots);
  EXPECT_EQ (shots->points.size (), indices.size ());

  EXPECT_NEAR (shots->points[103].descriptor[10], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[11], 0.0021887729, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[21], 0.062557608, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[42], 0.011778189, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[53], 0.0065085669, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[54], 0.012025614, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[55], 0.0044803056, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[64], 0.064429596, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[65], 0.046486385, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[86], 0.011518310, 1e-5);

  EXPECT_NEAR (shots->points[103].descriptor[357], 0.0020453099, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[360], 0.0027993850, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[386], 0.045115642, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[387], 0.059068538, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[389], 0.0047547864, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[453], 0.0051176427, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[481], 0.0053625242, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[482], 0.012025614, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[511], 0.0057367259, 1e-5);
  EXPECT_NEAR (shots->points[103].descriptor[512], 0.048357654, 1e-5);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (i);

  testSHOTIndicesAndSearchSurfaceEigen<SHOTEstimationOMP<PointXYZRGBA, Normal, Eigen::MatrixXf>, PointXYZRGBA, Normal>
    (cloudWithColors.makeShared (), normals, test_indices);
}
 */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, 3DSCEstimationEigen)
{
  float meshRes = 0.002;
  size_t nBinsL = 4;
  size_t nBinsK = 4;
  size_t nBinsJ = 4;
  float radius = 20.0 * meshRes;
  float rmin = radius / 10.0;
  float ptDensityRad = radius / 5.0;

  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> ne;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  ne.setInputCloud (cloudptr);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (radius);
  // estimate
  ne.compute (*normals);
  ShapeContext3DEstimation<PointXYZ, Normal, Eigen::MatrixXf> sc3d;
  sc3d.setInputCloud (cloudptr);
  sc3d.setInputNormals (normals);
  sc3d.setSearchMethod (tree);
  sc3d.setRadiusSearch (radius);
  sc3d.setAzimuthBins (nBinsL);
  sc3d.setElevationBins (nBinsK);
  sc3d.setRadiusBins (nBinsJ);
  sc3d.setMinimalRadius (rmin);
  sc3d.setPointDensityRadius (ptDensityRad);
  // Compute the features
  PointCloud<Eigen::MatrixXf>::Ptr sc3ds (new PointCloud<Eigen::MatrixXf>);
  sc3d.computeEigen (*sc3ds);
  EXPECT_EQ (sc3ds->points.rows (), cloud.size ());

  // 3DSC does not define a repeatable local RF, we set it to zero to signal it to the user
  //EXPECT_NEAR ((*sc3ds)[0].rf[0], 0.2902f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[1], 0.7334f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[2], -0.6146f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[3], 0.9486f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[4], -0.3051f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[5], 0.0838f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[6], -0.1261f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[7], -0.6074f, 1e-4f);
  //EXPECT_NEAR ((*sc3ds)[0].rf[8], -0.7843f, 1e-4f);

  EXPECT_NEAR (sc3ds->points (0, 0), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 1), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 2), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 3), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 4), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 5), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 6), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 7), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 8), 0.0f, 1e-4f);

  EXPECT_EQ   (sc3ds->points.row (0).size (), 64 + 9);
  EXPECT_NEAR (sc3ds->points (0, 9 + 4), 52.2474f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 9 + 6), 150.901611328125, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 9 + 7), 169.09703063964844, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 9 + 8), 0, 1e-4f);
  EXPECT_NEAR (sc3ds->points (0, 9 + 21), 39.1745f, 1e-4f);

  EXPECT_NEAR (sc3ds->points (2, 9 + 4), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (2, 9 + 6), 73.7986f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (2, 9 + 7), 209.97763061523438, 1e-4f);

  EXPECT_NEAR (sc3ds->points (2, 9 + 9), 68.5553f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (2, 9 + 16), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (2, 9 + 17), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (2, 9 + 18), 0.0f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (2, 9 + 20), 0.0f, 1e-4f);

  EXPECT_NEAR (sc3ds->points (2, 9 + 21), 39.1745f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (2, 9 + 22), 154.2060f, 1e-4f);
  EXPECT_NEAR (sc3ds->points (2, 9 + 23), 275.63433837890625, 1e-4f);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i++)
    test_indices->push_back (i);

  testSHOTIndicesAndSearchSurfaceEigen<ShapeContext3DEstimation<PointXYZ, Normal, Eigen::MatrixXf>, PointXYZ, Normal>
  (cloudptr, normals, test_indices);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, USCEstimation)
{
  float meshRes = 0.002;
  size_t nBinsL = 4;
  size_t nBinsK = 4;
  size_t nBinsJ = 4;
  float radius = 20.0 * meshRes;
  float rmin = radius / 10.0;
  float ptDensityRad = radius / 5.0;

  // estimate
  UniqueShapeContext<PointXYZ, Eigen::MatrixXf> uscd;
  uscd.setInputCloud (cloud.makeShared ());
  uscd.setSearchMethod (tree);
  uscd.setRadiusSearch (radius);
  uscd.setAzimuthBins (nBinsL);
  uscd.setElevationBins (nBinsK);
  uscd.setRadiusBins (nBinsJ);
  uscd.setMinimalRadius (rmin);
  uscd.setPointDensityRadius (ptDensityRad);
  uscd.setLocalRadius (radius);
  // Compute the features
  PointCloud<Eigen::MatrixXf>::Ptr uscds (new PointCloud<Eigen::MatrixXf>);
  uscd.computeEigen (*uscds);
  EXPECT_EQ (uscds->points.rows (), cloud.size ());

  EXPECT_NEAR (uscds->points (0, 0), 0.9876f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 1), -0.1408f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 2), -0.06949f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 3), -0.06984f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 4), -0.7904f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 5), 0.6086f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 6), -0.1406f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 7), -0.5962f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 8), -0.7904f, 1e-4f);

  EXPECT_EQ   (uscds->points.row (0).size (), 9+64);
  EXPECT_NEAR (uscds->points (0, 9 + 4), 52.2474f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 9 + 5), 39.1745f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 9 + 6), 176.2354f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 9 + 7), 199.4478f, 1e-4f);
  EXPECT_NEAR (uscds->points (0, 9 + 8), 0.0f, 1e-4f);

  EXPECT_NEAR (uscds->points (2, 9 + 6), 110.1472f, 1e-4f);
  EXPECT_NEAR (uscds->points (2, 9 + 7), 145.5597f, 1e-4f);
  EXPECT_NEAR (uscds->points (2, 9 + 8), 69.6632f, 1e-4f);
  EXPECT_NEAR (uscds->points (2, 9 + 22), 57.2765f, 1e-4f);
  EXPECT_NEAR (uscds->points (2, 9 + 23), 172.8134f, 1e-4f);
  EXPECT_NEAR (uscds->points (2, 9 + 25), 68.5554f, 1e-4f);
  EXPECT_NEAR (uscds->points (2, 9 + 26), 0.0f, 1e-4f);
  EXPECT_NEAR (uscds->points (2, 9 + 27), 0.0f, 1e-4f);
  EXPECT_NEAR (uscds->points (2, 9 + 37), 39.1745f, 1e-4f);
  EXPECT_NEAR (uscds->points (2, 9 + 38), 71.5957f, 1e-4f);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (i);

  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  testSHOTIndicesAndSearchSurfaceEigen<UniqueShapeContext<PointXYZ, Eigen::MatrixXf>, PointXYZ, Normal>
  (cloud.makeShared (), normals, test_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PFHEstimationEigen)
{
  float f1, f2, f3, f4;

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);

  PFHEstimation<PointXYZ, Normal, Eigen::MatrixXf> pfh;
  pfh.setInputNormals (normals);
  EXPECT_EQ (pfh.getInputNormals (), normals);

  // computePairFeatures
  pfh.computePairFeatures (cloud, *normals, 0, 12, f1, f2, f3, f4);
  EXPECT_NEAR (f1, -0.072575, 1e-4);
  EXPECT_NEAR (f2, -0.040221, 1e-4);
  EXPECT_NEAR (f3, 0.068133, 1e-4);
  EXPECT_NEAR (f4, 0.006130, 1e-4);

  // computePointPFHSignature
  int nr_subdiv = 3;
  Eigen::VectorXf pfh_histogram (nr_subdiv * nr_subdiv * nr_subdiv);
  pfh.computePointPFHSignature (cloud, *normals, indices, nr_subdiv, pfh_histogram);
  EXPECT_NEAR (pfh_histogram[0],  0.932506, 1e-2);
  EXPECT_NEAR (pfh_histogram[1],  2.32429 , 1e-2);
  EXPECT_NEAR (pfh_histogram[2],  0.357477, 1e-2);
  EXPECT_NEAR (pfh_histogram[3],  0.848541, 1e-2);
  EXPECT_NEAR (pfh_histogram[4],  3.65565 , 2e-2); // larger error w.r.t. considering all point pairs (feature bins=0,1,1 where 1 is middle, so angle of 0)
  EXPECT_NEAR (pfh_histogram[5],  0.178104, 1e-2);
  EXPECT_NEAR (pfh_histogram[6],  1.45284 , 1e-2);
  EXPECT_NEAR (pfh_histogram[7],  3.60666 , 1e-2);
  EXPECT_NEAR (pfh_histogram[8],  0.298959, 1e-2);
  EXPECT_NEAR (pfh_histogram[9],  0.295143, 1e-2);
  EXPECT_NEAR (pfh_histogram[10], 2.13474 , 1e-2);
  EXPECT_NEAR (pfh_histogram[11], 0.41218 , 1e-2);
  EXPECT_NEAR (pfh_histogram[12], 0.165382, 1e-2);
  EXPECT_NEAR (pfh_histogram[13], 8.97407 , 1e-2);
  EXPECT_NEAR (pfh_histogram[14], 0.306592, 1e-2);
  EXPECT_NEAR (pfh_histogram[15], 0.455432, 1e-2);
  EXPECT_NEAR (pfh_histogram[16], 4.5977 ,  1e-2);
  EXPECT_NEAR (pfh_histogram[17], 0.393097, 1e-2);
  EXPECT_NEAR (pfh_histogram[18], 7.54668 , 1e-2);
  EXPECT_NEAR (pfh_histogram[19], 6.78336 , 1e-2);
  EXPECT_NEAR (pfh_histogram[20], 1.63858 , 1e-2);
  EXPECT_NEAR (pfh_histogram[21], 9.93842 , 1e-2);
  EXPECT_NEAR (pfh_histogram[22], 18.4947 , 2e-2); // larger error w.r.t. considering all point pairs (feature bins=2,1,1 where 1 is middle, so angle of 0)
  EXPECT_NEAR (pfh_histogram[23], 1.96553 , 1e-4);
  EXPECT_NEAR (pfh_histogram[24], 8.04793 , 1e-4);
  EXPECT_NEAR (pfh_histogram[25], 11.2793  , 1e-4);
  EXPECT_NEAR (pfh_histogram[26], 2.91714 , 1e-4);

  // Sum of values should be 100
  EXPECT_NEAR (pfh_histogram.sum (), 100.0, 1e-2);

  // Object
  PointCloud<Eigen::MatrixXf>::Ptr pfhs (new PointCloud<Eigen::MatrixXf>);

  // set parameters
  pfh.setInputCloud (cloud.makeShared ());
  pfh.setIndices (indicesptr);
  pfh.setSearchMethod (tree);
  pfh.setKSearch (indices.size ());

  // estimate
  pfh.computeEigen (*pfhs);
  EXPECT_EQ (pfhs->points.rows (), indices.size ());

  for (int i = 0; i < pfhs->points.rows (); ++i)
  {
    EXPECT_NEAR (pfhs->points (i, 0),  0.156477  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 1),  0.539396  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 2),  0.410907  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 3),  0.184465  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 4),  0.115767  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 5),  0.0572475 , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 6),  0.206092  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 7),  0.339667  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 8),  0.265883  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 9),  0.0038165 , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 10), 0.103046  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 11), 0.214997  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 12), 0.398186  , 3e-2); // larger error w.r.t. considering all point pairs (feature bins=0,2,2 where 2 is middle, so angle of 0)
    EXPECT_NEAR (pfhs->points (i, 13), 0.298959  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 14), 0.00127217, 1e-4);
    EXPECT_NEAR (pfhs->points (i, 15), 0.11704   , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 16), 0.255706  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 17), 0.356205  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 18), 0.265883  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 19), 0.00127217, 1e-4);
    EXPECT_NEAR (pfhs->points (i, 20), 0.148844  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 21), 0.721316  , 1e-2);
    EXPECT_NEAR (pfhs->points (i, 22), 0.438899  , 1e-2);
    EXPECT_NEAR (pfhs->points (i, 23), 0.22263   , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 24), 0.0216269 , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 25), 0.223902  , 1e-4);
    EXPECT_NEAR (pfhs->points (i, 26), 0.07633   , 1e-4);
  }


  // Test results when setIndices and/or setSearchSurface are used

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (i);

  testIndicesAndSearchSurfaceEigen<PFHEstimation<PointXYZ, Normal, Eigen::MatrixXf>, PointXYZ, Normal>
  (cloud.makeShared (), normals, test_indices, 125);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FPFHEstimationEigen)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);

  FPFHEstimation<PointXYZ, Normal, Eigen::MatrixXf> fpfh;
  fpfh.setInputNormals (normals);
  EXPECT_EQ (fpfh.getInputNormals (), normals);

  // computePointSPFHSignature
  int nr_subdiv = 11; // use the same number of bins for all three angular features
  Eigen::MatrixXf hist_f1 (indices.size (), nr_subdiv), hist_f2 (indices.size (), nr_subdiv), hist_f3 (indices.size (), nr_subdiv);
  hist_f1.setZero (); hist_f2.setZero (); hist_f3.setZero ();
  for (size_t i = 0; i < indices.size (); ++i)
    fpfh.computePointSPFHSignature (cloud, *normals, i, i, indices, hist_f1, hist_f2, hist_f3);

  EXPECT_NEAR (hist_f1 (0, 0), 0.757576, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 1), 0.757576, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 2), 4.54545,  1e-4);
  EXPECT_NEAR (hist_f1 (0, 3), 19.697,   1e-4);
  EXPECT_NEAR (hist_f1 (0, 4), 40.6566,  1e-4);
  EXPECT_NEAR (hist_f1 (0, 5), 21.4647,  1e-4);
  EXPECT_NEAR (hist_f1 (0, 6), 7.575759, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 7), 0.000000, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 8), 0.000000, 1e-4);
  EXPECT_NEAR (hist_f1 (0, 9), 0.50505,  1e-4);
  EXPECT_NEAR (hist_f1 (0, 10), 4.0404,  1e-4);

  EXPECT_NEAR (hist_f2 (0, 0), 0.757576, 1e-4);
  EXPECT_NEAR (hist_f2 (0, 1), 1.51515,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 2), 6.31313,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 3), 9.59596,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 4), 20.7071,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 5), 18.9394,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 6), 15.9091,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 7), 12.8788,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 8), 6.56566,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 9), 4.29293,  1e-4);
  EXPECT_NEAR (hist_f2 (0, 10), 2.52525, 1e-4);

  EXPECT_NEAR (hist_f3 (0, 0), 0.000000, 1e-4);
  EXPECT_NEAR (hist_f3 (0, 1), 5.05051,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 2), 4.54545,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 3), 5.05051,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 4), 1.76768,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 5), 3.0303,   1e-4);
  EXPECT_NEAR (hist_f3 (0, 6), 9.09091,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 7), 31.8182,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 8), 22.2222,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 9), 11.8687,  1e-4);
  EXPECT_NEAR (hist_f3 (0, 10), 5.55556, 1e-4);

  // weightPointSPFHSignature
  Eigen::VectorXf fpfh_histogram (nr_subdiv + nr_subdiv + nr_subdiv);
  fpfh_histogram.setZero ();
  vector<float> dists (indices.size ());
  for (size_t i = 0; i < dists.size (); ++i) dists[i] = i;
  fpfh.weightPointSPFHSignature (hist_f1, hist_f2, hist_f3, indices, dists, fpfh_histogram);

  EXPECT_NEAR (fpfh_histogram[0],  1.9798 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[1],  2.86927,  1e-2);
  EXPECT_NEAR (fpfh_histogram[2],  8.47911,  1e-2);
  EXPECT_NEAR (fpfh_histogram[3],  22.8784,  1e-2);
  EXPECT_NEAR (fpfh_histogram[4],  29.8597,  1e-2);
  EXPECT_NEAR (fpfh_histogram[5],  19.6877,  1e-2);
  EXPECT_NEAR (fpfh_histogram[6],  7.38611,  1e-2);
  EXPECT_NEAR (fpfh_histogram[7],  1.44265,  1e-2);
  EXPECT_NEAR (fpfh_histogram[8],  0.69677,  1e-2);
  EXPECT_NEAR (fpfh_histogram[9],  1.72609,  1e-2);
  EXPECT_NEAR (fpfh_histogram[10], 2.99435,  1e-2);
  EXPECT_NEAR (fpfh_histogram[11], 2.26313,  1e-2);
  EXPECT_NEAR (fpfh_histogram[12], 5.16573,  1e-2);
  EXPECT_NEAR (fpfh_histogram[13], 8.3263 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[14], 9.92427,  1e-2);
  EXPECT_NEAR (fpfh_histogram[15], 16.8062,  1e-2);
  EXPECT_NEAR (fpfh_histogram[16], 16.2767,  1e-2);
  EXPECT_NEAR (fpfh_histogram[17], 12.251 ,  1e-2);
  //EXPECT_NEAR (fpfh_histogram[18], 10.354,  1e-2);
  //EXPECT_NEAR (fpfh_histogram[19], 6.65578,  1e-2);
  EXPECT_NEAR (fpfh_histogram[20], 6.1437 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[21], 5.83341,  1e-2);
  EXPECT_NEAR (fpfh_histogram[22], 1.08809,  1e-2);
  EXPECT_NEAR (fpfh_histogram[23], 3.34133,  1e-2);
  EXPECT_NEAR (fpfh_histogram[24], 5.59236,  1e-2);
  EXPECT_NEAR (fpfh_histogram[25], 5.6355 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[26], 3.03257,  1e-2);
  EXPECT_NEAR (fpfh_histogram[27], 1.37437,  1e-2);
  EXPECT_NEAR (fpfh_histogram[28], 7.99746,  1e-2);
  EXPECT_NEAR (fpfh_histogram[29], 18.0343,  1e-2);
  EXPECT_NEAR (fpfh_histogram[30], 23.691 ,  1e-2);
  EXPECT_NEAR (fpfh_histogram[31], 19.8475,  1e-2);
  EXPECT_NEAR (fpfh_histogram[32], 10.3655,  1e-2);

  // Object
  PointCloud<Eigen::MatrixXf>::Ptr fpfhs (new PointCloud<Eigen::MatrixXf>);

  // set parameters
  fpfh.setInputCloud (cloud.makeShared ());
  fpfh.setNrSubdivisions (11, 11, 11);
  fpfh.setIndices (indicesptr);
  fpfh.setSearchMethod (tree);
  fpfh.setKSearch (indices.size ());

  // estimate
  fpfh.computeEigen (*fpfhs);
  EXPECT_EQ (fpfhs->points.rows (), indices.size ());

  EXPECT_NEAR (fpfhs->points (0, 0),  1.58591, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 1),  1.68365, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 2),  6.71   , 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 3),  23.0717, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 4),  33.3844, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 5),  20.4002, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 6),  7.31067, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 7),  1.02635, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 8),  0.48591, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 9),  1.47069, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 10), 2.87061, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 11), 1.78321, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 12), 4.30795, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 13), 7.05514, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 14), 9.37615, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 15), 17.963 , 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 16), 18.2801, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 17), 14.2766, 1e-2);
  //EXPECT_NEAR (fpfhs->points (0, 18), 10.8542, 1e-2);
  //EXPECT_NEAR (fpfhs->points (0, 19), 6.07925, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 20), 5.28565, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 21), 4.73887, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 22), 0.56984, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 23), 3.29826, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 24), 5.28156, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 25), 5.26939, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 26), 3.13191, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 27), 1.74453, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 28), 9.41971, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 29), 21.5894, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 30), 24.6302, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 31), 17.7764, 1e-2);
  EXPECT_NEAR (fpfhs->points (0, 32), 7.28878, 1e-2);

  // Test results when setIndices and/or setSearchSurface are used
  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (i);

  testIndicesAndSearchSurfaceEigen <FPFHEstimation<PointXYZ, Normal, Eigen::MatrixXf>, PointXYZ, Normal>
  (cloud.makeShared (), normals, test_indices, 33);
}
/*
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FPFHEstimationOpenMP)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);
  FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh (4); // instantiate 4 threads
  fpfh.setInputNormals (normals);

  // Object
  PointCloud<FPFHSignature33>::Ptr fpfhs (new PointCloud<FPFHSignature33> ());

  // set parameters
  fpfh.setInputCloud (cloud.makeShared ());
  fpfh.setNrSubdivisions (11, 11, 11);
  fpfh.setIndices (indicesptr);
  fpfh.setSearchMethod (tree);
  fpfh.setKSearch (indices.size ());

  // estimate
  fpfh.compute (*fpfhs);
  EXPECT_EQ (fpfhs->points.size (), indices.size ());

  EXPECT_NEAR (fpfhs->points[0].histogram[0], 2.11328, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[1], 3.13866, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[2], 7.07176, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[3], 23.0986, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[4], 32.988, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[5], 18.74372, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[6], 8.118416, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[7], 1.9162, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[8], 1.19554, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[9], 0.577558, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[10], 1.03827, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[11], 0.631236, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[12], 2.13356, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[13], 5.67842, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[14], 10.8759, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[15], 20.2439, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[16], 19.674, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[17], 15.3302, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[18], 10.773, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[19], 6.80136, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[20], 4.03065, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[21], 3.82776, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[22], 0.208905, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[23], 0.392544, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[24], 1.27637, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[25], 2.61976, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[26], 5.12960, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[27], 12.35568, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[28], 21.89877, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[29], 25.55738, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[30], 19.1552, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[31], 9.22763, 1e-4);
  EXPECT_NEAR (fpfhs->points[0].histogram[32], 2.17815, 1e-4);



  // Test results when setIndices and/or setSearchSurface are used

  boost::shared_ptr<vector<int> > test_indices (new vector<int> (0));
  for (size_t i = 0; i < cloud.size (); i+=3)
    test_indices->push_back (i);

  testIndicesAndSearchSurface<FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33>, PointXYZ, Normal, FPFHSignature33>
    (cloud.makeShared (), normals, test_indices, 33);
}
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PPFEstimation)
{
  // Estimate normals
  NormalEstimation<PointXYZ, Normal> normal_estimation;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  normal_estimation.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  normal_estimation.setIndices (indicesptr);
  normal_estimation.setSearchMethod (tree);
  normal_estimation.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  normal_estimation.compute (*normals);

  PPFEstimation <PointXYZ, Normal, PPFSignature> ppf_estimation;
  ppf_estimation.setInputCloud (cloud.makeShared ());
  ppf_estimation.setInputNormals (normals);
  PointCloud<PPFSignature>::Ptr feature_cloud (new PointCloud<PPFSignature> ());
  ppf_estimation.compute (*feature_cloud);

  // Check for size of output
  EXPECT_EQ (feature_cloud->points.size (), indices.size () * cloud.points.size ());

  // Now check for a few values in the feature cloud
  EXPECT_EQ (pcl_isnan (feature_cloud->points[0].f1), true);
  EXPECT_EQ (pcl_isnan (feature_cloud->points[0].f2), true);
  EXPECT_EQ (pcl_isnan (feature_cloud->points[0].f3), true);
  EXPECT_EQ (pcl_isnan (feature_cloud->points[0].f4), true);
  EXPECT_TRUE (pcl_isnan (feature_cloud->points[0].alpha_m));
  EXPECT_NEAR (feature_cloud->points[15127].f1, -2.51637, 1e-4);
  EXPECT_NEAR (feature_cloud->points[15127].f2, -0.00365916, 1e-4);
  EXPECT_NEAR (feature_cloud->points[15127].f3, -0.521141, 1e-4);
  EXPECT_NEAR (feature_cloud->points[15127].f4, 0.0106809, 1e-4);
  EXPECT_NEAR (feature_cloud->points[15127].alpha_m, -0.255664, 1e-4);
  EXPECT_NEAR (feature_cloud->points[30254].f1, 0.185142, 1e-4);
  EXPECT_NEAR (feature_cloud->points[30254].f2, 0.0425001, 1e-4);
  EXPECT_NEAR (feature_cloud->points[30254].f3, -0.191276, 1e-4);
  EXPECT_NEAR (feature_cloud->points[30254].f4, 0.0138508, 1e-4);
  EXPECT_NEAR (feature_cloud->points[30254].alpha_m, 2.42955, 1e-4);
  EXPECT_NEAR (feature_cloud->points[45381].f1, -1.96263, 1e-4);
  EXPECT_NEAR (feature_cloud->points[45381].f2, -0.431919, 1e-4);
  EXPECT_NEAR (feature_cloud->points[45381].f3, 0.868716, 1e-4);
  EXPECT_NEAR (feature_cloud->points[45381].f4, 0.140129, 1e-4);
  EXPECT_NEAR (feature_cloud->points[45381].alpha_m, -1.97276, 1e-4);
}
/*
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, VFHEstimation)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);
  VFHEstimation<PointXYZ, Normal, VFHSignature308> vfh;
  vfh.setInputNormals (normals);

  //  PointCloud<PointNormal> cloud_normals;
  //  concatenateFields (cloud, normals, cloud_normals);
  //  savePCDFile ("bun0_n.pcd", cloud_normals);

  // Object
  PointCloud<VFHSignature308>::Ptr vfhs (new PointCloud<VFHSignature308> ());

  // set parameters
  vfh.setInputCloud (cloud.makeShared ());
  vfh.setIndices (indicesptr);
  vfh.setSearchMethod (tree);

  // estimate
  vfh.compute (*vfhs);
  EXPECT_EQ ((int)vfhs->points.size (), 1);

  //for (size_t d = 0; d < 308; ++d)
  //  std::cerr << vfhs.points[0].histogram[d] << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GFPFH)
{
  PointCloud<PointXYZL>::Ptr cloud (new PointCloud<PointXYZL>());

  const unsigned num_classes = 3;

  // Build a cubic shape with a hole and changing labels.
  for (int z = -10; z < 10; ++z)
  for (int y = -10; y < 10; ++y)
  for (int x = -10; x < 10; ++x)
  {
    if (x >= -9 && x < 9 && y >= -9 && y < 9 && z >= -9 && z < 9)
      continue;
    unsigned label = 1 + (std::abs (x+y+z) % num_classes);
    PointXYZL p;
    p.label = label;
    p.x = x;
    p.y = y;
    p.z = z;
    cloud->points.push_back (p);
  }
  cloud->width = cloud->points.size ();
  cloud->height = 1;

  GFPFHEstimation<PointXYZL, PointXYZL, GFPFHSignature16> gfpfh;
  gfpfh.setNumberOfClasses (num_classes);
  gfpfh.setOctreeLeafSize (2);
  gfpfh.setInputCloud (cloud);
  gfpfh.setInputLabels (cloud);
  PointCloud<GFPFHSignature16> descriptor;
  gfpfh.compute (descriptor);

  const float ref_values[] = { 3216, 7760, 8740, 26584, 4645, 2995, 3029, 4349, 6192, 5440, 9514, 47563, 21814, 22073, 5734, 1253 };

  EXPECT_EQ (descriptor.points.size (), 1);
  for (size_t i = 0; i < (size_t) descriptor.points[0].descriptorSize (); ++i)
  {
    EXPECT_EQ (descriptor.points[0].histogram[i], ref_values[i]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, RSDEstimation)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);
  RSDEstimation<PointXYZ, Normal, PrincipalRadiiRSD> rsd;
  rsd.setInputNormals (normals);

  // Object
  PointCloud<PrincipalRadiiRSD>::Ptr rsds (new PointCloud<PrincipalRadiiRSD> ());

  // set parameters
  rsd.setInputCloud (cloud.makeShared ());
  rsd.setIndices (indicesptr);
  rsd.setSearchMethod (tree);
  rsd.setRadiusSearch (0.015);

  // estimate
  rsd.computeEigen (*rsds);
  //  EXPECT_NEAR (rsds->points[0].r_min, 0.04599, 0.005);
  //  EXPECT_NEAR (rsds->points[0].r_max, 0.07053, 0.005);

  // Save output
  //PointCloud<PointNormal> normal_cloud;
  //concatenateFields (cloud, *normals, normal_cloud);
  //savePCDFile ("./test/bun0-normal.pcd", normal_cloud);
  //savePCDFile ("./test/bun0-rsd.pcd", *rsds);
}
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IntensityGradientEstimation)
{
  // Create a test cloud
  PointCloud<PointXYZI> cloud_xyzi;
  cloud_xyzi.height = 1;
  cloud_xyzi.is_dense = true;
  for (float x = -5.0; x <= 5.0; x += 0.1)
  {
    for (float y = -5.0; y <= 5.0; y += 0.1)
    {
      PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = 0.1 * pow (x, 2) + 0.5 * y + 1.0;
      p.intensity = 0.1 * pow (x, 3) + 0.2 * pow (y, 2) + 1.0 * p.z + 20000.0;

      cloud_xyzi.points.push_back (p);
    }
  }
  cloud_xyzi.width = cloud_xyzi.points.size ();
  PointCloud<PointXYZI>::ConstPtr cloud_ptr = cloud_xyzi.makeShared ();

  // Estimate surface normals
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  NormalEstimation<PointXYZI, Normal> norm_est;
  norm_est.setInputCloud (cloud_ptr);
  search::KdTree<PointXYZI>::Ptr treept1 (new search::KdTree<PointXYZI> (false));
  norm_est.setSearchMethod (treept1);
  norm_est.setRadiusSearch (0.25);
  norm_est.compute (*normals);

  // Estimate intensity gradient
  PointCloud<Eigen::MatrixXf> gradient;
  IntensityGradientEstimation<PointXYZI, Normal, Eigen::MatrixXf> grad_est;
  grad_est.setInputCloud (cloud_ptr);
  grad_est.setInputNormals (normals);
  search::KdTree<PointXYZI>::Ptr treept2 (new search::KdTree<PointXYZI> (false));
  grad_est.setSearchMethod (treept2);
  grad_est.setRadiusSearch (0.25);
  grad_est.computeEigen (gradient);

  // Compare to gradient estimates to actual values
  for (size_t i = 0; i < cloud_ptr->points.size (); ++i)
  {
    const PointXYZI &p = cloud_ptr->points[i];

    // Compute the surface normal analytically.
    float nx = -0.2 * p.x;
    float ny = -0.5;
    float nz = 1.0;
    float magnitude = sqrt (nx * nx + ny * ny + nz * nz);
    nx /= magnitude;
    ny /= magnitude;
    nz /= magnitude;

    // Compute the intensity gradient analytically...
    float tmpx = (0.3 * pow (p.x, 2));
    float tmpy = (0.4 * p.y);
    float tmpz = (1.0);
    // ...and project the 3-D gradient vector onto the surface's tangent plane.
    float gx = (1 - nx * nx) * tmpx + (-nx * ny) * tmpy + (-nx * nz) * tmpz;
    float gy = (-ny * nx) * tmpx + (1 - ny * ny) * tmpy + (-ny * nz) * tmpz;
    float gz = (-nz * nx) * tmpx + (-nz * ny) * tmpy + (1 - nz * nz) * tmpz;

    // Compare the estimates to the derived values.
    const float tolerance = 0.11;
    ASSERT_NEAR (gradient.points (i, 0), gx, tolerance);
    ASSERT_NEAR (gradient.points (i, 1), gy, tolerance);
    ASSERT_NEAR (gradient.points (i, 2), gz, tolerance);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SpinImageEstimationEigen)
{
  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  EXPECT_NEAR (normals->points[103].normal_x, 0.36683175, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_y, -0.44696972, 1e-4);
  EXPECT_NEAR (normals->points[103].normal_z, -0.81587529, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_x, -0.71414840, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_y, -0.06002361, 1e-4);
  EXPECT_NEAR (normals->points[200].normal_z, -0.69741613, 1e-4);

  EXPECT_NEAR (normals->points[140].normal_x, -0.45109111, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_y, -0.19499126, 1e-4);
  EXPECT_NEAR (normals->points[140].normal_z, -0.87091631, 1e-4);

  SpinImageEstimation<PointXYZ, Normal, Eigen::MatrixXf> spin_est (8, 0.5, 16);
  // set parameters
  //spin_est.setInputWithNormals (cloud.makeShared (), normals);
  spin_est.setInputCloud (cloud.makeShared ());
  spin_est.setInputNormals (normals);
  spin_est.setIndices (indicesptr);
  spin_est.setSearchMethod (tree);
  spin_est.setRadiusSearch (40*mr);

  // Object
  PointCloud<Eigen::MatrixXf>::Ptr spin_images (new PointCloud<Eigen::MatrixXf>);

  // radial SI
  spin_est.setRadialStructure ();

  // estimate
  spin_est.computeEigen (*spin_images);
  EXPECT_EQ (spin_images->points.rows (), indices.size ());

  EXPECT_NEAR (spin_images->points (100, 0), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 12), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 24), 0.00233226, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 36), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 48), 8.48662e-005, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 60), 0.0266387, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 72), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 84), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 96), 0.0414662, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 108), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 120), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 132), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 144), 0.0128513, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 0), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 12), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 24), 0.00932424, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 36), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 48), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 60), 0.0145733, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 72), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 84), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 96), 0.00034457, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 108), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 120), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 132), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 144), 0.0121195, 1e-5);


  // radial SI, angular spin-images
  spin_est.setAngularDomain ();

  // estimate
  spin_est.computeEigen (*spin_images);
  EXPECT_EQ (spin_images->points.rows (), indices.size ());

  EXPECT_NEAR (spin_images->points (100, 0), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 12), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 24), 0.13213, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 36), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 48), 0.908804, 1.1e-4);
  EXPECT_NEAR (spin_images->points (100, 60), 0.63875, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 72), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 84), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 96), 0.550392, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 108), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 120), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 132), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 144), 0.25713, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 0), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 12), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 24), 0.230605, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 36), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 48), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 60), 0.764872, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 72), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 84), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 96), 1.02824, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 108), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 120), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 132), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 144), 0.293567, 1e-4);


  // rectangular SI
  spin_est.setRadialStructure (false);
  spin_est.setAngularDomain (false);

  // estimate
  spin_est.computeEigen (*spin_images);
  EXPECT_EQ (spin_images->points.rows (), indices.size ());

  EXPECT_NEAR (spin_images->points (100, 0), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 12), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 24), 0.000889345, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 36), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 48), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 60), 0.0489534, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 72), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 84), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 96), 0.0747141, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 108), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 120), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 132), 0.0173423, 1e-5);
  EXPECT_NEAR (spin_images->points (100, 144), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 0), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 12), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 24), 0.0267132, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 36), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 48), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 60), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 72), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 84), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 96), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 108), 0.0209709, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 120), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 132), 0, 1e-5);
  EXPECT_NEAR (spin_images->points (300, 144), 0.029372, 1e-5);

  // rectangular SI, angular spin-images
  spin_est.setAngularDomain ();

  // estimate
  spin_est.computeEigen (*spin_images);
  EXPECT_EQ (spin_images->points.rows (), indices.size ());

  EXPECT_NEAR (spin_images->points (100, 0), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 12), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 24), 0.132126, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 36), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 48), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 60), 0.388011, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 72), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 84), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 96), 0.468881, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 108), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 120), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 132), 0.678995, 1e-4);
  EXPECT_NEAR (spin_images->points (100, 144), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 0), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 12), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 24), 0.143845, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 36), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 48), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 60), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 72), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 84), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 96), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 108), 0.706084, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 120), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 132), 0, 1e-4);
  EXPECT_NEAR (spin_images->points (300, 144), 0.272542, 1e-4);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IntensitySpinEstimation)
{
  // Generate a sample point cloud
  PointCloud<PointXYZI> cloud_xyzi;
  cloud_xyzi.height = 1;
  cloud_xyzi.is_dense = true;

  for (float x = -10.0; x <= 10.0; x += 1.0)
  {
    for (float y = -10.0; y <= 10.0; y += 1.0)
    {
      PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = sqrt (400 - x * x - y * y);
      p.intensity = exp (-(pow (x - 3, 2) + pow (y + 2, 2)) / (2 * 25.0)) + exp (-(pow (x + 5, 2) + pow (y - 5, 2))
                                                                                 / (2 * 4.0));

      cloud_xyzi.points.push_back (p);
    }
  }
  cloud_xyzi.width = cloud_xyzi.points.size ();

  // Compute the intensity-domain spin features
  typedef Histogram<20> IntensitySpin;
  IntensitySpinEstimation<PointXYZI, Eigen::MatrixXf> ispin_est;
  search::KdTree<PointXYZI>::Ptr treept3 (new search::KdTree<PointXYZI> (false));
  ispin_est.setSearchMethod (treept3);
  ispin_est.setRadiusSearch (10.0);
  ispin_est.setNrDistanceBins (4);
  ispin_est.setNrIntensityBins (5);

  ispin_est.setInputCloud (cloud_xyzi.makeShared ());
  PointCloud<Eigen::MatrixXf> ispin_output;
  ispin_est.computeEigen (ispin_output);

  // Compare to independently verified values
  Eigen::VectorXf ispin = ispin_output.points.row (220);
  const float correct_ispin_feature_values[20] = {2.4387, 9.4737, 21.3232, 28.3025, 22.5639, 13.2426, 35.7026, 60.0755,
                                                  66.9240, 50.4225, 42.7086, 83.5818, 105.4513, 97.8454, 67.3801,
                                                  75.7127, 119.4726, 120.9649, 93.4829, 55.4045};
  for (int i = 0; i < 20; ++i)
  {
    EXPECT_NEAR (ispin[i], correct_ispin_feature_values[i], 1e-4);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, RIFTEstimation)
{
  // Generate a sample point cloud
  PointCloud<PointXYZI> cloud_xyzi;
  cloud_xyzi.height = 1;
  cloud_xyzi.is_dense = true;
  for (float x = -10.0; x <= 10.0; x += 1.0)
  {
    for (float y = -10.0; y <= 10.0; y += 1.0)
    {
      PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = sqrt (400 - x * x - y * y);
      p.intensity = exp ((-pow (x - 3, 2) + pow (y + 2, 2)) / (2 * 25.0)) + exp ((-pow (x + 5, 2) + pow (y - 5, 2))
                                                                                 / (2 * 4.0));

      cloud_xyzi.points.push_back (p);
    }
  }
  cloud_xyzi.width = cloud_xyzi.points.size ();

  // Generate the intensity gradient data
  PointCloud<IntensityGradient> gradient;
  gradient.height = 1;
  gradient.width = cloud_xyzi.points.size ();
  gradient.is_dense = true;
  gradient.points.resize (gradient.width);
  for (size_t i = 0; i < cloud_xyzi.points.size (); ++i)
  {
    const PointXYZI &p = cloud_xyzi.points[i];

    // Compute the surface normal analytically.
    float nx = p.x;
    float ny = p.y;
    float nz = p.z;
    float magnitude = sqrt (nx * nx + ny * ny + nz * nz);
    nx /= magnitude;
    ny /= magnitude;
    nz /= magnitude;

    // Compute the intensity gradient analytically...
    float tmpx = -(p.x + 5) / 4.0 / exp ((pow (p.x + 5, 2) + pow (p.y - 5, 2)) / 8.0) - (p.x - 3) / 25.0
        / exp ((pow (p.x - 3, 2) + pow (p.y + 2, 2)) / 50.0);
    float tmpy = -(p.y - 5) / 4.0 / exp ((pow (p.x + 5, 2) + pow (p.y - 5, 2)) / 8.0) - (p.y + 2) / 25.0
        / exp ((pow (p.x - 3, 2) + pow (p.y + 2, 2)) / 50.0);
    float tmpz = 0;
    // ...and project the 3-D gradient vector onto the surface's tangent plane.
    float gx = (1 - nx * nx) * tmpx + (-nx * ny) * tmpy + (-nx * nz) * tmpz;
    float gy = (-ny * nx) * tmpx + (1 - ny * ny) * tmpy + (-ny * nz) * tmpz;
    float gz = (-nz * nx) * tmpx + (-nz * ny) * tmpy + (1 - nz * nz) * tmpz;

    gradient.points[i].gradient[0] = gx;
    gradient.points[i].gradient[1] = gy;
    gradient.points[i].gradient[2] = gz;
  }

  // Compute the RIFT features
  RIFTEstimation<PointXYZI, IntensityGradient, Eigen::MatrixXf> rift_est;
  search::KdTree<PointXYZI>::Ptr treept4 (new search::KdTree<PointXYZI> (false));
  rift_est.setSearchMethod (treept4);
  rift_est.setRadiusSearch (10.0);
  rift_est.setNrDistanceBins (4);
  rift_est.setNrGradientBins (8);

  rift_est.setInputCloud (cloud_xyzi.makeShared ());
  rift_est.setInputGradient (gradient.makeShared ());
  PointCloud<Eigen::MatrixXf> rift_output;
  rift_est.computeEigen (rift_output);

  // Compare to independently verified values
  Eigen::VectorXf rift = rift_output.points.row (220);
  const float correct_rift_feature_values[32] = {0.0187, 0.0349, 0.0647, 0.0881, 0.0042, 0.0131, 0.0346, 0.0030,
                                                 0.0076, 0.0218, 0.0463, 0.0030, 0.0087, 0.0288, 0.0920, 0.0472,
                                                 0.0076, 0.0420, 0.0726, 0.0669, 0.0090, 0.0901, 0.1274, 0.2185,
                                                 0.0147, 0.1222, 0.3568, 0.4348, 0.0149, 0.0806, 0.2787, 0.6864};
  for (int i = 0; i < 32; ++i)
  {
    ASSERT_NEAR (rift[i], correct_rift_feature_values[i], 1e-4);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  sensor_msgs::PointCloud2 cloud_blob;
  if (loadPCDFile (argv[1], cloud_blob) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  fromROSMsg (cloud_blob, cloud);

  indices.resize (cloud.points.size ());
  for (size_t i = 0; i < indices.size (); ++i)
  {
    indices[i] = i;
  }

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

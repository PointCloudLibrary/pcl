/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *  FOR a PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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

#ifndef PCL_COMMON_EIGEN_IMPL_HPP_
#define PCL_COMMON_EIGEN_IMPL_HPP_

#include <pcl/pcl_macros.h>

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::getTransFromUnitVectorsZY (const Eigen::Vector3f& z_axis, 
                                const Eigen::Vector3f& y_direction, 
                                Eigen::Affine3f& transformation)
{
  Eigen::Vector3f tmp0 = (y_direction.cross(z_axis)).normalized();
  Eigen::Vector3f tmp1 = (z_axis.cross(tmp0)).normalized();
  Eigen::Vector3f tmp2 = z_axis.normalized();
  
  transformation(0,0)=tmp0[0]; transformation(0,1)=tmp0[1]; transformation(0,2)=tmp0[2]; transformation(0,3)=0.0f;
  transformation(1,0)=tmp1[0]; transformation(1,1)=tmp1[1]; transformation(1,2)=tmp1[2]; transformation(1,3)=0.0f;
  transformation(2,0)=tmp2[0]; transformation(2,1)=tmp2[1]; transformation(2,2)=tmp2[2]; transformation(2,3)=0.0f;
  transformation(3,0)=0.0f;    transformation(3,1)=0.0f;    transformation(3,2)=0.0f;    transformation(3,3)=1.0f;
}

//////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3f 
pcl::getTransFromUnitVectorsZY (const Eigen::Vector3f& z_axis, 
                                const Eigen::Vector3f& y_direction)
{
  Eigen::Affine3f transformation;
  getTransFromUnitVectorsZY (z_axis, y_direction, transformation);
  return (transformation);
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::getTransFromUnitVectorsXY (const Eigen::Vector3f& x_axis, 
                                const Eigen::Vector3f& y_direction, 
                                Eigen::Affine3f& transformation)
{
  Eigen::Vector3f tmp2 = (x_axis.cross(y_direction)).normalized();
  Eigen::Vector3f tmp1 = (tmp2.cross(x_axis)).normalized();
  Eigen::Vector3f tmp0 = x_axis.normalized();
  
  transformation(0,0)=tmp0[0]; transformation(0,1)=tmp0[1]; transformation(0,2)=tmp0[2]; transformation(0,3)=0.0f;
  transformation(1,0)=tmp1[0]; transformation(1,1)=tmp1[1]; transformation(1,2)=tmp1[2]; transformation(1,3)=0.0f;
  transformation(2,0)=tmp2[0]; transformation(2,1)=tmp2[1]; transformation(2,2)=tmp2[2]; transformation(2,3)=0.0f;
  transformation(3,0)=0.0f;    transformation(3,1)=0.0f;    transformation(3,2)=0.0f;    transformation(3,3)=1.0f;
}

//////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3f 
pcl::getTransFromUnitVectorsXY (const Eigen::Vector3f& x_axis, 
                                const Eigen::Vector3f& y_direction)
{
  Eigen::Affine3f transformation;
  getTransFromUnitVectorsXY (x_axis, y_direction, transformation);
  return (transformation);
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::getTransformationFromTwoUnitVectors (const Eigen::Vector3f& y_direction, 
                                          const Eigen::Vector3f& z_axis, 
                                          Eigen::Affine3f& transformation)
{
  getTransFromUnitVectorsZY (z_axis, y_direction, transformation);
}

//////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3f 
pcl::getTransformationFromTwoUnitVectors (const Eigen::Vector3f& y_direction, 
                                          const Eigen::Vector3f& z_axis)
{
  Eigen::Affine3f transformation;
  getTransformationFromTwoUnitVectors (y_direction, z_axis, transformation);
  return (transformation);
}

void 
pcl::getTransformationFromTwoUnitVectorsAndOrigin (const Eigen::Vector3f& y_direction, 
                                                   const Eigen::Vector3f& z_axis,
                                                   const Eigen::Vector3f& origin, 
                                                   Eigen::Affine3f& transformation)
{
  getTransformationFromTwoUnitVectors(y_direction, z_axis, transformation);
  Eigen::Vector3f translation = transformation*origin;
  transformation(0,3)=-translation[0];  transformation(1,3)=-translation[1];  transformation(2,3)=-translation[2];
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::getEulerAngles (const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw)
{
  roll  = atan2f(t(2,1), t(2,2));
  pitch = asinf(-t(2,0));
  yaw   = atan2f(t(1,0), t(0,0));
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::getTranslationAndEulerAngles (const Eigen::Affine3f& t, 
                                   float& x, float& y, float& z, 
                                   float& roll, float& pitch, float& yaw)
{
  x = t(0,3);
  y = t(1,3);
  z = t(2,3);
  roll  = atan2f(t(2,1), t(2,2));
  pitch = asinf(-t(2,0));
  yaw   = atan2f(t(1,0), t(0,0));
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> void 
pcl::getTransformation (Scalar x, Scalar y, Scalar z, 
                        Scalar roll, Scalar pitch, Scalar yaw, 
                        Eigen::Transform<Scalar, 3, Eigen::Affine> &t)
{
  Scalar A = cos (yaw),  B = sin (yaw),  C  = cos (pitch), D  = sin (pitch),
         E = cos (roll), F = sin (roll), DE = D*E,         DF = D*F;

  t (0, 0) = A*C;  t (0, 1) = A*DF - B*E;  t (0, 2) = B*F + A*DE;  t (0, 3) = x;
  t (1, 0) = B*C;  t (1, 1) = A*E + B*DF;  t (1, 2) = B*DE - A*F;  t (1, 3) = y;
  t (2, 0) = -D;   t (2, 1) = C*F;         t (2, 2) = C*E;         t (2, 3) = z;
  t (3, 0) = 0;    t (3, 1) = 0;           t (3, 2) = 0;           t (3, 3) = 1;
}

//////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3f 
pcl::getTransformation (float x, float y, float z, float roll, float pitch, float yaw)
{
  Eigen::Affine3f t;
  getTransformation (x, y, z, roll, pitch, yaw, t);
  return (t);
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename Derived> void 
pcl::saveBinary (const Eigen::MatrixBase<Derived>& matrix, std::ostream& file)
{
  uint32_t rows = static_cast<uint32_t> (matrix.rows ()), cols = static_cast<uint32_t> (matrix.cols ());
  file.write (reinterpret_cast<char*> (&rows), sizeof (rows));
  file.write (reinterpret_cast<char*> (&cols), sizeof (cols));
  for (uint32_t i = 0; i < rows; ++i)
    for (uint32_t j = 0; j < cols; ++j)
    {
      typename Derived::Scalar tmp = matrix(i,j);
      file.write (reinterpret_cast<const char*> (&tmp), sizeof (tmp));
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename Derived> void 
pcl::loadBinary (Eigen::MatrixBase<Derived> const & matrix_, std::istream& file)
{
  Eigen::MatrixBase<Derived> &matrix = const_cast<Eigen::MatrixBase<Derived> &> (matrix_);

  uint32_t rows, cols;
  file.read (reinterpret_cast<char*> (&rows), sizeof (rows));
  file.read (reinterpret_cast<char*> (&cols), sizeof (cols));
  if (matrix.rows () != static_cast<int>(rows) || matrix.cols () != static_cast<int>(cols))
    matrix.derived().resize(rows, cols);
  
  for (uint32_t i = 0; i < rows; ++i)
    for (uint32_t j = 0; j < cols; ++j)
    {
      typename Derived::Scalar tmp;
      file.read (reinterpret_cast<char*> (&tmp), sizeof (tmp));
      matrix (i, j) = tmp;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename Derived, typename OtherDerived> 
typename Eigen::internal::umeyama_transform_matrix_type<Derived, OtherDerived>::type
pcl::umeyama (const Eigen::MatrixBase<Derived>& src, const Eigen::MatrixBase<OtherDerived>& dst, bool with_scaling)
{
  typedef typename Eigen::internal::umeyama_transform_matrix_type<Derived, OtherDerived>::type TransformationMatrixType;
  typedef typename Eigen::internal::traits<TransformationMatrixType>::Scalar Scalar;
  typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;
  typedef typename Derived::Index Index;

  EIGEN_STATIC_ASSERT (!Eigen::NumTraits<Scalar>::IsComplex, NUMERIC_TYPE_MUST_BE_REAL)
  EIGEN_STATIC_ASSERT ((Eigen::internal::is_same<Scalar, typename Eigen::internal::traits<OtherDerived>::Scalar>::value),
    YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

  enum { Dimension = PCL_EIGEN_SIZE_MIN_PREFER_DYNAMIC (Derived::RowsAtCompileTime, OtherDerived::RowsAtCompileTime) };

  typedef Eigen::Matrix<Scalar, Dimension, 1> VectorType;
  typedef Eigen::Matrix<Scalar, Dimension, Dimension> MatrixType;
  typedef typename Eigen::internal::plain_matrix_type_row_major<Derived>::type RowMajorMatrixType;

  const Index m = src.rows (); // dimension
  const Index n = src.cols (); // number of measurements

  // required for demeaning ...
  const RealScalar one_over_n = 1 / static_cast<RealScalar> (n);

  // computation of mean
  const VectorType src_mean = src.rowwise ().sum () * one_over_n;
  const VectorType dst_mean = dst.rowwise ().sum () * one_over_n;

  // demeaning of src and dst points
  const RowMajorMatrixType src_demean = src.colwise () - src_mean;
  const RowMajorMatrixType dst_demean = dst.colwise () - dst_mean;

  // Eq. (36)-(37)
  const Scalar src_var = src_demean.rowwise ().squaredNorm ().sum () * one_over_n;

  // Eq. (38)
  const MatrixType sigma (one_over_n * dst_demean * src_demean.transpose ());

  Eigen::JacobiSVD<MatrixType> svd (sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Initialize the resulting transformation with an identity matrix...
  TransformationMatrixType Rt = TransformationMatrixType::Identity (m + 1, m + 1);

  // Eq. (39)
  VectorType S = VectorType::Ones (m);
  if (sigma.determinant () < 0) 
    S (m - 1) = -1;

  // Eq. (40) and (43)
  const VectorType& d = svd.singularValues ();
  Index rank = 0; 
  for (Index i = 0; i < m; ++i) 
    if (!Eigen::internal::isMuchSmallerThan (d.coeff (i), d.coeff (0))) 
      ++rank;
  if (rank == m - 1) 
  {
    if (svd.matrixU ().determinant () * svd.matrixV ().determinant () > 0) 
      Rt.block (0, 0, m, m).noalias () = svd.matrixU () * svd.matrixV ().transpose ();
    else 
    {
      const Scalar s = S (m - 1); 
      S (m - 1) = -1;
      Rt.block (0, 0, m, m).noalias () = svd.matrixU () * S.asDiagonal () * svd.matrixV ().transpose ();
      S (m - 1) = s;
    }
  } 
  else 
  {
    Rt.block (0, 0, m, m).noalias () = svd.matrixU () * S.asDiagonal () * svd.matrixV ().transpose ();
  }

  // Eq. (42)
  if (with_scaling)
  {
    // Eq. (42)
    const Scalar c = 1 / src_var * svd.singularValues ().dot (S);

    // Eq. (41)
    Rt.col (m).head (m) = dst_mean;
    Rt.col (m).head (m).noalias () -= c * Rt.topLeftCorner (m, m) * src_mean;
    Rt.block (0, 0, m, m) *= c;
  }
  else
  {
    Rt.col (m).head (m) = dst_mean;
    Rt.col (m).head (m).noalias () -= Rt.topLeftCorner (m, m) * src_mean;
  }

  return (Rt);
}

#endif  //PCL_COMMON_EIGEN_IMPL_HPP_


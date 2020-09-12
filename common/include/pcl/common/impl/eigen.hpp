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

#pragma once

#include <pcl/common/eigen.h>
#include <pcl/console/print.h>

#include <array>
#include <algorithm>
#include <cmath>


namespace pcl
{

template <typename Scalar, typename Roots> inline void
computeRoots2 (const Scalar& b, const Scalar& c, Roots& roots)
{
  roots (0) = Scalar (0);
  Scalar d = Scalar (b * b - 4.0 * c);
  if (d < 0.0)  // no real roots ! THIS SHOULD NOT HAPPEN!
    d = 0.0;

  Scalar sd = std::sqrt (d);

  roots (2) = 0.5f * (b + sd);
  roots (1) = 0.5f * (b - sd);
}


template <typename Matrix, typename Roots> inline void
computeRoots (const Matrix& m, Roots& roots)
{
  using Scalar = typename Matrix::Scalar;

  // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
  // eigenvalues are the roots to this equation, all guaranteed to be
  // real-valued, because the matrix is symmetric.
  Scalar c0 =      m (0, 0) * m (1, 1) * m (2, 2)
      + Scalar (2) * m (0, 1) * m (0, 2) * m (1, 2)
             - m (0, 0) * m (1, 2) * m (1, 2)
             - m (1, 1) * m (0, 2) * m (0, 2)
             - m (2, 2) * m (0, 1) * m (0, 1);
  Scalar c1 = m (0, 0) * m (1, 1) -
        m (0, 1) * m (0, 1) +
        m (0, 0) * m (2, 2) -
        m (0, 2) * m (0, 2) +
        m (1, 1) * m (2, 2) -
        m (1, 2) * m (1, 2);
  Scalar c2 = m (0, 0) + m (1, 1) + m (2, 2);

  if (std::abs (c0) < Eigen::NumTraits < Scalar > ::epsilon ())  // one root is 0 -> quadratic equation
    computeRoots2 (c2, c1, roots);
  else
  {
    const Scalar s_inv3 = Scalar (1.0 / 3.0);
    const Scalar s_sqrt3 = std::sqrt (Scalar (3.0));
    // Construct the parameters used in classifying the roots of the equation
    // and in solving the equation for the roots in closed form.
    Scalar c2_over_3 = c2 * s_inv3;
    Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
    if (a_over_3 > Scalar (0))
      a_over_3 = Scalar (0);

    Scalar half_b = Scalar (0.5) * (c0 + c2_over_3 * (Scalar (2) * c2_over_3 * c2_over_3 - c1));

    Scalar q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
    if (q > Scalar (0))
      q = Scalar (0);

    // Compute the eigenvalues by solving for the roots of the polynomial.
    Scalar rho = std::sqrt (-a_over_3);
    Scalar theta = std::atan2 (std::sqrt (-q), half_b) * s_inv3;
    Scalar cos_theta = std::cos (theta);
    Scalar sin_theta = std::sin (theta);
    roots (0) = c2_over_3 + Scalar (2) * rho * cos_theta;
    roots (1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
    roots (2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

    // Sort in increasing order.
    if (roots (0) >= roots (1))
      std::swap (roots (0), roots (1));
    if (roots (1) >= roots (2))
    {
      std::swap (roots (1), roots (2));
      if (roots (0) >= roots (1))
        std::swap (roots (0), roots (1));
    }

    if (roots (0) <= 0)  // eigenval for symmetric positive semi-definite matrix can not be negative! Set it to 0
      computeRoots2 (c2, c1, roots);
  }
}


template <typename Matrix, typename Vector> inline void
eigen22 (const Matrix& mat, typename Matrix::Scalar& eigenvalue, Vector& eigenvector)
{
  // if diagonal matrix, the eigenvalues are the diagonal elements
  // and the eigenvectors are not unique, thus set to Identity
  if (std::abs (mat.coeff (1)) <= std::numeric_limits<typename Matrix::Scalar>::min ())
  {
    if (mat.coeff (0) < mat.coeff (2))
    {
      eigenvalue = mat.coeff (0);
      eigenvector[0] = 1.0;
      eigenvector[1] = 0.0;
    }
    else
    {
      eigenvalue = mat.coeff (2);
      eigenvector[0] = 0.0;
      eigenvector[1] = 1.0;
    }
    return;
  }

  // 0.5 to optimize further calculations
  typename Matrix::Scalar trace = static_cast<typename Matrix::Scalar> (0.5) * (mat.coeff (0) + mat.coeff (3));
  typename Matrix::Scalar determinant = mat.coeff (0) * mat.coeff (3) - mat.coeff (1) * mat.coeff (1);

  typename Matrix::Scalar temp = trace * trace - determinant;

  if (temp < 0)
    temp = 0;

  eigenvalue = trace - std::sqrt (temp);

  eigenvector[0] = -mat.coeff (1);
  eigenvector[1] = mat.coeff (0) - eigenvalue;
  eigenvector.normalize ();
}


template <typename Matrix, typename Vector> inline void
eigen22 (const Matrix& mat, Matrix& eigenvectors, Vector& eigenvalues)
{
  // if diagonal matrix, the eigenvalues are the diagonal elements
  // and the eigenvectors are not unique, thus set to Identity
  if (std::abs (mat.coeff (1)) <= std::numeric_limits<typename Matrix::Scalar>::min ())
  {
    if (mat.coeff (0) < mat.coeff (3))
    {
      eigenvalues.coeffRef (0) = mat.coeff (0);
      eigenvalues.coeffRef (1) = mat.coeff (3);
      eigenvectors.coeffRef (0) = 1.0;
      eigenvectors.coeffRef (1) = 0.0;
      eigenvectors.coeffRef (2) = 0.0;
      eigenvectors.coeffRef (3) = 1.0;
    }
    else
    {
      eigenvalues.coeffRef (0) = mat.coeff (3);
      eigenvalues.coeffRef (1) = mat.coeff (0);
      eigenvectors.coeffRef (0) = 0.0;
      eigenvectors.coeffRef (1) = 1.0;
      eigenvectors.coeffRef (2) = 1.0;
      eigenvectors.coeffRef (3) = 0.0;
    }
    return;
  }

  // 0.5 to optimize further calculations
  typename Matrix::Scalar trace = static_cast<typename Matrix::Scalar> (0.5) * (mat.coeff (0) + mat.coeff (3));
  typename Matrix::Scalar determinant = mat.coeff (0) * mat.coeff (3) - mat.coeff (1) * mat.coeff (1);

  typename Matrix::Scalar temp = trace * trace - determinant;

  if (temp < 0)
    temp = 0;
  else
    temp = std::sqrt (temp);

  eigenvalues.coeffRef (0) = trace - temp;
  eigenvalues.coeffRef (1) = trace + temp;

  // either this is in a row or column depending on RowMajor or ColumnMajor
  eigenvectors.coeffRef (0) = -mat.coeff (1);
  eigenvectors.coeffRef (2) = mat.coeff (0) - eigenvalues.coeff (0);
  typename Matrix::Scalar norm = static_cast<typename Matrix::Scalar> (1.0)
      / static_cast<typename Matrix::Scalar> (std::sqrt (eigenvectors.coeffRef (0) * eigenvectors.coeffRef (0) + eigenvectors.coeffRef (2) * eigenvectors.coeffRef (2)));
  eigenvectors.coeffRef (0) *= norm;
  eigenvectors.coeffRef (2) *= norm;
  eigenvectors.coeffRef (1) = eigenvectors.coeffRef (2);
  eigenvectors.coeffRef (3) = -eigenvectors.coeffRef (0);
}


template <typename Matrix, typename Vector> inline void
computeCorrespondingEigenVector (const Matrix& mat, const typename Matrix::Scalar& eigenvalue, Vector& eigenvector)
{
  using Scalar = typename Matrix::Scalar;
  // Scale the matrix so its entries are in [-1,1].  The scaling is applied
  // only when at least one matrix entry has magnitude larger than 1.

  Scalar scale = mat.cwiseAbs ().maxCoeff ();
  if (scale <= std::numeric_limits < Scalar > ::min ())
    scale = Scalar (1.0);

  Matrix scaledMat = mat / scale;

  scaledMat.diagonal ().array () -= eigenvalue / scale;

  Vector vec1 = scaledMat.row (0).cross (scaledMat.row (1));
  Vector vec2 = scaledMat.row (0).cross (scaledMat.row (2));
  Vector vec3 = scaledMat.row (1).cross (scaledMat.row (2));

  Scalar len1 = vec1.squaredNorm ();
  Scalar len2 = vec2.squaredNorm ();
  Scalar len3 = vec3.squaredNorm ();

  if (len1 >= len2 && len1 >= len3)
    eigenvector = vec1 / std::sqrt (len1);
  else if (len2 >= len1 && len2 >= len3)
    eigenvector = vec2 / std::sqrt (len2);
  else
    eigenvector = vec3 / std::sqrt (len3);
}

namespace detail
{

template <typename Vector, typename Scalar>
struct EigenVector {
  Vector vector;
  Scalar length;
};  // struct EigenVector

/**
 * @brief returns the unit vector along the largest eigen value as well as the
 *        length of the largest eigenvector
 * @tparam Vector Requested result type, needs to be explicitly provided and has
 *                to be implicitly constructible from ConstRowExpr
 * @tparam Matrix deduced input type providing similar in API as Eigen::Matrix
 */
template <typename Vector, typename Matrix> static EigenVector<Vector, typename Matrix::Scalar>
getLargest3x3Eigenvector (const Matrix scaledMatrix)
{
  using Scalar = typename Matrix::Scalar;
  using Index = typename Matrix::Index;

  Matrix crossProduct;
  crossProduct << scaledMatrix.row (0).cross (scaledMatrix.row (1)),
                  scaledMatrix.row (0).cross (scaledMatrix.row (2)),
                  scaledMatrix.row (1).cross (scaledMatrix.row (2));

  // expression template, no evaluation here
  const auto len = crossProduct.rowwise ().norm ();

  Index index;
  const Scalar length = len.maxCoeff (&index);  // <- first evaluation
  return EigenVector<Vector, Scalar> {crossProduct.row (index) / length,
                                      length};
}

}  // namespace detail


template <typename Matrix, typename Vector> inline void
eigen33 (const Matrix& mat, typename Matrix::Scalar& eigenvalue, Vector& eigenvector)
{
  using Scalar = typename Matrix::Scalar;
  // Scale the matrix so its entries are in [-1,1].  The scaling is applied
  // only when at least one matrix entry has magnitude larger than 1.

  Scalar scale = mat.cwiseAbs ().maxCoeff ();
  if (scale <= std::numeric_limits < Scalar > ::min ())
    scale = Scalar (1.0);

  Matrix scaledMat = mat / scale;

  Vector eigenvalues;
  computeRoots (scaledMat, eigenvalues);

  eigenvalue = eigenvalues (0) * scale;

  scaledMat.diagonal ().array () -= eigenvalues (0);

  eigenvector = detail::getLargest3x3Eigenvector<Vector> (scaledMat).vector;
}


template <typename Matrix, typename Vector> inline void
eigen33 (const Matrix& mat, Vector& evals)
{
  using Scalar = typename Matrix::Scalar;
  Scalar scale = mat.cwiseAbs ().maxCoeff ();
  if (scale <= std::numeric_limits < Scalar > ::min ())
    scale = Scalar (1.0);

  Matrix scaledMat = mat / scale;
  computeRoots (scaledMat, evals);
  evals *= scale;
}


template <typename Matrix, typename Vector> inline void
eigen33 (const Matrix& mat, Matrix& evecs, Vector& evals)
{
  using Scalar = typename Matrix::Scalar;
  // Scale the matrix so its entries are in [-1,1].  The scaling is applied
  // only when at least one matrix entry has magnitude larger than 1.

  Scalar scale = mat.cwiseAbs ().maxCoeff ();
  if (scale <= std::numeric_limits < Scalar > ::min ())
    scale = Scalar (1.0);

  Matrix scaledMat = mat / scale;

  // Compute the eigenvalues
  computeRoots (scaledMat, evals);

  if ( (evals (2) - evals (0)) <= Eigen::NumTraits < Scalar > ::epsilon ())
  {
    // all three equal
    evecs.setIdentity ();
  }
  else if ( (evals (1) - evals (0)) <= Eigen::NumTraits < Scalar > ::epsilon ())
  {
    // first and second equal
    Matrix tmp;
    tmp = scaledMat;
    tmp.diagonal ().array () -= evals (2);

    evecs.col (2) = detail::getLargest3x3Eigenvector<Vector> (tmp).vector;
    evecs.col (1) = evecs.col (2).unitOrthogonal ();
    evecs.col (0) = evecs.col (1).cross (evecs.col (2));
  }
  else if ( (evals (2) - evals (1)) <= Eigen::NumTraits < Scalar > ::epsilon ())
  {
    // second and third equal
    Matrix tmp;
    tmp = scaledMat;
    tmp.diagonal ().array () -= evals (0);

    evecs.col (0) = detail::getLargest3x3Eigenvector<Vector> (tmp).vector;
    evecs.col (1) = evecs.col (0).unitOrthogonal ();
    evecs.col (2) = evecs.col (0).cross (evecs.col (1));
  }
  else
  {
    std::array<Scalar, 3> eigenVecLen;

    for (int i = 0; i < 3; ++i)
    {
      Matrix tmp = scaledMat;
      tmp.diagonal ().array () -= evals (i);
      const auto vec_len = detail::getLargest3x3Eigenvector<Vector> (tmp);
      evecs.col (i) = vec_len.vector;
      eigenVecLen[i] = vec_len.length;
    }

    // @TODO: might be redundant or over-complicated as per @SergioRAgostinho
    // see: https://github.com/PointCloudLibrary/pcl/pull/3441#discussion_r341024181
    const auto minmax_it = std::minmax_element (eigenVecLen.cbegin (), eigenVecLen.cend ());
    int min_idx = std::distance (eigenVecLen.cbegin (), minmax_it.first);
    int max_idx = std::distance (eigenVecLen.cbegin (), minmax_it.second);
    int mid_idx = 3 - min_idx - max_idx;

    evecs.col (min_idx) = evecs.col ( (min_idx + 1) % 3).cross (evecs.col ( (min_idx + 2) % 3)).normalized ();
    evecs.col (mid_idx) = evecs.col ( (mid_idx + 1) % 3).cross (evecs.col ( (mid_idx + 2) % 3)).normalized ();
  }
  // Rescale back to the original size.
  evals *= scale;
}


template <typename Matrix> inline typename Matrix::Scalar
invert2x2 (const Matrix& matrix, Matrix& inverse)
{
  using Scalar = typename Matrix::Scalar;
  Scalar det = matrix.coeff (0) * matrix.coeff (3) - matrix.coeff (1) * matrix.coeff (2);

  if (det != 0)
  {
    //Scalar inv_det = Scalar (1.0) / det;
    inverse.coeffRef (0) = matrix.coeff (3);
    inverse.coeffRef (1) = -matrix.coeff (1);
    inverse.coeffRef (2) = -matrix.coeff (2);
    inverse.coeffRef (3) = matrix.coeff (0);
    inverse /= det;
  }
  return det;
}


template <typename Matrix> inline typename Matrix::Scalar
invert3x3SymMatrix (const Matrix& matrix, Matrix& inverse)
{
  using Scalar = typename Matrix::Scalar;
  // elements
  // a b c
  // b d e
  // c e f
  //| a b c |-1             |   fd-ee    ce-bf   be-cd  |
  //| b d e |    =  1/det * |   ce-bf    af-cc   bc-ae  |
  //| c e f |               |   be-cd    bc-ae   ad-bb  |

  //det = a(fd-ee) + b(ec-fb) + c(eb-dc)

  Scalar fd_ee = matrix.coeff (4) * matrix.coeff (8) - matrix.coeff (7) * matrix.coeff (5);
  Scalar ce_bf = matrix.coeff (2) * matrix.coeff (5) - matrix.coeff (1) * matrix.coeff (8);
  Scalar be_cd = matrix.coeff (1) * matrix.coeff (5) - matrix.coeff (2) * matrix.coeff (4);

  Scalar det = matrix.coeff (0) * fd_ee + matrix.coeff (1) * ce_bf + matrix.coeff (2) * be_cd;

  if (det != 0)
  {
    //Scalar inv_det = Scalar (1.0) / det;
    inverse.coeffRef (0) = fd_ee;
    inverse.coeffRef (1) = inverse.coeffRef (3) = ce_bf;
    inverse.coeffRef (2) = inverse.coeffRef (6) = be_cd;
    inverse.coeffRef (4) = (matrix.coeff (0) * matrix.coeff (8) - matrix.coeff (2) * matrix.coeff (2));
    inverse.coeffRef (5) = inverse.coeffRef (7) = (matrix.coeff (1) * matrix.coeff (2) - matrix.coeff (0) * matrix.coeff (5));
    inverse.coeffRef (8) = (matrix.coeff (0) * matrix.coeff (4) - matrix.coeff (1) * matrix.coeff (1));
    inverse /= det;
  }
  return det;
}


template <typename Matrix> inline typename Matrix::Scalar
invert3x3Matrix (const Matrix& matrix, Matrix& inverse)
{
  using Scalar = typename Matrix::Scalar;

  //| a b c |-1             |   ie-hf    hc-ib   fb-ec  |
  //| d e f |    =  1/det * |   gf-id    ia-gc   dc-fa  |
  //| g h i |               |   hd-ge    gb-ha   ea-db  |
  //det = a(ie-hf) + d(hc-ib) + g(fb-ec)

  Scalar ie_hf = matrix.coeff (8) * matrix.coeff (4) - matrix.coeff (7) * matrix.coeff (5);
  Scalar hc_ib = matrix.coeff (7) * matrix.coeff (2) - matrix.coeff (8) * matrix.coeff (1);
  Scalar fb_ec = matrix.coeff (5) * matrix.coeff (1) - matrix.coeff (4) * matrix.coeff (2);
  Scalar det = matrix.coeff (0) * (ie_hf) + matrix.coeff (3) * (hc_ib) + matrix.coeff (6) * (fb_ec);

  if (det != 0)
  {
    inverse.coeffRef (0) = ie_hf;
    inverse.coeffRef (1) = hc_ib;
    inverse.coeffRef (2) = fb_ec;
    inverse.coeffRef (3) = matrix.coeff (6) * matrix.coeff (5) - matrix.coeff (8) * matrix.coeff (3);
    inverse.coeffRef (4) = matrix.coeff (8) * matrix.coeff (0) - matrix.coeff (6) * matrix.coeff (2);
    inverse.coeffRef (5) = matrix.coeff (3) * matrix.coeff (2) - matrix.coeff (5) * matrix.coeff (0);
    inverse.coeffRef (6) = matrix.coeff (7) * matrix.coeff (3) - matrix.coeff (6) * matrix.coeff (4);
    inverse.coeffRef (7) = matrix.coeff (6) * matrix.coeff (1) - matrix.coeff (7) * matrix.coeff (0);
    inverse.coeffRef (8) = matrix.coeff (4) * matrix.coeff (0) - matrix.coeff (3) * matrix.coeff (1);

    inverse /= det;
  }
  return det;
}


template <typename Matrix> inline typename Matrix::Scalar
determinant3x3Matrix (const Matrix& matrix)
{
  // result is independent of Row/Col Major storage!
  return matrix.coeff (0) * (matrix.coeff (4) * matrix.coeff (8) - matrix.coeff (5) * matrix.coeff (7)) +
         matrix.coeff (1) * (matrix.coeff (5) * matrix.coeff (6) - matrix.coeff (3) * matrix.coeff (8)) +
         matrix.coeff (2) * (matrix.coeff (3) * matrix.coeff (7) - matrix.coeff (4) * matrix.coeff (6)) ;
}


void
getTransFromUnitVectorsZY (const Eigen::Vector3f& z_axis,
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


Eigen::Affine3f
getTransFromUnitVectorsZY (const Eigen::Vector3f& z_axis,
                           const Eigen::Vector3f& y_direction)
{
  Eigen::Affine3f transformation;
  getTransFromUnitVectorsZY (z_axis, y_direction, transformation);
  return (transformation);
}


void
getTransFromUnitVectorsXY (const Eigen::Vector3f& x_axis,
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


Eigen::Affine3f
getTransFromUnitVectorsXY (const Eigen::Vector3f& x_axis,
                           const Eigen::Vector3f& y_direction)
{
  Eigen::Affine3f transformation;
  getTransFromUnitVectorsXY (x_axis, y_direction, transformation);
  return (transformation);
}


void
getTransformationFromTwoUnitVectors (const Eigen::Vector3f& y_direction,
                                     const Eigen::Vector3f& z_axis,
                                     Eigen::Affine3f& transformation)
{
  getTransFromUnitVectorsZY (z_axis, y_direction, transformation);
}


Eigen::Affine3f
getTransformationFromTwoUnitVectors (const Eigen::Vector3f& y_direction,
                                     const Eigen::Vector3f& z_axis)
{
  Eigen::Affine3f transformation;
  getTransformationFromTwoUnitVectors (y_direction, z_axis, transformation);
  return (transformation);
}


void
getTransformationFromTwoUnitVectorsAndOrigin (const Eigen::Vector3f& y_direction,
                                              const Eigen::Vector3f& z_axis,
                                              const Eigen::Vector3f& origin,
                                              Eigen::Affine3f& transformation)
{
  getTransformationFromTwoUnitVectors(y_direction, z_axis, transformation);
  Eigen::Vector3f translation = transformation*origin;
  transformation(0,3)=-translation[0];  transformation(1,3)=-translation[1];  transformation(2,3)=-translation[2];
}


template <typename Scalar> void
getEulerAngles (const Eigen::Transform<Scalar, 3, Eigen::Affine> &t, Scalar &roll, Scalar &pitch, Scalar &yaw)
{
  roll = std::atan2 (t (2, 1), t (2, 2));
  pitch = asin (-t (2, 0));
  yaw = std::atan2 (t (1, 0), t (0, 0));
}


template <typename Scalar> void
getTranslationAndEulerAngles (const Eigen::Transform<Scalar, 3, Eigen::Affine> &t,
                              Scalar &x, Scalar &y, Scalar &z,
                              Scalar &roll, Scalar &pitch, Scalar &yaw)
{
  x = t (0, 3);
  y = t (1, 3);
  z = t (2, 3);
  roll = std::atan2 (t (2, 1), t (2, 2));
  pitch = asin (-t (2, 0));
  yaw = std::atan2 (t (1, 0), t (0, 0));
}


template <typename Scalar> void
getTransformation (Scalar x, Scalar y, Scalar z,
                   Scalar roll, Scalar pitch, Scalar yaw,
                   Eigen::Transform<Scalar, 3, Eigen::Affine> &t)
{
  Scalar A = std::cos (yaw),  B = sin (yaw),  C  = std::cos (pitch), D  = sin (pitch),
         E = std::cos (roll), F = sin (roll), DE = D*E,         DF = D*F;

  t (0, 0) = A*C;  t (0, 1) = A*DF - B*E;  t (0, 2) = B*F + A*DE;  t (0, 3) = x;
  t (1, 0) = B*C;  t (1, 1) = A*E + B*DF;  t (1, 2) = B*DE - A*F;  t (1, 3) = y;
  t (2, 0) = -D;   t (2, 1) = C*F;         t (2, 2) = C*E;         t (2, 3) = z;
  t (3, 0) = 0;    t (3, 1) = 0;           t (3, 2) = 0;           t (3, 3) = 1;
}


template <typename Derived> void
saveBinary (const Eigen::MatrixBase<Derived>& matrix, std::ostream& file)
{
  std::uint32_t rows = static_cast<std::uint32_t> (matrix.rows ()), cols = static_cast<std::uint32_t> (matrix.cols ());
  file.write (reinterpret_cast<char*> (&rows), sizeof (rows));
  file.write (reinterpret_cast<char*> (&cols), sizeof (cols));
  for (std::uint32_t i = 0; i < rows; ++i)
    for (std::uint32_t j = 0; j < cols; ++j)
    {
      typename Derived::Scalar tmp = matrix(i,j);
      file.write (reinterpret_cast<const char*> (&tmp), sizeof (tmp));
    }
}


template <typename Derived> void
loadBinary (Eigen::MatrixBase<Derived> const & matrix_, std::istream& file)
{
  Eigen::MatrixBase<Derived> &matrix = const_cast<Eigen::MatrixBase<Derived> &> (matrix_);

  std::uint32_t rows, cols;
  file.read (reinterpret_cast<char*> (&rows), sizeof (rows));
  file.read (reinterpret_cast<char*> (&cols), sizeof (cols));
  if (matrix.rows () != static_cast<int>(rows) || matrix.cols () != static_cast<int>(cols))
    matrix.derived().resize(rows, cols);

  for (std::uint32_t i = 0; i < rows; ++i)
    for (std::uint32_t j = 0; j < cols; ++j)
    {
      typename Derived::Scalar tmp;
      file.read (reinterpret_cast<char*> (&tmp), sizeof (tmp));
      matrix (i, j) = tmp;
    }
}


template <typename Derived, typename OtherDerived>
typename Eigen::internal::umeyama_transform_matrix_type<Derived, OtherDerived>::type
umeyama (const Eigen::MatrixBase<Derived>& src, const Eigen::MatrixBase<OtherDerived>& dst, bool with_scaling)
{
#if EIGEN_VERSION_AT_LEAST (3, 3, 0)
  return Eigen::umeyama (src, dst, with_scaling);
#else
  using TransformationMatrixType = typename Eigen::internal::umeyama_transform_matrix_type<Derived, OtherDerived>::type;
  using Scalar = typename Eigen::internal::traits<TransformationMatrixType>::Scalar;
  using RealScalar = typename Eigen::NumTraits<Scalar>::Real;
  using Index = typename Derived::Index;

  static_assert (!Eigen::NumTraits<Scalar>::IsComplex, "Numeric type must be real.");
  static_assert ((Eigen::internal::is_same<Scalar, typename Eigen::internal::traits<OtherDerived>::Scalar>::value),
    "You mixed different numeric types. You need to use the cast method of matrixbase to cast numeric types explicitly.");

  enum { Dimension = PCL_EIGEN_SIZE_MIN_PREFER_DYNAMIC (Derived::RowsAtCompileTime, OtherDerived::RowsAtCompileTime) };

  using VectorType = Eigen::Matrix<Scalar, Dimension, 1>;
  using MatrixType = Eigen::Matrix<Scalar, Dimension, Dimension>;
  using RowMajorMatrixType = typename Eigen::internal::plain_matrix_type_row_major<Derived>::type;

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

  if  ( svd.matrixU ().determinant () * svd.matrixV ().determinant () < 0 )
    S (m - 1) = -1;

  // Eq. (40) and (43)
  Rt.block (0,0,m,m).noalias () = svd.matrixU () * S.asDiagonal () * svd.matrixV ().transpose ();

  if (with_scaling)
  {
    // Eq. (42)
    const Scalar c = Scalar (1)/ src_var * svd.singularValues ().dot (S);

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
#endif
}


template <typename Scalar> bool
transformLine (const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &line_in,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &line_out,
               const Eigen::Transform<Scalar, 3, Eigen::Affine> &transformation)
{
  if (line_in.innerSize () != 6 || line_out.innerSize () != 6)
  {
    PCL_DEBUG ("transformLine: lines size != 6\n");
    return (false);
  }

  Eigen::Matrix<Scalar, 3, 1> point, vector;
  point << line_in.template head<3> ();
  vector << line_out.template tail<3> ();

  pcl::transformPoint (point, point, transformation);
  pcl::transformVector (vector, vector, transformation);
  line_out << point, vector;
  return (true);
}


template <typename Scalar> void
transformPlane (const Eigen::Matrix<Scalar, 4, 1> &plane_in,
                      Eigen::Matrix<Scalar, 4, 1> &plane_out,
                const Eigen::Transform<Scalar, 3, Eigen::Affine> &transformation)
{
  Eigen::Hyperplane < Scalar, 3 > plane;
  plane.coeffs () << plane_in;
  plane.transform (transformation);
  plane_out << plane.coeffs ();

  // Versions prior to 3.3.2 don't normalize the result
  #if !EIGEN_VERSION_AT_LEAST (3, 3, 2)
    plane_out /= plane_out.template head<3> ().norm ();
  #endif
}


template <typename Scalar> void
transformPlane (const pcl::ModelCoefficients::ConstPtr plane_in,
                      pcl::ModelCoefficients::Ptr plane_out,
                const Eigen::Transform<Scalar, 3, Eigen::Affine> &transformation)
{
  std::vector<Scalar> values (plane_in->values.begin (), plane_in->values.end ());
  Eigen::Matrix < Scalar, 4, 1 > v_plane_in (values.data ());
  pcl::transformPlane (v_plane_in, v_plane_in, transformation);
  plane_out->values.resize (4);
  std::copy_n(v_plane_in.data (), 4, plane_out->values.begin ());
}


template <typename Scalar> bool
checkCoordinateSystem (const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &line_x,
                       const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &line_y,
                       const Scalar norm_limit,
                       const Scalar dot_limit)
{
  if (line_x.innerSize () != 6 || line_y.innerSize () != 6)
  {
    PCL_DEBUG ("checkCoordinateSystem: lines size != 6\n");
    return (false);
  }

  if (line_x.template head<3> () != line_y.template head<3> ())
  {
    PCL_DEBUG ("checkCoorZdinateSystem: vector origins are different !\n");
    return (false);
  }

  // Make a copy of vector directions
  // X^Y = Z | Y^Z = X | Z^X = Y
  Eigen::Matrix<Scalar, 3, 1> v_line_x (line_x.template tail<3> ()),
                              v_line_y (line_y.template tail<3> ()),
                              v_line_z (v_line_x.cross (v_line_y));

  // Check vectors norms
  if (v_line_x.norm () < 1 - norm_limit || v_line_x.norm () > 1 + norm_limit)
  {
    PCL_DEBUG ("checkCoordinateSystem: line_x norm %d != 1\n", v_line_x.norm ());
    return (false);
  }

  if (v_line_y.norm () < 1 - norm_limit || v_line_y.norm () > 1 + norm_limit)
  {
    PCL_DEBUG ("checkCoordinateSystem: line_y norm %d != 1\n", v_line_y.norm ());
    return (false);
  }

  if (v_line_z.norm () < 1 - norm_limit || v_line_z.norm () > 1 + norm_limit)
  {
    PCL_DEBUG ("checkCoordinateSystem: line_z norm %d != 1\n", v_line_z.norm ());
    return (false);
  }

  // Check vectors perendicularity
  if (std::abs (v_line_x.dot (v_line_y)) > dot_limit)
  {
    PCL_DEBUG ("checkCSAxis: line_x dot line_y %e =  > %e\n", v_line_x.dot (v_line_y), dot_limit);
    return (false);
  }

  if (std::abs (v_line_x.dot (v_line_z)) > dot_limit)
  {
    PCL_DEBUG ("checkCSAxis: line_x dot line_z = %e > %e\n", v_line_x.dot (v_line_z), dot_limit);
    return (false);
  }

  if (std::abs (v_line_y.dot (v_line_z)) > dot_limit)
  {
    PCL_DEBUG ("checkCSAxis: line_y dot line_z = %e > %e\n", v_line_y.dot (v_line_z), dot_limit);
    return (false);
  }

  return (true);
}


template <typename Scalar> bool
transformBetween2CoordinateSystems (const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> from_line_x,
                                    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> from_line_y,
                                    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> to_line_x,
                                    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> to_line_y,
                                    Eigen::Transform<Scalar, 3, Eigen::Affine> &transformation)
{
  if (from_line_x.innerSize () != 6 || from_line_y.innerSize () != 6 || to_line_x.innerSize () != 6 || to_line_y.innerSize () != 6)
  {
    PCL_DEBUG ("transformBetween2CoordinateSystems: lines size != 6\n");
    return (false);
  }

  // Check if coordinate systems are valid
  if (!pcl::checkCoordinateSystem (from_line_x, from_line_y) || !pcl::checkCoordinateSystem (to_line_x, to_line_y))
  {
    PCL_DEBUG ("transformBetween2CoordinateSystems: coordinate systems invalid !\n");
    return (false);
  }

  // Convert lines into Vector3 :
  Eigen::Matrix<Scalar, 3, 1> fr0 (from_line_x.template head<3>()),
                              fr1 (from_line_x.template head<3>() + from_line_x.template tail<3>()),
                              fr2 (from_line_y.template head<3>() + from_line_y.template tail<3>()),

                              to0 (to_line_x.template head<3>()),
                              to1 (to_line_x.template head<3>() + to_line_x.template tail<3>()),
                              to2 (to_line_y.template head<3>() + to_line_y.template tail<3>());

  // Code is inspired from http://stackoverflow.com/a/15277421/1816078
  // Define matrices and points :
  Eigen::Transform<Scalar, 3, Eigen::Affine> T2, T3 = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity ();
  Eigen::Matrix<Scalar, 3, 1> x1, y1, z1, x2, y2, z2;

  // Axes of the coordinate system "fr"
  x1 = (fr1 - fr0).normalized ();  // the versor (unitary vector) of the (fr1-fr0) axis vector
  y1 = (fr2 - fr0).normalized ();

  // Axes of the coordinate system "to"
  x2 = (to1 - to0).normalized ();
  y2 = (to2 - to0).normalized ();

  // Transform from CS1 to CS2
  // Note: if fr0 == (0,0,0) --> CS1 == CS2 --> T2 = Identity
  T2.linear () << x1, y1, x1.cross (y1);

  // Transform from CS1 to CS3
  T3.linear () << x2, y2, x2.cross (y2);

  // Identity matrix = transform to CS2 to CS3
  // Note: if CS1 == CS2 --> transformation = T3
  transformation = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity ();
  transformation.linear () = T3.linear () * T2.linear ().inverse ();
  transformation.translation () = to0 - (transformation.linear () * fr0);
  return (true);
}

} // namespace pcl


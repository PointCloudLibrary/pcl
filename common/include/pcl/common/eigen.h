/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (C) 2010 Gael Guennebaud <gael.guennebaud@inria.fr>
 *  Copyright (C) 2009 Hauke Heibel <hauke.heibel@gmail.com>
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
 * $Id$
 *
 */
#ifndef PCL_COMMON_EIGEN_H_
#define PCL_COMMON_EIGEN_H_

#ifndef NOMINMAX
#define NOMINMAX
#endif

#if defined __GNUC__
#  pragma GCC system_header
#elif defined __SUNPRO_CC
#  pragma disable_warn
#endif

#include <cmath>

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace pcl
{
  /** \brief Compute the roots of a quadratic polynom x^2 + b*x + c = 0
    * \param[in] b linear parameter
    * \param[in] c constant parameter
    * \param[out] roots solutions of x^2 + b*x + c = 0
    */
  template <typename Scalar, typename Roots> void
  computeRoots2 (const Scalar &b, const Scalar &c, Roots &roots);

  /** \brief computes the roots of the characteristic polynomial of the input matrix m, which are the eigenvalues
    * \param[in] m input matrix
    * \param[out] roots roots of the characteristic polynomial of the input matrix m, which are the eigenvalues
    */
  template <typename Matrix, typename Roots> void
  computeRoots (const Matrix &m, Roots &roots);

  /** \brief determine the smallest eigenvalue and its corresponding eigenvector
    * \param[in] mat input matrix that needs to be symmetric and positive semi definite
    * \param[out] eigenvalue the smallest eigenvalue of the input matrix
    * \param[out] eigenvector the corresponding eigenvector to the smallest eigenvalue of the input matrix
    * \ingroup common
    */
  template <typename Matrix, typename Vector> void
  eigen22 (const Matrix &mat, typename Matrix::Scalar &eigenvalue, Vector &eigenvector);

  /** \brief determine the smallest eigenvalue and its corresponding eigenvector
    * \param[in] mat input matrix that needs to be symmetric and positive semi definite
    * \param[out] eigenvectors the corresponding eigenvector to the smallest eigenvalue of the input matrix
    * \param[out] eigenvalues the smallest eigenvalue of the input matrix
    * \ingroup common
    */
  template <typename Matrix, typename Vector> void
  eigen22 (const Matrix &mat, Matrix &eigenvectors, Vector &eigenvalues);

  /** \brief determines the corresponding eigenvector to the given eigenvalue of the symmetric positive semi definite input matrix
    * \param[in] mat symmetric positive semi definite input matrix
    * \param[in] eigenvalue the eigenvalue which corresponding eigenvector is to be computed
    * \param[out] eigenvector the corresponding eigenvector for the input eigenvalue
    * \ingroup common
    */
  template <typename Matrix, typename Vector> void
  computeCorrespondingEigenVector (const Matrix &mat, const typename Matrix::Scalar &eigenvalue, Vector &eigenvector);
  
  /** \brief determines the eigenvector and eigenvalue of the smallest eigenvalue of the symmetric positive semi definite input matrix
    * \param[in] mat symmetric positive semi definite input matrix
    * \param[out] eigenvalue smallest eigenvalue of the input matrix
    * \param[out] eigenvector the corresponding eigenvector for the input eigenvalue
    * \note if the smallest eigenvalue is not unique, this function may return any eigenvector that is consistent to the eigenvalue.
    * \ingroup common
    */
  template <typename Matrix, typename Vector> void
  eigen33 (const Matrix &mat, typename Matrix::Scalar &eigenvalue, Vector &eigenvector);

  /** \brief determines the eigenvalues of the symmetric positive semi definite input matrix
    * \param[in] mat symmetric positive semi definite input matrix
    * \param[out] evals resulting eigenvalues in ascending order
    * \ingroup common
    */
  template <typename Matrix, typename Vector> void
  eigen33 (const Matrix &mat, Vector &evals);

  /** \brief determines the eigenvalues and corresponding eigenvectors of the symmetric positive semi definite input matrix
    * \param[in] mat symmetric positive semi definite input matrix
    * \param[out] evecs resulting eigenvalues in ascending order
    * \param[out] evals corresponding eigenvectors in correct order according to eigenvalues
    * \ingroup common
    */
  template <typename Matrix, typename Vector> void
  eigen33 (const Matrix &mat, Matrix &evecs, Vector &evals);

  /** \brief Calculate the inverse of a 2x2 matrix
    * \param[in] matrix matrix to be inverted
    * \param[out] inverse the resultant inverted matrix
    * \note only the upper triangular part is taken into account => non symmetric matrices will give wrong results
    * \return determinant of the original matrix => if 0 no inverse exists => result is invalid
    * \ingroup common
    */
  template <typename Matrix> typename Matrix::Scalar
  invert2x2 (const Matrix &matrix, Matrix &inverse);

  /** \brief Calculate the inverse of a 3x3 symmetric matrix.
    * \param[in] matrix matrix to be inverted
    * \param[out] inverse the resultant inverted matrix
    * \note only the upper triangular part is taken into account => non symmetric matrices will give wrong results
    * \return determinant of the original matrix => if 0 no inverse exists => result is invalid
    * \ingroup common
    */
  template <typename Matrix> typename Matrix::Scalar
  invert3x3SymMatrix (const Matrix &matrix, Matrix &inverse);

  /** \brief Calculate the inverse of a general 3x3 matrix.
    * \param[in] matrix matrix to be inverted
    * \param[out] inverse the resultant inverted matrix
    * \return determinant of the original matrix => if 0 no inverse exists => result is invalid
    * \ingroup common
    */
  template <typename Matrix> typename Matrix::Scalar
  invert3x3Matrix (const Matrix &matrix, Matrix &inverse);

  template<typename Matrix> inline typename Matrix::Scalar
  determinant3x3Matrix (const Matrix& matrix)
  {
    // result is independent of Row/Col Major storage!
    return matrix.coeff (0) * (matrix.coeff (4) * matrix.coeff (8) - matrix.coeff (5) * matrix.coeff (7)) +
           matrix.coeff (1) * (matrix.coeff (5) * matrix.coeff (6) - matrix.coeff (3) * matrix.coeff (8)) +
           matrix.coeff (2) * (matrix.coeff (3) * matrix.coeff (7) - matrix.coeff (4) * matrix.coeff (6)) ;
  }
  
  /** \brief Get the unique 3D rotation that will rotate \a z_axis into (0,0,1) and \a y_direction into a vector
    * with x=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis)
    * \param[in] z_axis the z-axis
    * \param[in] y_direction the y direction
    * \param[out] transformation the resultant 3D rotation
    * \ingroup common
    */
  inline void
  getTransFromUnitVectorsZY (const Eigen::Vector3f& z_axis, 
                             const Eigen::Vector3f& y_direction,
                             Eigen::Affine3f& transformation);

  /** \brief Get the unique 3D rotation that will rotate \a z_axis into (0,0,1) and \a y_direction into a vector
    * with x=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis)
    * \param[in] z_axis the z-axis
    * \param[in] y_direction the y direction
    * \return the resultant 3D rotation
    * \ingroup common
    */
  inline Eigen::Affine3f
  getTransFromUnitVectorsZY (const Eigen::Vector3f& z_axis, 
                             const Eigen::Vector3f& y_direction);

  /** \brief Get the unique 3D rotation that will rotate \a x_axis into (1,0,0) and \a y_direction into a vector
    * with z=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis)
    * \param[in] x_axis the x-axis
    * \param[in] y_direction the y direction
    * \param[out] transformation the resultant 3D rotation
    * \ingroup common
    */
  inline void
  getTransFromUnitVectorsXY (const Eigen::Vector3f& x_axis, 
                             const Eigen::Vector3f& y_direction,
                             Eigen::Affine3f& transformation);

  /** \brief Get the unique 3D rotation that will rotate \a x_axis into (1,0,0) and \a y_direction into a vector
    * with z=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis)
    * \param[in] x_axis the x-axis
    * \param[in] y_direction the y direction
    * \return the resulting 3D rotation
    * \ingroup common
    */
  inline Eigen::Affine3f
  getTransFromUnitVectorsXY (const Eigen::Vector3f& x_axis, 
                             const Eigen::Vector3f& y_direction);

  /** \brief Get the unique 3D rotation that will rotate \a z_axis into (0,0,1) and \a y_direction into a vector
    * with x=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis)
    * \param[in] y_direction the y direction
    * \param[in] z_axis the z-axis
    * \param[out] transformation the resultant 3D rotation
    * \ingroup common
    */
  inline void
  getTransformationFromTwoUnitVectors (const Eigen::Vector3f& y_direction, 
                                       const Eigen::Vector3f& z_axis,
                                       Eigen::Affine3f& transformation);

  /** \brief Get the unique 3D rotation that will rotate \a z_axis into (0,0,1) and \a y_direction into a vector
    * with x=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis)
    * \param[in] y_direction the y direction
    * \param[in] z_axis the z-axis
    * \return transformation the resultant 3D rotation
    * \ingroup common
    */
  inline Eigen::Affine3f
  getTransformationFromTwoUnitVectors (const Eigen::Vector3f& y_direction, 
                                       const Eigen::Vector3f& z_axis);

  /** \brief Get the transformation that will translate \a orign to (0,0,0) and rotate \a z_axis into (0,0,1)
    * and \a y_direction into a vector with x=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis)
    * \param[in] y_direction the y direction
    * \param[in] z_axis the z-axis
    * \param[in] origin the origin
    * \param[in] transformation the resultant transformation matrix
    * \ingroup common
    */
  inline void
  getTransformationFromTwoUnitVectorsAndOrigin (const Eigen::Vector3f& y_direction, 
                                                const Eigen::Vector3f& z_axis,
                                                const Eigen::Vector3f& origin, 
                                                Eigen::Affine3f& transformation);

  /** \brief Extract the Euler angles (XYZ-convention) from the given transformation
    * \param[in] t the input transformation matrix
    * \param[in] roll the resulting roll angle
    * \param[in] pitch the resulting pitch angle
    * \param[in] yaw the resulting yaw angle
    * \ingroup common
    */
  inline void
  getEulerAngles (const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw);

  /** Extract x,y,z and the Euler angles (XYZ-convention) from the given transformation
    * \param[in] t the input transformation matrix
    * \param[out] x the resulting x translation
    * \param[out] y the resulting y translation
    * \param[out] z the resulting z translation
    * \param[out] roll the resulting roll angle
    * \param[out] pitch the resulting pitch angle
    * \param[out] yaw the resulting yaw angle
    * \ingroup common
    */
  inline void
  getTranslationAndEulerAngles (const Eigen::Affine3f& t, 
                                float& x, float& y, float& z,
                                float& roll, float& pitch, float& yaw);

  /** \brief Create a transformation from the given translation and Euler angles (XYZ-convention)
    * \param[in] x the input x translation
    * \param[in] y the input y translation
    * \param[in] z the input z translation
    * \param[in] roll the input roll angle
    * \param[in] pitch the input pitch angle
    * \param[in] yaw the input yaw angle
    * \param[out] t the resulting transformation matrix
    * \ingroup common
    */
  template <typename Scalar> inline void
  getTransformation (Scalar x, Scalar y, Scalar z, Scalar roll, Scalar pitch, Scalar yaw, 
                     Eigen::Transform<Scalar, 3, Eigen::Affine> &t);

  inline void
  getTransformation (float x, float y, float z, float roll, float pitch, float yaw, 
                     Eigen::Affine3f &t)
  {
    return (getTransformation<float> (x, y, z, roll, pitch, yaw, t));
  }

  inline void
  getTransformation (double x, double y, double z, double roll, double pitch, double yaw, 
                     Eigen::Affine3d &t)
  {
    return (getTransformation<double> (x, y, z, roll, pitch, yaw, t));
  }

  /** \brief Create a transformation from the given translation and Euler angles (XYZ-convention)
    * \param[in] x the input x translation
    * \param[in] y the input y translation
    * \param[in] z the input z translation
    * \param[in] roll the input roll angle
    * \param[in] pitch the input pitch angle
    * \param[in] yaw the input yaw angle
    * \return the resulting transformation matrix
    * \ingroup common
    */
  inline Eigen::Affine3f
  getTransformation (float x, float y, float z, float roll, float pitch, float yaw);

  /** \brief Write a matrix to an output stream
    * \param[in] matrix the matrix to output
    * \param[out] file the output stream
    * \ingroup common
    */
  template <typename Derived> void
  saveBinary (const Eigen::MatrixBase<Derived>& matrix, std::ostream& file);

  /** \brief Read a matrix from an input stream
    * \param[out] matrix the resulting matrix, read from the input stream
    * \param[in,out] file the input stream
    * \ingroup common
    */
  template <typename Derived> void
  loadBinary (Eigen::MatrixBase<Derived> const& matrix, std::istream& file);

// PCL_EIGEN_SIZE_MIN_PREFER_DYNAMIC gives the min between compile-time sizes. 0 has absolute priority, followed by 1,
// followed by Dynamic, followed by other finite values. The reason for giving Dynamic the priority over
// finite values is that min(3, Dynamic) should be Dynamic, since that could be anything between 0 and 3.
#define PCL_EIGEN_SIZE_MIN_PREFER_DYNAMIC(a,b) ((int (a) == 0 || int (b) == 0) ? 0 \
                           : (int (a) == 1 || int (b) == 1) ? 1 \
                           : (int (a) == Eigen::Dynamic || int (b) == Eigen::Dynamic) ? Eigen::Dynamic \
                           : (int (a) <= int (b)) ? int (a) : int (b))

  /** \brief Returns the transformation between two point sets. 
    * The algorithm is based on: 
    * "Least-squares estimation of transformation parameters between two point patterns",
    * Shinji Umeyama, PAMI 1991, DOI: 10.1109/34.88573
    *
    * It estimates parameters \f$ c, \mathbf{R}, \f$ and \f$ \mathbf{t} \f$ such that
    * \f{align*}
    *   \frac{1}{n} \sum_{i=1}^n \vert\vert y_i - (c\mathbf{R}x_i + \mathbf{t}) \vert\vert_2^2
    * \f}
    * is minimized.
    *
    * The algorithm is based on the analysis of the covariance matrix
    * \f$ \Sigma_{\mathbf{x}\mathbf{y}} \in \mathbb{R}^{d \times d} \f$
    * of the input point sets \f$ \mathbf{x} \f$ and \f$ \mathbf{y} \f$ where
    * \f$d\f$ is corresponding to the dimension (which is typically small).
    * The analysis is involving the SVD having a complexity of \f$O(d^3)\f$
    * though the actual computational effort lies in the covariance
    * matrix computation which has an asymptotic lower bound of \f$O(dm)\f$ when
    * the input point sets have dimension \f$d \times m\f$.
    *
    * \param[in] src Source points \f$ \mathbf{x} = \left( x_1, \hdots, x_n \right) \f$
    * \param[in] dst Destination points \f$ \mathbf{y} = \left( y_1, \hdots, y_n \right) \f$.
    * \param[in] with_scaling Sets \f$ c=1 \f$ when <code>false</code> is passed. (default: false)
    * \return The homogeneous transformation 
    * \f{align*}
    *   T = \begin{bmatrix} c\mathbf{R} & \mathbf{t} \\ \mathbf{0} & 1 \end{bmatrix}
    * \f}
    * minimizing the resudiual above. This transformation is always returned as an
    * Eigen::Matrix.
    */
  template <typename Derived, typename OtherDerived> 
  typename Eigen::internal::umeyama_transform_matrix_type<Derived, OtherDerived>::type
  umeyama (const Eigen::MatrixBase<Derived>& src, const Eigen::MatrixBase<OtherDerived>& dst, bool with_scaling = false);
}

#include <pcl/common/impl/eigen.hpp>

#if defined __SUNPRO_CC
#  pragma enable_warn
#endif

#endif  //PCL_COMMON_EIGEN_H_

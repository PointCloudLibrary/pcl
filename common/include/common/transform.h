/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */

#ifndef PCL_TRANSFORM_H_
#define PCL_TRANSFORM_H_

#include <Eigen/Geometry>

/*
 *  Define some libeigen based transformation methods
 */

namespace pcl
{
  /** \brief Get the unique 3D rotation that will rotate \a z_axis into (0,0,1) and \a y_direction into a vector 
    * with x=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis) 
    * \param z_axis the z-axis
    * \param y_direction the y direction
    * \param transformation the resultant 3D rotation
    */
  inline void
  getTransFromUnitVectorsZY (const Eigen::Vector3f& z_axis, const Eigen::Vector3f& y_direction,
                             Eigen::Affine3f& transformation);
  
  /** \brief Get the unique 3D rotation that will rotate \a z_axis into (0,0,1) and \a y_direction into a vector 
    * with x=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis) 
    * \param z_axis the z-axis
    * \param y_direction the y direction
    * \return the resultant 3D rotation
    */
  inline Eigen::Affine3f
  getTransFromUnitVectorsZY (const Eigen::Vector3f& z_axis, const Eigen::Vector3f& y_direction);
 
  /** \brief Get the unique 3D rotation that will rotate \a x_axis into (1,0,0) and \a y_direction into a vector 
    * with z=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis)
    * \param x_axis the x-axis
    * \param y_direction the y direction
    * \param transformation the resultant 3D rotation
    */
  inline void
  getTransFromUnitVectorsXY (const Eigen::Vector3f& x_axis, const Eigen::Vector3f& y_direction,
                             Eigen::Affine3f& transformation);

  /** \brief Get the unique 3D rotation that will rotate \a x_axis into (1,0,0) and \a y_direction into a vector 
    * with z=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis)
    * \param x_axis the x-axis
    * \param y_direction the y direction
    * \return the resulting 3D rotation
    */
  inline Eigen::Affine3f
  getTransFromUnitVectorsXY (const Eigen::Vector3f& x_axis, const Eigen::Vector3f& y_direction);
  
  /** Same as getTransFromUnitVectorsZY - for downwards compatibility */
  inline void
  getTransformationFromTwoUnitVectors (const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis,
                                       Eigen::Affine3f& transformation);
  
  /** Same as getTransFromUnitVectorsZY - for downwards compatibility */
  inline Eigen::Affine3f
  getTransformationFromTwoUnitVectors (const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis);

  /** \brief Get the transformation that will translate \a orign to (0,0,0) and rotate \a z_axis into (0,0,1)
    * and \a y_direction into a vector with x=0 (or into (0,1,0) should \a y_direction be orthogonal to \a z_axis) 
    * \param y_direction the y direction
    * \param z_axis the z-axis
    * \param origin the origin
    * \param transformation the resultant transformation matrix
    */
  inline void
  getTransformationFromTwoUnitVectorsAndOrigin (const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis,
                                                const Eigen::Vector3f& origin, Eigen::Affine3f& transformation);

  /** \brief Output operator for Tranform3f
    * \param os the output stream
    * \param m the affine transformation to output
    */
  inline std::ostream&
  operator<< (std::ostream& os, const Eigen::Affine3f& m)
  {
    os << "{(" << m (0,0) << "," <<m (0,1) << "," << m (0,2) << "," << m (0,3)
       << ")(" << m (1,0) << "," <<m (1,1) << "," << m (1,2) << "," << m (1,3)
       << ")(" << m (2,0) << "," <<m (2,1) << "," << m (2,2) << "," << m (2,3)
       << ")(" << m (3,0) << "," <<m (3,1) << "," << m (3,2) << "," << m (3,3) << ")}";
    return (os);
  }

  /** \brief Get only the rotation part of the transformation 
    * \param transformation the input transformation matrix 
    * \return the resulting 3D rotation matrix
    */
  inline Eigen::Affine3f
  getRotation (const Eigen::Affine3f& transformation);

  /** \brief Get only the translation part of the transformation 
    * \param transformation the input transformation matrix
    * \return the resulting translation matrix
    */
  inline Eigen::Vector3f
  getTranslation (const Eigen::Affine3f& transformation);

  /** \brief Get the inverse of an Eigen::Affine3f object 
    * \param transformation the input transformation matrix
    * \param inverse_transformation the resultant inverse of \a transformation
    */
  inline void
  getInverse (const Eigen::Affine3f& transformation, Eigen::Affine3f& inverse_transformation);

  /** \brief Get the inverse of an Eigen::Affine3f object 
    * \param transformation the input transformation matrix
    * \return the resulting inverse of \a transformation
    */
  inline Eigen::Affine3f getInverse (const Eigen::Affine3f& transformation);

  /** \brief Transform a point with members x,y,z
    * \param transformation the transformation to apply
    * \param point the point to transform
    * \return the transformed point
    */
  template <typename PointType> inline PointType 
  transformXYZ (const Eigen::Affine3f& transformation, const PointType& point);
  
  /** \brief Transform each point in the given point cloud according to the given transformation
    * \param input the input point cloud
    * \param transformation the transformation matrix to apply
    * \param output the resulting transformed point cloud
    */
  template <typename PointCloudType> inline void
  getTransformedPointCloud (const PointCloudType& input, const Eigen::Affine3f& transformation,
                            PointCloudType& output);
  
  /** \brief Extract the Euler angles (XYZ-convention) from the given transformation
    * \param t the input transformation matrix
    * \param roll the resulting roll angle
    * \param pitch the resulting pitch angle
    * \param yaw the resulting yaw angle
    */
  inline void
  getEulerAngles (const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw);

  /** Extract x,y,z and the Euler angles (XYZ-convention) from the given transformation
    * \param t the input transformation matrix
    * \param x the resulting x translation
    * \param y the resulting y translation
    * \param z the resulting z translation
    * \param roll the resulting roll angle
    * \param pitch the resulting pitch angle
    * \param yaw the resulting yaw angle
    */
  inline void
  getTranslationAndEulerAngles (const Eigen::Affine3f& t, float& x, float& y, float& z,
                                float& roll, float& pitch, float& yaw);

  /** \brief Create a transformation from the given translation and Euler angles (XYZ-convention)
    * \param x the input x translation
    * \param y the input y translation
    * \param z the input z translation
    * \param roll the input roll angle
    * \param pitch the input pitch angle
    * \param yaw the input yaw angle
    * \param t the resulting transformation matrix
    */
  inline void
  getTransformation (float x, float y, float z, float roll, float pitch, float yaw, Eigen::Affine3f& t);

  /** \brief Create a transformation from the given translation and Euler angles (XYZ-convention)
    * \param x the input x translation
    * \param y the input y translation
    * \param z the input z translation
    * \param roll the input roll angle
    * \param pitch the input pitch angle
    * \param yaw the input yaw angle
    * \return the resulting transformation matrix
    */
  inline Eigen::Affine3f
  getTransformation (float x, float y, float z, float roll, float pitch, float yaw);
  
  /** \brief Write a matrix to an output stream
    * \param matrix the matrix to output
    * \param file the output stream
    */
  template <typename Derived> void
  saveBinary (const Eigen::MatrixBase<Derived>& matrix, std::ostream& file);

  /** \brief Read a matrix from an input stream
   * \param matrix the resulting matrix, read from the input stream
   * \param file the input stream
   */
  template <typename Derived> void
  loadBinary (Eigen::MatrixBase<Derived>& matrix, std::istream& file);
  
}  // namespasce end

#include "pcl/common/transform.hpp"

#endif  //#ifndef PCL_NORMS_H_

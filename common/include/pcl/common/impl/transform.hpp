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

#include "pcl/pcl_macros.h"

namespace pcl
{

void getTransFromUnitVectorsZY(const Eigen::Vector3f& z_axis, const Eigen::Vector3f& y_direction, Eigen::Affine3f& transformation)
{
  Eigen::Vector3f tmp0 = (y_direction.cross(z_axis)).normalized();
  Eigen::Vector3f tmp1 = (z_axis.cross(tmp0)).normalized();
  Eigen::Vector3f tmp2 = z_axis.normalized();
  
  transformation(0,0)=tmp0[0]; transformation(0,1)=tmp0[1]; transformation(0,2)=tmp0[2]; transformation(0,3)=0.0f;
  transformation(1,0)=tmp1[0]; transformation(1,1)=tmp1[1]; transformation(1,2)=tmp1[2]; transformation(1,3)=0.0f;
  transformation(2,0)=tmp2[0]; transformation(2,1)=tmp2[1]; transformation(2,2)=tmp2[2]; transformation(2,3)=0.0f;
  transformation(3,0)=0.0f;    transformation(3,1)=0.0f;    transformation(3,2)=0.0f;    transformation(3,3)=1.0f;
}

Eigen::Affine3f getTransFromUnitVectorsZY(const Eigen::Vector3f& z_axis, const Eigen::Vector3f& y_direction)
{
  Eigen::Affine3f transformation;
  getTransFromUnitVectorsZY(z_axis, y_direction, transformation);
  return transformation;
}

void getTransFromUnitVectorsXY(const Eigen::Vector3f& x_axis, const Eigen::Vector3f& y_direction, Eigen::Affine3f& transformation)
{
  Eigen::Vector3f tmp2 = (x_axis.cross(y_direction)).normalized();
  Eigen::Vector3f tmp1 = (tmp2.cross(x_axis)).normalized();
  Eigen::Vector3f tmp0 = x_axis.normalized();
  
  transformation(0,0)=tmp0[0]; transformation(0,1)=tmp0[1]; transformation(0,2)=tmp0[2]; transformation(0,3)=0.0f;
  transformation(1,0)=tmp1[0]; transformation(1,1)=tmp1[1]; transformation(1,2)=tmp1[2]; transformation(1,3)=0.0f;
  transformation(2,0)=tmp2[0]; transformation(2,1)=tmp2[1]; transformation(2,2)=tmp2[2]; transformation(2,3)=0.0f;
  transformation(3,0)=0.0f;    transformation(3,1)=0.0f;    transformation(3,2)=0.0f;    transformation(3,3)=1.0f;
}

Eigen::Affine3f getTransFromUnitVectorsXY(const Eigen::Vector3f& x_axis, const Eigen::Vector3f& y_direction)
{
  Eigen::Affine3f transformation;
  getTransFromUnitVectorsXY(x_axis, y_direction, transformation);
  return transformation;
}

void getTransformationFromTwoUnitVectors(const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis, Eigen::Affine3f& transformation)
{
  getTransFromUnitVectorsZY(z_axis, y_direction, transformation);
}

Eigen::Affine3f getTransformationFromTwoUnitVectors(const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis)
{
  Eigen::Affine3f transformation;
  getTransformationFromTwoUnitVectors(y_direction, z_axis, transformation);
  return transformation;
}

void getTransformationFromTwoUnitVectorsAndOrigin(const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis,
                                                  const Eigen::Vector3f& origin, Eigen::Affine3f& transformation)
{
  getTransformationFromTwoUnitVectors(y_direction, z_axis, transformation);
  Eigen::Vector3f translation = transformation*origin;
  transformation(0,3)=-translation[0];  transformation(1,3)=-translation[1];  transformation(2,3)=-translation[2];
}

inline Eigen::Affine3f getRotationOnly(const Eigen::Affine3f& transformation)
{
  Eigen::Affine3f ret = transformation;
  ret(0,3) = ret(1,3) = ret(2,3) = 0.0f;
  return ret;
}

inline Eigen::Vector3f getTranslation(const Eigen::Affine3f& transformation)
{
  return Eigen::Vector3f(transformation(0,3), transformation(1,3), transformation(2,3));
}

inline void getInverse(const Eigen::Affine3f& transformation, Eigen::Affine3f& inverse_transformation)
{
  inverse_transformation(0,0) = transformation(0,0);
  inverse_transformation(0,1) = transformation(1,0);
  inverse_transformation(0,2) = transformation(2,0);
  inverse_transformation(0,3) = -transformation(0,0)*transformation(0,3) - transformation(1,0)*transformation(1,3) - transformation(2,0)*transformation(2,3);
  inverse_transformation(1,0) = transformation(0,1);
  inverse_transformation(1,1) = transformation(1,1);
  inverse_transformation(1,2) = transformation(2,1);
  inverse_transformation(1,3) = -transformation(0,1)*transformation(0,3) - transformation(1,1)*transformation(1,3) - transformation(2,1)*transformation(2,3);
  inverse_transformation(2,0) = transformation(0,2);
  inverse_transformation(2,1) = transformation(1,2);
  inverse_transformation(2,2) = transformation(2,2);
  inverse_transformation(2,3) = -transformation(0,2)*transformation(0,3) - transformation(1,2)*transformation(1,3) - transformation(2,2)*transformation(2,3);
  inverse_transformation(3,0) = 0.0;
  inverse_transformation(3,1) = 0.0;
  inverse_transformation(3,2) = 0.0;
  inverse_transformation(3,3) = 1.0;
}

inline Eigen::Affine3f getInverse(const Eigen::Affine3f& transformation)
{
  Eigen::Affine3f inverse_transformation;
  getInverse(transformation, inverse_transformation);
  return inverse_transformation;
}

template <typename PointType>
inline PointType transformXYZ(const Eigen::Affine3f& tranformation, const PointType& point)
{
  PointType ret = point;
  ret.getVector3fMap() = tranformation * point.getVector3fMap();
  return ret;
}

template <typename PointCloudType>
void getTransformedPointCloud(const PointCloudType& input, const Eigen::Affine3f& transformation, PointCloudType& output)
{
  output = input;
  for (unsigned int i=0; i<output.points.size(); ++i)
    output.points[i].getVector3fMap() = transformation*input.points[i].getVector3fMap();
}

void getEulerAngles(const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw)
{
  roll  = atan2f(t(2,1), t(2,2));
  pitch = asinf(-t(2,0));
  yaw   = atan2f(t(1,0), t(0,0));
}

void getTranslationAndEulerAngles(const Eigen::Affine3f& t, float& x, float& y, float& z, float& roll, float& pitch, float& yaw)
{
  x = t(0,3);
  y = t(1,3);
  z = t(2,3);
  roll  = atan2f(t(2,1), t(2,2));
  pitch = asinf(-t(2,0));
  yaw   = atan2f(t(1,0), t(0,0));
}

void getTransformation(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Affine3f& t)
{
  float A=cosf(yaw),  B=sinf(yaw),  C=cosf(pitch), D=sinf(pitch),
        E=cosf(roll), F=sinf(roll), DE=D*E,        DF=D*F;
  t(0,0) = A*C;  t(0,1) = A*DF - B*E;  t(0,2) = B*F + A*DE;  t(0,3) = x;
  t(1,0) = B*C;  t(1,1) = A*E + B*DF;  t(1,2) = B*DE - A*F;  t(1,3) = y;
  t(2,0) = -D;   t(2,1) = C*F;         t(2,2) = C*E;         t(2,3) = z;
  t(3,0) = 0;    t(3,1) = 0;           t(3,2) = 0;           t(3,3) = 1;
}

Eigen::Affine3f getTransformation(float x, float y, float z, float roll, float pitch, float yaw)
{
  Eigen::Affine3f t;
  getTransformation(x, y, z, roll, pitch, yaw, t);
  return t;
}

template <typename Derived>
void saveBinary(const Eigen::MatrixBase<Derived>& matrix, std::ostream& file)
{
  uint32_t rows=matrix.rows(), cols=matrix.cols();
  file.write((char*) &rows, sizeof(rows));
  file.write((char*) &cols, sizeof(cols));
  for (uint32_t i=0; i<rows; ++i)
    for (uint32_t j=0; j<cols; ++j)
    {
      typename Derived::Scalar tmp = matrix(i,j);
      file.write((char*) &tmp, sizeof(tmp));
    }
}

template <typename Derived>
void loadBinary(Eigen::MatrixBase<Derived>& matrix, std::istream& file)
{
  uint32_t rows, cols;
  file.read((char*) &rows, sizeof(rows));
  file.read((char*) &cols, sizeof(cols));
  //std::cout << rows <<" rows and "<<cols<<" cols.\n";
  if (matrix.rows()!=(int)rows || matrix.cols()!=(int)cols)
  {
    //std::cerr << __PRETTY_FUNCTION__ << ": matrix size does not fit!\n";
    matrix.resize(rows, cols);
  }
  
  for (uint32_t i=0; i<rows; ++i)
    for (uint32_t j=0; j<cols; ++j)
    {
      typename Derived::Scalar tmp;
      file.read((char*) &tmp, sizeof(tmp));
      matrix(i,j) = tmp;
    }
}

}  // namespace end

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#pragma once

#include <Eigen/Core> // for Matrix
#include <Eigen/Geometry> // for AngleAxis
#include <pcl/point_types.h>

#define AUX_PI_FLOAT            3.14159265358979323846f
#define AUX_HALF_PI             1.57079632679489661923f
#define AUX_DEG_TO_RADIANS     (3.14159265358979323846f/180.0f)

namespace pcl
{
  namespace recognition
  {
    namespace aux
    {
      template<typename T> bool
      compareOrderedPairs (const std::pair<T,T>& a, const std::pair<T,T>& b)
      {
        if ( a.first == b.first )
          return a.second < b.second;

        return a.first < b.first;
      }

      template<typename T> T
      sqr (T a)
      {
        return (a*a);
      }

      template<typename T> T
      clamp (T value, T min, T max)
      {
        if ( value < min )
          return min;
        if ( value > max )
          return max;

        return value;
      }

      /** \brief Expands the destination bounding box 'dst' such that it contains 'src'. */
      template<typename T> void
      expandBoundingBox (T dst[6], const T src[6])
      {
        if ( src[0] < dst[0] ) dst[0] = src[0];
        if ( src[2] < dst[2] ) dst[2] = src[2];
        if ( src[4] < dst[4] ) dst[4] = src[4];

        if ( src[1] > dst[1] ) dst[1] = src[1];
        if ( src[3] > dst[3] ) dst[3] = src[3];
        if ( src[5] > dst[5] ) dst[5] = src[5];
      }

      /** \brief Expands the bounding box 'bbox' such that it contains the point 'p'. */
      template<typename T> void
      expandBoundingBoxToContainPoint (T bbox[6], const T p[3])
      {
        if      ( p[0] < bbox[0] ) bbox[0] = p[0];
        else if ( p[0] > bbox[1] ) bbox[1] = p[0];

        if      ( p[1] < bbox[2] ) bbox[2] = p[1];
        else if ( p[1] > bbox[3] ) bbox[3] = p[1];

        if      ( p[2] < bbox[4] ) bbox[4] = p[2];
        else if ( p[2] > bbox[5] ) bbox[5] = p[2];
      }

      /** \brief v[0] = v[1] = v[2] = value */
      template <typename T> void
      set3 (T v[3], T value)
      {
        v[0] = v[1] = v[2] = value;
      }

      /** \brief dst = src */
      template <typename T> void
      copy3 (const T src[3], T dst[3])
      {
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
      }

      /** \brief dst = src */
      template <typename T> void
      copy3 (const T src[3], pcl::PointXYZ& dst)
      {
        dst.x = src[0];
        dst.y = src[1];
        dst.z = src[2];
      }

      /** \brief a = -a */
      template <typename T> void
      flip3 (T a[3])
      {
        a[0] = -a[0];
        a[1] = -a[1];
        a[2] = -a[2];
      }
	  
      /** \brief a = b */
      template <typename T> bool
      equal3 (const T a[3], const T b[3])
      {
        return (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]);
      }
	 
      /** \brief a += b */
      template <typename T> void
      add3 (T a[3], const T b[3])
      {
        a[0] += b[0];
        a[1] += b[1];
        a[2] += b[2];
      }

      /** \brief c = a + b */
      template <typename T> void
      sum3 (const T a[3], const T b[3], T c[3])
      {
        c[0] = a[0] + b[0];
        c[1] = a[1] + b[1];
        c[2] = a[2] + b[2];
      }

      /** \brief c = a - b */
      template <typename T> void
      diff3 (const T a[3], const T b[3], T c[3])
      {
        c[0] = a[0] - b[0];
        c[1] = a[1] - b[1];
        c[2] = a[2] - b[2];
      }

      template <typename T> void
      cross3 (const T v1[3], const T v2[3], T out[3])
      {
        out[0] = v1[1]*v2[2] - v1[2]*v2[1];
        out[1] = v1[2]*v2[0] - v1[0]*v2[2];
        out[2] = v1[0]*v2[1] - v1[1]*v2[0];
      }

      /** \brief Returns the length of v. */
      template <typename T> T
      length3 (const T v[3])
      {
        return (std::sqrt (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
      }

      /** \brief Returns the Euclidean distance between a and b. */
      template <typename T> T
      distance3 (const T a[3], const T b[3])
      {
        T l[3] = {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
        return (length3 (l));
      }

      /** \brief Returns the squared Euclidean distance between a and b. */
      template <typename T> T
      sqrDistance3 (const T a[3], const T b[3])
      {
        return (aux::sqr (a[0]-b[0]) + aux::sqr (a[1]-b[1]) + aux::sqr (a[2]-b[2]));
      }

      /** \brief Returns the dot product a*b */
      template <typename T> T
      dot3 (const T a[3], const T b[3])
      {
        return (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]);
      }

      /** \brief Returns the dot product (x, y, z)*(u, v, w) = x*u + y*v + z*w */
      template <typename T> T
      dot3 (T x, T y, T z, T u, T v, T w)
      {
        return (x*u + y*v + z*w);
      }

      /** \brief v = scalar*v. */
      template <typename T> void
      mult3 (T* v, T scalar)
      {
        v[0] *= scalar;
        v[1] *= scalar;
        v[2] *= scalar;
      }

      /** \brief out = scalar*v. */
      template <typename T> void
      mult3 (const T* v, T scalar, T* out)
      {
        out[0] = v[0]*scalar;
        out[1] = v[1]*scalar;
        out[2] = v[2]*scalar;
      }

      /** \brief Normalize v */
      template <typename T> void
      normalize3 (T v[3])
      {
        T inv_len = (static_cast<T> (1.0))/aux::length3 (v);
        v[0] *= inv_len;
        v[1] *= inv_len;
        v[2] *= inv_len;
      }

      /** \brief Returns the square length of v. */
      template <typename T> T
      sqrLength3 (const T v[3])
      {
        return (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
      }

      /** Projects 'x' on the plane through 0 and with normal 'planeNormal' and saves the result in 'out'. */
      template <typename T> void
      projectOnPlane3 (const T x[3], const T planeNormal[3], T out[3])
      {
        T dot = aux::dot3 (planeNormal, x);
        // Project 'x' on the plane normal
        T nproj[3] = {-dot*planeNormal[0], -dot*planeNormal[1], -dot*planeNormal[2]};
        aux::sum3 (x, nproj, out);
      }

      /** \brief Sets 'm' to the 3x3 identity matrix. */
      template <typename T> void
      identity3x3 (T m[9])
      {
        m[0] = m[4] = m[8] = 1.0;
        m[1] = m[2] = m[3] = m[5] = m[6] = m[7] = 0.0;
      }

      /** \brief out = mat*v. 'm' is an 1D array of 9 elements treated as a 3x3 matrix (row major order). */
      template <typename T> void
      mult3x3(const T m[9], const T v[3], T out[3])
      {
      	out[0] = v[0]*m[0] + v[1]*m[1] + v[2]*m[2];
      	out[1] = v[0]*m[3] + v[1]*m[4] + v[2]*m[5];
      	out[2] = v[0]*m[6] + v[1]*m[7] + v[2]*m[8];
      }

      /** Let x, y, z be the columns of the matrix a = [x|y|z]. The method computes out = a*m.
        * Note that 'out' is a 1D array of 9 elements and the resulting matrix is stored in row
        * major order, i.e., the first matrix row is (out[0] out[1] out[2]), the second
        * (out[3] out[4] out[5]) and the third (out[6] out[7] out[8]). */
      template <typename T> void
      mult3x3 (const T x[3], const T y[3], const T z[3], const T m[3][3], T out[9])
      {
        out[0] = x[0]*m[0][0] + y[0]*m[1][0] + z[0]*m[2][0];
        out[1] = x[0]*m[0][1] + y[0]*m[1][1] + z[0]*m[2][1];
        out[2] = x[0]*m[0][2] + y[0]*m[1][2] + z[0]*m[2][2];

        out[3] = x[1]*m[0][0] + y[1]*m[1][0] + z[1]*m[2][0];
        out[4] = x[1]*m[0][1] + y[1]*m[1][1] + z[1]*m[2][1];
        out[5] = x[1]*m[0][2] + y[1]*m[1][2] + z[1]*m[2][2];

        out[6] = x[2]*m[0][0] + y[2]*m[1][0] + z[2]*m[2][0];
        out[7] = x[2]*m[0][1] + y[2]*m[1][1] + z[2]*m[2][1];
        out[8] = x[2]*m[0][2] + y[2]*m[1][2] + z[2]*m[2][2];
      }

      /** \brief The first 9 elements of 't' are treated as a 3x3 matrix (row major order) and the last 3 as a translation.
        * First, 'p' is multiplied by that matrix and then translated. The result is saved in 'out'. */
      template<class T> void
      transform(const T t[12], const T p[3], T out[3])
      {
        out[0] = t[0]*p[0] + t[1]*p[1] + t[2]*p[2] + t[9];
        out[1] = t[3]*p[0] + t[4]*p[1] + t[5]*p[2] + t[10];
        out[2] = t[6]*p[0] + t[7]*p[1] + t[8]*p[2] + t[11];
      }

      /** \brief The first 9 elements of 't' are treated as a 3x3 matrix (row major order) and the last 3 as a translation.
        * First, (x, y, z) is multiplied by that matrix and then translated. The result is saved in 'out'. */
      template<class T> void
      transform(const T t[12], T x, T y, T z, T out[3])
      {
        out[0] = t[0]*x + t[1]*y + t[2]*z + t[9];
        out[1] = t[3]*x + t[4]*y + t[5]*z + t[10];
        out[2] = t[6]*x + t[7]*y + t[8]*z + t[11];
      }

      /** \brief Compute out = (upper left 3x3 of mat)*p + last column of mat. */
      template<class T> void
      transform(const Eigen::Matrix<T,4,4>& mat, const pcl::PointXYZ& p, pcl::PointXYZ& out)
      {
        out.x = mat(0,0)*p.x + mat(0,1)*p.y + mat(0,2)*p.z + mat(0,3);
        out.y = mat(1,0)*p.x + mat(1,1)*p.y + mat(1,2)*p.z + mat(1,3);
        out.z = mat(2,0)*p.x + mat(2,1)*p.y + mat(2,2)*p.z + mat(2,3);
      }

      /** \brief The first 9 elements of 't' are treated as a 3x3 matrix (row major order) and the last 3 as a translation.
        * First, 'p' is multiplied by that matrix and then translated. The result is saved in 'out'. */
      template<class T> void
      transform(const T t[12], const pcl::PointXYZ& p, T out[3])
      {
        out[0] = t[0]*p.x + t[1]*p.y + t[2]*p.z + t[9];
        out[1] = t[3]*p.x + t[4]*p.y + t[5]*p.z + t[10];
        out[2] = t[6]*p.x + t[7]*p.y + t[8]*p.z + t[11];
      }

      /** \brief Returns true if the points 'p1' and 'p2' are co-planar and false otherwise. The method assumes that 'n1'
        * is a normal at 'p1' and 'n2' is a normal at 'p2'. 'max_angle' is the threshold used for the test. The bigger
        * the value the larger the deviation between the normals can be which still leads to a positive test result. The
        * angle has to be in radians. */
      template<typename T> bool
      pointsAreCoplanar (const T p1[3], const T n1[3], const T p2[3], const T n2[3], T max_angle)
      {
        // Compute the angle between 'n1' and 'n2' and compare it with 'max_angle'
        if ( std::acos (aux::clamp (aux::dot3 (n1, n2), -1.0f, 1.0f)) > max_angle )
          return (false);

        T cl[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
        aux::normalize3 (cl);

        // Compute the angle between 'cl' and 'n1'
        T tmp_angle = std::acos (aux::clamp (aux::dot3 (n1, cl), -1.0f, 1.0f));

        // 'tmp_angle' should not deviate too much from 90 degrees
        if ( std::fabs (tmp_angle - AUX_HALF_PI) > max_angle )
          return (false);

        // All tests passed => the points are coplanar
        return (true);
      }

      template<typename Scalar> void
      array12ToMatrix4x4 (const Scalar src[12], Eigen::Matrix<Scalar, 4, 4>& dst)
      {
        dst(0,0) = src[0]; dst(0,1) = src[1];  dst(0,2) = src[2]; dst(0,3) = src[9];
        dst(1,0) = src[3]; dst(1,1) = src[4];  dst(1,2) = src[5]; dst(1,3) = src[10];
        dst(2,0) = src[6]; dst(2,1) = src[7];  dst(2,2) = src[8]; dst(2,3) = src[11];
        dst(3,0) =         dst(3,1) =          dst(3,2) = 0.0;    dst(3,3) = 1.0;
      }

      template<typename Scalar> void
      matrix4x4ToArray12 (const Eigen::Matrix<Scalar, 4, 4>& src, Scalar dst[12])
      {
        dst[0] = src(0,0); dst[1] = src(0,1); dst[2] = src(0,2); dst[9]  = src(0,3);
        dst[3] = src(1,0); dst[4] = src(1,1); dst[5] = src(1,2); dst[10] = src(1,3);
        dst[6] = src(2,0); dst[7] = src(2,1); dst[8] = src(2,2); dst[11] = src(2,3);
      }

      /** \brief The method copies the input array 'src' to the eigen matrix 'dst' in row major order.
        * dst[0] = src(0,0); dst[1] = src(0,1); dst[2] = src(0,2);
        * dst[3] = src(1,0); dst[4] = src(1,1); dst[5] = src(1,2);
        * dst[6] = src(2,0); dst[7] = src(2,1); dst[8] = src(2,2);
        * */
      template <typename T> void
      eigenMatrix3x3ToArray9RowMajor (const Eigen::Matrix<T,3,3>& src, T dst[9])
      {
        dst[0] = src(0,0); dst[1] = src(0,1); dst[2] = src(0,2);
        dst[3] = src(1,0); dst[4] = src(1,1); dst[5] = src(1,2);
        dst[6] = src(2,0); dst[7] = src(2,1); dst[8] = src(2,2);
      }

      /** \brief The method copies the input array 'src' to the eigen matrix 'dst' in row major order.
        * dst(0,0) = src[0]; dst(0,1) = src[1]; dst(0,2) = src[2];
        * dst(1,0) = src[3]; dst(1,1) = src[4]; dst(1,2) = src[5];
        * dst(2,0) = src[6]; dst(2,1) = src[7]; dst(2,2) = src[8];
        * */
      template <typename T> void
      toEigenMatrix3x3RowMajor (const T src[9], Eigen::Matrix<T,3,3>& dst)
      {
        dst(0,0) = src[0]; dst(0,1) = src[1]; dst(0,2) = src[2];
        dst(1,0) = src[3]; dst(1,1) = src[4]; dst(1,2) = src[5];
        dst(2,0) = src[6]; dst(2,1) = src[7]; dst(2,2) = src[8];
      }

      /** brief Computes a rotation matrix from the provided input vector 'axis_angle'. The direction of 'axis_angle' is the rotation axis
        * and its magnitude is the angle of rotation about that axis. 'rotation_matrix' is the output rotation matrix saved in row major order. */
      template <typename T> void
      axisAngleToRotationMatrix (const T axis_angle[3], T rotation_matrix[9])
      {
        // Get the angle of rotation
        T angle = aux::length3 (axis_angle);
        if ( angle == 0.0 )
        {
          // Undefined rotation -> set to identity
          aux::identity3x3 (rotation_matrix);
          return;
        }

        // Normalize the input
        T normalized_axis_angle[3];
        aux::mult3 (axis_angle, static_cast<T> (1.0)/angle, normalized_axis_angle);

        // The eigen objects
        Eigen::Matrix<T,3,1> mat_axis(normalized_axis_angle);
        Eigen::AngleAxis<T> eigen_angle_axis (angle, mat_axis);

        // Save the output
        aux::eigenMatrix3x3ToArray9RowMajor (eigen_angle_axis.toRotationMatrix (), rotation_matrix);
      }

      /** brief Extracts the angle-axis representation from 'rotation_matrix', i.e., computes a rotation 'axis' and an 'angle'
        * of rotation about that axis from the provided input. The output 'angle' is in the range [0, pi] and 'axis' is normalized. */
      template <typename T> void
      rotationMatrixToAxisAngle (const T rotation_matrix[9], T axis[3], T& angle)
      {
        // The eigen objects
        Eigen::AngleAxis<T> angle_axis;
        Eigen::Matrix<T,3,3> rot_mat;
        // Copy the input matrix to the eigen matrix in row major order
        aux::toEigenMatrix3x3RowMajor (rotation_matrix, rot_mat);

        // Do the computation
        angle_axis.fromRotationMatrix (rot_mat);

        // Save the result
        axis[0] = angle_axis.axis () (0,0);
        axis[1] = angle_axis.axis () (1,0);
        axis[2] = angle_axis.axis () (2,0);
        angle = angle_axis.angle ();

        // Make sure that 'angle' is in the range [0, pi]
        if ( angle > AUX_PI_FLOAT )
        {
          angle = 2.0f*AUX_PI_FLOAT - angle;
          aux::flip3 (axis);
        }
      }
    } // namespace aux
  } // namespace recognition
} // namespace pcl

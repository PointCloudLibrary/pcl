/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2018-, Open Perception, Inc.
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
 *
 */

#include <gtest/gtest.h>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include <pcl/pcl_tests.h>

using namespace pcl;

using TransformTypes = ::testing::Types
        <Eigen::Transform<float, 3, Eigen::Affine>,
         Eigen::Transform<double, 3, Eigen::Affine>,
         Eigen::Matrix<float, 4, 4>,
         Eigen::Matrix<double, 4,4> >;

template <typename Transform>
class Transforms : public ::testing::Test
{
 public:
  using Scalar = typename Transform::Scalar;

  Transforms ()
  : ABS_ERROR (std::numeric_limits<Scalar>::epsilon () * 10)
  , CLOUD_SIZE (100)
  {
    Eigen::Matrix<Scalar, 6, 1> r = Eigen::Matrix<Scalar, 6, 1>::Random ();
    Eigen::Transform<Scalar, 3, Eigen::Affine> transform;
    pcl::getTransformation (r[0], r[1], r[2], r[3], r[4], r[5], transform);
    tf = transform.matrix ();

    p_xyz_normal.resize (CLOUD_SIZE);
    p_xyz_normal_trans.resize (CLOUD_SIZE);
    for (size_t i = 0; i < CLOUD_SIZE; ++i)
    {
      Eigen::Vector3f xyz = Eigen::Vector3f::Random ();
      Eigen::Vector3f normal = Eigen::Vector3f::Random ().normalized ();
      p_xyz_normal[i].getVector3fMap () = xyz;
      p_xyz_normal_trans[i].getVector3fMap () = (transform * xyz.template cast<typename Transform::Scalar> ()).template cast<float> ();
      p_xyz_normal[i].getNormalVector3fMap () = normal;
      p_xyz_normal_trans[i].getNormalVector3fMap () = (transform.rotation () * normal.template cast<typename Transform::Scalar> ()).template cast<float> ();
      p_xyz_normal[i].rgb = Eigen::Matrix<float, 1, 1>::Random ()[0];
      p_xyz_normal_trans[i].rgb = p_xyz_normal[i].rgb;
    }

    pcl::copyPointCloud(p_xyz_normal, p_xyz);
    pcl::copyPointCloud(p_xyz_normal_trans, p_xyz_trans);

    indices.resize (CLOUD_SIZE / 2);
    for (size_t i = 0; i < indices.size(); ++i)
      indices[i] = i * 2;
  }

  const Scalar ABS_ERROR;
  const size_t CLOUD_SIZE;

  Transform tf;

  // Random point clouds and their expected transformed versions
  pcl::PointCloud<pcl::PointXYZ> p_xyz, p_xyz_trans;
  pcl::PointCloud<pcl::PointXYZRGBNormal> p_xyz_normal, p_xyz_normal_trans;

  // Indices, every second point
  std::vector<int> indices;

  PCL_MAKE_ALIGNED_OPERATOR_NEW;
};

TYPED_TEST_CASE (Transforms, TransformTypes);

TYPED_TEST (Transforms, PointCloudXYZDense)
{
  pcl::PointCloud<pcl::PointXYZ> p;
  pcl::transformPointCloud (this->p_xyz, p, this->tf);
  ASSERT_METADATA_EQ (p, this->p_xyz);
  ASSERT_EQ (p.size (), this->p_xyz.size ());
  for (size_t i = 0; i < p.size (); ++i)
    ASSERT_XYZ_NEAR (p[i], this->p_xyz_trans[i], this->ABS_ERROR);
}

TYPED_TEST (Transforms, PointCloudXYZDenseIndexed)
{
  pcl::PointCloud<pcl::PointXYZ> p;
  pcl::transformPointCloud (this->p_xyz, this->indices, p, this->tf);
  ASSERT_EQ (p.size (), this->indices.size ());
  ASSERT_EQ (p.width, this->indices.size ());
  ASSERT_EQ (p.height, 1);
  for (size_t i = 0; i < p.size (); ++i)
    ASSERT_XYZ_NEAR (p[i], this->p_xyz_trans[i * 2], this->ABS_ERROR);
}

TYPED_TEST (Transforms, PointCloudXYZSparse)
{
  // Make first point infinite and point cloud not dense
  this->p_xyz.is_dense = false;
  this->p_xyz[0].x = std::numeric_limits<float>::quiet_NaN();

  pcl::PointCloud<pcl::PointXYZ> p;
  pcl::transformPointCloud (this->p_xyz, p, this->tf);
  ASSERT_METADATA_EQ (p, this->p_xyz);
  ASSERT_EQ (p.size (), this->p_xyz.size ());
  ASSERT_FALSE (pcl::isFinite (p[0]));
  for (size_t i = 1; i < p.size (); ++i)
  {
    ASSERT_TRUE (pcl::isFinite (p[i]));
    ASSERT_XYZ_NEAR (p[i], this->p_xyz_trans[i], this->ABS_ERROR);
  }
}

TYPED_TEST (Transforms, PointCloudXYZRGBNormalDense)
{
  // Copy all fields
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p;
    pcl::transformPointCloudWithNormals (this->p_xyz_normal, p, this->tf, true);
    ASSERT_METADATA_EQ (p, this->p_xyz_normal);
    ASSERT_EQ (p.size (), this->p_xyz_normal.size ());
    for (size_t i = 0; i < p.size (); ++i)
    {
      ASSERT_XYZ_NEAR (p[i], this->p_xyz_normal_trans[i], this->ABS_ERROR);
      ASSERT_NORMAL_NEAR (p[i], this->p_xyz_normal_trans[i], this->ABS_ERROR);
      ASSERT_RGBA_EQ (p[i], this->p_xyz_normal_trans[i]);
    }
  }
  // Do not copy all fields
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p;
    pcl::transformPointCloudWithNormals (this->p_xyz_normal, p, this->tf, false);
    ASSERT_METADATA_EQ (p, this->p_xyz_normal);
    ASSERT_EQ (p.size (), this->p_xyz_normal.size ());
    for (size_t i = 0; i < p.size (); ++i)
    {
      ASSERT_XYZ_NEAR (p[i], this->p_xyz_normal_trans[i], this->ABS_ERROR);
      ASSERT_NORMAL_NEAR (p[i], this->p_xyz_normal_trans[i], this->ABS_ERROR);
      ASSERT_NE (p[i].rgba, this->p_xyz_normal_trans[i].rgba);
    }
  }
}

TYPED_TEST (Transforms, PointCloudXYZRGBNormalDenseIndexed)
{
  // Copy all fields
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p;
    pcl::transformPointCloudWithNormals (this->p_xyz_normal, this->indices, p, this->tf, true);
    ASSERT_EQ (p.size (), this->indices.size ());
    ASSERT_EQ (p.width, this->indices.size ());
    ASSERT_EQ (p.height, 1);
    for (size_t i = 0; i < p.size (); ++i)
    {
      ASSERT_XYZ_NEAR (p[i], this->p_xyz_normal_trans[i * 2], this->ABS_ERROR);
      ASSERT_NORMAL_NEAR (p[i], this->p_xyz_normal_trans[i * 2], this->ABS_ERROR);
      ASSERT_RGBA_EQ (p[i], this->p_xyz_normal_trans[i * 2]);
    }
  }
  // Do not copy all fields
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p;
    pcl::transformPointCloudWithNormals (this->p_xyz_normal, this->indices, p, this->tf, false);
    ASSERT_EQ (p.size (), this->indices.size ());
    ASSERT_EQ (p.width, this->indices.size ());
    ASSERT_EQ (p.height, 1);
    for (size_t i = 0; i < p.size (); ++i)
    {
      ASSERT_XYZ_NEAR (p[i], this->p_xyz_normal_trans[i * 2], this->ABS_ERROR);
      ASSERT_NORMAL_NEAR (p[i], this->p_xyz_normal_trans[i * 2], this->ABS_ERROR);
      ASSERT_NE (p[i].rgba, this->p_xyz_normal_trans[i * 2].rgba);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Matrix4Affine3Transform)
{
  float rot_x = 2.8827f;
  float rot_y = -0.31190f;
  float rot_z = -0.93058f;
  Eigen::Affine3f affine;
  pcl::getTransformation (0.1f, 0.2f, 0.3f, rot_x, rot_y, rot_z, affine);
  Eigen::Matrix4f transformation = affine.matrix ();

  PointXYZ p (1.f, 2.f, 3.f);
  Eigen::Vector3f v3 = p.getVector3fMap ();
  Eigen::Vector4f v4 = p.getVector4fMap ();

  Eigen::Vector3f v3t (affine * v3);
  Eigen::Vector4f v4t (transformation * v4);

  PointXYZ pt = pcl::transformPoint (p, affine);

  EXPECT_NEAR (pt.x, v3t.x (), 1e-4); EXPECT_NEAR (pt.x, v4t.x (), 1e-4);
  EXPECT_NEAR (pt.y, v3t.y (), 1e-4); EXPECT_NEAR (pt.y, v4t.y (), 1e-4);
  EXPECT_NEAR (pt.z, v3t.z (), 1e-4); EXPECT_NEAR (pt.z, v4t.z (), 1e-4);

  PointNormal pn;
  pn.getVector3fMap () = p.getVector3fMap ();
  pn.getNormalVector3fMap () = Eigen::Vector3f (0.60f, 0.48f, 0.64f);
  Eigen::Vector3f n3 = pn.getNormalVector3fMap ();
  Eigen::Vector4f n4 = pn.getNormalVector4fMap ();

  Eigen::Vector3f n3t (affine.rotation() * n3);
  Eigen::Vector4f n4t (transformation * n4);

  PointNormal pnt = pcl::transformPointWithNormal (pn, affine);

  EXPECT_NEAR (pnt.x, v3t.x (), 1e-4); EXPECT_NEAR (pnt.x, v4t.x (), 1e-4);
  EXPECT_NEAR (pnt.y, v3t.y (), 1e-4); EXPECT_NEAR (pnt.y, v4t.y (), 1e-4);
  EXPECT_NEAR (pnt.z, v3t.z (), 1e-4); EXPECT_NEAR (pnt.z, v4t.z (), 1e-4);
  EXPECT_NEAR (pnt.normal_x, n3t.x (), 1e-4); EXPECT_NEAR (pnt.normal_x, n4t.x (), 1e-4);
  EXPECT_NEAR (pnt.normal_y, n3t.y (), 1e-4); EXPECT_NEAR (pnt.normal_y, n4t.y (), 1e-4);
  EXPECT_NEAR (pnt.normal_z, n3t.z (), 1e-4); EXPECT_NEAR (pnt.normal_z, n4t.z (), 1e-4);

  PointCloud<PointXYZ> c, ct;
  c.push_back (p);
  pcl::transformPointCloud (c, ct, affine);
  EXPECT_NEAR (pt.x, ct[0].x, 1e-4);
  EXPECT_NEAR (pt.y, ct[0].y, 1e-4);
  EXPECT_NEAR (pt.z, ct[0].z, 1e-4);

  pcl::transformPointCloud (c, ct, transformation);
  EXPECT_NEAR (pt.x, ct[0].x, 1e-4);
  EXPECT_NEAR (pt.y, ct[0].y, 1e-4);
  EXPECT_NEAR (pt.z, ct[0].z, 1e-4);

  affine = transformation;

  std::vector<int> indices (1); indices[0] = 0;

  pcl::transformPointCloud (c, indices, ct, affine);
  EXPECT_NEAR (pt.x, ct[0].x, 1e-4);
  EXPECT_NEAR (pt.y, ct[0].y, 1e-4);
  EXPECT_NEAR (pt.z, ct[0].z, 1e-4);

  pcl::transformPointCloud (c, indices, ct, transformation);
  EXPECT_NEAR (pt.x, ct[0].x, 1e-4);
  EXPECT_NEAR (pt.y, ct[0].y, 1e-4);
  EXPECT_NEAR (pt.z, ct[0].z, 1e-4);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

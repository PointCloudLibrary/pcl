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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl/pcl_tests.h>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Transform)
{
  PointCloud<PointXYZ> cloud (4, 1);
  cloud[0].getVector3fMap () << 0, 0, 0;
  cloud[1].getVector3fMap () << 1, 0, 0;
  cloud[2].getVector3fMap () << 0, 1, 0;
  cloud[3].getVector3fMap () << 0, 0, 1;

  Eigen::Vector3f offset (100, 0, 0);
  float angle = M_PI_4;
  Eigen::Quaternionf rotation (cos (angle / 2), 0, 0, sin (angle / 2));

  PointCloud<PointXYZ> cloud_out;
  transformPointCloud (cloud, cloud_out, offset, rotation);

  const float rho = M_SQRT2 / 2.0f;  // cos(PI/4) == sin(PI/4)

  EXPECT_EQ (cloud.size (), cloud_out.size ());
  EXPECT_EQ (100, cloud_out[0].x);
  EXPECT_EQ (0, cloud_out[0].y);
  EXPECT_EQ (0, cloud_out[0].z);
  EXPECT_NEAR (100+rho, cloud_out[1].x,  1e-4);
  EXPECT_NEAR (rho, cloud_out[1].y,  1e-4);
  EXPECT_EQ (0, cloud_out[1].z);
  EXPECT_NEAR (100-rho, cloud_out[2].x,  1e-4);
  EXPECT_NEAR (rho, cloud_out[2].y,  1e-4);
  EXPECT_EQ (0, cloud_out[2].z);
  EXPECT_EQ (100, cloud_out[3].x);
  EXPECT_EQ (0, cloud_out[3].y);
  EXPECT_EQ (1, cloud_out[3].z);

  PointCloud<PointXYZ> cloud_out2;
  Eigen::Translation3f translation (offset);
  Eigen::Affine3f transform;
  transform = translation * rotation;
  transformPointCloud (cloud, cloud_out2, transform);

  EXPECT_EQ (cloud.size (), cloud_out2.size ());
  EXPECT_EQ (100, cloud_out2[0].x);
  EXPECT_EQ (0, cloud_out2[0].y);
  EXPECT_EQ (0, cloud_out2[0].z);
  EXPECT_NEAR (100+rho, cloud_out2[1].x,  1e-4);
  EXPECT_NEAR (rho, cloud_out2[1].y,  1e-4);
  EXPECT_EQ (0, cloud_out2[1].z);
  EXPECT_NEAR (100-rho, cloud_out2[2].x,  1e-4);
  EXPECT_NEAR (rho, cloud_out2[2].y,  1e-4);
  EXPECT_EQ (0, cloud_out2[2].z);
  EXPECT_EQ (100, cloud_out2[3].x);
  EXPECT_EQ (0, cloud_out2[3].y);
  EXPECT_EQ (1, cloud_out2[3].z);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, TransformCopyFields)
{
  Eigen::Affine3f transform;
  transform = Eigen::Translation3f (100, 0, 0);

  PointXYZRGBNormal empty_point;
  std::vector<int> indices (1);

  PointCloud<PointXYZRGBNormal> cloud (2, 1);
  cloud[0].rgba = 0xFF0000;
  cloud[1].rgba = 0x00FF00;

  // Preserve data in all fields
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloud (cloud, cloud_out, transform, true);
    ASSERT_EQ (cloud.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (cloud[0], cloud_out[0]);
    EXPECT_RGBA_EQ (cloud[1], cloud_out[1]);
  }
  // Preserve data in all fields (with indices)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloud (cloud, indices, cloud_out, transform, true);
    ASSERT_EQ (indices.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (cloud[0], cloud_out[0]);
  }
  // Do not preserve data in all fields
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloud (cloud, cloud_out, transform, false);
    ASSERT_EQ (cloud.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (empty_point, cloud_out[0]);
    EXPECT_RGBA_EQ (empty_point, cloud_out[1]);
  }
  // Do not preserve data in all fields (with indices)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloud (cloud, indices, cloud_out, transform, false);
    ASSERT_EQ (indices.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (empty_point, cloud_out[0]);
  }
  // Preserve data in all fields (with normals version)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloudWithNormals (cloud, cloud_out, transform, true);
    ASSERT_EQ (cloud.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (cloud[0], cloud_out[0]);
    EXPECT_RGBA_EQ (cloud[1], cloud_out[1]);
  }
  // Preserve data in all fields (with normals and indices version)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloudWithNormals (cloud, indices, cloud_out, transform, true);
    ASSERT_EQ (indices.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (cloud[0], cloud_out[0]);
  }
  // Do not preserve data in all fields (with normals version)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloudWithNormals (cloud, cloud_out, transform, false);
    ASSERT_EQ (cloud.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (empty_point, cloud_out[0]);
    EXPECT_RGBA_EQ (empty_point, cloud_out[1]);
  }
  // Do not preserve data in all fields (with normals and indices version)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloudWithNormals (cloud, indices, cloud_out, transform, false);
    ASSERT_EQ (indices.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (empty_point, cloud_out[0]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Matrix4Affine3Transform)
{
  float rot_x = 2.8827f;
  float rot_y = -0.31190f;
  float rot_z = -0.93058f;
  Eigen::Affine3f affine;
  pcl::getTransformation (0, 0, 0, rot_x, rot_y, rot_z, affine);

  EXPECT_NEAR (affine (0, 0),  0.56854731f, 1e-4); EXPECT_NEAR (affine (0, 1), -0.82217032f, 1e-4); EXPECT_NEAR (affine (0, 2), -0.028107658f, 1e-4);
  EXPECT_NEAR (affine (1, 0), -0.76327348f, 1e-4); EXPECT_NEAR (affine (1, 1), -0.51445758f, 1e-4); EXPECT_NEAR (affine (1, 2), -0.39082864f, 1e-4);
  EXPECT_NEAR (affine (2, 0),  0.30686751f, 1e-4); EXPECT_NEAR (affine (2, 1),  0.24365838f, 1e-4); EXPECT_NEAR (affine (2, 2), -0.920034f, 1e-4);

  // Approximative!!! Uses SVD internally! See http://eigen.tuxfamily.org/dox/Transform_8h_source.html
  Eigen::Matrix3f rotation = affine.rotation ();

  EXPECT_NEAR (rotation (0, 0),  0.56854731f, 1e-4); EXPECT_NEAR (rotation (0, 1), -0.82217032f, 1e-4); EXPECT_NEAR (rotation (0, 2), -0.028107658f, 1e-4);
  EXPECT_NEAR (rotation (1, 0), -0.76327348f, 1e-4); EXPECT_NEAR (rotation (1, 1), -0.51445758f, 1e-4); EXPECT_NEAR (rotation (1, 2), -0.39082864f, 1e-4);
  EXPECT_NEAR (rotation (2, 0),  0.30686751f, 1e-4); EXPECT_NEAR (rotation (2, 1),  0.24365838f, 1e-4); EXPECT_NEAR (rotation (2, 2), -0.920034f, 1e-4);

  float trans_x, trans_y, trans_z;
  pcl::getTransformation (0.1f, 0.2f, 0.3f, rot_x, rot_y, rot_z, affine);
  pcl::getTranslationAndEulerAngles (affine, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z);
  EXPECT_FLOAT_EQ (trans_x, 0.1f);
  EXPECT_FLOAT_EQ (trans_y, 0.2f);
  EXPECT_FLOAT_EQ (trans_z, 0.3f);
  EXPECT_FLOAT_EQ (rot_x, 2.8827f);
  EXPECT_FLOAT_EQ (rot_y, -0.31190f);
  EXPECT_FLOAT_EQ (rot_z, -0.93058f);

  Eigen::Matrix4f transformation (Eigen::Matrix4f::Identity ());
  transformation.block<3, 3> (0, 0) = affine.rotation ();
  transformation.block<3, 1> (0, 3) = affine.translation ();

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

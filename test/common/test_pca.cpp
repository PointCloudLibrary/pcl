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
 */

/** \author Nizar Sallem */

#include <pcl/test/gtest.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/pcl_tests.h>

using namespace pcl::test;

pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PCA<pcl::PointXYZ> pca;

TEST(PCA, projection)
{
  pcl::PointXYZ projected, reconstructed;
  for(const auto &point : cloud)
  {
    pca.project (point, projected);
    pca.reconstruct (projected, reconstructed);
    EXPECT_NEAR_VECTORS (reconstructed.getVector3fMap (), point.getVector3fMap (), 5e-4);
  }
}

TEST(PCA, copy_constructor)
{
  // Test copy constructor
  pcl::PCA<pcl::PointXYZ> pca_copy(pca);
  try
  {
    Eigen::Matrix3f eigen_vectors_copy = pca_copy.getEigenVectors ();
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors ();
    for(std::size_t i = 0; i < 3; ++i)
      for(std::size_t j = 0; j < 3; ++j)
        EXPECT_EQ (eigen_vectors (i,j), eigen_vectors_copy (i,j));
  }
  catch (pcl::InitFailedException &/*e*/)
  {
    std::cerr << "something wrong" << std::endl;
  }
}

TEST(PCA, cloud_projection)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_projected, cloud_reconstructed;
  try
  {
    pca.project (cloud, cloud_projected);
    EXPECT_EQ (cloud.size (), cloud_projected.size ());
    pca.reconstruct (cloud_projected, cloud_reconstructed);
    EXPECT_EQ (cloud_reconstructed.size (), cloud_projected.size ());
    for(std::size_t i = 0; i < cloud.size(); i++)
      EXPECT_NEAR_VECTORS (cloud[i].getVector3fMap (),
                           cloud_reconstructed[i].getVector3fMap (),
                           5e-4);
  }
  catch (pcl::InitFailedException &/*e*/)
  {
    std::cerr << "something wrong" << std::endl;
  }
}

int main (int argc, char** argv)
{
  cloud.width = 5;
  cloud.height = 4 ;
  cloud.is_dense = true;
  cloud.resize(20);
  cloud[0].x = 100;   cloud[0].y = 8;    cloud[0].z = 5;
  cloud[1].x = 228;   cloud[1].y = 21;   cloud[1].z = 2;
  cloud[2].x = 341;   cloud[2].y = 31;   cloud[2].z = 10;
  cloud[3].x = 472;   cloud[3].y = 40;   cloud[3].z = 15;
  cloud[4].x = 578;   cloud[4].y = 48;   cloud[4].z = 3;
  cloud[5].x = 699;   cloud[5].y = 60;   cloud[5].z = 12;
  cloud[6].x = 807;   cloud[6].y = 71;   cloud[6].z = 14;
  cloud[7].x = 929;   cloud[7].y = 79;   cloud[7].z = 16;
  cloud[8].x = 1040;  cloud[8].y = 92;   cloud[8].z = 18;
  cloud[9].x = 1160;  cloud[9].y = 101;  cloud[9].z = 38;
  cloud[10].x = 1262; cloud[10].y = 109; cloud[10].z = 28;
  cloud[11].x = 1376; cloud[11].y = 121; cloud[11].z = 32;
  cloud[12].x = 1499; cloud[12].y = 128; cloud[12].z = 35;
  cloud[13].x = 1620; cloud[13].y = 143; cloud[13].z = 28;
  cloud[14].x = 1722; cloud[14].y = 150; cloud[14].z = 30;
  cloud[15].x = 1833; cloud[15].y = 159; cloud[15].z = 15;
  cloud[16].x = 1948; cloud[16].y = 172; cloud[16].z = 12;
  cloud[17].x = 2077; cloud[17].y = 181; cloud[17].z = 33;
  cloud[18].x = 2282; cloud[18].y = 190; cloud[18].z = 23;
  cloud[19].x = 2999; cloud[19].y = 202; cloud[19].z = 29;  

  pca.setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

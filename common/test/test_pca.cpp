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
 *
 */
/** \author Nizar Sallem */

#include <gtest/gtest.h>
#include "pcl/common/pca.h"
#include "pcl/point_types.h"
TEST(PCL, pca)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 5;
  cloud.height = 4 ;
  cloud.is_dense = true;
  cloud.points.resize(20);
  cloud.points[0].x = 100;   cloud.points[0].y = 8;    cloud.points[0].z = 5;
  cloud.points[1].x = 228;   cloud.points[1].y = 21;   cloud.points[1].z = 2;
  cloud.points[2].x = 341;   cloud.points[2].y = 31;   cloud.points[2].z = 10;
  cloud.points[3].x = 472;   cloud.points[3].y = 40;   cloud.points[3].z = 15;
  cloud.points[4].x = 578;   cloud.points[4].y = 48;   cloud.points[4].z = 3;
  cloud.points[5].x = 699;   cloud.points[5].y = 60;   cloud.points[5].z = 12;
  cloud.points[6].x = 807;   cloud.points[6].y = 71;   cloud.points[6].z = 14;
  cloud.points[7].x = 929;   cloud.points[7].y = 79;   cloud.points[7].z = 16;
  cloud.points[8].x = 1040;  cloud.points[8].y = 92;   cloud.points[8].z = 18;
  cloud.points[9].x = 1160;  cloud.points[9].y = 101;  cloud.points[9].z = 38;
  cloud.points[10].x = 1262; cloud.points[10].y = 109; cloud.points[10].z = 28;
  cloud.points[11].x = 1376; cloud.points[11].y = 121; cloud.points[11].z = 32;
  cloud.points[12].x = 1499; cloud.points[12].y = 128; cloud.points[12].z = 35;
  cloud.points[13].x = 1620; cloud.points[13].y = 143; cloud.points[13].z = 28;
  cloud.points[14].x = 1722; cloud.points[14].y = 150; cloud.points[14].z = 30;
  cloud.points[15].x = 1833; cloud.points[15].y = 159; cloud.points[15].z = 15;
  cloud.points[16].x = 1948; cloud.points[16].y = 172; cloud.points[16].z = 12;
  cloud.points[17].x = 2077; cloud.points[17].y = 181; cloud.points[17].z = 33;
  cloud.points[18].x = 2282; cloud.points[18].y = 190; cloud.points[18].z = 23;
  cloud.points[19].x = 2999; cloud.points[19].y = 202; cloud.points[19].z = 29;  
  pcl::PCA<pcl::PointXYZ> pca(cloud);
  pcl::PointXYZ projected, reconstructed;
  for(size_t i = 0; i < cloud.size(); i++)
  {
    pca.project(cloud.points[i], projected);
    pca.reconstruct(projected, reconstructed);
    std::cout << (reconstructed.getVector3fMap() - cloud.points[i].getVector3fMap()).norm() << std::endl;
    EXPECT_NEAR ((reconstructed.getVector3fMap() - cloud.points[i].getVector3fMap()).norm(), 0.0f, 2e-4);
  }
}

int main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

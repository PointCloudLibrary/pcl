/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <pcl/test/gtest.h>

#include <pcl/pcl_tests.h>
#include <pcl/PolygonMesh.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>

using namespace pcl;
using namespace pcl::test;

TEST(PolygonMesh, concatenate_header)
{
    PolygonMesh test, dummy;

    dummy.header.stamp = 4;
    test.header.stamp = 5;

    EXPECT_TRUE(PolygonMesh::concatenate(test, dummy));
    EXPECT_EQ(5, test.header.stamp);

    dummy.header.stamp = 5;
    test.header.stamp = 4;

    EXPECT_TRUE(PolygonMesh::concatenate(test, dummy));
    EXPECT_EQ(5, test.header.stamp);
}

TEST(PolygonMesh, concatenate_cloud)
{
    PointCloud<PointXYZ> cloud_template;
    const std::size_t size = 10 * 480;

    cloud_template.width = 10;
    cloud_template.height = 480;
    for (std::uint32_t i = 0; i < size; ++i)
    {
        cloud_template.emplace_back(3.0f * static_cast<float>(i) + 0, 
                                           3.0f * static_cast<float> (i) + 1,
                                           3.0f * static_cast<float> (i) + 2);
    }

    PCLPointCloud2 cloud, test_cloud, dummy_cloud;
    toPCLPointCloud2(cloud_template, cloud);

    PolygonMesh test, dummy;
    test.cloud = dummy.cloud = cloud;
    test_cloud = dummy_cloud = cloud;

    EXPECT_EQ(PCLPointCloud2::concatenate(test_cloud, dummy_cloud),
              PolygonMesh::concatenate(test, dummy));
    EXPECT_EQ(test_cloud.data, test.cloud.data);
}

TEST(PolygonMesh, concatenate_vertices)
{
    const std::size_t size = 15;

    PolygonMesh test, dummy;
    // The algorithm works regardless of the organization.
    test.cloud.width = dummy.cloud.width = size;
    test.cloud.height = dummy.cloud.height = 1;

    for (std::size_t i = 0; i < size; ++i)
    {
        dummy.polygons.emplace_back();
        test.polygons.emplace_back();
        for (std::size_t j = 0; j < size; ++j)
        {
            dummy.polygons.back().vertices.emplace_back(j);
            test.polygons.back().vertices.emplace_back(j);
        }
    }

    EXPECT_TRUE(PolygonMesh::concatenate(test, dummy));
    EXPECT_EQ(2 * dummy.polygons.size(), test.polygons.size());

    const auto cloud_size = test.cloud.width * test.cloud.height;
    for (const auto& polygon : test.polygons)
      for (const auto& vertex : polygon.vertices)
        EXPECT_LT(vertex, cloud_size);

    pcl::Indices vertices(size);
    for (std::size_t i = 0; i < size; ++i) {
        vertices = dummy.polygons[i].vertices;
        EXPECT_EQ_VECTORS(vertices, test.polygons[i].vertices);
        for (auto& vertex : vertices) { vertex += size; }
        EXPECT_EQ_VECTORS(vertices, test.polygons[i + size].vertices);
    }
}

int
main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return (RUN_ALL_TESTS());
}

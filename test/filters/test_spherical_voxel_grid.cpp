/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2018, Open Perception, Inc.
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

#include <gtest/gtest.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/spherical_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);

TEST (SphericalVoxelGrid, SphericalVoxelGrid)
{
  for (int i = 0; i < input_cloud->points.size(); i++)
  {
    if (input_cloud->points[i].z > 0)
      input_cloud->points[i].intensity = 20;
    else
      input_cloud->points[i].intensity = 10;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::SphericalVoxelGrid<pcl::PointXYZI> voxel;

  voxel.setInputCloud(input_cloud);
  voxel.setLeafSize(1, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(0, 0, 0));
  voxel.filter(*output_cloud);

  EXPECT_EQ(int (output_cloud->points.size()), 8);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 8);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[0].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[1].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[2].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[3].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[4].x, -0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[4].y, -0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[4].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[4].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[5].x,  0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[5].y, -0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[5].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[5].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[6].x,  0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[6].y,  0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[6].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[6].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[7].x, -0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[7].y,  0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[7].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[7].intensity, 15, 0.001);

  /* Test not downsampling all data */
  voxel.setInputCloud(input_cloud);
  voxel.setLeafSize(1, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(0, 0, 0));
  voxel.setDownsampleAllData(false);
  voxel.filter(*output_cloud);

  EXPECT_EQ(int (output_cloud->points.size()), 8);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 8);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.4, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[4].x, -0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[4].y, -0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[4].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[5].x,  0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[5].y, -0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[5].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[6].x,  0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[6].y,  0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[6].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[7].x, -0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[7].y,  0.9, 0.001);
  EXPECT_NEAR(output_cloud->points[7].z,    0, 0.001);

  /* Test different leaf size */
  voxel.setInputCloud(input_cloud);
  voxel.setLeafSize(2, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(0, 0, 0));
  voxel.setDownsampleAllData(true);
  voxel.filter(*output_cloud);

  EXPECT_EQ(int (output_cloud->points.size()), 4);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 4);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[0].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[1].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[2].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[3].intensity, 15, 0.001);


  /* Test origin movement */

  pcl::PointCloud<pcl::PointXYZI>::Ptr translated_input (new pcl::PointCloud<pcl::PointXYZI>);
  *translated_input = *input_cloud;

  for (int i = 0; i < translated_input->points.size(); i++)
  {
    translated_input->points[i].x += 5;
    translated_input->points[i].y += 6;
    translated_input->points[i].z += 7;
  }

  voxel.setInputCloud(translated_input);
  voxel.setLeafSize(2, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(5, 6, 7));
  voxel.setDownsampleAllData(true);
  voxel.filter(*output_cloud);

  for (int i = 0; i < output_cloud->points.size(); i++)
  {
    output_cloud->points[i].x -= 5;
    output_cloud->points[i].y -= 6;
    output_cloud->points[i].z -= 7;
  }

  EXPECT_EQ(int (output_cloud->points.size()), 4);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 4);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[0].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[1].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[2].intensity, 15, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[3].intensity, 15, 0.001);


  /* Test data filter */
  voxel.setInputCloud(input_cloud);
  voxel.setLeafSize(2, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(0, 0, 0));
  voxel.setFilterFieldName("intensity");
  voxel.setFilterLimits(15, 50);
  voxel.setDownsampleAllData(true);
  voxel.filter(*output_cloud);

  EXPECT_EQ(int (output_cloud->points.size()), 4);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 4);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    0.919238, 0.001);
  EXPECT_NEAR(output_cloud->points[0].intensity, 20, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    0.919238, 0.001);
  EXPECT_NEAR(output_cloud->points[1].intensity, 20, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    0.919238, 0.001);
  EXPECT_NEAR(output_cloud->points[2].intensity, 20, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    0.919238, 0.001);
  EXPECT_NEAR(output_cloud->points[3].intensity, 20, 0.001);


  /* Test data filter negative */
  voxel.setInputCloud(input_cloud);
  voxel.setLeafSize(2, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(0, 0, 0));
  voxel.setFilterFieldName("intensity");
  voxel.setFilterLimits(15, 50);
  voxel.setFilterLimitsNegative(true);
  voxel.setDownsampleAllData(true);
  voxel.filter(*output_cloud);

  EXPECT_EQ(int (output_cloud->points.size()), 4);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 4);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    -0.919238, 0.001);
  EXPECT_NEAR(output_cloud->points[0].intensity, 10, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    -0.919238, 0.001);
  EXPECT_NEAR(output_cloud->points[1].intensity, 10, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    -0.919238, 0.001);
  EXPECT_NEAR(output_cloud->points[2].intensity, 10, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    -0.919238, 0.001);
  EXPECT_NEAR(output_cloud->points[3].intensity, 10, 0.001);
}

TEST (SphericalVoxelGrid_RGB, SphericalVoxelGrid)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::copyPointCloud(*input_cloud, *input_cloudrgb);

  for (int i = 0; i < input_cloudrgb->points.size(); i++)
  {
    if (input_cloudrgb->points[i].z > 0)
    {
      input_cloudrgb->points[i].r = 10;
      input_cloudrgb->points[i].g = 20;
      input_cloudrgb->points[i].b = 30;
    }
    else
    {
      input_cloudrgb->points[i].r = 40;
      input_cloudrgb->points[i].g = 50;
      input_cloudrgb->points[i].b = 60;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::SphericalVoxelGrid<pcl::PointXYZRGB> voxel;

  voxel.setInputCloud(input_cloudrgb);
  voxel.setLeafSize(2, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(0, 0, 0));
  voxel.filter(*output_cloud);

  EXPECT_EQ(int (output_cloud->points.size()), 4);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 4);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[0].r, 25, 0.001);
  EXPECT_NEAR(output_cloud->points[0].g, 35, 0.001);
  EXPECT_NEAR(output_cloud->points[0].b, 45, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[1].r, 25, 0.001);
  EXPECT_NEAR(output_cloud->points[1].g, 35, 0.001);
  EXPECT_NEAR(output_cloud->points[1].b, 45, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[2].r, 25, 0.001);
  EXPECT_NEAR(output_cloud->points[2].g, 35, 0.001);
  EXPECT_NEAR(output_cloud->points[2].b, 45, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[3].r, 25, 0.001);
  EXPECT_NEAR(output_cloud->points[3].g, 35, 0.001);
  EXPECT_NEAR(output_cloud->points[3].b, 45, 0.001);

  voxel.setInputCloud(input_cloudrgb);
  voxel.setLeafSize(2, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(0, 0, 0));
  voxel.setDownsampleAllData(false);
  voxel.filter(*output_cloud);

  EXPECT_EQ(int (output_cloud->points.size()), 4);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 4);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    0, 0.001);
}

TEST (SphericalVoxelGrid_RGBA, SphericalVoxelGrid)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloudrgba(new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::copyPointCloud(*input_cloud, *input_cloudrgba);

  for (int i = 0; i < input_cloudrgba->points.size(); i++)
  {
    if (input_cloudrgba->points[i].z > 0)
    {
      input_cloudrgba->points[i].r = 10;
      input_cloudrgba->points[i].g = 20;
      input_cloudrgba->points[i].b = 30;
      input_cloudrgba->points[i].a = 40;
    }
    else
    {
      input_cloudrgba->points[i].r = 40;
      input_cloudrgba->points[i].g = 50;
      input_cloudrgba->points[i].b = 60;
      input_cloudrgba->points[i].a = 70;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::SphericalVoxelGrid<pcl::PointXYZRGBA> voxel;

  voxel.setInputCloud(input_cloudrgba);
  voxel.setLeafSize(2, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(0, 0, 0));
  voxel.filter(*output_cloud);

  EXPECT_EQ(int (output_cloud->points.size()), 4);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 4);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[0].r, 25, 0.001);
  EXPECT_NEAR(output_cloud->points[0].g, 35, 0.001);
  EXPECT_NEAR(output_cloud->points[0].b, 45, 0.001);
  EXPECT_NEAR(output_cloud->points[0].a, 55, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[1].r, 25, 0.001);
  EXPECT_NEAR(output_cloud->points[1].g, 35, 0.001);
  EXPECT_NEAR(output_cloud->points[1].b, 45, 0.001);
  EXPECT_NEAR(output_cloud->points[1].a, 55, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[2].r, 25, 0.001);
  EXPECT_NEAR(output_cloud->points[2].g, 35, 0.001);
  EXPECT_NEAR(output_cloud->points[2].b, 45, 0.001);
  EXPECT_NEAR(output_cloud->points[2].a, 55, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    0, 0.001);
  EXPECT_NEAR(output_cloud->points[3].r, 25, 0.001);
  EXPECT_NEAR(output_cloud->points[3].g, 35, 0.001);
  EXPECT_NEAR(output_cloud->points[3].b, 45, 0.001);
  EXPECT_NEAR(output_cloud->points[3].a, 55, 0.001);

  voxel.setInputCloud(input_cloudrgba);
  voxel.setLeafSize(2, 1, 4);
  voxel.setOrigin(Eigen::Vector3f(0, 0, 0));
  voxel.setDownsampleAllData(false);
  voxel.filter(*output_cloud);

  EXPECT_EQ(int (output_cloud->points.size()), 4);
  EXPECT_EQ(int (output_cloud->height), 1);
  EXPECT_EQ(int (output_cloud->width), 4);
  EXPECT_EQ(bool (output_cloud->is_dense), true);

  EXPECT_NEAR(output_cloud->points[0].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[0].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[1].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].y, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[1].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[2].x,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[2].z,    0, 0.001);

  EXPECT_NEAR(output_cloud->points[3].x, -0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].y,  0.65, 0.001);
  EXPECT_NEAR(output_cloud->points[3].z,    0, 0.001);
}

int
main (int argc, char** argv)
{
  // Load a standard PCD file from disk
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download 'spherical_voxel_grid_test.pcd' its path to the test." << std::endl;
    return (-1);
  }

  char* file_name = argv[1];
  std::cout << file_name << std::endl;
  // Load a standard PCD file from disk
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (file_name, *input_cloud) == -1)
  {
    std::cerr << "Could not load test file. Please specify correct location of 'spherical_voxel_grid_test.pcd'." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
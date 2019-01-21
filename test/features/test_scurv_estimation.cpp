/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2018, University of Innsbruck (Antonio J Rodríguez-Sánchez, Tomas Turecek, Alex Melniciuc)
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

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/scurv.h>
#include <pcl/io/pcd_io.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SCurVEstimation)
{
  // Create cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width    = 100;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = i;
    cloud->points[i].y = i*i - i;
    cloud->points[i].z = i*i + i;
  }

  // Estimate normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  n.setInputCloud (cloud);
  n.setRadiusSearch (200.0f);
  n.compute (*normals);

  pcl::SCurVEstimation<pcl::PointXYZ, pcl::Normal> scurv;
  scurv.setKSearch(6);
  scurv.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
  scurv.setInputCloud (cloud);
  scurv.setInputNormals (normals);

  // SCurVSignature object
  pcl::PointCloud<pcl::SCurVSignature210>::Ptr scurvs (new pcl::PointCloud<pcl::SCurVSignature210> ());

  // estimate
  scurv.compute (*scurvs);

  // compare results
  EXPECT_EQ (scurvs->points.size (), 1u);
  int results[210] = {6,0,0,0,0,0,0,0,0,0,7,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,5,2,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,1,2,2,0,0,0,0,0,0,0,0,0,1,0,1,1,0,2,1,0,0,0,0,0,0,1,0,3,3,0,0,0,0,0,0,1,1,0,8,0,0,0,0,0,0,0,4,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  for (size_t i = 0; i < 210; ++i)
  {
    EXPECT_EQ (scurvs->points[0].histogram[i], results[i]);
  } 
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

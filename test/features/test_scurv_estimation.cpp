/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, OpenPerception
 *
 *  \authors Antonio J Rodríguez-Sánchez, University of Innsbruck
 *           Tomáš Tureček, Tomas Bata University in Zlín
 *           Alex Melniciuc, University of Innsbruck
 *
 *  All rights reserved.
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

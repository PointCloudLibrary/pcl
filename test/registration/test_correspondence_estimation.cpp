/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (CorrespondenceEstimation, CorrespondenceEstimationNormalShooting)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());

  // Defining two parallel planes differing only by the y co-ordinate
  for (float i = 0.0f; i < 10.0f; i += 0.2f)
  {
    for (float z = 0.0f; z < 5.0f; z += 0.2f)
    {
      cloud1->points.push_back (pcl::PointXYZ (i, 0, z));
      cloud2->points.push_back (pcl::PointXYZ (i, 2, z)); // Ideally this should be the corresponding point to the point defined in the previous line
    }
  }
        
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud1); 

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud1_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch (5);
  ne.compute (*cloud1_normals); // All normals are perpendicular to the plane defined

  pcl::CorrespondencesPtr corr (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimationNormalShooting <pcl::PointXYZ, pcl::PointXYZ, pcl::Normal> ce;
  ce.setInputSource (cloud1);
  ce.setKSearch (10);
  ce.setSourceNormals (cloud1_normals);
  ce.setInputTarget (cloud2);
  ce.determineCorrespondences (*corr);

  // Based on the data defined, the correspondence indices should be 1 <-> 1 , 2 <-> 2 , 3 <-> 3 etc.
  for (unsigned int i = 0; i < corr->size (); i++)
  {
    EXPECT_EQ ((*corr)[i].index_query, (*corr)[i].index_match);
  }
}

//////////////////////////////////////////////////////////////////////////////////////
TEST (CorrespondenceEstimation, CorrespondenceEstimationSetSearchMethod)
{
  // Generating 3 random clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
  for ( size_t i = 0; i < 50; i++ )
  {
    cloud1->points.push_back(pcl::PointXYZ(float (rand()), float (rand()), float (rand())));
    cloud2->points.push_back(pcl::PointXYZ(float (rand()), float (rand()), float (rand())));
  }
  // Build a KdTree for each
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ> ());
  tree1->setInputCloud (cloud1);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
  tree2->setInputCloud (cloud2);
  // Compute correspondences
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, double> ce;
  ce.setInputSource (cloud1);
  ce.setInputTarget (cloud2);
  pcl::Correspondences corr_orig;
  ce.determineCorrespondences(corr_orig);
  // Now set the kd trees
  ce.setSearchMethodSource (tree1, true);
  ce.setSearchMethodTarget (tree2, true);
  pcl::Correspondences corr_cached;
  ce.determineCorrespondences (corr_cached);
  // Ensure they're the same
  EXPECT_EQ(corr_orig.size(), corr_cached.size());
  for(size_t i = 0; i < corr_orig.size(); i++)
  {
    EXPECT_EQ(corr_orig[i].index_query, corr_cached[i].index_query);
    EXPECT_EQ(corr_orig[i].index_match, corr_cached[i].index_match);
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

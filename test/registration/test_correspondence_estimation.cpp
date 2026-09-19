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

#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>

#include <utility>

namespace
{

template <typename PointT>
PointT makeRandomPoint()
{
  return PointT{};
}

template <>
pcl::PointXYZ makeRandomPoint()
{
  return {static_cast<float>(rand()), static_cast<float>(rand()), static_cast<float>(rand())};
}

template <>
pcl::PointXYZI makeRandomPoint()
{
  return {static_cast<float>(rand()), static_cast<float>(rand()), static_cast<float>(rand()), static_cast<float>(rand())};
}

template <typename PointT>
PointT makePointWithXYZ(float x, float y, float z)
{
  PointT point{};
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

}

template <typename T>
class CorrespondenceEstimationTestSuite : public ::testing::Test { };

using PointTypesForCorrespondenceEstimationTest = 
  ::testing::Types<std::pair<pcl::PointXYZ, pcl::PointXYZ>, std::pair<pcl::PointXYZ, pcl::PointXYZI>>;

using PointTypesForCorrespondenceEstimationMatrixTest =
  ::testing::Types<
    std::pair<pcl::PointXYZ, pcl::PointXYZ>,
    std::pair<pcl::PointXYZ, pcl::PointXYZI>,
    std::pair<pcl::PointXYZ, pcl::PointNormal>,
    std::pair<pcl::PointXYZ, pcl::PointXYZRGB>,
    std::pair<pcl::PointXYZ, pcl::PointXYZRGBNormal>,
    std::pair<pcl::PointXYZI, pcl::PointXYZ>,
    std::pair<pcl::PointXYZI, pcl::PointXYZI>,
    std::pair<pcl::PointXYZI, pcl::PointNormal>,
    std::pair<pcl::PointXYZI, pcl::PointXYZRGB>,
    std::pair<pcl::PointXYZI, pcl::PointXYZRGBNormal>,
    std::pair<pcl::PointNormal, pcl::PointXYZ>,
    std::pair<pcl::PointNormal, pcl::PointXYZI>,
    std::pair<pcl::PointNormal, pcl::PointNormal>,
    std::pair<pcl::PointNormal, pcl::PointXYZRGB>,
    std::pair<pcl::PointNormal, pcl::PointXYZRGBNormal>,
    std::pair<pcl::PointXYZRGB, pcl::PointXYZ>,
    std::pair<pcl::PointXYZRGB, pcl::PointXYZI>,
    std::pair<pcl::PointXYZRGB, pcl::PointNormal>,
    std::pair<pcl::PointXYZRGB, pcl::PointXYZRGB>,
    std::pair<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>,
    std::pair<pcl::PointXYZRGBNormal, pcl::PointXYZ>,
    std::pair<pcl::PointXYZRGBNormal, pcl::PointXYZI>,
    std::pair<pcl::PointXYZRGBNormal, pcl::PointNormal>,
    std::pair<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>,
    std::pair<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>>;

TYPED_TEST_SUITE(CorrespondenceEstimationTestSuite, PointTypesForCorrespondenceEstimationTest);

template <typename T>
class CorrespondenceEstimationMatrixTest : public ::testing::Test { };

TYPED_TEST_SUITE(CorrespondenceEstimationMatrixTest,
                 PointTypesForCorrespondenceEstimationMatrixTest);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST(CorrespondenceEstimationTestSuite, CorrespondenceEstimationNormalShooting)
{
  using PointSource = typename TypeParam::first_type;
  using PointTarget = typename TypeParam::second_type;

  auto cloud1 (pcl::make_shared<pcl::PointCloud<PointSource>> ());
  auto cloud2 (pcl::make_shared<pcl::PointCloud<PointTarget>> ());

  // Defining two parallel planes differing only by the y coordinate
  for (std::size_t i = 0; i < 50; ++i)
  {
    for (std::size_t j = 0; j < 25; ++j)
    {
      cloud1->push_back(makePointWithXYZ<PointSource>(i * 0.2f, 0.f, j * 0.2f));
      cloud2->push_back(makePointWithXYZ<PointTarget>(i * 0.2f, 2.f, j * 0.2f)); // Ideally this should be the corresponding point to the point defined in the previous line
    }
  }
        
  pcl::NormalEstimation<PointSource, pcl::Normal> ne;
  ne.setInputCloud (cloud1); 

  auto tree (pcl::make_shared<pcl::search::KdTree<PointSource>> ());
  ne.setSearchMethod (tree);

  auto cloud1_normals (pcl::make_shared<pcl::PointCloud<pcl::Normal>> ());
  ne.setKSearch (5);
  ne.compute (*cloud1_normals); // All normals are perpendicular to the plane defined

  auto corr (pcl::make_shared<pcl::Correspondences> ());
  pcl::registration::CorrespondenceEstimationNormalShooting <PointSource, PointTarget, pcl::Normal> ce;
  ce.setInputSource (cloud1);
  ce.setKSearch (10);
  ce.setSourceNormals (cloud1_normals);
  ce.setInputTarget (cloud2);
  ce.determineCorrespondences (*corr);

  // Based on the data defined, the correspondence indices should be 1 <-> 1 , 2 <-> 2 , 3 <-> 3 etc.
  for (std::size_t i = 0; i < corr->size (); i++)
  {
    EXPECT_EQ ((*corr)[i].index_query, (*corr)[i].index_match);
  }
}

//////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST(CorrespondenceEstimationMatrixTest, DifferentPointTypes)
{
  using PointSource = typename TypeParam::first_type;
  using PointTarget = typename TypeParam::second_type;

  auto source = pcl::make_shared<pcl::PointCloud<PointSource>>();
  auto target = pcl::make_shared<pcl::PointCloud<PointTarget>>();
  for (std::size_t i = 0; i < 4; ++i) {
    const float coordinate = static_cast<float>(i);
    source->push_back(makePointWithXYZ<PointSource>(coordinate, 0.f, 0.f));
    target->push_back(makePointWithXYZ<PointTarget>(coordinate, 0.f, 0.f));
  }

  pcl::registration::CorrespondenceEstimation<PointSource, PointTarget> ce;
  ce.setInputSource(source);
  ce.setInputTarget(target);

  pcl::Correspondences correspondences;
  ce.determineCorrespondences(correspondences);
  ASSERT_EQ(correspondences.size(), source->size());
  for (std::size_t i = 0; i < correspondences.size(); ++i) {
    EXPECT_EQ(correspondences[i].index_query, static_cast<int>(i));
    EXPECT_EQ(correspondences[i].index_match, static_cast<int>(i));
  }

  pcl::Correspondences reciprocal_correspondences;
  ce.determineReciprocalCorrespondences(reciprocal_correspondences);
  ASSERT_EQ(reciprocal_correspondences.size(), source->size());
  for (std::size_t i = 0; i < reciprocal_correspondences.size(); ++i) {
    EXPECT_EQ(reciprocal_correspondences[i].index_query, static_cast<int>(i));
    EXPECT_EQ(reciprocal_correspondences[i].index_match, static_cast<int>(i));
  }
}

//////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST (CorrespondenceEstimationTestSuite, CorrespondenceEstimationSetSearchMethod)
{
  using PointSource = typename TypeParam::first_type;
  using PointTarget = typename TypeParam::second_type;
  // Generating 3 random clouds
  auto cloud1 (pcl::make_shared<pcl::PointCloud<PointSource>> ());
  auto cloud2 (pcl::make_shared<pcl::PointCloud<PointTarget>> ());
  for (std::size_t i = 0; i < 50; i++)
  {
    cloud1->push_back(makeRandomPoint<PointSource>());
    cloud2->push_back(makeRandomPoint<PointTarget>());
  }
  // Build a KdTree for each
  auto tree1 (pcl::make_shared<pcl::search::KdTree<PointSource>> ());
  tree1->setInputCloud (cloud1);
  auto tree2 (pcl::make_shared<pcl::search::KdTree<PointTarget>> ());
  tree2->setInputCloud (cloud2);
  // Compute correspondences
  pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, double> ce;
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
  for(std::size_t i = 0; i < corr_orig.size(); i++)
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

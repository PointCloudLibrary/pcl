/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2021-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace pcl;

//////////////////////////////////////////////
struct pointCloudTest : public testing::Test {
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    PointCloud<PointXYZ> cloud;
};

TEST_F (pointCloudTest, is_organized)
{
  cloud.width = 640;
  cloud.height = 480;
  EXPECT_TRUE (cloud.isOrganized ());
}

TEST_F (pointCloudTest, not_organized)
{
  cloud.width = 640;
  cloud.height = 1;
  EXPECT_FALSE (cloud.isOrganized ());
}

TEST_F (pointCloudTest, getMatrixXfMap)
{
  cloud.height = 1;
  cloud.width = 10;
  for (std::uint32_t i = 0; i < cloud.width*cloud.height; ++i)
  {
    float j = static_cast<float> (i);
    cloud.emplace_back(3.0f * j + 0.0f, 3.0f * j + 1.0f, 3.0f * j + 2.0f);
  }

  Eigen::MatrixXf mat_xyz1 = cloud.getMatrixXfMap ();
  Eigen::MatrixXf mat_xyz = cloud.getMatrixXfMap (3, 4, 0);

  if (Eigen::MatrixXf::Flags & Eigen::RowMajorBit)
  {
    EXPECT_EQ (mat_xyz1.cols (), 4);
    EXPECT_EQ (mat_xyz1.rows (), cloud.width);
    EXPECT_EQ (mat_xyz1 (0, 0), 0);
    EXPECT_EQ (mat_xyz1 (cloud.width - 1, 2), 3 * cloud.width - 1);   // = 29

    EXPECT_EQ (mat_xyz.cols (), 3);
    EXPECT_EQ (mat_xyz.rows (), cloud.width);
    EXPECT_EQ (mat_xyz (0, 0), 0);
    EXPECT_EQ (mat_xyz (cloud.width - 1, 2), 3 * cloud.width - 1);    // = 29
  }
  else
  {
    EXPECT_EQ (mat_xyz1.cols (), cloud.width);
    EXPECT_EQ (mat_xyz1.rows (), 4);
    EXPECT_EQ (mat_xyz1 (0, 0), 0);
    EXPECT_EQ (mat_xyz1 (2, cloud.width - 1), 3 * cloud.width - 1);   // = 29

    EXPECT_EQ (mat_xyz.cols (), cloud.width);
    EXPECT_EQ (mat_xyz.rows (), 3);
    EXPECT_EQ (mat_xyz (0, 0), 0);
    EXPECT_EQ (mat_xyz (2, cloud.width - 1), 3 * cloud.width - 1);    // = 29
  }

#ifdef NDEBUG
  if (Eigen::MatrixXf::Flags & Eigen::RowMajorBit)
  {
    Eigen::MatrixXf mat_yz = cloud.getMatrixXfMap (2, 4, 1);
    EXPECT_EQ (mat_yz.cols (), 2);
    EXPECT_EQ (mat_yz.rows (), cloud.width);
    EXPECT_EQ (mat_yz (0, 0), 1);
    EXPECT_EQ (mat_yz (cloud.width - 1, 1), 3 * cloud.width - 1);
    std::uint32_t j = 1;
    for (std::uint32_t i = 1; i < cloud.width*cloud.height; i+=4, j+=3)
    {
      Eigen::MatrixXf mat_yz = cloud.getMatrixXfMap (2, 4, i);
      EXPECT_EQ (mat_yz.cols (), 2);
      EXPECT_EQ (mat_yz.rows (), cloud.width);
      EXPECT_EQ (mat_yz (0, 0), j);
    }
  }
  else
  {
    Eigen::MatrixXf mat_yz = cloud.getMatrixXfMap (2, 4, 1);
    EXPECT_EQ (mat_yz.cols (), cloud.width);
    EXPECT_EQ (mat_yz.rows (), 2);
    EXPECT_EQ (mat_yz (0, 0), 1);
    EXPECT_EQ (mat_yz (1, cloud.width - 1), 3 * cloud.width - 1);
    std::uint32_t j = 1;
    for (std::uint32_t i = 1; i < cloud.width*cloud.height; i+=4, j+=3)
    {
      Eigen::MatrixXf mat_yz = cloud.getMatrixXfMap (2, 4, i);
      EXPECT_EQ (mat_yz.cols (), cloud.width);
      EXPECT_EQ (mat_yz.rows (), 2);
      EXPECT_EQ (mat_yz (0, 0), j);
    }
  }
#endif
}

TEST_F (pointCloudTest, clear)
{
  cloud.insert (cloud.end (), PointXYZ (1, 1, 1));
  EXPECT_EQ (cloud.size(), 1);
  cloud.clear ();
  EXPECT_EQ (cloud.width, 0);
  EXPECT_EQ (cloud.height, 0);
}

TEST_F (pointCloudTest, insert)
{
  cloud.insert (cloud.end (), PointXYZ (1, 1, 1));
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 1);
}

TEST_F (pointCloudTest, insert_with_height)
{
  cloud.insert (cloud.end (), 5, PointXYZ (1, 1, 1));
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 5);
}

TEST_F (pointCloudTest, erase_at_position)
{
  cloud.insert (cloud.end (), 5, PointXYZ (1, 1, 1));
  cloud.erase (cloud.end () - 1);
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 4);
}

TEST_F (pointCloudTest, erase_with_iterator)
{
  cloud.insert (cloud.end (), 5, PointXYZ (1, 1, 1));
  cloud.erase (cloud.begin (), cloud.end ());
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 0);
}

TEST_F (pointCloudTest, emplace)
{
  cloud.emplace (cloud.end (), 1, 1, 1);
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 1);
}

TEST_F (pointCloudTest, emplace_back)
{
  auto& new_point = cloud.emplace_back (1, 1, 1);
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 1);
  EXPECT_EQ (&new_point, &cloud.back ());
}

TEST_F (pointCloudTest, resize_with_count_elements)
{
  cloud.resize (640*360);
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640*360);
}

TEST_F (pointCloudTest, resize_with_new_width_and_height)
{
  cloud.resize (640, 480);
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640);
  EXPECT_EQ (cloud.height, 480);
}

TEST_F (pointCloudTest, resize_with_initialized_count_elements)
{
  cloud.resize (640*360, PointXYZ (1, 1, 1));
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640*360);
}

TEST_F (pointCloudTest, resize_with_initialized_count_and_new_width_and_height)
{
  cloud.resize (640, 480, PointXYZ (1, 1, 1));
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640);
}

TEST_F (pointCloudTest, assign_with_copies)
{
  cloud.assign (640*360, PointXYZ (1, 1, 1));
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640*360);
}

TEST_F (pointCloudTest, assign_with_new_width_and_height_copies)
{
  cloud.assign(640, 480, PointXYZ (1, 1, 1));
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640);
}

TEST_F (pointCloudTest, assign_with_copies_in_range)
{
  std::vector<PointXYZ> pointVec;
  pointVec.resize (640*360, PointXYZ (2, 3, 4));
  cloud.assign (pointVec.begin(), pointVec.end());
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640*360);
}

TEST_F (pointCloudTest, assign_with_copies_in_range_and_new_width)
{
  std::vector<PointXYZ> pointVec;
  pointVec.resize (640*360, PointXYZ (2, 3, 4));
  cloud.assign (pointVec.begin(), pointVec.end(), 640);
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640);
}

TEST_F (pointCloudTest, assign_mismatch_size_and_width_height)
{
  std::vector<PointXYZ> pointVec;
  pointVec.resize (640*480, PointXYZ (7, 7, 7));
  cloud.assign (pointVec.begin(), pointVec.end(), 460);
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640*480);
}

TEST_F (pointCloudTest, assign_initializer_list)
{
  cloud.assign ({PointXYZ (3, 4, 5), PointXYZ (3, 4, 5), PointXYZ (3, 4, 5)});
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 3);
}

TEST_F (pointCloudTest, assign_initializer_list_with_new_width)
{
  cloud.assign ({PointXYZ (3, 4, 5), PointXYZ (3, 4, 5), PointXYZ (3, 4, 5), PointXYZ (3, 4, 5)}, 2);
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 2);
}

TEST_F (pointCloudTest, assign_initializer_list_with_unorganized_cloud)
{
  cloud.assign ({PointXYZ (3, 4, 5), PointXYZ (3, 4, 5), PointXYZ (3, 4, 5)}, 6);
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 3);
}

TEST_F (pointCloudTest, push_back_to_unorganized_cloud)
{
  cloud.push_back (PointXYZ (3, 4, 5));
  EXPECT_FALSE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 1);
}

TEST_F (pointCloudTest, push_back_to_organized_cloud)
{
  cloud.resize (80, 80, PointXYZ (1, 1, 1));
  EXPECT_TRUE (cloud.isOrganized ());
  cloud.push_back (PointXYZ (3, 4, 5));
  EXPECT_EQ (cloud.width, (80*80) + 1);
}

/////////////////////////////////////////////////
struct organizedPointCloudTest : public pointCloudTest {
  protected:
    void SetUp() override
    {
      cloud.resize (640, 480, PointXYZ (1, 1, 1));
    }
};

TEST_F (organizedPointCloudTest, transient_push_back)
{
  cloud.transient_push_back (PointXYZ(2, 2, 2));
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640);
  EXPECT_EQ (cloud.size(), (640*480) + 1);
}

TEST_F (organizedPointCloudTest, transient_emplace_back)
{
  auto& new_pointXYZ = cloud.transient_emplace_back (3, 3, 3);
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640);
  EXPECT_EQ (cloud.size(), (640*480) + 1);
  EXPECT_EQ (&new_pointXYZ, &cloud.back ());
}

TEST_F (organizedPointCloudTest, transient_insert_one_element)
{
  cloud.transient_insert (cloud.end (), PointXYZ (1, 1, 1));
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.size(), (640*480) + 1);
  EXPECT_EQ (cloud.width, 640);
}

TEST_F (organizedPointCloudTest, transient_insert_with_n_elements)
{
  cloud.transient_insert (cloud.end (), 10, PointXYZ (1, 1, 1));
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.size(), (640*480) + 10);
  EXPECT_EQ (cloud.width, 640);
}

TEST_F (organizedPointCloudTest, transient_emplace)
{
  cloud.transient_emplace (cloud.end (), 4, 4, 4);
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640);
  EXPECT_EQ (cloud.size(), (640*480) + 1);
}

TEST_F (organizedPointCloudTest, transient_erase_at_position)
{
  cloud.transient_erase (cloud.end () - 1);
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640);
  EXPECT_EQ (cloud.size(), (640*480) - 1);
}

TEST_F (organizedPointCloudTest, transient_erase_with_iterator)
{
  cloud.transient_erase (cloud.begin (), cloud.end ());
  EXPECT_TRUE (cloud.isOrganized ());
  EXPECT_EQ (cloud.width, 640);
  EXPECT_EQ (cloud.size(), 0);
}

TEST_F (organizedPointCloudTest, unorganized_concatenate)
{
  PointCloud<PointXYZ> new_unorganized_cloud;
  PointCloud<PointXYZ>::concatenate (new_unorganized_cloud, cloud);
  EXPECT_FALSE (new_unorganized_cloud.isOrganized ());
  EXPECT_EQ (new_unorganized_cloud.width, 640*480);
}

TEST_F (organizedPointCloudTest, unorganized_concatenate_with_argument_return)
{
  PointCloud<PointXYZ> new_unorganized_cloud;
  PointCloud<PointXYZ>::concatenate (new_unorganized_cloud, cloud);
  PointCloud<PointXYZ> unorganized_cloud_out;
  PointCloud<PointXYZ>::concatenate (new_unorganized_cloud, cloud, unorganized_cloud_out);
  EXPECT_FALSE (unorganized_cloud_out.isOrganized ());
  EXPECT_EQ (unorganized_cloud_out.width, 640*480*2);
}

TEST_F (organizedPointCloudTest, unorganized_concatenate_with_assignment_return)
{
  PointCloud<PointXYZ> unorganized_cloud;
  PointCloud<PointXYZ>::concatenate (unorganized_cloud, cloud);
  PointCloud<PointXYZ> unorganized_cloud_out = cloud + unorganized_cloud;
  EXPECT_FALSE (unorganized_cloud_out.isOrganized ());
  EXPECT_EQ (unorganized_cloud_out.width, 640*480*2);
}

TEST_F (organizedPointCloudTest, unorganized_concatenate_with_plus_operator)
{
  PointCloud<PointXYZ> unorganized_cloud;
  unorganized_cloud += cloud;
  EXPECT_FALSE (unorganized_cloud.isOrganized ());
  EXPECT_EQ (unorganized_cloud.width, 640*480);
}

TEST_F (organizedPointCloudTest, at_with_throw)
{
  PointCloud<PointXYZ> unorganized_cloud;
  unorganized_cloud += cloud;
  EXPECT_THROW({unorganized_cloud.at (5, 5);}, UnorganizedPointCloudException);
}

TEST_F (organizedPointCloudTest, at_no_throw)
{
  const auto& point_at = cloud.at (cloud.width - 1, cloud.height - 1);
  EXPECT_EQ(&point_at, &cloud.back());
}

TEST_F (organizedPointCloudTest, organized_concatenate)
{
  PointCloud<PointXYZ> organized_cloud1 = cloud;
  PointCloud<PointXYZ> organized_cloud2 = cloud;
  EXPECT_TRUE (organized_cloud1.isOrganized ());
  EXPECT_TRUE (organized_cloud2.isOrganized ());
  PointCloud<PointXYZ> organized_cloud_out = organized_cloud1 + organized_cloud2;
  std::size_t total_size = organized_cloud1.size() + organized_cloud2.size();
  EXPECT_FALSE (organized_cloud_out.isOrganized ());
  EXPECT_EQ(total_size, 614400);
  EXPECT_EQ (organized_cloud_out.width, total_size);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

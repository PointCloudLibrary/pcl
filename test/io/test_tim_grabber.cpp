/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *  Copyright (c) 2020, ysuzuki19
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/tim_grabber.h>

#include <string>
#include <random>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct TestableTimGrabber : public pcl::TimGrabber {
  using TimGrabber::TimGrabber;
  using TimGrabber::point_cloud_xyz_ptr_;
  using TimGrabber::processTimPacket;
  using TimGrabber::updateLookupTables;
  using TimGrabber::toPointClouds;
};

class TimGrabberTest : public ::testing::Test {
  protected:
    TimGrabberTest () = default;
    ~TimGrabberTest () override = default;

    std::vector<std::string> packets_;
    std::vector<CloudT> correct_clouds_;
    TestableTimGrabber grabber_;

    void SetUp () override {
      constexpr float angle_start = - 1.0 * M_PI / 4.0;
      constexpr float angle_range = 2.0 * M_PI * 3.0 / 4.0;

      std::default_random_engine generator;
      std::uniform_int_distribution<int> i_distribution (0, 1000);
      std::uniform_real_distribution<float> f_distribution (0.0, 20.0);

      CloudT cloud;

      for (int i=0; i<1000; ++i) {
        const size_t amount_of_data = i_distribution (generator);

        cloud.reserve (amount_of_data);
        cloud.clear ();

        std::ostringstream ss;
        ss << std::hex;
        ss << " " << amount_of_data;

        float angle = angle_start;
        const float angle_step = angle_range / amount_of_data;

        for (size_t i=0; i<amount_of_data; ++i, angle += angle_step) {
          float distance = f_distribution (generator);
          cloud.emplace_back (distance * std::cos (angle), distance * std::sin (angle), 0.0);
          ss << " " << static_cast<int>(distance * 1000);
        }
        correct_clouds_.push_back (cloud);

        std::string header_sample = "sRA LMDscandata 1 1 1291B11 0 0 AED5 AED7 FDB36397 FDB3779F 0 0 1 0 0 5DC A2 0 1 DIST1 3F800000 00000000 FFF92230 D05";
        std::string packet = header_sample + ss.str ();
        packets_.push_back (packet);
      }
    }
};

TEST_F (TimGrabberTest, Test1)
{
  CloudT::ConstPtr answer_cloud = grabber_.point_cloud_xyz_ptr_;

  for (std::size_t i=0; i<packets_.size (); ++i) {
    std::string packet = packets_.at(i);

    grabber_.processTimPacket (packet);
    grabber_.updateLookupTables ();
    grabber_.toPointClouds ();

    ASSERT_EQ (correct_clouds_.at(i).size (), answer_cloud->size ());

    for (std::size_t j = 0; j < correct_clouds_.at(i).size (); j++) {
      PointT const& correct_point = correct_clouds_.at(i).at(j);
      PointT const& answer_point = answer_cloud->at(j);
      EXPECT_NEAR (correct_point.x, answer_point.x, 1.0e-3);
      EXPECT_NEAR (correct_point.y, answer_point.y, 1.0e-3);
      EXPECT_NEAR (correct_point.z, answer_point.z, 1.0e-3);
    }
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  ::testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/io/hdl_grabber.h>

#include <cstdio>
#include <fstream>
#include <string>

struct TestableHDLGrabber : public pcl::HDLGrabber
{
  using HDLGrabber::HDLGrabber;
  using HDLGrabber::laser_corrections_;
};

static void
writeCorrectionItem (std::ofstream& file, const std::string& id, double dist_correction)
{
  file << "<item><px>"
       << "<id_>" << id << "</id_>"
       << "<rotCorrection_>-5</rotCorrection_>"
       << "<vertCorrection_>-7</vertCorrection_>"
       << "<distCorrection_>" << dist_correction << "</distCorrection_>"
       << "<vertOffsetCorrection_>20</vertOffsetCorrection_>"
       << "<horizOffsetCorrection_>2.6</horizOffsetCorrection_>"
       << "</px></item>";
}

TEST (PCL, HDLGrabberCorrectionsFileInvalidLaserId)
{
  const std::string file_name = "test_hdl_grabber_corrections.xml";
  {
    std::ofstream file (file_name);
    file << "<?xml version=\"1.0\"?>\n<boost_serialization><DB><points_>";
    // laser ids outside [0, 64) must be ignored, not written past laser_corrections_
    writeCorrectionItem (file, "64", 1.0);
    writeCorrectionItem (file, "1000", 1.0);
    writeCorrectionItem (file, "-2", 1.0);
    writeCorrectionItem (file, "3", 111.0);
    file << "</points_></DB></boost_serialization>\n";
  }

  {
    TestableHDLGrabber grabber (file_name, "");
    EXPECT_DOUBLE_EQ (1.11, grabber.laser_corrections_[3].distanceCorrection);
    EXPECT_DOUBLE_EQ (-5.0, grabber.laser_corrections_[3].azimuthCorrection);
  }

  std::remove (file_name.c_str ());
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

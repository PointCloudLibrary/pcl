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
#include <sstream>
#include <string>

struct TestableHDLGrabber : public pcl::HDLGrabber
{
  using HDLGrabber::HDLGrabber;
  using HDLGrabber::laser_corrections_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
laserCorrection (const std::string& id, double rot_correction, double vert_correction)
{
  std::ostringstream ss;
  ss << "<item class_id=\"2\" tracking_level=\"0\" version=\"1\">"
     << "<px class_id=\"3\" tracking_level=\"0\" version=\"1\">"
     << "<id_>" << id << "</id_>"
     << "<rotCorrection_>" << rot_correction << "</rotCorrection_>"
     << "<vertCorrection_>" << vert_correction << "</vertCorrection_>"
     << "<distCorrection_>0</distCorrection_>"
     << "<vertOffsetCorrection_>0</vertOffsetCorrection_>"
     << "<horizOffsetCorrection_>0</horizOffsetCorrection_>"
     << "</px></item>";
  return ss.str ();
}

void
writeCorrectionsFile (const std::string& file_name, const std::string& db)
{
  std::ofstream fs (file_name);
  fs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>\n"
     << "<!DOCTYPE boost_serialization>\n"
     << "<boost_serialization signature=\"serialization::archive\" version=\"4\">\n"
     << "<DB class_id=\"0\" tracking_level=\"0\" version=\"0\">" << db << "</DB>\n"
     << "</boost_serialization>\n";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (HDLGrabber, LoadCorrectionsFileLaserIdOutOfRange)
{
  // The laser ids come from the calibration file. Ids outside of the range of
  // lasers the grabber knows about must be ignored, not used as an index.
  const std::string file_name = "test_hdl_corrections_out_of_range.xml";
  writeCorrectionsFile (file_name,
                        "<points_ class_id=\"1\" tracking_level=\"0\" version=\"0\">"
                        "<count>5</count><item_version>1</item_version>"
                        + laserCorrection ("5", 2.5, 1.5)
                        + laserCorrection ("64", 3.0, 3.0)
                        + laserCorrection ("-2", 3.0, 3.0)
                        + laserCorrection ("1000000", 3.0, 3.0)
                        + laserCorrection ("-1000000", 3.0, 3.0)
                        + "</points_>");

  TestableHDLGrabber grabber (file_name, "");
  EXPECT_DOUBLE_EQ (grabber.laser_corrections_[5].azimuthCorrection, 2.5);
  EXPECT_DOUBLE_EQ (grabber.laser_corrections_[5].verticalCorrection, 1.5);

  remove (file_name.c_str ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (HDLGrabber, LoadCorrectionsFileWithoutPoints)
{
  // A calibration file without the list of corrections is reported, not thrown
  // out of the constructor.
  const std::string file_name = "test_hdl_corrections_without_points.xml";
  writeCorrectionsFile (file_name, "<count>0</count>");

  EXPECT_NO_THROW (TestableHDLGrabber grabber (file_name, ""));

  remove (file_name.c_str ());
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

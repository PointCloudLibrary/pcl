/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
/** \author Radu Bogdan Rusu */

#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/ros/point_traits.h"
#include "pcl/point_types.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"

using namespace pcl;
using namespace pcl::io;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConcatenatePoints)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;

  // Fill in the cloud data
  cloud_a.width  = 5;
  cloud_b.width  = 3;
  cloud_a.height = cloud_b.height = 1;
  cloud_a.points.resize (cloud_a.width * cloud_a.height);
  cloud_b.points.resize (cloud_b.width * cloud_b.height);

  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    cloud_a.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_a.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_a.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
  }

  for (size_t i = 0; i < cloud_b.points.size (); ++i)
  {
    cloud_b.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_b.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_b.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
  }

  // Copy the point cloud data
  cloud_c  = cloud_a;
  cloud_c += cloud_b;
  EXPECT_EQ (cloud_c.points.size (), cloud_a.points.size () + cloud_b.points.size ());
  EXPECT_EQ (cloud_c.width, cloud_a.width + cloud_b.width);
  EXPECT_EQ ((int)cloud_c.height, 1);

  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    EXPECT_EQ (cloud_c.points[i].x, cloud_a.points[i].x);
    EXPECT_EQ (cloud_c.points[i].y, cloud_a.points[i].y);
    EXPECT_EQ (cloud_c.points[i].z, cloud_a.points[i].z);
  }
  for (size_t i = cloud_a.points.size (); i < cloud_c.points.size (); ++i)
  {
    EXPECT_EQ (cloud_c.points[i].x, cloud_b.points[i - cloud_a.points.size ()].x);
    EXPECT_EQ (cloud_c.points[i].y, cloud_b.points[i - cloud_a.points.size ()].y);
    EXPECT_EQ (cloud_c.points[i].z, cloud_b.points[i - cloud_a.points.size ()].z);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConcatenateFields)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_a;
  pcl::PointCloud<pcl::Normal> cloud_b;
  pcl::PointCloud<pcl::PointNormal> cloud_c;

  // Fill in the cloud data
  cloud_a.width  = cloud_b.width  = 5;
  cloud_a.height = cloud_b.height = 1;
  cloud_a.points.resize (cloud_a.width * cloud_a.height);
  cloud_b.points.resize (cloud_b.width * cloud_b.height);

  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    cloud_a.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_a.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_a.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
  }

  for (size_t i = 0; i < cloud_b.points.size (); ++i)
  {
    cloud_b.points[i].normal[0] = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_b.points[i].normal[1] = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_b.points[i].normal[2] = 1024 * rand () / (RAND_MAX + 1.0);
  }

  pcl::concatenateFields (cloud_a, cloud_b, cloud_c);
  EXPECT_EQ (cloud_c.points.size (), cloud_a.points.size ());
  EXPECT_EQ (cloud_c.width, cloud_a.width);
  EXPECT_EQ (cloud_c.height, cloud_a.height);

  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    EXPECT_EQ (cloud_c.points[i].x, cloud_a.points[i].x);
    EXPECT_EQ (cloud_c.points[i].y, cloud_a.points[i].y);
    EXPECT_EQ (cloud_c.points[i].z, cloud_a.points[i].z);
    EXPECT_EQ (cloud_c.points[i].normal[0], cloud_b.points[i].normal[0]);
    EXPECT_EQ (cloud_c.points[i].normal[1], cloud_b.points[i].normal[1]);
    EXPECT_EQ (cloud_c.points[i].normal[2], cloud_b.points[i].normal[2]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IO)
{
  sensor_msgs::PointCloud2 cloud_blob;
  PointCloud<PointXYZI> cloud;

  cloud.width  = 640;
  cloud.height = 480;
  cloud.points.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  srand (time (NULL));
  size_t nr_p = cloud.points.size ();
  // Randomly create a new point cloud
  for (size_t i = 0; i < nr_p; ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].intensity = i;
  }
  PointXYZI first, last;
  first.x = cloud.points[0].x;       first.y = cloud.points[0].y;       first.z = cloud.points[0].z;       first.intensity = cloud.points[0].intensity;
  last.x = cloud.points[nr_p - 1].x; last.y = cloud.points[nr_p - 1].y; last.z = cloud.points[nr_p - 1].z; last.intensity  = cloud.points[nr_p - 1].intensity;

  // Tests for PointCloud::operator()
  EXPECT_EQ (first.x, cloud (0, 0).x);
  EXPECT_EQ (first.y, cloud (0, 0).y);
  EXPECT_EQ (first.z, cloud (0, 0).z);
  EXPECT_EQ (first.intensity, (float)0);
  EXPECT_EQ (last.x, cloud (cloud.width-1, cloud.height-1).x);
  EXPECT_EQ (last.y, cloud (cloud.width-1, cloud.height-1).y);
  EXPECT_EQ (last.z, cloud (cloud.width-1, cloud.height-1).z);
  EXPECT_EQ (last.intensity, nr_p - 1);

  // Test getFieldIndex
  std::vector<sensor_msgs::PointField> fields;
  pcl::getFields (cloud, fields);
  EXPECT_EQ (fields.size (), (size_t)4);
  int x_idx = pcl::getFieldIndex (cloud, "x", fields);
  EXPECT_EQ (x_idx, 0);
  EXPECT_EQ (fields[x_idx].offset, (uint32_t)0);
  EXPECT_EQ (fields[x_idx].name, "x");
  EXPECT_EQ (fields[x_idx].datatype, sensor_msgs::PointField::FLOAT32);
  EXPECT_EQ (fields[x_idx].count, (uint32_t)1);

  int y_idx = pcl::getFieldIndex (cloud, "y", fields);
  EXPECT_EQ (y_idx, 1);
  EXPECT_EQ (fields[y_idx].offset, (uint32_t)4);
  EXPECT_EQ (fields[y_idx].name, "y");
  EXPECT_EQ (fields[y_idx].datatype, sensor_msgs::PointField::FLOAT32);
  EXPECT_EQ (fields[y_idx].count, (uint32_t)1);

  int z_idx = pcl::getFieldIndex (cloud, "z", fields);
  EXPECT_EQ (z_idx, 2);
  EXPECT_EQ (fields[z_idx].offset, (uint32_t)8);
  EXPECT_EQ (fields[z_idx].name, "z");
  EXPECT_EQ (fields[z_idx].datatype, sensor_msgs::PointField::FLOAT32);
  EXPECT_EQ (fields[z_idx].count, (uint32_t)1);

  int intensity_idx = pcl::getFieldIndex (cloud, "intensity", fields);
  EXPECT_EQ (intensity_idx, 3);
  EXPECT_EQ (fields[intensity_idx].offset, (uint32_t)16);      // NOTE: intensity_idx.offset should be 12, but we are padding in PointXYZ (!)
  EXPECT_EQ (fields[intensity_idx].name, "intensity");
  EXPECT_EQ (fields[intensity_idx].datatype, sensor_msgs::PointField::FLOAT32);
  EXPECT_EQ (fields[intensity_idx].count, (uint32_t)1);

  // Convert from data type to blob
  toROSMsg (cloud, cloud_blob);

  // Test getFieldIndex
  x_idx = pcl::getFieldIndex (cloud_blob, "x");
  EXPECT_EQ (x_idx, 0);
  EXPECT_EQ (cloud_blob.fields[x_idx].offset, (uint32_t)0);
  EXPECT_EQ (cloud_blob.fields[x_idx].name, "x");
  EXPECT_EQ (cloud_blob.fields[x_idx].datatype, sensor_msgs::PointField::FLOAT32);
  EXPECT_EQ (cloud_blob.fields[x_idx].count, (uint32_t)1);
  y_idx = pcl::getFieldIndex (cloud_blob, "y");
  EXPECT_EQ (y_idx, 1);
  EXPECT_EQ (cloud_blob.fields[y_idx].offset, (uint32_t)4);
  EXPECT_EQ (cloud_blob.fields[y_idx].name, "y");
  EXPECT_EQ (cloud_blob.fields[y_idx].datatype, sensor_msgs::PointField::FLOAT32);
  EXPECT_EQ (cloud_blob.fields[y_idx].count, (uint32_t)1);
  z_idx = pcl::getFieldIndex (cloud_blob, "z");
  EXPECT_EQ (z_idx, 2);
  EXPECT_EQ (cloud_blob.fields[z_idx].offset, (uint32_t)8);
  EXPECT_EQ (cloud_blob.fields[z_idx].name, "z");
  EXPECT_EQ (cloud_blob.fields[z_idx].datatype, sensor_msgs::PointField::FLOAT32);
  EXPECT_EQ (cloud_blob.fields[z_idx].count, (uint32_t)1);
  intensity_idx = pcl::getFieldIndex (cloud_blob, "intensity");
  EXPECT_EQ (intensity_idx, 3);
  //EXPECT_EQ (cloud_blob.fields[intensity_idx].offset, (uint32_t)12);      // NOTE: the fields.offset is 16 in PointCloud<PointXYZI>, but we are obtaining the correct offset in toROSMsg
  EXPECT_EQ (cloud_blob.fields[intensity_idx].offset, (uint32_t)16);      // NOTE: the fields.offset is 16 in PointCloud<PointXYZI>, but we are obtaining the correct offset in toROSMsg
  EXPECT_EQ (cloud_blob.fields[intensity_idx].name, "intensity");
  EXPECT_EQ (cloud_blob.fields[intensity_idx].datatype, sensor_msgs::PointField::FLOAT32);
  EXPECT_EQ (cloud_blob.fields[intensity_idx].count, (uint32_t)1);
  
  fromROSMsg (cloud_blob, cloud);
  for (size_t i = 0; i < nr_p; ++i)
    EXPECT_EQ (cloud.points[i].intensity, i);

  EXPECT_EQ ((uint32_t)cloud_blob.width, cloud.width);    // test for toROSMsg ()
  EXPECT_EQ ((uint32_t)cloud_blob.height, cloud.height);  // test for toROSMsg ()
  EXPECT_EQ ((bool)cloud_blob.is_dense, cloud.is_dense);  // test for toROSMsg ()
  //EXPECT_EQ ((size_t)cloud_blob.data.size () * 2,         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
  //           cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for toROSMsg ()
  EXPECT_EQ ((size_t)cloud_blob.data.size (),             // PointXYZI is 16*2 (XYZ+1, Intensity+3)
             cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for toROSMsg ()

  // Make sure we have permissions to write there
  PCDWriter w;
  int res = w.writeASCII ("/tmp/test_pcl_io.pcd", cloud_blob, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), 10);
  EXPECT_EQ ((int)res, 0);                            // test for savePCDFileASCII ()

  // Please make sure that this file exists, otherwise the test will fail.
  res = loadPCDFile ("/tmp/test_pcl_io.pcd", cloud_blob);
  EXPECT_NE ((int)res, -1);                               // test for loadPCDFile ()
  EXPECT_EQ ((uint32_t)cloud_blob.width, cloud.width);    // test for loadPCDFile ()
  EXPECT_EQ ((uint32_t)cloud_blob.height, cloud.height);  // test for loadPCDFile ()
  EXPECT_EQ ((bool)cloud_blob.is_dense, cloud.is_dense);  // test for loadPCDFile ()
  EXPECT_EQ ((size_t)cloud_blob.data.size () * 2,         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
              cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()

  // Convert from blob to data type
  fromROSMsg (cloud_blob, cloud);

  EXPECT_EQ ((uint32_t)cloud.width, cloud_blob.width);    // test for fromROSMsg ()
  EXPECT_EQ ((uint32_t)cloud.height, cloud_blob.height);  // test for fromROSMsg ()
  EXPECT_EQ ((int)cloud.is_dense, cloud_blob.is_dense);   // test for fromROSMsg ()
  EXPECT_EQ ((size_t)cloud.points.size (), nr_p);         // test for fromROSMsg ()

  EXPECT_NEAR ((float)cloud.points[0].x, first.x, 1e-5);     // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[0].y, first.y, 1e-5);     // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[0].z, first.z, 1e-5);     // test for fromROSMsg ()
  EXPECT_NEAR ((uint32_t)cloud.points[0].intensity, first.intensity, 1e-5);  // test for fromROSMsg ()

  EXPECT_NEAR ((float)cloud.points[nr_p - 1].x, last.x, 1e-5);    // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[nr_p - 1].y, last.y, 1e-5);    // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[nr_p - 1].z, last.z, 1e-5);    // test for fromROSMsg ()
  EXPECT_NEAR ((uint32_t)cloud.points[nr_p - 1].intensity, last.intensity, 1e-5); // test for fromROSMsg ()

  // Make sure we have permissions to write there
  res = savePCDFile ("/tmp/test_pcl_io.pcd", cloud_blob, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
  EXPECT_EQ ((int)res, 0);                            // test for savePCDFileBinary ()

  // Please make sure that this file exists, otherwise the test will fail.
  res = loadPCDFile ("/tmp/test_pcl_io.pcd", cloud_blob);
  EXPECT_NE ((int)res, -1);                               // test for loadPCDFile ()
  EXPECT_EQ ((uint32_t)cloud_blob.width, cloud.width);    // test for loadPCDFile ()
  EXPECT_EQ ((uint32_t)cloud_blob.height, cloud.height);  // test for loadPCDFile ()
  EXPECT_EQ ((bool)cloud_blob.is_dense, false);  // @todo: let is_dense be false for now when reading from binary files
  EXPECT_EQ ((size_t)cloud_blob.data.size () * 2,         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
              cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()

  // Convert from blob to data type
  fromROSMsg (cloud_blob, cloud);

  EXPECT_EQ ((uint32_t)cloud.width, cloud_blob.width);    // test for fromROSMsg ()
  EXPECT_EQ ((uint32_t)cloud.height, cloud_blob.height);  // test for fromROSMsg ()
  EXPECT_EQ ((int)cloud.is_dense, cloud_blob.is_dense);   // test for fromROSMsg ()
  EXPECT_EQ ((size_t)cloud.points.size (), nr_p);         // test for fromROSMsg ()

  EXPECT_NEAR ((float)cloud.points[0].x, first.x, 1e-5);     // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[0].y, first.y, 1e-5);     // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[0].z, first.z, 1e-5);     // test for fromROSMsg ()
  EXPECT_NEAR ((uint32_t)cloud.points[0].intensity, first.intensity, 1e-5);  // test for fromROSMsg ()

  EXPECT_NEAR ((float)cloud.points[nr_p - 1].x, last.x, 1e-5);    // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[nr_p - 1].y, last.y, 1e-5);    // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[nr_p - 1].z, last.z, 1e-5);    // test for fromROSMsg ()
  EXPECT_NEAR ((uint32_t)cloud.points[nr_p - 1].intensity, last.intensity, 1e-5); // test for fromROSMsg ()
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PCDReaderWriter)
{
  sensor_msgs::PointCloud2 cloud_blob;
  PointCloud<PointXYZI> cloud;

  cloud.width  = 640;
  cloud.height = 480;
  cloud.points.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  srand (time (NULL));
  size_t nr_p = cloud.points.size ();
  // Randomly create a new point cloud
  for (size_t i = 0; i < nr_p; ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].intensity = i;
  }
  PointXYZI first, last;
  first.x = cloud.points[0].x;       first.y = cloud.points[0].y;       first.z = cloud.points[0].z;       first.intensity = cloud.points[0].intensity;
  last.x = cloud.points[nr_p - 1].x; last.y = cloud.points[nr_p - 1].y; last.z = cloud.points[nr_p - 1].z; last.intensity  = cloud.points[nr_p - 1].intensity;

  // Convert from data type to blob
  toROSMsg (cloud, cloud_blob);

  EXPECT_EQ ((uint32_t)cloud_blob.width, cloud.width);    // test for toROSMsg ()
  EXPECT_EQ ((uint32_t)cloud_blob.height, cloud.height);  // test for toROSMsg ()
  EXPECT_EQ ((bool)cloud_blob.is_dense, cloud.is_dense);  // test for toROSMsg ()
  //EXPECT_EQ ((size_t)cloud_blob.data.size () * 2,         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
  //           cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for toROSMsg ()
  EXPECT_EQ ((size_t)cloud_blob.data.size (),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
             cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for toROSMsg ()

  PCDWriter writer;
  writer.write ("/tmp/test_pcl_io.pcd", cloud_blob, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);

  PCDReader reader;
  reader.read ("/tmp/test_pcl_io.pcd", cloud_blob);
  EXPECT_EQ ((uint32_t)cloud_blob.width, cloud.width);
  EXPECT_EQ ((uint32_t)cloud_blob.height, cloud.height);
  EXPECT_EQ ((bool)cloud_blob.is_dense, false);   // @todo: let is_dense be false for now when reading from binary files
  //EXPECT_EQ ((size_t)cloud_blob.data.size () * 2,         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
  //           cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()
  EXPECT_EQ ((size_t)cloud_blob.data.size (),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
             cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()

  // Convert from blob to data type
  fromROSMsg (cloud_blob, cloud);

  EXPECT_EQ ((uint32_t)cloud.width, cloud_blob.width);    // test for fromROSMsg ()
  EXPECT_EQ ((uint32_t)cloud.height, cloud_blob.height);  // test for fromROSMsg ()
  EXPECT_EQ ((int)cloud.is_dense, cloud_blob.is_dense);   // test for fromROSMsg ()
  EXPECT_EQ ((size_t)cloud.points.size (), nr_p);         // test for fromROSMsg ()

  EXPECT_NEAR ((float)cloud.points[0].x, first.x, 1e-5);     // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[0].y, first.y, 1e-5);     // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[0].z, first.z, 1e-5);     // test for fromROSMsg ()
  EXPECT_NEAR ((uint32_t)cloud.points[0].intensity, first.intensity, 1e-5);  // test for fromROSMsg ()

  EXPECT_NEAR ((float)cloud.points[nr_p - 1].x, last.x, 1e-5);    // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[nr_p - 1].y, last.y, 1e-5);    // test for fromROSMsg ()
  EXPECT_NEAR ((float)cloud.points[nr_p - 1].z, last.z, 1e-5);    // test for fromROSMsg ()
  EXPECT_NEAR ((uint32_t)cloud.points[nr_p - 1].intensity, last.intensity, 1e-5); // test for fromROSMsg ()
}

struct PointXYZFPFH33
{
  float x, y, z;
  float histogram[33];
};
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZFPFH33,
    (float, x, x) (float, y, y) (float, z, z)
    (float[33], histogram, fpfh));

inline std::ostream& operator << (std::ostream& os, const PointXYZFPFH33& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << ")";
  for (int i = 0; i < 33; ++i) 
    os << (i == 0 ? "(" :"") << p.histogram[i] << (i < 32 ? ", " : ")");
  return os;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ExtendedIO)
{
  PointCloud<PointXYZFPFH33> cloud;
  cloud.points.resize (2);

  cloud.points[0].x = cloud.points[0].y = cloud.points[0].z = 1;
  cloud.points[1].x = cloud.points[1].y = cloud.points[1].z = 2;
  for (int i = 0; i < 33; ++i)
  {
    cloud.points[0].histogram[i] = i;
    cloud.points[1].histogram[i] = 33 - i;
  }

  savePCDFile ("/tmp/v.pcd", cloud);
  cloud.points.clear ();
  loadPCDFile ("/tmp/v.pcd", cloud);

  EXPECT_EQ ((int)cloud.width, 2);
  EXPECT_EQ ((int)cloud.height, 1);
  EXPECT_EQ (cloud.is_dense, true);
  EXPECT_EQ ((int)cloud.points.size (), 2);
  
  EXPECT_EQ (cloud.points[0].x, 1); EXPECT_EQ (cloud.points[0].y, 1); EXPECT_EQ (cloud.points[0].z, 1);
  EXPECT_EQ (cloud.points[1].x, 2); EXPECT_EQ (cloud.points[1].y, 2); EXPECT_EQ (cloud.points[1].z, 2);
  for (int i = 0; i < 33; ++i)
  {
    EXPECT_EQ (cloud.points[0].histogram[i], i);
    EXPECT_EQ (cloud.points[1].histogram[i], 33-i);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, EigenConversions)
{
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (5);

  for (size_t i = 0; i < cloud.points.size (); ++i)
    cloud.points[i].x = cloud.points[i].y = cloud.points[i].z = i;

  sensor_msgs::PointCloud2 blob;
  toROSMsg (cloud, blob);

  Eigen::MatrixXf mat;
  getPointCloudAsEigen (blob, mat);
  EXPECT_EQ (mat.cols (), (int)cloud.points.size ());
  EXPECT_EQ (mat.rows (), 4);
  
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    EXPECT_EQ (mat (0, i), cloud.points[i].x);
    EXPECT_EQ (mat (1, i), cloud.points[i].y);
    EXPECT_EQ (mat (2, i), cloud.points[i].z);
    EXPECT_EQ (mat (3, i), 1);
  }
  
  getEigenAsPointCloud (mat, blob);
  fromROSMsg (blob, cloud);
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    EXPECT_EQ (cloud.points[i].x, i);
    EXPECT_EQ (cloud.points[i].y, i);
    EXPECT_EQ (cloud.points[i].z, i);
  }

  getPointCloudAsEigen (blob, mat);
  EXPECT_EQ (mat.cols (), (int)cloud.points.size ());
  EXPECT_EQ (mat.rows (), 4);
  
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    EXPECT_EQ (mat (0, i), cloud.points[i].x);
    EXPECT_EQ (mat (1, i), cloud.points[i].y);
    EXPECT_EQ (mat (2, i), cloud.points[i].z);
    EXPECT_EQ (mat (3, i), 1);
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

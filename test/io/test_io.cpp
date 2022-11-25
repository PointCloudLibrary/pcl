/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <pcl/test/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/type_traits.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/obj_io.h>
#include <fstream>
#include <locale>
#include <stdexcept>

using namespace pcl;
using namespace pcl::io;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ComplexPCDFileASCII)
{
  std::ofstream fs;
  fs.open ("complex_ascii.pcd");
  fs << "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS fpfh _ x y z\n"
        "SIZE 4 1 4 4 4\n"
        "TYPE F F F F F\n"
        "COUNT 33 10 1 1 1\n"
        "WIDTH 1\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 1\n"
        "DATA ascii\n"
        "0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 -69.234001 -65.460999 19.173";
  fs.close ();

  pcl::PCLPointCloud2 blob;
  int res = loadPCDFile ("complex_ascii.pcd", blob);
  EXPECT_NE (res, -1);
  EXPECT_EQ (blob.width, 1);
  EXPECT_EQ (blob.height, 1);
  EXPECT_TRUE (blob.is_dense);
  EXPECT_EQ (blob.data.size (), 4 * 33 + 10 * 1 + 4 * 3);

  // Check fields
  EXPECT_EQ (blob.fields[0].name, "fpfh");
  EXPECT_EQ (blob.fields[0].offset, 0);
  EXPECT_EQ (blob.fields[0].count, 33);
  EXPECT_EQ (blob.fields[0].datatype, pcl::PCLPointField::FLOAT32);

  EXPECT_EQ (blob.fields[1].name, "_");
  EXPECT_EQ (blob.fields[1].offset, 4 * 33);
  EXPECT_EQ (blob.fields[1].count, 10);
  EXPECT_EQ (blob.fields[1].datatype, (std::uint8_t) -1);
  
  EXPECT_EQ (blob.fields[2].name, "x");
  EXPECT_EQ (blob.fields[2].offset, 4 * 33 + 10 * 1);
  EXPECT_EQ (blob.fields[2].count, 1);
  EXPECT_EQ (blob.fields[2].datatype, pcl::PCLPointField::FLOAT32);
  
  EXPECT_EQ (blob.fields[3].name, "y");
  EXPECT_EQ (blob.fields[3].offset, 4 * 33 + 10 * 1 + 4);
  EXPECT_EQ (blob.fields[3].count, 1);
  EXPECT_EQ (blob.fields[3].datatype, pcl::PCLPointField::FLOAT32);
  
  EXPECT_EQ (blob.fields[4].name, "z");
  EXPECT_EQ (blob.fields[4].offset, 4 * 33 + 10 * 1 + 4 + 4);
  EXPECT_EQ (blob.fields[4].count, 1);
  EXPECT_EQ (blob.fields[4].datatype, pcl::PCLPointField::FLOAT32);

  int x_idx = pcl::getFieldIndex (blob, "x");
  EXPECT_EQ (x_idx, 2);
  float x, y, z;
  memcpy (&x, &blob.data[0 * blob.point_step + blob.fields[x_idx + 0].offset], sizeof (float));
  memcpy (&y, &blob.data[0 * blob.point_step + blob.fields[x_idx + 1].offset], sizeof (float));
  memcpy (&z, &blob.data[0 * blob.point_step + blob.fields[x_idx + 2].offset], sizeof (float));
  EXPECT_FLOAT_EQ (x, -69.234001f);
  EXPECT_FLOAT_EQ (y, -65.460999f);
  EXPECT_FLOAT_EQ (z, 19.173f);

  int fpfh_idx = pcl::getFieldIndex (blob, "fpfh");
  EXPECT_EQ (fpfh_idx, 0);
  float val[33];
  for (uindex_t i = 0; i < blob.fields[fpfh_idx].count; ++i)
    memcpy (&val[i], &blob.data[0 * blob.point_step + blob.fields[fpfh_idx + 0].offset + i * sizeof (float)], sizeof (float));

  EXPECT_EQ (val[0], 0); 
  EXPECT_EQ (val[1], 0); 
  EXPECT_EQ (val[2], 0); 
  EXPECT_EQ (val[3], 0); 
  EXPECT_EQ (val[4], 0); 
  EXPECT_EQ (val[5], 100); 
  EXPECT_EQ (val[6], 0); 
  EXPECT_EQ (val[7], 0); 
  EXPECT_EQ (val[8], 0); 
  EXPECT_EQ (val[9], 0); 
  EXPECT_EQ (val[10], 0); 
  EXPECT_EQ (val[11], 0); 
  EXPECT_EQ (val[12], 0); 
  EXPECT_EQ (val[13], 0); 
  EXPECT_EQ (val[14], 0); 
  EXPECT_EQ (val[15], 0); 
  EXPECT_EQ (val[16], 100); 
  EXPECT_EQ (val[17], 0); 
  EXPECT_EQ (val[18], 0); 
  EXPECT_EQ (val[19], 0); 
  EXPECT_EQ (val[20], 0); 
  EXPECT_EQ (val[21], 0); 
  EXPECT_EQ (val[22], 0); 
  EXPECT_EQ (val[23], 0); 
  EXPECT_EQ (val[24], 0); 
  EXPECT_EQ (val[25], 0); 
  EXPECT_EQ (val[26], 0); 
  EXPECT_EQ (val[27], 100); 
  EXPECT_EQ (val[28], 0); 
  EXPECT_EQ (val[29], 0); 
  EXPECT_EQ (val[30], 0); 
  EXPECT_EQ (val[31], 0); 
  EXPECT_EQ (val[32], 0); 

  remove ("complex_ascii.pcd");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, AllTypesPCDFile)
{
  std::ofstream fs;
  fs.open ("all_types.pcd");
  fs << "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS a1 a2 a3 a4 a5 a6 a7 a8 a9 a10\n"
        "SIZE    1  1  2  2  4  4  4  8  8   8\n"
        "TYPE    I  U  I  U  I  U  F  F  I   U\n"
        "COUNT   1  2  1  2  1  2  1  2  1   2\n"
        "WIDTH 1\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 1\n"
        "DATA ascii\n"
        "-50 250 251 -250 2500 2501 -250000 250000 250001 250.05 -250.05 -251.05 -5000000000 10000000000 10000000001";
  fs.close ();

  pcl::PCLPointCloud2 blob;
  int res = loadPCDFile ("all_types.pcd", blob);
  EXPECT_NE (res, -1);
  EXPECT_EQ (blob.width, 1);
  EXPECT_EQ (blob.height, 1);
  EXPECT_EQ (blob.data.size (), 1 * 1 + 1 * 2 + // {,u}int8_t
                                2 * 1 + 2 * 2 + // {,u}in16_t
                                4 * 1 + 4 * 2 + // {,u}int32_t
                                4 * 1 + 8 * 2 + // f32, f64
                                8 * 1 + 8 * 2); // {,u}int64_t
  EXPECT_TRUE (blob.is_dense);

  EXPECT_EQ (blob.fields.size (), 10);
  // Check fields
  EXPECT_EQ (blob.fields[0].name, "a1");
  EXPECT_EQ (blob.fields[1].name, "a2");
  EXPECT_EQ (blob.fields[2].name, "a3");
  EXPECT_EQ (blob.fields[3].name, "a4");
  EXPECT_EQ (blob.fields[4].name, "a5");
  EXPECT_EQ (blob.fields[5].name, "a6");
  EXPECT_EQ (blob.fields[6].name, "a7");
  EXPECT_EQ (blob.fields[7].name, "a8");
  EXPECT_EQ (blob.fields[8].name, "a9");
  EXPECT_EQ (blob.fields[9].name, "a10");

  EXPECT_EQ (blob.fields[0].offset, 0);
  EXPECT_EQ (blob.fields[1].offset, blob.fields[0].offset + 1);      // 1 int8_t
  EXPECT_EQ (blob.fields[2].offset, blob.fields[1].offset + 1 * 2);  // 2 uint8_t
  EXPECT_EQ (blob.fields[3].offset, blob.fields[2].offset + 2 * 1);  // 1 int16_t
  EXPECT_EQ (blob.fields[4].offset, blob.fields[3].offset + 2 * 2);  // 2 uint16_t
  EXPECT_EQ (blob.fields[5].offset, blob.fields[4].offset + 4 * 1);  // 1 int32_t
  EXPECT_EQ (blob.fields[6].offset, blob.fields[5].offset + 4 * 2);  // 2 uint32_t
  EXPECT_EQ (blob.fields[7].offset, blob.fields[6].offset + 4 * 1);  // 1 float
  EXPECT_EQ (blob.fields[8].offset, blob.fields[7].offset + 8 * 2);  // 2 doubles
  EXPECT_EQ (blob.fields[9].offset, blob.fields[8].offset + 8 * 1);  // 1 int64_t

  EXPECT_EQ (blob.fields[0].count, 1);
  EXPECT_EQ (blob.fields[1].count, 2);
  EXPECT_EQ (blob.fields[2].count, 1);
  EXPECT_EQ (blob.fields[3].count, 2);
  EXPECT_EQ (blob.fields[4].count, 1);
  EXPECT_EQ (blob.fields[5].count, 2);
  EXPECT_EQ (blob.fields[6].count, 1);
  EXPECT_EQ (blob.fields[7].count, 2);
  EXPECT_EQ (blob.fields[8].count, 1);
  EXPECT_EQ (blob.fields[9].count, 2);

  EXPECT_EQ (blob.fields[0].datatype, pcl::PCLPointField::INT8);
  EXPECT_EQ (blob.fields[1].datatype, pcl::PCLPointField::UINT8);
  EXPECT_EQ (blob.fields[2].datatype, pcl::PCLPointField::INT16);
  EXPECT_EQ (blob.fields[3].datatype, pcl::PCLPointField::UINT16);
  EXPECT_EQ (blob.fields[4].datatype, pcl::PCLPointField::INT32);
  EXPECT_EQ (blob.fields[5].datatype, pcl::PCLPointField::UINT32);
  EXPECT_EQ (blob.fields[6].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (blob.fields[7].datatype, pcl::PCLPointField::FLOAT64);
  EXPECT_EQ (blob.fields[8].datatype, pcl::PCLPointField::INT64);
  EXPECT_EQ (blob.fields[9].datatype, pcl::PCLPointField::UINT64);

  std::int8_t b1;
  std::uint8_t b2;
  std::int16_t b3;
  std::uint16_t b4;
  std::int32_t b5;
  std::uint32_t b6;
  float b7;
  double b8;
  std::int64_t b9;
  std::uint64_t b10;

  memcpy (&b1, &blob.data[blob.fields[0].offset], sizeof (std::int8_t));
  EXPECT_FLOAT_EQ (b1, -50);
  memcpy (&b2, &blob.data[blob.fields[1].offset], sizeof (std::uint8_t));
  EXPECT_FLOAT_EQ (b2, 250);
  memcpy (&b2, &blob.data[blob.fields[1].offset + sizeof (std::uint8_t)], sizeof (std::uint8_t));
  EXPECT_FLOAT_EQ (b2, 251);

  memcpy (&b3, &blob.data[blob.fields[2].offset], sizeof (std::int16_t));
  EXPECT_FLOAT_EQ (b3, -250);
  memcpy (&b4, &blob.data[blob.fields[3].offset], sizeof (std::uint16_t));
  EXPECT_FLOAT_EQ (b4, 2500);
  memcpy (&b4, &blob.data[blob.fields[3].offset + sizeof (std::uint16_t)], sizeof (std::uint16_t));
  EXPECT_FLOAT_EQ (b4, 2501);

  memcpy (&b5, &blob.data[blob.fields[4].offset], sizeof (std::int32_t));
  EXPECT_FLOAT_EQ (float (b5), float (-250000));
  memcpy (&b6, &blob.data[blob.fields[5].offset], sizeof (std::uint32_t));
  EXPECT_FLOAT_EQ (float (b6), float (250000));
  memcpy (&b6, &blob.data[blob.fields[5].offset + sizeof (std::uint32_t)], sizeof (std::uint32_t));
  EXPECT_FLOAT_EQ (float (b6), float (250001));

  memcpy (&b7, &blob.data[blob.fields[6].offset], sizeof (float));
  EXPECT_FLOAT_EQ (b7, 250.05f);
  memcpy (&b8, &blob.data[blob.fields[7].offset], sizeof (double));
  EXPECT_FLOAT_EQ (float (b8), -250.05f);
  memcpy (&b8, &blob.data[blob.fields[7].offset + sizeof (double)], sizeof (double));
  EXPECT_FLOAT_EQ (float (b8), -251.05f);

  memcpy (&b9, &blob.data[blob.fields[8].offset], sizeof (std::int64_t));
  EXPECT_EQ (b9, -5000000000);
  memcpy (&b10, &blob.data[blob.fields[9].offset], sizeof (std::uint64_t));
  EXPECT_EQ (b10, 10000000000);
  memcpy (&b10, &blob.data[blob.fields[9].offset + sizeof (std::uint64_t)], sizeof (std::uint64_t));
  EXPECT_EQ (b10, 10000000001);

  remove ("all_types.pcd");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConcatenatePoints)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;

  // Fill in the cloud data
  cloud_a.width  = 5;
  cloud_b.width  = 3;
  cloud_a.height = cloud_b.height = 1;
  cloud_a.resize (cloud_a.width * cloud_a.height);
  cloud_b.resize (cloud_b.width * cloud_b.height);

  for (auto &point : cloud_a.points)
  {
    point.x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    point.y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    point.z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
  }

  for (auto &point : cloud_b.points)
  {
    point.x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    point.y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    point.z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
  }

  // Copy the point cloud data
  cloud_c  = cloud_a;
  cloud_c += cloud_b;
  EXPECT_EQ (cloud_c.size (), cloud_a.size () + cloud_b.size ());
  EXPECT_EQ (cloud_c.width, cloud_a.width + cloud_b.width);
  EXPECT_EQ (cloud_c.height, 1);

  for (std::size_t i = 0; i < cloud_a.size (); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_c[i].x, cloud_a[i].x);
    EXPECT_FLOAT_EQ (cloud_c[i].y, cloud_a[i].y);
    EXPECT_FLOAT_EQ (cloud_c[i].z, cloud_a[i].z);
  }
  for (std::size_t i = cloud_a.size (); i < cloud_c.size (); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_c[i].x, cloud_b[i - cloud_a.size ()].x);
    EXPECT_FLOAT_EQ (cloud_c[i].y, cloud_b[i - cloud_a.size ()].y);
    EXPECT_FLOAT_EQ (cloud_c[i].z, cloud_b[i - cloud_a.size ()].z);
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
  cloud_a.resize (cloud_a.width * cloud_a.height);
  cloud_b.resize (cloud_b.width * cloud_b.height);

  for (auto& point: cloud_a)
  {
    point.x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    point.y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    point.z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
  }

  for (auto& point: cloud_b)
  {
    point.normal_x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    point.normal_y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    point.normal_z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
  }

  pcl::concatenateFields (cloud_a, cloud_b, cloud_c);
  EXPECT_EQ (cloud_c.size (), cloud_a.size ());
  EXPECT_EQ (cloud_c.width, cloud_a.width);
  EXPECT_EQ (cloud_c.height, cloud_a.height);

  for (std::size_t i = 0; i < cloud_a.size (); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_c[i].x, cloud_a[i].x);
    EXPECT_FLOAT_EQ (cloud_c[i].y, cloud_a[i].y);
    EXPECT_FLOAT_EQ (cloud_c[i].z, cloud_a[i].z);
    EXPECT_FLOAT_EQ (cloud_c[i].normal[0], cloud_b[i].normal[0]);
    EXPECT_FLOAT_EQ (cloud_c[i].normal[1], cloud_b[i].normal[1]);
    EXPECT_FLOAT_EQ (cloud_c[i].normal[2], cloud_b[i].normal[2]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IO)
{
  pcl::PCLPointCloud2 cloud_blob;
  PointCloud<PointXYZI> cloud;

  cloud.width  = 640;
  cloud.height = 480;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  srand (static_cast<unsigned int> (time (nullptr)));
  const auto nr_p = cloud.size ();
  // Randomly create a new point cloud
  for (std::size_t i = 0; i < nr_p; ++i)
  {
    cloud[i].x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].intensity = static_cast<float> (i);
  }
  PointXYZI first, last;
  first.x = cloud[0].x;       first.y = cloud[0].y;       first.z = cloud[0].z;       first.intensity = cloud[0].intensity;
  last.x = cloud[nr_p - 1].x; last.y = cloud[nr_p - 1].y; last.z = cloud[nr_p - 1].z; last.intensity  = cloud[nr_p - 1].intensity;

  // Tests for PointCloud::operator()
  EXPECT_FLOAT_EQ (first.x, cloud (0, 0).x);
  EXPECT_FLOAT_EQ (first.y, cloud (0, 0).y);
  EXPECT_FLOAT_EQ (first.z, cloud (0, 0).z);
  EXPECT_FLOAT_EQ (first.intensity, 0.0f);
  EXPECT_FLOAT_EQ (last.x, cloud (cloud.width-1, cloud.height-1).x);
  EXPECT_FLOAT_EQ (last.y, cloud (cloud.width-1, cloud.height-1).y);
  EXPECT_FLOAT_EQ (last.z, cloud (cloud.width-1, cloud.height-1).z);
  EXPECT_FLOAT_EQ (last.intensity, static_cast<float> (nr_p - 1));

  // Test getFieldIndex
  const auto fields = pcl::getFields<PointXYZI> ();
  EXPECT_EQ (fields.size (), std::size_t (4));
  int x_idx = pcl::getFieldIndex<PointXYZI> ("x", fields);
  EXPECT_EQ (x_idx, 0);
  EXPECT_EQ (fields[x_idx].offset, 0);
  EXPECT_EQ (fields[x_idx].name, "x");
  EXPECT_EQ (fields[x_idx].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (fields[x_idx].count, 1);

  int y_idx = pcl::getFieldIndex<PointXYZI> ("y", fields);
  EXPECT_EQ (y_idx, 1);
  EXPECT_EQ (fields[y_idx].offset, 4);
  EXPECT_EQ (fields[y_idx].name, "y");
  EXPECT_EQ (fields[y_idx].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (fields[y_idx].count, 1);

  int z_idx = pcl::getFieldIndex<PointXYZI> ("z", fields);
  EXPECT_EQ (z_idx, 2);
  EXPECT_EQ (fields[z_idx].offset, 8);
  EXPECT_EQ (fields[z_idx].name, "z");
  EXPECT_EQ (fields[z_idx].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (fields[z_idx].count, 1);

  int intensity_idx = pcl::getFieldIndex<PointXYZI> ("intensity", fields);
  EXPECT_EQ (intensity_idx, 3);
  EXPECT_EQ (fields[intensity_idx].offset, 16);      // NOTE: intensity_idx.offset should be 12, but we are padding in PointXYZ (!)
  EXPECT_EQ (fields[intensity_idx].name, "intensity");
  EXPECT_EQ (fields[intensity_idx].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (fields[intensity_idx].count, 1);

  // Convert from data type to blob
  toPCLPointCloud2 (cloud, cloud_blob);

  // Test getFieldIndex
  x_idx = pcl::getFieldIndex (cloud_blob, "x");
  EXPECT_EQ (x_idx, 0);
  EXPECT_EQ (cloud_blob.fields[x_idx].offset, 0);
  EXPECT_EQ (cloud_blob.fields[x_idx].name, "x");
  EXPECT_EQ (cloud_blob.fields[x_idx].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (cloud_blob.fields[x_idx].count, 1);
  y_idx = pcl::getFieldIndex (cloud_blob, "y");
  EXPECT_EQ (y_idx, 1);
  EXPECT_EQ (cloud_blob.fields[y_idx].offset, 4);
  EXPECT_EQ (cloud_blob.fields[y_idx].name, "y");
  EXPECT_EQ (cloud_blob.fields[y_idx].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (cloud_blob.fields[y_idx].count, 1);
  z_idx = pcl::getFieldIndex (cloud_blob, "z");
  EXPECT_EQ (z_idx, 2);
  EXPECT_EQ (cloud_blob.fields[z_idx].offset, 8);
  EXPECT_EQ (cloud_blob.fields[z_idx].name, "z");
  EXPECT_EQ (cloud_blob.fields[z_idx].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (cloud_blob.fields[z_idx].count, 1);
  intensity_idx = pcl::getFieldIndex (cloud_blob, "intensity");
  EXPECT_EQ (intensity_idx, 3);
  //EXPECT_EQ (cloud_blob.fields[intensity_idx].offset, (std::uint32_t)12);      // NOTE: the fields.offset is 16 in PointCloud<PointXYZI>, but we are obtaining the correct offset in toPCLPointCloud2
  EXPECT_EQ (cloud_blob.fields[intensity_idx].offset, 16);      // NOTE: the fields.offset is 16 in PointCloud<PointXYZI>, but we are obtaining the correct offset in toPCLPointCloud2
  EXPECT_EQ (cloud_blob.fields[intensity_idx].name, "intensity");
  EXPECT_EQ (cloud_blob.fields[intensity_idx].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (cloud_blob.fields[intensity_idx].count, 1);
  
  fromPCLPointCloud2 (cloud_blob, cloud);
  for (std::size_t i = 0; i < nr_p; ++i)
    EXPECT_EQ (cloud[i].intensity, i);

  EXPECT_EQ (cloud_blob.width, cloud.width);    // test for toPCLPointCloud2 ()
  EXPECT_EQ (cloud_blob.height, cloud.height);  // test for toPCLPointCloud2 ()
  EXPECT_EQ (bool (cloud_blob.is_dense), cloud.is_dense);  // test for toPCLPointCloud2 ()
  //EXPECT_EQ ((std::size_t)cloud_blob.data.size () * 2,         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
  //           cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for toPCLPointCloud2 ()
  EXPECT_EQ (std::size_t (cloud_blob.data.size ()),             // PointXYZI is 16*2 (XYZ+1, Intensity+3)
             cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for toPCLPointCloud2 ()

  // Make sure we have permissions to write there
  PCDWriter w;
  int res = w.writeASCII ("test_pcl_io.pcd", cloud_blob, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), 10);
  EXPECT_EQ (res, 0);                            // test for savePCDFileASCII ()

  // Please make sure that this file exists, otherwise the test will fail.
  res = loadPCDFile ("test_pcl_io.pcd", cloud_blob);
  EXPECT_NE (res, -1);                               // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.width, cloud.width);    // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.height, cloud.height);  // test for loadPCDFile ()
  EXPECT_EQ (bool (cloud_blob.is_dense), cloud.is_dense);  // test for loadPCDFile ()
  EXPECT_EQ (std::size_t (cloud_blob.data.size () * 2),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
              cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()

  // Convert from blob to data type
  fromPCLPointCloud2 (cloud_blob, cloud);

  EXPECT_EQ (cloud.width, cloud_blob.width);    // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.height, cloud_blob.height);  // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.is_dense, cloud_blob.is_dense);   // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.size (), nr_p);         // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[0].x, first.x);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].y, first.y);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].z, first.z);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].intensity, first.intensity);  // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[nr_p - 1].x, last.x);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].y, last.y);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].z, last.z);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].intensity, last.intensity); // test for fromPCLPointCloud2 ()

  // Make sure we have permissions to write there
  res = savePCDFile ("test_pcl_io.pcd", cloud_blob, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
  EXPECT_EQ (res, 0);                            // test for savePCDFileBinary ()

  // Please make sure that this file exists, otherwise the test will fail.
  res = loadPCDFile ("test_pcl_io.pcd", cloud_blob);
  EXPECT_NE (res, -1);                               // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.width, cloud.width);    // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.height, cloud.height);  // test for loadPCDFile ()
  EXPECT_EQ (bool (cloud_blob.is_dense), cloud.is_dense);
  EXPECT_EQ (std::size_t (cloud_blob.data.size () * 2),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
              cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()
  remove ("test_pcl_io.pcd");

  // Convert from blob to data type
  fromPCLPointCloud2 (cloud_blob, cloud);

  EXPECT_EQ (cloud.width, cloud_blob.width);    // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.height, cloud_blob.height);  // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.is_dense, cloud_blob.is_dense);   // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.size (), nr_p);         // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[0].x, first.x);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].y, first.y);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].z, first.z);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].intensity, first.intensity);  // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[nr_p - 1].x, last.x);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].y, last.y);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].z, last.z);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].intensity, last.intensity); // test for fromPCLPointCloud2 ()

  // Save as binary
  try
  {
    w.write<PointXYZI> ("test_pcl_io_binary.pcd", cloud, true);
  }
  catch (pcl::IOException &e)
  {
    std::cerr << e.detailedMessage () << std::endl;
  }
  res = loadPCDFile ("test_pcl_io_binary.pcd", cloud_blob);
  EXPECT_NE (res, -1);                               // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.width, cloud.width);    // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.height, cloud.height);  // test for loadPCDFile ()
  EXPECT_EQ (bool (cloud_blob.is_dense), cloud.is_dense);
  EXPECT_EQ (std::size_t (cloud_blob.data.size () * 2),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
              cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()

  // Convert from blob to data type
  fromPCLPointCloud2 (cloud_blob, cloud);

  EXPECT_EQ (cloud.width, cloud_blob.width);    // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.height, cloud_blob.height);  // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.is_dense, cloud_blob.is_dense);   // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.size (), nr_p);         // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[0].x, first.x);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].y, first.y);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].z, first.z);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].intensity, first.intensity);  // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[nr_p - 1].x, last.x);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].y, last.y);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].z, last.z);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].intensity, last.intensity); // test for fromPCLPointCloud2 ()

  // Save as ASCII
  try
  {
    w.write<PointXYZI> ("test_pcl_io_ascii.pcd", cloud, false);
  }
  catch (pcl::IOException &e)
  {
    std::cerr << e.detailedMessage () << std::endl;
  }
  res = loadPCDFile ("test_pcl_io_ascii.pcd", cloud_blob);
  EXPECT_NE (res, -1);                               // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.width, cloud.width);    // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.height, cloud.height);  // test for loadPCDFile ()
  EXPECT_TRUE (cloud_blob.is_dense);
  EXPECT_EQ (std::size_t (cloud_blob.data.size () * 2),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
              cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()

  // Convert from blob to data type
  fromPCLPointCloud2 (cloud_blob, cloud);

  EXPECT_EQ (cloud.width, cloud_blob.width);    // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.height, cloud_blob.height);  // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.is_dense, cloud_blob.is_dense);   // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.size (), nr_p);         // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[0].x, first.x);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].y, first.y);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].z, first.z);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].intensity, first.intensity);  // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[nr_p - 1].x, last.x);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].y, last.y);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].z, last.z);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (float (cloud[nr_p - 1].intensity), float (last.intensity)); // test for fromPCLPointCloud2 ()

  pcl::Indices indices (cloud.width * cloud.height / 2);
  for (int i = 0; i < static_cast<int> (indices.size ()); ++i) indices[i] = i;
  // Save as binary
  try
  {
    w.write<PointXYZI> ("test_pcl_io_binary.pcd", cloud, indices, true);
  }
  catch (pcl::IOException &e)
  {
    std::cerr << e.detailedMessage () << std::endl;
  }
  res = loadPCDFile ("test_pcl_io_binary.pcd", cloud_blob);
  EXPECT_NE (res, -1);                               // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.width, cloud.width * cloud.height / 2);    // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.height, 1);  // test for loadPCDFile ()
  EXPECT_EQ (bool (cloud_blob.is_dense), cloud.is_dense);
  EXPECT_EQ (std::size_t (cloud_blob.data.size () * 2),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
              cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()
  remove ("test_pcl_io_binary.pcd");

  // Convert from blob to data type
  fromPCLPointCloud2 (cloud_blob, cloud);

  EXPECT_EQ (cloud.width, cloud_blob.width);    // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.height, cloud_blob.height);  // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.is_dense, cloud_blob.is_dense);   // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.size (), nr_p / 2);         // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[0].x, first.x);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].y, first.y);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].z, first.z);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (float (cloud[0].intensity), float (first.intensity));  // test for fromPCLPointCloud2 ()

  indices.resize (cloud.width * cloud.height / 2);
  for (int i = 0; i < static_cast<int> (indices.size ()); ++i) indices[i] = i;
  // Save as ASCII
  try
  {
    w.write<PointXYZI> ("test_pcl_io_ascii.pcd", cloud, indices, false);
  }
  catch (pcl::IOException &e)
  {
    std::cerr << e.detailedMessage () << std::endl;
  }
  res = loadPCDFile ("test_pcl_io_ascii.pcd", cloud_blob);
  EXPECT_NE (res, -1);                               // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.width, cloud.width * cloud.height / 2);    // test for loadPCDFile ()
  EXPECT_EQ (cloud_blob.height, 1);  // test for loadPCDFile ()
  EXPECT_TRUE (cloud_blob.is_dense);
  EXPECT_EQ (std::size_t (cloud_blob.data.size () * 2),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
              cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()
  remove ("test_pcl_io_ascii.pcd");

  // Convert from blob to data type
  fromPCLPointCloud2 (cloud_blob, cloud);

  EXPECT_EQ (cloud.width, cloud_blob.width);    // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.height, cloud_blob.height);  // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.is_dense, cloud_blob.is_dense);   // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.size (), nr_p / 4);         // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[0].x, first.x);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].y, first.y);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].z, first.z);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].intensity, first.intensity);  // test for fromPCLPointCloud2 ()
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, EmptyCloudToPCD)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int res = pcl::io::savePCDFileASCII("ascii.pcd", cloud);
  EXPECT_EQ (res, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud_in_ascii;
  cloud_in_ascii.width = 10; // Make sure loadPCDFile overwrites this
  cloud_in_ascii.height = 10; // Make sure loadPCDFile overwrites this
  res = pcl::io::loadPCDFile("ascii.pcd", cloud_in_ascii);
  EXPECT_EQ (0, res);
  EXPECT_EQ(cloud.width, cloud_in_ascii.width);
  EXPECT_EQ(cloud.height, cloud_in_ascii.height);
  remove ("ascii.pcd");
  
  pcl::Indices indices;
  res = pcl::io::savePCDFile("ascii_indices.pcd", cloud, indices);
  EXPECT_EQ (0, res);
  pcl::PointCloud<pcl::PointXYZ> cloud_in_indices;
  cloud_in_indices.width = 10; // Make sure loadPCDFile overwrites this
  res = pcl::io::loadPCDFile("ascii_indices.pcd", cloud_in_indices);
  EXPECT_EQ (0, res);
  EXPECT_EQ(cloud.width, cloud_in_indices.width);
  EXPECT_EQ(1, cloud_in_indices.height); // if we specify indices height must be 1
  remove ("ascii_indices.pcd");
  
  res = pcl::io::savePCDFileBinary("binary.pcd", cloud);
  EXPECT_EQ (0, res);
  pcl::PointCloud<pcl::PointXYZ> cloud_in_binary;
  cloud_in_binary.width = 10; // Make sure loadPCDFile overwrites this
  cloud_in_binary.height = 10; // Make sure loadPCDFile overwrites this
  res = pcl::io::loadPCDFile("binary.pcd", cloud_in_binary);
  EXPECT_EQ (0, res);
  EXPECT_EQ(cloud.width, cloud_in_binary.width);
  EXPECT_EQ(cloud.height, cloud_in_binary.height);
  remove ("binary.pcd");

  res = pcl::io::savePCDFileBinaryCompressed("binary_compressed.pcd", cloud);
  EXPECT_EQ (0, res);
  pcl::PointCloud<pcl::PointXYZ> cloud_in_compressed;
  cloud_in_compressed.width = 10; // Make sure loadPCDFile overwrites this
  cloud_in_compressed.height = 10; // Make sure loadPCDFile overwrites this
  res = pcl::io::loadPCDFile("binary_compressed.pcd", cloud_in_compressed);
  EXPECT_EQ (0, res);
  EXPECT_EQ(cloud.width, cloud_in_compressed.width);
  EXPECT_EQ(cloud.height, cloud_in_compressed.height);
  remove ("binary_compressed.pcd");

  // Data initialization for pcl::PCLPointCloud2 interface
  pcl::PCLPointCloud2 cloud2;
  pcl::PCLPointField x, y, z;
  x.name = "x";
  x.datatype = pcl::PCLPointField::FLOAT32;
  y.name = "y";
  y.datatype = pcl::PCLPointField::FLOAT32;
  z.name = "z";
  z.datatype = pcl::PCLPointField::FLOAT32;
  cloud2.fields.push_back(x);
  cloud2.fields.push_back(y);
  cloud2.fields.push_back(z);
  cloud2.is_dense = true;

  res = pcl::io::savePCDFile ("ascii_pc2.pcd", cloud2,
                              Eigen::Vector4f::Zero (),
                              Eigen::Quaternionf::Identity ());
  EXPECT_EQ (0, res);
  pcl::PCLPointCloud2 cloud2_in_ascii;
  cloud2_in_ascii.width = 10;
  cloud2_in_ascii.height = 10;
  res = loadPCDFile ("ascii_pc2.pcd", cloud2_in_ascii);
  EXPECT_EQ (0, res);
  EXPECT_EQ (cloud2.width, cloud2_in_ascii.width);
  EXPECT_EQ (cloud2.height, cloud2_in_ascii.height);
  remove ("ascii_pc2.pcd");

  res = pcl::io::savePCDFile ("binary_pc2.pcd", cloud2,
                              Eigen::Vector4f::Zero (),
                              Eigen::Quaternionf::Identity (),
                              true);
  EXPECT_EQ (0, res);

  pcl::PCLPointCloud2 cloud2_in_binary;
  cloud2_in_binary.width = 10;
  cloud2_in_binary.height = 10;
  res = loadPCDFile ("binary_pc2.pcd", cloud2_in_binary);
  EXPECT_EQ (0, res);
  EXPECT_EQ (cloud2.width, cloud2_in_binary.width);
  EXPECT_EQ (cloud2.height, cloud2_in_binary.height);
  remove ("binary_pc2.pcd");

  PCDWriter w;
  res = w.writeBinaryCompressed ("compressed_pc2.pcd", cloud2);
  EXPECT_EQ (0, res);
  pcl::PCLPointCloud2 cloud2_in_compressed;
  cloud2_in_compressed.width = 10;
  cloud2_in_compressed.height = 10;
  res = loadPCDFile ("compressed_pc2.pcd", cloud2_in_compressed);
  EXPECT_EQ (0, res);
  EXPECT_EQ (cloud2.width, cloud2_in_compressed.width);
  EXPECT_EQ (cloud2.height, cloud2_in_compressed.height);
  remove ("compressed_pc2.pcd");

  // Test when WIDTH and HEIGHT are not defined
  std::ofstream fs;
  fs.open ("incomplete_ascii.pcd");
  fs << "# .PCD v0.5 - Point Cloud Data file format\n"
        "VERSION 0.5\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "POINTS 2\n"
        "DATA ascii\n"
        "1 2 3 4\n"
        "5 6 7 8";
  fs.close ();
  pcl::PCLPointCloud2 incomplete_cloud2_in;
  res = loadPCDFile ("incomplete_ascii.pcd", incomplete_cloud2_in);
  EXPECT_EQ (0, res);
  EXPECT_EQ (2, incomplete_cloud2_in.width);
  EXPECT_EQ (1, incomplete_cloud2_in.height);
  EXPECT_EQ (true, bool (incomplete_cloud2_in.is_dense));
  EXPECT_EQ (2 * 4 * 4, std::size_t (incomplete_cloud2_in.data.size ()));
  remove ("incomplete_ascii.pcd");

  // Test when HEIGHT are not defined
  fs.open ("incomplete_height_ascii.pcd");
  fs << "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH 2\n"
        "POINTS 2\n"
        "DATA ascii\n"
        "1 2 3 4\n"
        "5 6 7 8";
  fs.close ();
  pcl::PCLPointCloud2 incomplete_height_cloud2_in;
  res = loadPCDFile ("incomplete_height_ascii.pcd", incomplete_height_cloud2_in);
  EXPECT_EQ (0, res);
  EXPECT_EQ (2, incomplete_height_cloud2_in.width);
  EXPECT_EQ (1, incomplete_height_cloud2_in.height);
  EXPECT_EQ (true, bool (incomplete_height_cloud2_in.is_dense));
  EXPECT_EQ (2 * 4 * 4, std::size_t (incomplete_height_cloud2_in.data.size ()));
  remove ("incomplete_height_ascii.pcd");

  // Test invalid height
  fs.open ("invalid_height_ascii.pcd");
  fs << "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH 2\n"
        "HEIGHT a\n"
        "POINTS 2\n"
        "DATA ascii\n"
        "1 2 3 4\n"
        "5 6 7 8";
  fs.close ();
  pcl::PCLPointCloud2 invalid_height_cloud2_in;
  res = loadPCDFile ("invalid_height_ascii.pcd", invalid_height_cloud2_in);
  EXPECT_EQ (-1, res);
  remove ("invalid_height_ascii.pcd");

  // Test for no field data
  pcl::PCLPointCloud2 empty_cloud;
  res = pcl::io::savePCDFile ("empty_cloud_ascii.pcd", empty_cloud,
                              Eigen::Vector4f::Zero (),
                              Eigen::Quaternionf::Identity ());
  EXPECT_EQ (-1, res);
  remove ("empty_cloud_ascii.pcd");

  res = pcl::io::savePCDFile ("empty_cloud_binary.pcd", empty_cloud,
                              Eigen::Vector4f::Zero (),
                              Eigen::Quaternionf::Identity (),
                              true);
  EXPECT_EQ (-1, res);
  remove ("empty_cloud_binary.pcd");

  EXPECT_THROW(w.writeBinaryCompressed ("empty_cloud_compressed.pcd", empty_cloud),
               pcl::IOException);
  remove ("empty_cloud_compressed.pcd");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PCDReaderWriter)
{
  pcl::PCLPointCloud2 cloud_blob;
  PointCloud<PointXYZI> cloud;

  cloud.width  = 640;
  cloud.height = 480;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  srand (static_cast<unsigned int> (time (nullptr)));
  const auto nr_p = cloud.size ();
  // Randomly create a new point cloud
  for (std::size_t i = 0; i < nr_p; ++i)
  {
    cloud[i].x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].intensity = static_cast<float> (i);
  }
  PointXYZI first, last;
  first.x = cloud[0].x;       first.y = cloud[0].y;       first.z = cloud[0].z;       first.intensity = cloud[0].intensity;
  last.x = cloud[nr_p - 1].x; last.y = cloud[nr_p - 1].y; last.z = cloud[nr_p - 1].z; last.intensity  = cloud[nr_p - 1].intensity;

  // Convert from data type to blob
  toPCLPointCloud2 (cloud, cloud_blob);

  EXPECT_EQ (cloud_blob.width, cloud.width);    // test for toPCLPointCloud2 ()
  EXPECT_EQ (cloud_blob.height, cloud.height);  // test for toPCLPointCloud2 ()
  EXPECT_EQ (bool (cloud_blob.is_dense), cloud.is_dense);  // test for toPCLPointCloud2 ()
  //EXPECT_EQ ((std::size_t)cloud_blob.data.size () * 2,         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
  //           cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for toPCLPointCloud2 ()
  EXPECT_EQ (std::size_t (cloud_blob.data.size ()),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
             cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for toPCLPointCloud2 ()

  PCDWriter writer;
  writer.write ("test_pcl_io.pcd", cloud_blob, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);

  PCDReader reader;
  reader.read ("test_pcl_io.pcd", cloud_blob);
  EXPECT_EQ (cloud_blob.width, cloud.width);
  EXPECT_EQ (cloud_blob.height, cloud.height);
  EXPECT_EQ (bool (cloud_blob.is_dense), cloud.is_dense);
  //EXPECT_EQ ((std::size_t)cloud_blob.data.size () * 2,         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
  //           cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()
  EXPECT_EQ (std::size_t (cloud_blob.data.size ()),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
             cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));  // test for loadPCDFile ()

  // Convert from blob to data type
  fromPCLPointCloud2 (cloud_blob, cloud);

  EXPECT_EQ (cloud.width, cloud_blob.width);    // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.height, cloud_blob.height);  // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.is_dense, cloud_blob.is_dense);   // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.size (), nr_p);         // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[0].x, first.x);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].y, first.y);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].z, first.z);     // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[0].intensity, first.intensity);  // test for fromPCLPointCloud2 ()

  EXPECT_FLOAT_EQ (cloud[nr_p - 1].x, last.x);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].y, last.y);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].z, last.z);    // test for fromPCLPointCloud2 ()
  EXPECT_FLOAT_EQ (cloud[nr_p - 1].intensity, last.intensity); // test for fromPCLPointCloud2 ()

  remove ("test_pcl_io.pcd");
}

TEST (PCL, PCDReaderWriterASCIIColorPrecision)
{
  PointCloud<PointXYZRGB> cloud;
  cloud.reserve (256 / 4 * 256 / 4 * 256 / 4 * 256 / 16);
  for (std::size_t r_i = 0; r_i < 256; r_i += 5)
    for (std::size_t g_i = 0; g_i < 256; g_i += 5)
      for (std::size_t b_i = 0; b_i < 256; b_i += 5)
          for (std::size_t a_i = 0; a_i < 256; a_i += 10)
          {
            PointXYZRGB p;
            p.r = static_cast<unsigned char> (r_i);
            p.g = static_cast<unsigned char> (g_i);
            p.b = static_cast<unsigned char> (b_i);
            p.a = static_cast<unsigned char> (a_i);
            p.x = p.y = p.z = 0.f;

            cloud.push_back (p);
          }
  cloud.height = 1;
  cloud.width = cloud.size ();
  cloud.is_dense = true;

  io::savePCDFile ("temp_binary_color.pcd", cloud, true);
  PointCloud<PointXYZRGB> cloud_binary;
  io::loadPCDFile ("temp_binary_color.pcd", cloud_binary);
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    EXPECT_EQ (cloud[i].r, cloud_binary[i].r);
    EXPECT_EQ (cloud[i].g, cloud_binary[i].g);
    EXPECT_EQ (cloud[i].b, cloud_binary[i].b);
  }

  io::savePCDFile ("temp_ascii_color.pcd", cloud, false);
  PointCloud<PointXYZRGB> cloud_ascii;
  io::loadPCDFile ("temp_ascii_color.pcd", cloud_ascii);
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    EXPECT_EQ (cloud[i].r, cloud_ascii[i].r);
    EXPECT_EQ (cloud[i].g, cloud_ascii[i].g);
    EXPECT_EQ (cloud[i].b, cloud_ascii[i].b);
  }

  remove ("temp_binary_color.pcd");
  remove ("temp_ascii_color.pcd");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ASCIIRead)
{
  PointCloud<PointXYZI> cloud, rcloud;

  cloud.width  = 300;
  cloud.height = 1;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;


  std::fstream afile ("test_pcd.txt", std::iostream::out);

  afile<< std::setprecision(10);

  srand (static_cast<unsigned int> (time (nullptr)));
  const auto nr_p = cloud.size ();
  // Randomly create a new point cloud
  for (std::size_t i = 0; i < nr_p; ++i)
  {
    cloud[i].x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].intensity = static_cast<float> (i);
   afile << cloud[i].x << " , " << cloud[i].y  << " , " << cloud[i].z << " , "  << cloud[i].intensity << " \n";
  }
  afile.close();

  ASCIIReader reader;
  reader.setInputFields<pcl::PointXYZI> ();

  EXPECT_GE(reader.read("test_pcd.txt", rcloud), 0);
  EXPECT_EQ(cloud.size(), rcloud.size() );

  for(std::size_t i=0;i < rcloud.size(); i++){
    EXPECT_FLOAT_EQ(cloud[i].x, rcloud[i].x);
    EXPECT_FLOAT_EQ(cloud[i].y,rcloud[i].y);
    EXPECT_FLOAT_EQ(cloud[i].z, rcloud[i].z);
    EXPECT_FLOAT_EQ(cloud[i].intensity, rcloud[i].intensity);
  }

  remove ("test_pcd.txt");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(PCL, OBJRead)
{
  std::ofstream fs;
  fs.open ("test_obj.obj");
  fs << "# Blender v2.79 (sub 4) OBJ File: ''\n"
        "mtllib test_obj.mtl\n"
        "o Cube_Cube.001\n"
        "v -1.000000 -1.000000 1.000000\n"
        "v -1.000000 1.000000 1.000000\n"
        "v -1.000000 -1.000000 -1.000000\n"
        "v -1.000000 1.000000 -1.000000\n"
        "v 1.000000 -1.000000 1.000000\n"
        "v 1.000000 1.000000 1.000000\n"
        "v 1.000000 -1.000000 -1.000000\n"
        "v 1.000000 1.000000 -1.000000\n"
        "vn -1.0000 0.0000 0.0000\n"
        "vn 0.0000 0.0000 -1.0000\n"
        "vn 1.0000 0.0000 0.0000\n"
        "vn 0.0000 0.0000 1.0000\n"
        "vn 0.0000 -1.0000 0.0000\n"
        "vn 0.0000 1.0000 0.0000\n"
        "# Redundant vertex normal to test error handling\n"
        "vn 0.0000 0.0000 0.0000\n"
        "usemtl None\n"
        "s off\n"
        "f 1//1 2//1 4//1 3//1\n"
        "f 3//2 4//2 8//2 7//2\n"
        "f 7//3 8//3 6//3 5//3\n"
        "f 5//4 6//4 2//4 1//4\n"
        "f 3//5 7//5 5//5 1//5\n"
        "f 8//6 4//6 2//6 6//6\n";

  fs.close ();
  fs.open ("test_obj.mtl");
  fs << "# Blender MTL File: 'None'\n"
        "# Material Count: 1\n"
        "newmtl None\n"
        "Ns 0\n"
        "Ka 0.000000 0.000000 0.000000\n"
        "Kd 0.8 0.8 0.8\n"
        "Ks 0.8 0.8 0.8\n"
        "d 1\n"
        "illum 2\n";

  fs.close ();

  pcl::PCLPointCloud2 blob;
  pcl::OBJReader objreader = pcl::OBJReader();
  int res = objreader.read ("test_obj.obj", blob);
  EXPECT_NE (res, -1);
  EXPECT_EQ (blob.width, 8);
  EXPECT_EQ (blob.height, 1);
  EXPECT_TRUE (blob.is_dense);
  EXPECT_EQ (blob.data.size (), 8 * 6 * 4); // 8 verts, 6 values per vertex (vx,vy,vz,vnx,vny,vnz), 4 byte per value

  // Check fields
  EXPECT_EQ (blob.fields[0].name, "x");
  EXPECT_EQ (blob.fields[0].offset, 0);
  EXPECT_EQ (blob.fields[0].count, 1);
  EXPECT_EQ (blob.fields[0].datatype, pcl::PCLPointField::FLOAT32);

  EXPECT_EQ (blob.fields[1].name, "y");
  EXPECT_EQ (blob.fields[1].offset, 4 * 1);
  EXPECT_EQ (blob.fields[1].count, 1);
  EXPECT_EQ (blob.fields[1].datatype, pcl::PCLPointField::FLOAT32);

  EXPECT_EQ (blob.fields[2].name, "z");
  EXPECT_EQ (blob.fields[2].offset, 4 * 2);
  EXPECT_EQ (blob.fields[2].count, 1);
  EXPECT_EQ (blob.fields[2].datatype, pcl::PCLPointField::FLOAT32);

  EXPECT_EQ (blob.fields[3].name, "normal_x");
  EXPECT_EQ (blob.fields[3].offset, 4 * 3);
  EXPECT_EQ (blob.fields[3].count, 1);
  EXPECT_EQ (blob.fields[3].datatype, pcl::PCLPointField::FLOAT32);

  EXPECT_EQ (blob.fields[4].name, "normal_y");
  EXPECT_EQ (blob.fields[4].offset, 4 * 4);
  EXPECT_EQ (blob.fields[4].count, 1);
  EXPECT_EQ (blob.fields[4].datatype, pcl::PCLPointField::FLOAT32);

  EXPECT_EQ (blob.fields[5].name, "normal_z");
  EXPECT_EQ (blob.fields[5].offset, 4 * 5);
  EXPECT_EQ (blob.fields[5].count, 1);
  EXPECT_EQ (blob.fields[5].datatype, pcl::PCLPointField::FLOAT32);

  remove ("test_obj.obj");
  remove ("test_obj.mtl");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct PointXYZFPFH33
{
  float x, y, z;
  float histogram[33];
};
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZFPFH33,
    (float, x, x) (float, y, y) (float, z, z)
    (float[33], histogram, fpfh))

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
  cloud.width = 2; cloud.height = 1;
  cloud.resize (2);

  cloud[0].x = cloud[0].y = cloud[0].z = 1;
  cloud[1].x = cloud[1].y = cloud[1].z = 2;
  for (int i = 0; i < 33; ++i)
  {
    cloud[0].histogram[i] = static_cast<float> (i);
    cloud[1].histogram[i] = 33.0f - static_cast<float> (i);
  }

  savePCDFile ("v.pcd", cloud);
  cloud.clear ();
  loadPCDFile ("v.pcd", cloud);

  EXPECT_EQ (cloud.width, 2);
  EXPECT_EQ (cloud.height, 1);
  EXPECT_TRUE (cloud.is_dense);
  EXPECT_EQ (cloud.size (), 2);
  
  EXPECT_EQ (cloud[0].x, 1); EXPECT_EQ (cloud[0].y, 1); EXPECT_EQ (cloud[0].z, 1);
  EXPECT_EQ (cloud[1].x, 2); EXPECT_EQ (cloud[1].y, 2); EXPECT_EQ (cloud[1].z, 2);
  for (int i = 0; i < 33; ++i)
  {
    ASSERT_EQ (cloud[0].histogram[i], i);
    ASSERT_EQ (cloud[1].histogram[i], 33-i);
  }

  remove ("v.pcd");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, EigenConversions)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize (5);

  for (std::size_t i = 0; i < cloud.size (); ++i)
    cloud[i].x = cloud[i].y = cloud[i].z = static_cast<float> (i);

  pcl::PCLPointCloud2 blob;
  toPCLPointCloud2 (cloud, blob);

  Eigen::MatrixXf mat;
  getPointCloudAsEigen (blob, mat);
  EXPECT_EQ (mat.cols (), static_cast<Eigen::Index>(cloud.size ()));
  EXPECT_EQ (mat.rows (), 4);
  
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    EXPECT_EQ (mat (0, i), cloud[i].x);
    EXPECT_EQ (mat (1, i), cloud[i].y);
    EXPECT_EQ (mat (2, i), cloud[i].z);
    EXPECT_EQ (mat (3, i), 1);
  }
  
  getEigenAsPointCloud (mat, blob);
  fromPCLPointCloud2 (blob, cloud);
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    EXPECT_EQ (cloud[i].x, i);
    EXPECT_EQ (cloud[i].y, i);
    EXPECT_EQ (cloud[i].z, i);
  }

  getPointCloudAsEigen (blob, mat);
  EXPECT_EQ (mat.cols (), static_cast<Eigen::Index>(cloud.size ()));
  EXPECT_EQ (mat.rows (), 4);
  
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    EXPECT_EQ (mat (0, i), cloud[i].x);
    EXPECT_EQ (mat (1, i), cloud[i].y);
    EXPECT_EQ (mat (2, i), cloud[i].z);
    EXPECT_EQ (mat (3, i), 1);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CopyPointCloud)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_a;
  pcl::PointCloud<pcl::PointXYZRGBA> cloud_b;

  // Fill in the cloud data
  cloud_a.width  = cloud_b.width  = 3;
  cloud_a.height = cloud_b.height = 1;
  cloud_a.resize (cloud_a.width * cloud_a.height);
  cloud_b.resize (cloud_b.width * cloud_b.height);

  for (std::size_t i = 0; i < cloud_a.size (); ++i)
  {
    cloud_a[i].x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud_a[i].y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud_a[i].z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud_b[i].rgba = 255;
  }

  pcl::copyPointCloud (cloud_a, cloud_b);

  for (std::size_t i = 0; i < cloud_a.size (); ++i)
  {
    EXPECT_EQ (cloud_b[i].x, cloud_a[i].x);
    EXPECT_EQ (cloud_b[i].y, cloud_a[i].y);
    EXPECT_EQ (cloud_b[i].z, cloud_a[i].z);
    EXPECT_EQ (cloud_b[i].rgba, 255);
    cloud_a[i].x = cloud_a[i].y = cloud_a[i].z = 0;
  }

  pcl::copyPointCloud (cloud_b, cloud_a);

  for (std::size_t i = 0; i < cloud_a.size (); ++i)
  {
    EXPECT_EQ (cloud_b[i].x, cloud_a[i].x);
    EXPECT_EQ (cloud_b[i].y, cloud_a[i].y);
    EXPECT_EQ (cloud_b[i].z, cloud_a[i].z);
    EXPECT_EQ (cloud_b[i].rgba, 255);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, LZF)
{
  PointCloud<PointXYZ> cloud, cloud2;
  cloud.width  = 640;
  cloud.height = 480;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  srand (static_cast<unsigned int> (time (nullptr)));
  const auto nr_p = cloud.size ();
  // Randomly create a new point cloud
  for (std::size_t i = 0; i < nr_p; ++i)
  {
    cloud[i].x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
  }
  PCDWriter writer;
  int res = writer.writeBinaryCompressed<PointXYZ> ("test_pcl_io_compressed.pcd", cloud);
  EXPECT_EQ (res, 0);

  PCDReader reader;
  res = reader.read<PointXYZ> ("test_pcl_io_compressed.pcd", cloud2);
  EXPECT_EQ (res, 0);

  EXPECT_EQ (cloud2.width, cloud.width);
  EXPECT_EQ (cloud2.height, cloud.height);
  EXPECT_EQ (cloud2.is_dense, cloud.is_dense);
  EXPECT_EQ (cloud2.size (), cloud.size ());

  for (std::size_t i = 0; i < cloud2.size (); ++i)
  {
    ASSERT_EQ (cloud2[i].x, cloud[i].x);
    ASSERT_EQ (cloud2[i].y, cloud[i].y);
    ASSERT_EQ (cloud2[i].z, cloud[i].z);
  }

  pcl::PCLPointCloud2 blob;
  pcl::toPCLPointCloud2 (cloud, blob);
  res = writer.writeBinaryCompressed ("test_pcl_io_compressed.pcd", blob);
  EXPECT_EQ (res, 0);

  reader.read<PointXYZ> ("test_pcl_io_compressed.pcd", cloud2);

  EXPECT_EQ (cloud2.width, blob.width);
  EXPECT_EQ (cloud2.height, blob.height);
  EXPECT_EQ (cloud2.is_dense, cloud.is_dense);
  EXPECT_EQ (cloud2.size (), cloud.size ());

  for (std::size_t i = 0; i < cloud2.size (); ++i)
  {
    EXPECT_EQ (cloud2[i].x, cloud[i].x);
    EXPECT_EQ (cloud2[i].y, cloud[i].y);
    EXPECT_EQ (cloud2[i].z, cloud[i].z);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, LZFExtended)
{
  PointCloud<PointXYZRGBNormal> cloud, cloud2;
  cloud.width  = 640;
  cloud.height = 480;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  srand (static_cast<unsigned int> (time (nullptr)));
  const auto nr_p = cloud.size ();
  // Randomly create a new point cloud
  for (std::size_t i = 0; i < nr_p; ++i)
  {
    cloud[i].x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].normal_x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].normal_y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].normal_z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].rgb = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
  }

  pcl::PCLPointCloud2 blob;
  pcl::toPCLPointCloud2 (cloud, blob);

  PCDWriter writer;
  int res = writer.writeBinaryCompressed ("test_pcl_io_compressed.pcd", blob);
  EXPECT_EQ (res, 0);

  PCDReader reader;
  res = reader.read<PointXYZRGBNormal> ("test_pcl_io_compressed.pcd", cloud2);
  EXPECT_EQ (res, 0);

  EXPECT_EQ (cloud2.width, blob.width);
  EXPECT_EQ (cloud2.height, blob.height);
  EXPECT_EQ (cloud2.is_dense, cloud.is_dense);
  EXPECT_EQ (cloud2.size (), cloud.size ());

  for (std::size_t i = 0; i < cloud2.size (); ++i)
  {
    EXPECT_EQ (cloud2[i].x, cloud[i].x);
    EXPECT_EQ (cloud2[i].y, cloud[i].y);
    EXPECT_EQ (cloud2[i].z, cloud[i].z);
    EXPECT_EQ (cloud2[i].normal_x, cloud[i].normal_x);
    EXPECT_EQ (cloud2[i].normal_y, cloud[i].normal_y);
    EXPECT_EQ (cloud2[i].normal_z, cloud[i].normal_z);
    EXPECT_EQ (cloud2[i].rgb, cloud[i].rgb);
  }

  remove ("test_pcl_io_compressed.pcd");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, LZFInMem)
{
  PointCloud<PointXYZRGBNormal> cloud;
  cloud.width  = 640;
  cloud.height = 480;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  srand (static_cast<unsigned int> (time (nullptr)));
  const auto nr_p = cloud.size ();
  // Randomly create a new point cloud
  for (std::size_t i = 0; i < nr_p; ++i)
  {
    cloud[i].x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].normal_x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].normal_y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].normal_z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].rgb = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
  }

  pcl::PCLPointCloud2 blob;
  pcl::toPCLPointCloud2 (cloud, blob);

  std::ostringstream oss;
  PCDWriter writer;
  int res = writer.writeBinaryCompressed (oss, blob);
  EXPECT_EQ (res, 0);
  std::string pcd_str = oss.str ();

  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int pcd_version = -1;
  int data_type = -1;
  unsigned int data_idx = 0;
  std::istringstream iss (pcd_str, std::ios::binary);
  PCDReader reader;
  pcl::PCLPointCloud2 blob2;
  res = reader.readHeader (iss, blob2, origin, orientation, pcd_version, data_type, data_idx);
  EXPECT_EQ (res, 0);
  EXPECT_EQ (blob2.width, blob.width);
  EXPECT_EQ (blob2.height, blob.height);
  EXPECT_EQ (data_type, 2); // since it was written by writeBinaryCompressed(), it should be compressed.

  const auto *data = reinterpret_cast<const unsigned char *> (pcd_str.data ());
  res = reader.readBodyBinary (data, blob2, pcd_version, data_type == 2, data_idx);
  PointCloud<PointXYZRGBNormal> cloud2;
  pcl::fromPCLPointCloud2 (blob2, cloud2);
  EXPECT_EQ (res, 0);
  EXPECT_EQ (cloud2.width, blob.width);
  EXPECT_EQ (cloud2.height, blob.height);
  EXPECT_EQ (cloud2.is_dense, cloud.is_dense);
  EXPECT_EQ (cloud2.size (), cloud.size ());

  for (std::size_t i = 0; i < cloud2.size (); ++i)
  {
    EXPECT_EQ (cloud2[i].x, cloud[i].x);
    EXPECT_EQ (cloud2[i].y, cloud[i].y);
    EXPECT_EQ (cloud2[i].z, cloud[i].z);
    EXPECT_EQ (cloud2[i].normal_x, cloud[i].normal_x);
    EXPECT_EQ (cloud2[i].normal_y, cloud[i].normal_y);
    EXPECT_EQ (cloud2[i].normal_z, cloud[i].normal_z);
    EXPECT_EQ (cloud2[i].rgb, cloud[i].rgb);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Locale)
{
#ifndef __APPLE__
  try
  {
    PointCloud<PointXYZ> cloud, cloud2;
    cloud.width  = 640;
    cloud.height = 480;
    cloud.resize (cloud.width * cloud.height);
    cloud.is_dense = true;

    srand (static_cast<unsigned int> (time (nullptr)));
    const auto nr_p = cloud.size ();
    // Randomly create a new point cloud
    cloud[0].x = std::numeric_limits<float>::quiet_NaN ();
    cloud[0].y = std::numeric_limits<float>::quiet_NaN ();
    cloud[0].z = std::numeric_limits<float>::quiet_NaN ();
  
    for (std::size_t i = 1; i < nr_p; ++i)
    {
      cloud[i].x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
      cloud[i].y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
      cloud[i].z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    }
    PCDWriter writer;
    try
    {
#ifdef _WIN32
      std::locale::global (std::locale ("German_germany"));
#else
      std::locale::global (std::locale ("de_DE.UTF-8"));
#endif
    }
    catch (const std::runtime_error&)
    {
      PCL_WARN ("Failed to set locale, skipping test.\n");
    }
    int res = writer.writeASCII<PointXYZ> ("test_pcl_io_ascii_locale.pcd", cloud);
    EXPECT_EQ (res, 0);

    PCDReader reader;
    try
    {
#ifdef _WIN32
      std::locale::global (std::locale ("English_US"));
#else
      std::locale::global (std::locale ("en_US.UTF-8"));
#endif
    }
    catch (const std::runtime_error&)
    {
      PCL_WARN ("Failed to set locale, skipping test.\n");
    }
    reader.read<PointXYZ> ("test_pcl_io_ascii_locale.pcd", cloud2);
    std::locale::global (std::locale::classic ());

    EXPECT_EQ (cloud2.width, cloud.width);
    EXPECT_EQ (cloud2.height, cloud.height);
    EXPECT_FALSE (cloud2.is_dense);
    EXPECT_EQ (cloud2.size (), cloud.size ());
  
    EXPECT_TRUE (std::isnan(cloud2[0].x));
    EXPECT_TRUE (std::isnan(cloud2[0].y));
    EXPECT_TRUE (std::isnan(cloud2[0].z));
    for (std::size_t i = 1; i < cloud2.size (); ++i)
    {
      ASSERT_FLOAT_EQ (cloud2[i].x, cloud[i].x);
      ASSERT_FLOAT_EQ (cloud2[i].y, cloud[i].y);
      ASSERT_FLOAT_EQ (cloud2[i].z, cloud[i].z);
    }
  }
  catch (const std::exception&)
  {
  }

  remove ("test_pcl_io_ascii_locale.pcd");
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> class AutoIOTest : public testing::Test { };
using PCLXyzNormalPointTypes = ::testing::Types<BOOST_PP_SEQ_ENUM (PCL_XYZ_POINT_TYPES PCL_NORMAL_POINT_TYPES)>;
TYPED_TEST_SUITE (AutoIOTest, PCLXyzNormalPointTypes);
TYPED_TEST (AutoIOTest, AutoLoadCloudFiles)
{
  PointCloud<TypeParam> cloud;
  PointCloud<TypeParam> cloud_pcd;
  PointCloud<TypeParam> cloud_ply;
  PointCloud<TypeParam> cloud_ifs;

  cloud.width  = 10;
  cloud.height = 5;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  save ("test_autoio.pcd", cloud);
  save ("test_autoio.ply", cloud);
  save ("test_autoio.ifs", cloud);

  load ("test_autoio.pcd", cloud_pcd);
  EXPECT_EQ (cloud_pcd.width * cloud_pcd.height, cloud.width * cloud.height);
  EXPECT_EQ (cloud_pcd.is_dense, cloud.is_dense);

  load ("test_autoio.ply", cloud_ply);
  EXPECT_EQ (cloud_ply.width * cloud_ply.height, cloud.width * cloud.height);
  EXPECT_EQ (cloud_ply.is_dense, cloud.is_dense);

  load ("test_autoio.ifs", cloud_ifs);
  EXPECT_EQ (cloud_ifs.width * cloud_ifs.height, cloud.width * cloud.height);
  EXPECT_EQ (cloud_ifs.is_dense, cloud.is_dense);

  remove ("test_autoio.pcd");
  remove ("test_autoio.ply");
  remove ("test_autoio.ifs");
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

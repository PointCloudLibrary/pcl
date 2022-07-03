/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2017-, Open Perception, Inc.
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
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/test/gtest.h>
#include <fstream> // for ofstream

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PLYReaderWriter)
{
  using pcl::PointXYZI;
  using pcl::PointXYZ;

  pcl::PCLPointCloud2 cloud_blob, cloud_blob2;
  pcl::PointCloud<PointXYZI> cloud, cloud2;

  cloud.width  = 640;
  cloud.height = 480;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;

  srand (static_cast<unsigned int> (time (nullptr)));
  std::size_t nr_p = cloud.size ();
  // Randomly create a new point cloud
  for (std::size_t i = 0; i < nr_p; ++i)
  {
    cloud[i].x = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].y = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].z = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud[i].intensity = static_cast<float> (i);
  }

  // Convert from data type to blob
  toPCLPointCloud2 (cloud, cloud_blob);

  EXPECT_EQ (cloud_blob.width, cloud.width);    // test for toPCLPointCloud2 ()
  EXPECT_EQ (cloud_blob.height, cloud.height);  // test for toPCLPointCloud2 ()
  EXPECT_EQ (cloud_blob.is_dense, cloud.is_dense);  // test for toPCLPointCloud2 ()
  EXPECT_EQ (cloud_blob.data.size (),
             cloud_blob.width * cloud_blob.height * sizeof (PointXYZI));

  // test for toPCLPointCloud2 ()
  pcl::PLYWriter writer;
  writer.write ("test_pcl_io.ply", cloud_blob, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true, true);

  pcl::PLYReader reader;
  reader.read ("test_pcl_io.ply", cloud_blob2);
  //PLY DOES preserve organiziation
  EXPECT_EQ (cloud_blob.width * cloud_blob.height, cloud_blob2.width * cloud_blob2.height);
  EXPECT_EQ (cloud_blob.is_dense, cloud.is_dense);
  EXPECT_EQ (std::size_t (cloud_blob2.data.size ()),         // PointXYZI is 16*2 (XYZ+1, Intensity+3)
             cloud_blob2.width * cloud_blob2.height * sizeof (PointXYZ));  // test for loadPLYFile ()

  // Convert from blob to data type
  fromPCLPointCloud2 (cloud_blob2, cloud2);

  // EXPECT_EQ (cloud.width, cloud2.width);    // test for fromPCLPointCloud2 ()
  // EXPECT_EQ (cloud.height, cloud2.height);  // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.is_dense, cloud2.is_dense);   // test for fromPCLPointCloud2 ()
  EXPECT_EQ (cloud.size (), cloud2.size ());         // test for fromPCLPointCloud2 ()

  for (std::size_t counter = 0; counter < cloud.size (); ++counter)
  {
    EXPECT_FLOAT_EQ (cloud[counter].x, cloud2[counter].x);     // test for fromPCLPointCloud2 ()
    EXPECT_FLOAT_EQ (cloud[counter].y, cloud2[counter].y);     // test for fromPCLPointCloud2 ()
    EXPECT_FLOAT_EQ (cloud[counter].z, cloud2[counter].z);     // test for fromPCLPointCloud2 ()
    EXPECT_FLOAT_EQ (cloud[counter].intensity, cloud2[counter].intensity);  // test for fromPCLPointCloud2 ()
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct PLYTest : public ::testing::Test
{
  PLYTest () : mesh_file_ply_("ply_file.ply")
  {}

  
  ~PLYTest () override { remove (mesh_file_ply_.c_str ()); }

  std::string mesh_file_ply_;
};

struct PLYColorTest : public PLYTest
{
  void SetUp () override
  {
    // Test colors from ply_benchmark.ply
    clr_1_.r = 255;
    clr_1_.g = 0;
    clr_1_.b = 0;
    clr_1_.a = 255;

    clr_2_.r = 0;
    clr_2_.g = 255;
    clr_2_.b = 0;
    clr_2_.a = 0;

    clr_3_.r = 0;
    clr_3_.g = 0;
    clr_3_.b = 255;
    clr_3_.a = 128;

    clr_4_.r = 255;
    clr_4_.g = 255;
    clr_4_.b = 255;
    clr_4_.a = 128;

    std::ofstream fs;
    fs.open (mesh_file_ply_.c_str ());
    fs << "ply\n"
          "format ascii 1.0\n"
          "element vertex 4\n"
          "property float x\n"
          "property float y\n"
          "property float z\n"
          "property uchar red\n"
          "property uchar green\n"
          "property uchar blue\n"
          "property uchar alpha\n"
          "element face 2\n"
          "property list uchar int vertex_indices\n"
          "end_header\n"
          "4.23607 0 1.61803 "
            << unsigned (clr_1_.r) << ' '
            << unsigned (clr_1_.g) << ' '
            << unsigned (clr_1_.b) << ' '
            << unsigned (clr_1_.a) << "\n"
          "2.61803 2.61803 2.61803 "
            << unsigned (clr_2_.r) << ' '
            << unsigned (clr_2_.g) << ' '
            << unsigned (clr_2_.b) << ' '
            << unsigned (clr_2_.a) << "\n"
          "0 1.61803 4.23607 "
            << unsigned (clr_3_.r) << ' '
            << unsigned (clr_3_.g) << ' '
            << unsigned (clr_3_.b) << ' '
            << unsigned (clr_3_.a) << "\n"
          "0 -1.61803 4.23607 "
            << unsigned (clr_4_.r) << ' '
            << unsigned (clr_4_.g) << ' '
            << unsigned (clr_4_.b) << ' '
            << unsigned (clr_4_.a) << "\n"
          "3 0 1 2\n"
          "3 1 2 3\n";
    fs.close ();
  }

  pcl::RGB clr_1_;
  pcl::RGB clr_2_;
  pcl::RGB clr_3_;
  pcl::RGB clr_4_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST_F (PLYColorTest, LoadPLYFileColoredASCIIIntoBlob)
{
  int res;
  std::uint32_t rgba;

  pcl::PCLPointCloud2 cloud_blob;
  std::uint32_t ps;
  std::int32_t offset = -1;

  // check if loading is ok
  res = pcl::io::loadPLYFile (mesh_file_ply_, cloud_blob);
  ASSERT_EQ (res, 0);

  // blob has proper structure
  EXPECT_EQ (cloud_blob.height, 1);
  EXPECT_EQ (cloud_blob.width, 4);
  EXPECT_EQ (cloud_blob.fields.size(), 4);
  EXPECT_FALSE (cloud_blob.is_bigendian);
  EXPECT_EQ (cloud_blob.point_step, 16);
  EXPECT_EQ (cloud_blob.row_step, 16 * 4);
  EXPECT_EQ (cloud_blob.data.size(), 16 * 4);
  EXPECT_TRUE (cloud_blob.is_dense);

  // scope blob data
  ps = cloud_blob.point_step;
  for (const auto &field : cloud_blob.fields)
    if (field.name == std::string("rgba"))
      offset = static_cast<std::int32_t> (field.offset);

  ASSERT_GE (offset, 0);

  // 1st point
  rgba = *reinterpret_cast<std::uint32_t *> (&cloud_blob.data[offset]);
  ASSERT_EQ (rgba, clr_1_.rgba);

  // 2th point
  rgba = *reinterpret_cast<std::uint32_t *> (&cloud_blob.data[ps + offset]);
  ASSERT_EQ (rgba, clr_2_.rgba);

  // 3th point
  rgba = *reinterpret_cast<std::uint32_t *> (&cloud_blob.data[2 * ps + offset]);
  ASSERT_EQ (rgba, clr_3_.rgba);

  // 4th point
  rgba = *reinterpret_cast<std::uint32_t *> (&cloud_blob.data[3 * ps + offset]);
  ASSERT_EQ (rgba, clr_4_.rgba);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST_F (PLYColorTest, LoadPLYFileColoredASCIIIntoPolygonMesh)
{
  int res;
  std::uint32_t rgba;
  pcl::PolygonMesh mesh;
  std::uint32_t ps;
  std::int32_t offset = -1;

  // check if loading is ok
  res = pcl::io::loadPLYFile (mesh_file_ply_, mesh);
  ASSERT_EQ (res, 0);

  // blob has proper structure
  EXPECT_EQ (mesh.cloud.height, 1);
  EXPECT_EQ (mesh.cloud.width, 4);
  EXPECT_EQ (mesh.cloud.fields.size(), 4);
  EXPECT_FALSE (mesh.cloud.is_bigendian);
  EXPECT_EQ (mesh.cloud.point_step, 16);
  EXPECT_EQ (mesh.cloud.row_step, 16 * 4);
  EXPECT_EQ (mesh.cloud.data.size(), 16 * 4);
  EXPECT_TRUE (mesh.cloud.is_dense);

  // scope blob data
  ps = mesh.cloud.point_step;
  for (const auto &field : mesh.cloud.fields)
    if (field.name == std::string("rgba"))
      offset = static_cast<std::int32_t> (field.offset);

  ASSERT_GE (offset, 0);

  // 1st point
  rgba = *reinterpret_cast<std::uint32_t *> (&mesh.cloud.data[offset]);
  ASSERT_EQ (rgba, clr_1_.rgba);

  // 2th point
  rgba = *reinterpret_cast<std::uint32_t *> (&mesh.cloud.data[ps + offset]);
  ASSERT_EQ (rgba, clr_2_.rgba);

  // 3th point
  rgba = *reinterpret_cast<std::uint32_t *> (&mesh.cloud.data[2 * ps + offset]);
  ASSERT_EQ (rgba, clr_3_.rgba);

  // 4th point
  rgba = *reinterpret_cast<std::uint32_t *> (&mesh.cloud.data[3 * ps + offset]);
  ASSERT_EQ (rgba, clr_4_.rgba);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> class PLYPointCloudTest : public PLYColorTest { };
using RGBPointTypes = ::testing::Types<BOOST_PP_SEQ_ENUM (PCL_RGB_POINT_TYPES)>;
TYPED_TEST_SUITE (PLYPointCloudTest, RGBPointTypes);
TYPED_TEST (PLYPointCloudTest, LoadPLYFileColoredASCIIIntoPointCloud)
{
  int res;
  pcl::PointCloud<TypeParam> cloud_rgb;

  // check if loading is ok
  res = pcl::io::loadPLYFile (PLYTest::mesh_file_ply_, cloud_rgb);
  ASSERT_EQ (res, 0);

  // cloud has proper structure
  EXPECT_EQ (cloud_rgb.height, 1);
  EXPECT_EQ (cloud_rgb.width, 4);
  EXPECT_EQ (cloud_rgb.size(), 4);
  EXPECT_TRUE (cloud_rgb.is_dense);

  // scope cloud data
  ASSERT_EQ (cloud_rgb[0].rgba, PLYColorTest::clr_1_.rgba);
  ASSERT_EQ (cloud_rgb[1].rgba, PLYColorTest::clr_2_.rgba);
  ASSERT_EQ (cloud_rgb[2].rgba, PLYColorTest::clr_3_.rgba);
  ASSERT_EQ (cloud_rgb[3].rgba, PLYColorTest::clr_4_.rgba);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
struct PLYCoordinatesIsDenseTest : public PLYTest {};

using XYZPointTypes = ::testing::Types<BOOST_PP_SEQ_ENUM (PCL_XYZ_POINT_TYPES)>;
TYPED_TEST_SUITE (PLYCoordinatesIsDenseTest, XYZPointTypes);

TYPED_TEST (PLYCoordinatesIsDenseTest, NaNInCoordinates)
{
  // create file
  std::ofstream fs;
  fs.open (PLYTest::mesh_file_ply_.c_str ());
  fs << "ply\n"
        "format ascii 1.0\n"
        "element vertex 4\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
        "4.23607 NaN 1.61803 \n"
        "2.61803 2.61803 2.61803 \n"
        "0 1.61803 4.23607 \n"
        "0 -1.61803 4.23607 \n";
  fs.close ();

  // Set up cloud
  pcl::PointCloud<TypeParam> cloud;

  // check if loading is ok
  const int res = pcl::io::loadPLYFile (PLYTest::mesh_file_ply_, cloud);
  ASSERT_EQ (res, 0);

  // cloud has proper structure
  EXPECT_FALSE (cloud.is_dense);
}

TYPED_TEST (PLYCoordinatesIsDenseTest, nanInCoordinates)
{
  // create file
  std::ofstream fs;
  fs.open (PLYTest::mesh_file_ply_.c_str ());
  fs << "ply\n"
        "format ascii 1.0\n"
        "element vertex 4\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
        "4.23607 0 1.61803 \n"
        "2.61803 2.61803 2.61803 \n"
        "nan 1.61803 4.23607 \n"
        "0 -1.61803 4.23607 \n";
  fs.close ();

  // Set up cloud
  pcl::PointCloud<TypeParam> cloud;

  // check if loading is ok
  const int res = pcl::io::loadPLYFile (PLYTest::mesh_file_ply_, cloud);
  ASSERT_EQ (res, 0);

  // cloud has proper structure
  EXPECT_FALSE (cloud.is_dense);
}

TYPED_TEST (PLYCoordinatesIsDenseTest, InfInCoordinates)
{
  // create file
  std::ofstream fs;
  fs.open (PLYTest::mesh_file_ply_.c_str ());
  fs << "ply\n"
        "format ascii 1.0\n"
        "element vertex 4\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
        "4.23607 0 1.61803 \n"
        "2.61803 2.61803 Inf \n"
        "0 1.61803 4.23607 \n"
        "0 -1.61803 4.23607 \n";
  fs.close ();

  // Set up cloud
  pcl::PointCloud<TypeParam> cloud;

  // check if loading is ok
  const int res = pcl::io::loadPLYFile (PLYTest::mesh_file_ply_, cloud);
  ASSERT_EQ (res, 0);

  // cloud has proper structure
  EXPECT_FALSE (cloud.is_dense);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
struct PLYNormalsIsDenseTest : public PLYTest {};

using NormalPointTypes = ::testing::Types<BOOST_PP_SEQ_ENUM (PCL_NORMAL_POINT_TYPES)>;
TYPED_TEST_SUITE (PLYNormalsIsDenseTest, NormalPointTypes);

TYPED_TEST (PLYNormalsIsDenseTest, NaNInNormals)
{
  // create file
  std::ofstream fs;
  fs.open (PLYTest::mesh_file_ply_.c_str ());
  fs << "ply\n"
        "format ascii 1.0\n"
        "element vertex 4\n"
        "property float normal_x\n"
        "property float normal_y\n"
        "property float normal_z\n"
        "end_header\n"
        "4.23607 0 1.61803 \n"
        "2.61803 2.61803 NaN \n"
        "0 1.61803 4.23607 \n"
        "0 -1.61803 4.23607 \n";
  fs.close ();

  // Set up cloud
  pcl::PointCloud<TypeParam> cloud;

  // check if loading is ok
  const int res = pcl::io::loadPLYFile (PLYTest::mesh_file_ply_, cloud);
  ASSERT_EQ (res, 0);

  // cloud has proper structure
  EXPECT_FALSE (cloud.is_dense);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST_F (PLYTest, NaNInIntensity)
{
  // create file
  std::ofstream fs;
  fs.open (mesh_file_ply_.c_str ());
  fs << "ply\n"
        "format ascii 1.0\n"
        "element vertex 4\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property float intensity\n"
        "end_header\n"
        "4.23607 0 1.61803 3.13223\n"
        "2.61803 2.61803 0 3.13223\n"
        "0 1.61803 4.23607 NaN\n"
        "0 -1.61803 4.23607 3.13223\n";
  fs.close ();

  // Set up cloud
  pcl::PointCloud<pcl::PointXYZI> cloud;

  // check if loading is ok
  const int res = pcl::io::loadPLYFile (PLYTest::mesh_file_ply_, cloud);
  ASSERT_EQ (res, 0);

  // cloud has proper structure
  EXPECT_FALSE (cloud.is_dense);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST_F (PLYTest, NoEndofLine)
{
  // create file
  std::ofstream fs;
  fs.open (mesh_file_ply_.c_str ());
  fs << "ply\n"
        "format ascii 1.0\n"
        "element vertex 4\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property float intensity\n"
        "end_header\n"
        "4.23607 0 1.61803 3.13223\n"
        "2.61803 2.61803 0 3.13223\n"
        "0 1.61803 4.23607 NaN\n"
        "0 -1.61803 4.23607 3.13223";
  fs.close ();

  // Set up cloud
  pcl::PointCloud<pcl::PointXYZI> cloud;

  pcl::PLYReader Reader;
  Reader.read(PLYTest::mesh_file_ply_, cloud);

  ASSERT_FALSE (cloud.empty());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST_F (PLYTest, CommentAtTheEnd)
{
  // create file
  std::ofstream fs;
  fs.open (mesh_file_ply_.c_str ());
  fs << "ply\n"
        "format ascii 1.0\n"
        "element vertex 4\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property float intensity\n"
        "end_header\n"
        "4.23607 0 1.61803 3.13223\n"
        "2.61803 2.61803 0 3.13223\n"
        "0 1.61803 4.23607 NaN\n"
        "0 -1.61803 4.23607 3.13223\n"
        "comment hi\n";
  fs.close ();

  // Set up cloud
  pcl::PointCloud<pcl::PointXYZI> cloud;

  pcl::PLYReader Reader;
  Reader.read(PLYTest::mesh_file_ply_, cloud);

  ASSERT_FALSE (cloud.empty());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST_F (PLYTest, EmptyCloud)
{
  // create file
  std::ofstream fs;
  fs.open (mesh_file_ply_.c_str ());
  fs << "ply\n"
        "format ascii 1.0\n"
        "element vertex 0\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n";
  fs.close ();

  // Set up cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // check if loading is ok
  const int res = pcl::io::loadPLYFile (PLYTest::mesh_file_ply_, cloud);
  ASSERT_EQ (res, 0);

  ASSERT_TRUE (cloud.empty());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST_F (PLYTest, Float64Cloud)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(4.23, 0.42, 1.61));
  cloud.push_back(pcl::PointXYZ(-1.61, 4.32, 3.13));

  // create file
  std::ofstream fs;
  fs.open (mesh_file_ply_.c_str ());
  fs << "ply\n"
        "format ascii 1.0\n"
        "element vertex 2\n"
        "property float64 x\n"
        "property float64 y\n"
        "property float64 z\n"
        "end_header\n"
        << cloud[0].x << " " << cloud[0].y << " " << cloud[0].z << "\n"
        << cloud[1].x << " " << cloud[1].y << " " << cloud[1].z << "\n"
        ;
  fs.close ();

  pcl::PCLPointCloud2 cloud2;
  const int res = pcl::io::loadPLYFile (PLYTest::mesh_file_ply_, cloud2);
  ASSERT_EQ (res, 0);

  ASSERT_EQ (cloud2.height*cloud2.width, cloud.size());
  for (auto & field : cloud2.fields) {
    ASSERT_EQ (field.datatype, pcl::PCLPointField::FLOAT64);
  }
  for (size_t pointIdx = 0; pointIdx < cloud.size(); ++pointIdx)
  {
    unsigned char const * ptr = &cloud2.data[0] + pointIdx*cloud2.point_step;
    double xValue, yValue, zValue;
    memcpy(
        reinterpret_cast<char*>(&xValue),
        ptr + cloud2.fields.at(getFieldIndex(cloud2, "x")).offset,
        8);
    memcpy(
        reinterpret_cast<char*>(&yValue),
        ptr + cloud2.fields.at(getFieldIndex(cloud2, "y")).offset,
        8);
    memcpy(
        reinterpret_cast<char*>(&zValue),
        ptr + cloud2.fields.at(getFieldIndex(cloud2, "z")).offset,
        8);

    EXPECT_FLOAT_EQ(cloud[pointIdx].x, xValue);
    EXPECT_FLOAT_EQ(cloud[pointIdx].y, yValue);
    EXPECT_FLOAT_EQ(cloud[pointIdx].z, zValue);
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

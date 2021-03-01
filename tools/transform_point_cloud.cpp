/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("           -trans dx,dy,dz           = the translation (default: "); 
  print_value ("%0.1f, %0.1f, %0.1f", 0, 0, 0); print_info (")\n");
  print_info ("           -quat x,y,z,w             = rotation as quaternion\n");
  print_info ("           -axisangle ax,ay,az,theta = rotation in axis-angle form\n"); 
  print_info ("           -scale x,y,z              = scale each dimension with these values\n"); 
  print_info ("           -matrix v1,v2,...,v8,v9   = a 3x3 affine transform\n");
  print_info ("           -matrix v1,v2,...,v15,v16 = a 4x4 transformation matrix\n");
  print_info ("   Note: If a rotation is not specified, it will default to no rotation.\n");
  print_info ("         If redundant or conflicting transforms are specified, then:\n");
  print_info ("           -axisangle will override -quat\n");
  print_info ("           -matrix (3x3) will take override -axisangle and -quat\n");
  print_info ("           -matrix (4x4) will take override all other arguments\n");

}

void 
printElapsedTimeAndNumberOfPoints (double t, int w, int h = 1)
{
  print_info ("[done, "); print_value ("%g", t); print_info (" ms : "); 
  print_value ("%d", w*h); print_info (" points]\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);

  printElapsedTimeAndNumberOfPoints (tt.toc (), cloud.width, cloud.height);

  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

template <typename PointT> void
transformPointCloudHelper (PointCloud<PointT> & input, PointCloud<PointT> & output,
                           Eigen::Matrix4f &tform)
{
  transformPointCloud (input, output, tform);
}

template <> void
transformPointCloudHelper (PointCloud<PointNormal> & input, PointCloud<PointNormal> & output, 
                           Eigen::Matrix4f &tform)
{
  transformPointCloudWithNormals (input, output, tform);
}

template <> void
transformPointCloudHelper<PointXYZRGBNormal> (PointCloud<PointXYZRGBNormal> & input, 
                                              PointCloud<PointXYZRGBNormal> & output, 
                                              Eigen::Matrix4f &tform)
{
  transformPointCloudWithNormals (input, output, tform);
}


template <typename PointT> void
transformPointCloud2AsType (const pcl::PCLPointCloud2 &input, pcl::PCLPointCloud2 &output,
                            Eigen::Matrix4f &tform)
{
  PointCloud<PointT> cloud;
  fromPCLPointCloud2 (input, cloud);
  transformPointCloudHelper (cloud, cloud, tform);
  toPCLPointCloud2 (cloud, output);
}

void
transformPointCloud2 (const pcl::PCLPointCloud2 &input, pcl::PCLPointCloud2 &output,
                      Eigen::Matrix4f &tform)
{
  // Check for 'rgb' and 'normals' fields
  bool has_rgb = false;
  bool has_normals = false;
  for (const auto &field : input.fields)
  {
    if (field.name.find("rgb") != std::string::npos)
      has_rgb = true;
    if (field.name == "normal_x")
      has_normals = true;
  }

  // Handle the following four point types differently: PointXYZ, PointXYZRGB, PointNormal, PointXYZRGBNormal
  if (!has_rgb && !has_normals)
    transformPointCloud2AsType<PointXYZ> (input, output, tform);
  else if (has_rgb && !has_normals)
    transformPointCloud2AsType<PointXYZRGB> (input, output, tform);
  else if (!has_rgb && has_normals)
    transformPointCloud2AsType<PointNormal> (input, output, tform);
  else // (has_rgb && has_normals)
    transformPointCloud2AsType<PointXYZRGBNormal> (input, output, tform);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         Eigen::Matrix4f &tform)
{
  TicToc tt;
  tt.tic ();
  
  print_highlight ("Transforming ");

  transformPointCloud2 (*input, output, tform);

  printElapsedTimeAndNumberOfPoints (tt.toc (), output.width, output.height);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output);
  
  printElapsedTimeAndNumberOfPoints (tt.toc (), output.width, output.height);
}

template <typename T> void
multiply (pcl::PCLPointCloud2 &cloud, int field_offset, double multiplier)
{
  T val;
  memcpy (&val, &cloud.data[field_offset], sizeof (T));
  val = static_cast<T> (val * static_cast<T> (multiplier));
  memcpy (&cloud.data[field_offset], &val, sizeof (T));
}
template <> void
multiply<bool> (pcl::PCLPointCloud2 &cloud, int field_offset, double multiplier)
{
  if (multiplier != static_cast<bool>(multiplier)) {
    PCL_WARN("Invalid value for scaling a boolean: %f. Only 0 or 1 is valid", multiplier);
    return;
  }

  bool val;
  memcpy (&val, &cloud.data[field_offset], sizeof (bool));
  val = val && static_cast<bool> (multiplier);
  memcpy (&cloud.data[field_offset], &val, sizeof (bool));
}

void
scaleInPlace (pcl::PCLPointCloud2 &cloud, double* multiplier)
{
  // Obtain the x, y, and z indices
  int x_idx = pcl::getFieldIndex (cloud, "x");
  int y_idx = pcl::getFieldIndex (cloud, "y");
  int z_idx = pcl::getFieldIndex (cloud, "z");
  Eigen::Array3i xyz_offset (cloud.fields[x_idx].offset, cloud.fields[y_idx].offset, cloud.fields[z_idx].offset);

  if (cloud.fields[x_idx].datatype == pcl::PCLPointField::BOOL) {
    PCL_WARN("Datatype of point was deduced as boolean. Please check, there might be "
             "an error somewhere");
  }

  for (uindex_t cp = 0; cp < cloud.width * cloud.height; ++cp)
  {
    // Assume all 3 fields are the same (XYZ)
    assert ((cloud.fields[x_idx].datatype == cloud.fields[y_idx].datatype));
    assert ((cloud.fields[x_idx].datatype == cloud.fields[z_idx].datatype));
#define MULTIPLY(CASE_LABEL)                                                           \
  case CASE_LABEL: {                                                                   \
    for (int i = 0; i < 3; ++i)                                                        \
      multiply<pcl::traits::asType_t<CASE_LABEL>>(                                     \
          cloud, xyz_offset[i], multiplier[i]);                                        \
    break;                                                                             \
  }
    switch (cloud.fields[x_idx].datatype)
    {
      MULTIPLY(pcl::PCLPointField::BOOL)
      MULTIPLY(pcl::PCLPointField::INT8)
      MULTIPLY(pcl::PCLPointField::UINT8)
      MULTIPLY(pcl::PCLPointField::INT16)
      MULTIPLY(pcl::PCLPointField::UINT16)
      MULTIPLY(pcl::PCLPointField::INT32)
      MULTIPLY(pcl::PCLPointField::UINT32)
      MULTIPLY(pcl::PCLPointField::INT64)
      MULTIPLY(pcl::PCLPointField::UINT64)
      MULTIPLY(pcl::PCLPointField::FLOAT32)
      MULTIPLY(pcl::PCLPointField::FLOAT64)
    }
#undef MULTIPLY
    xyz_offset += cloud.point_step;
  }
}


/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Transform a cloud. For more information, use: %s -h\n", argv[0]);

  bool help = false;
  parse_argument (argc, argv, "-h", help);
  if (argc < 3 || help)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Initialize the transformation matrix
  Eigen::Matrix4f tform; 
  tform.setIdentity ();

  // Command line parsing
  float dx, dy, dz;
  std::vector<float> values;

  if (parse_3x_arguments (argc, argv, "-trans", dx, dy, dz) > -1)
  {
    tform (0, 3) = dx;
    tform (1, 3) = dy;
    tform (2, 3) = dz;
  }

  if (parse_x_arguments (argc, argv, "-quat", values) > -1)
  {
    if (values.size () == 4)
    {
      const float& x = values[0];
      const float& y = values[1];
      const float& z = values[2];
      const float& w = values[3];
      tform.topLeftCorner (3, 3) = Eigen::Matrix3f (Eigen::Quaternionf (w, x, y, z));
    }
    else
    {
      print_error ("Wrong number of values given (%lu): ", values.size ());
      print_error ("The quaternion specified with -quat must contain 4 elements (w,x,y,z).\n");
    }
  }

  if (parse_x_arguments (argc, argv, "-axisangle", values) > -1)
  {
    if (values.size () == 4)
    {
      const float& ax = values[0];
      const float& ay = values[1];
      const float& az = values[2];
      const float& theta = values[3];
      tform.topLeftCorner (3, 3) = Eigen::Matrix3f (Eigen::AngleAxisf (theta, Eigen::Vector3f (ax, ay, az)));
    }
    else
    {
      print_error ("Wrong number of values given (%lu): ", values.size ());
      print_error ("The rotation specified with -axisangle must contain 4 elements (ax,ay,az,theta).\n");
    }
  }

  if (parse_x_arguments (argc, argv, "-matrix", values) > -1)
  {
    if (values.size () == 9 || values.size () == 16)
    {
      int n = values.size () == 9 ? 3 : 4;
      for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c)
          tform (r, c) = values[n*r+c];
    }
    else
    {
      print_error ("Wrong number of values given (%lu): ", values.size ());
      print_error ("The transformation specified with -matrix must be 3x3 (9) or 4x4 (16).\n");
    }
  }

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud)) 
    return (-1);

  // Apply the transform
  pcl::PCLPointCloud2 output;
  compute (cloud, output, tform);

  // Check if a scaling parameter has been given
  double divider[3];
  if (parse_3x_arguments (argc, argv, "-scale", divider[0], divider[1], divider[2]) > -1)
  {
    print_highlight ("Scaling XYZ data with the following values: %f, %f, %f\n", divider[0], divider[1], divider[2]);
    scaleInPlace (output, divider);
  }

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}


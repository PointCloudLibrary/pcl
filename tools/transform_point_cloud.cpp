/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <cmath>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int argc, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("           -trans dx,dy,dz           = the translation (default: "); 
  print_value ("%0.1f, %0.1f, %0.1f", 0, 0, 0); print_info (")\n");
  print_info ("           -quat w,x,y,z             = rotation as quaternion\n"); 
  print_info ("           -axisangle ax,ay,az,theta = rotation in axis-angle form\n"); 
  print_info ("           -matrix v1,v2,...,v8,v9   = a 3x3 affine transform\n");
  print_info ("           -matrix v1,v2,...,v15,v16 = a 4x4 transformation matrix\n");
  print_info ("   Note: If a rotation is not specified, it will default to no rotation.\n");
  print_info ("         If redundant or conflicting transforms are specified, then:\n");
  print_info ("           -axisangle will override -quat\n");
  print_info ("           -matrix (3x3) will take override -axisangle and -quat\n");
  print_info ("           -matrix (4x4) will take override all other arguments\n");

}

void printElapsedTimeAndNumberOfPoints (double t, int w, int h=1)
{
  print_info ("[done, "); print_value ("%g", t); print_info (" ms : "); 
  print_value ("%d", w*h); print_info (" points]\n");
}

bool
loadCloud (const std::string &filename, sensor_msgs::PointCloud2 &cloud)
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

template <typename PointT>
void
transformPointCloudHelper (PointCloud<PointT> & input, PointCloud<PointT> & output,
                           Eigen::Matrix4f &tform)
{
  transformPointCloud (input, output, tform);
}

template <>
void
transformPointCloudHelper (PointCloud<PointNormal> & input, PointCloud<PointNormal> & output, 
                           Eigen::Matrix4f &tform)
{
  transformPointCloudWithNormals (input, output, tform);
}

template <>
void
transformPointCloudHelper<PointXYZRGBNormal> (PointCloud<PointXYZRGBNormal> & input, 
                                              PointCloud<PointXYZRGBNormal> & output, 
                                              Eigen::Matrix4f &tform)
{
  transformPointCloudWithNormals (input, output, tform);
}


template <typename PointT>
void
transformPointCloud2AsType (const sensor_msgs::PointCloud2 &input, sensor_msgs::PointCloud2 &output,
                            Eigen::Matrix4f &tform)
{
  PointCloud<PointT> cloud;
  fromROSMsg (input, cloud);
  transformPointCloudHelper (cloud, cloud, tform);
  toROSMsg (cloud, output);
}

void
transformPointCloud2 (const sensor_msgs::PointCloud2 &input, sensor_msgs::PointCloud2 &output,
                      Eigen::Matrix4f &tform)
{
  // Check for 'rgb' and 'normals' fields
  bool has_rgb = false;
  bool has_normals = false;
  for (size_t i = 0; i < input.fields.size (); ++i)
  {
    if (input.fields[i].name == "rgb")
      has_rgb = true;
    if (input.fields[i].name == "normals")
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
compute (const sensor_msgs::PointCloud2::ConstPtr &input, sensor_msgs::PointCloud2 &output,
                Eigen::Matrix4f &tform)
{
  TicToc tt;
  tt.tic ();
  
  print_highlight ("Transforming ");

  transformPointCloud2 (*input, output, tform);

  printElapsedTimeAndNumberOfPoints (tt.toc (), output.width, output.height);

}

void
saveCloud (const std::string &filename, const sensor_msgs::PointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  pcl::io::savePCDFile (filename, output);
  
  printElapsedTimeAndNumberOfPoints (tt.toc (), output.width, output.height);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Transform a cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
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
  double dx, dy, dz;
  std::vector<double> values;

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
      const double & x = values[0];
      const double & y = values[1];
      const double & z = values[2];
      const double & w = values[3];
      tform.topLeftCorner (3, 3) = Eigen::Matrix3f (Eigen::Quaternionf (w, x, y, z));
    }
    else
    {
      print_error ("Wrong number of values given (%lu): ", (unsigned long) values.size ());
      print_error ("The quaternion specified with -quat must contain 4 elements (w,x,y,z).\n");
    }
  }

  if (parse_x_arguments (argc, argv, "-axisangle", values) > -1)
  {
    if (values.size () == 4)
    {
      const double & ax = values[0];
      const double & ay = values[1];
      const double & az = values[2];
      const double & theta = values[3];
      tform.topLeftCorner (3, 3) = Eigen::Matrix3f (Eigen::AngleAxisf (theta, Eigen::Vector3f (ax, ay, az)));
    }
    else
    {
      print_error ("Wrong number of values given (%lu): ", (unsigned long) values.size ());
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
      print_error ("Wrong number of values given (%lu): ", (unsigned long) values.size ());
      print_error ("The transformation specified with -matrix must be 3x3 (9) or 4x4 (16).\n");
    }
  }

  // Load the first file
  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud)) 
    return (-1);

  // Apply the transform
  sensor_msgs::PointCloud2 output;
  compute (cloud, output, tform);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}


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
 * $Id: io.cpp 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

/**
\author Radu Bogdan Rusu

**/

#include "pcl/point_types.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"

//////////////////////////////////////////////////////////////////////////
bool
pcl::concatenatePointCloud (const sensor_msgs::PointCloud2 &cloud1, 
                            const sensor_msgs::PointCloud2 &cloud2, 
                            sensor_msgs::PointCloud2 &cloud_out)
{
  if (cloud1.fields.size () != cloud2.fields.size ())
    return (false);

  for (size_t i = 0; i < cloud1.fields.size (); ++i)
    if (cloud1.fields[i].name != cloud2.fields[i].name)
      return (false);

  // Copy cloud1 into cloud_out
  cloud_out = cloud1;
  size_t nrpts = cloud_out.data.size ();
  cloud_out.data.resize (nrpts + cloud2.data.size ());
  memcpy (&cloud_out.data[nrpts], &cloud2.data[0], cloud2.data.size ());

  // Height = 1 => no more organized
  cloud_out.width    = cloud1.width * cloud1.height + cloud2.width * cloud2.height;
  cloud_out.height   = 1;
  if (!cloud1.is_dense || !cloud2.is_dense)
    cloud_out.is_dense = false;
  else
    cloud_out.is_dense = true;

  return (true);
}

//////////////////////////////////////////////////////////////////////////
bool
pcl::getPointCloudAsEigen (const sensor_msgs::PointCloud2 &in, Eigen::MatrixXf &out)
{
  // Get X-Y-Z indices
  int x_idx = getFieldIndex (in, "x");
  int y_idx = getFieldIndex (in, "y");
  int z_idx = getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    PCL_ERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.\n");
    return (false);
  }

  if (in.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    PCL_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.\n");
    return (false);
  }

  size_t npts = in.width * in.height;
  out = Eigen::MatrixXf::Ones (4, npts);

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  // Copy the input dataset into Eigen format
  for (size_t i = 0; i < npts; ++i)
  {
     // Unoptimized memcpys: assume fields x, y, z are in random order
     memcpy (&out (0, i), &in.data[xyz_offset[0]], sizeof (float));
     memcpy (&out (1, i), &in.data[xyz_offset[1]], sizeof (float));
     memcpy (&out (2, i), &in.data[xyz_offset[2]], sizeof (float));

     xyz_offset += in.point_step;
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////
bool 
pcl::getEigenAsPointCloud (Eigen::MatrixXf &in, sensor_msgs::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = getFieldIndex (out, "x");
  int y_idx = getFieldIndex (out, "y");
  int z_idx = getFieldIndex (out, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    PCL_ERROR ("Output dataset has no X-Y-Z coordinates set up as fields! Cannot convert from Eigen format.\n");
    return (false);
  }

  if (out.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      out.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      out.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    PCL_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.\n");
    return (false);
  }

  if (in.cols () != (int)(out.width * out.height))
  {
    PCL_ERROR ("Number of points in the point cloud differs from the Eigen matrix. Cannot continue.\n");
    return (false);
  }

  size_t npts = in.cols ();

  Eigen::Array4i xyz_offset (out.fields[x_idx].offset, out.fields[y_idx].offset, out.fields[z_idx].offset, 0);

  // Copy the input dataset into Eigen format
  for (size_t i = 0; i < npts; ++i)
  {
     // Unoptimized memcpys: assume fields x, y, z are in random order
     memcpy (&out.data[xyz_offset[0]], &in (0, i), sizeof (float));
     memcpy (&out.data[xyz_offset[1]], &in (1, i), sizeof (float));
     memcpy (&out.data[xyz_offset[2]], &in (2, i), sizeof (float));

     xyz_offset += out.point_step;
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////
void 
pcl::copyPointCloud (
    const sensor_msgs::PointCloud2 &cloud_in, const std::vector<int> &indices, 
    sensor_msgs::PointCloud2 &cloud_out)
{
  cloud_out.header       = cloud_in.header;
  cloud_out.height       = 1;
  cloud_out.width        = indices.size (); 
  cloud_out.fields       = cloud_in.fields;
  cloud_out.is_bigendian = cloud_in.is_bigendian;
  cloud_out.point_step   = cloud_in.point_step;
  cloud_out.row_step     = cloud_in.row_step;
  if (cloud_in.is_dense)
    cloud_out.is_dense = true;
  else
    // It's not necessarily true that is_dense is false if cloud_in.is_dense is false
    // To verify this, we would need to iterate over all points and check for NaNs
    cloud_out.is_dense = false;

  cloud_out.data.resize (cloud_out.width * cloud_out.height * cloud_out.point_step);

  // Iterate over each point
  for (size_t i = 0; i < indices.size (); ++i)
  {
    memcpy (&cloud_out.data[i * cloud_out.point_step], &cloud_in.data[indices[i] * cloud_in.point_step], cloud_in.point_step);
    // Check for NaNs, set is_dense to true/false based on this
    // ...
  }
}


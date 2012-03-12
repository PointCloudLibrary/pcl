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

#include <sensor_msgs/PointCloud2.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
//#include <pcl/filters/conditional_removal.h>

/*
 //#include <pcl/filters/pixel_grid.h>
 //#include <pcl/filters/filter_dimension.h>
 */
// Include the implementations instead of compiling them separately to speed up compile time
//#include "passthrough.cpp"
//#include "extract_indices.cpp"
//#include "project_inliers.cpp"
//#include "radius_outlier_removal.cpp"
//#include "statistical_outlier_removal.cpp"
//#include "voxel_grid.cpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Base method for feature estimation for all points given in <setInputCloud (), setIndices ()> using
 * the surface in setSearchSurface () and the spatial locator in setSearchMethod ()
 * \param output the resultant filtered point cloud dataset
 */
void
pcl::Filter<sensor_msgs::PointCloud2>::filter (PointCloud2 &output)
{
  if (!initCompute ())
    return;

  // Copy fields and header at a minimum
  output.header = input_->header;
  output.fields = input_->fields;

  // Apply the actual filter
  applyFilter (output);

  deinitCompute ();
}

// Instantiations of specific point types
PCL_INSTANTIATE(removeNanFromPointCloud, PCL_XYZ_POINT_TYPES)


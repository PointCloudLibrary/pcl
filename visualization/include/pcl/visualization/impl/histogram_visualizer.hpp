/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#ifndef PCL_PCL_HISTOGRAM_VISUALIZER_IMPL_H_
#define PCL_PCL_HISTOGRAM_VISUALIZER_IMPL_H_

#include <vtkDoubleArray.h>


namespace pcl
{

namespace visualization
{

template <typename PointT> bool
PCLHistogramVisualizer::addFeatureHistogram (
    const pcl::PointCloud<PointT> &cloud, int hsize,
    const std::string &id, int win_width, int win_height)
{
  auto am_it = wins_.find (id);
  if (am_it != wins_.end ())
  {
    PCL_WARN ("[addFeatureHistogram] A window with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (hsize);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (int d = 0; d < hsize; ++d)
  {
    xy[0] = d;
    xy[1] = cloud[0].histogram[d];
    xy_array->SetTuple (d, xy);
  }
  RenWinInteract renwinint;
  createActor (xy_array, renwinint, id, win_width, win_height);

  // Save the pointer/ID pair to the global window map
  wins_[id] = renwinint;

  return (true);
}


template <typename PointT> bool
PCLHistogramVisualizer::addFeatureHistogram (
    const pcl::PointCloud<PointT> &cloud,
    const std::string &field_name,
    const pcl::index_t index,
    const std::string &id, int win_width, int win_height)
{
  if (index < 0 || index >= cloud.size ())
  {
    PCL_ERROR ("[addFeatureHistogram] Invalid point index (%d) given!\n", index);
    return (false);
  }

  // Get the fields present in this cloud
  std::vector<pcl::PCLPointField> fields;
  // Check if our field exists
  int field_idx = pcl::getFieldIndex<PointT> (cloud, field_name, fields);
  if (field_idx == -1)
  {
    PCL_ERROR ("[addFeatureHistogram] The specified field <%s> does not exist!\n", field_name.c_str ());
    return (false);
  }

  auto am_it = wins_.find (id);
  if (am_it != wins_.end ())
  {
    PCL_WARN ("[addFeatureHistogram] A window with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (fields[field_idx].count);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (uindex_t d = 0; d < fields[field_idx].count; ++d)
  {
    xy[0] = d;
    //xy[1] = cloud[index].histogram[d];
    float data;
    memcpy (&data, reinterpret_cast<const char*> (&cloud[index]) + fields[field_idx].offset + d * sizeof (float), sizeof (float));
    xy[1] = data;
    xy_array->SetTuple (d, xy);
  }
  RenWinInteract renwinint;
  createActor (xy_array, renwinint, id, win_width, win_height);

  // Save the pointer/ID pair to the global window map
  wins_[id] = renwinint;
  return (true);
}


template <typename PointT> bool
PCLHistogramVisualizer::updateFeatureHistogram (
    const pcl::PointCloud<PointT> &cloud, int hsize,
    const std::string &id)
{
  auto am_it = wins_.find (id);
  if (am_it == wins_.end ())
  {
    PCL_WARN ("[updateFeatureHistogram] A window with id <%s> does not exists!.\n", id.c_str ());
    return (false);
  }
  RenWinInteract* renwinupd = &wins_[id];

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (hsize);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (int d = 0; d < hsize; ++d)
  {
    xy[0] = d;
    xy[1] = cloud[0].histogram[d];
    xy_array->SetTuple (d, xy);
  }
  reCreateActor (xy_array, renwinupd, hsize);
  return (true);
}


template <typename PointT> bool
PCLHistogramVisualizer::updateFeatureHistogram (
    const pcl::PointCloud<PointT> &cloud, const std::string &field_name, const pcl::index_t index,
    const std::string &id)
{
  if (index < 0 || index >= cloud.size ())
  {
    PCL_ERROR ("[updateFeatureHistogram] Invalid point index (%d) given!\n", index);
    return (false);
  }

  // Get the fields present in this cloud
  std::vector<pcl::PCLPointField> fields;
  // Check if our field exists
  int field_idx = pcl::getFieldIndex<PointT> (cloud, field_name, fields);
  if (field_idx == -1)
  {
    PCL_ERROR ("[updateFeatureHistogram] The specified field <%s> does not exist!\n", field_name.c_str ());
    return (false);
  }

  auto am_it = wins_.find (id);
  if (am_it == wins_.end ())
  {
    PCL_WARN ("[updateFeatureHistogram] A window with id <%s> does not exists!.\n", id.c_str ());
    return (false);
  }
  RenWinInteract* renwinupd = &wins_[id];

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (fields[field_idx].count);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (std::uint32_t d = 0; d < fields[field_idx].count; ++d)
  {
    xy[0] = d;
    //xy[1] = cloud[index].histogram[d];
    float data;
    memcpy (&data, reinterpret_cast<const char*> (&cloud[index]) + fields[field_idx].offset + d * sizeof (float), sizeof (float));
    xy[1] = data;
    xy_array->SetTuple (d, xy);
  }

  reCreateActor (xy_array, renwinupd, cloud.fields[field_idx].count - 1);
  return (true);
}

} // namespace visualization
} // namespace pcl

#endif


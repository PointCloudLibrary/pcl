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
 */

#ifndef PCL_VISUALUALIZATION_PCL_PLOTTER_IMPL_H_
#define	PCL_VISUALUALIZATION_PCL_PLOTTER_IMPL_H_


namespace pcl
{

namespace visualization
{

template <typename PointT> bool
PCLPlotter::addFeatureHistogram (
    const pcl::PointCloud<PointT> &cloud, int hsize,
    const std::string &id, int win_width, int win_height)
{
  std::vector<double> array_x(hsize), array_y(hsize);

  // Parse the cloud data and store it in the array
  for (int i = 0; i < hsize; ++i)
  {
    array_x[i] = i;
    array_y[i] = cloud[0].histogram[i];
  }

  this->addPlotData(array_x, array_y, id.c_str(), vtkChart::LINE);
  setWindowSize (win_width, win_height);
  return true;
}


template <typename PointT> bool
PCLPlotter::addFeatureHistogram (
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

  int hsize = fields[field_idx].count;
  std::vector<double> array_x (hsize), array_y (hsize);

  for (int i = 0; i < hsize; ++i)
  {
    array_x[i] = i;
    float data;
    // TODO: replace float with the real data type
    memcpy (&data, reinterpret_cast<const char*> (&cloud[index]) + fields[field_idx].offset + i * sizeof (float), sizeof (float));
    array_y[i] = data;
  }

  this->addPlotData(array_x, array_y, id.c_str(), vtkChart::LINE);
  setWindowSize (win_width, win_height);
  return (true);
}

} // namespace visualization
} // namespace pcl

#endif	/* PCL_VISUALUALIZATION_PCL_PLOTTER_IMPL_H_ */


/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
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

#ifndef PCL_POINT_CLOUD_IMAGE_EXTRACTORS_IMPL_HPP_
#define PCL_POINT_CLOUD_IMAGE_EXTRACTORS_IMPL_HPP_

#include <set>
#include <map>
#include <ctime>
#include <cstdlib>

#include <pcl/common/io.h>
#include <pcl/common/colors.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::PointCloudImageExtractor<PointT>::extract (const PointCloud& cloud, pcl::PCLImage& img) const
{
  if (!cloud.isOrganized () || cloud.points.size () != cloud.width * cloud.height)
    return (false);

  bool result = this->extractImpl (cloud, img);

  if (paint_nans_with_black_ && result)
  {
    size_t size = img.encoding == "mono16" ? 2 : 3;
    for (size_t i = 0; i < cloud.points.size (); ++i)
      if (!pcl::isFinite (cloud[i]))
        std::memset (&img.data[i * size], 0, size);
  }

  return (result);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::PointCloudImageExtractorFromNormalField<PointT>::extractImpl (const PointCloud& cloud, pcl::PCLImage& img) const
{
  std::vector<pcl::PCLPointField> fields;
  int field_x_idx = pcl::getFieldIndex (cloud, "normal_x", fields);
  int field_y_idx = pcl::getFieldIndex (cloud, "normal_y", fields);
  int field_z_idx = pcl::getFieldIndex (cloud, "normal_z", fields);
  if (field_x_idx == -1 || field_y_idx == -1 || field_z_idx == -1)
    return (false);
  const size_t offset_x = fields[field_x_idx].offset;
  const size_t offset_y = fields[field_y_idx].offset;
  const size_t offset_z = fields[field_z_idx].offset;

  img.encoding = "rgb8";
  img.width = cloud.width;
  img.height = cloud.height;
  img.step = img.width * sizeof (unsigned char) * 3;
  img.data.resize (img.step * img.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    float x;
    float y;
    float z;
    pcl::getFieldValue<PointT, float> (cloud.points[i], offset_x, x);
    pcl::getFieldValue<PointT, float> (cloud.points[i], offset_y, y);
    pcl::getFieldValue<PointT, float> (cloud.points[i], offset_z, z);
    img.data[i * 3 + 0] = static_cast<unsigned char>((x + 1.0) * 127);
    img.data[i * 3 + 1] = static_cast<unsigned char>((y + 1.0) * 127);
    img.data[i * 3 + 2] = static_cast<unsigned char>((z + 1.0) * 127);
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::PointCloudImageExtractorFromRGBField<PointT>::extractImpl (const PointCloud& cloud, pcl::PCLImage& img) const
{
  std::vector<pcl::PCLPointField> fields;
  int field_idx = pcl::getFieldIndex (cloud, "rgb", fields);
  if (field_idx == -1)
  {
    field_idx = pcl::getFieldIndex (cloud, "rgba", fields);
    if (field_idx == -1)
      return (false);
  }
  const size_t offset = fields[field_idx].offset;

  img.encoding = "rgb8";
  img.width = cloud.width;
  img.height = cloud.height;
  img.step = img.width * sizeof (unsigned char) * 3;
  img.data.resize (img.step * img.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    uint32_t val;
    pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], offset, val);
    img.data[i * 3 + 0] = (val >> 16) & 0x0000ff;
    img.data[i * 3 + 1] = (val >> 8) & 0x0000ff;
    img.data[i * 3 + 2] = (val) & 0x0000ff;
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::PointCloudImageExtractorFromLabelField<PointT>::extractImpl (const PointCloud& cloud, pcl::PCLImage& img) const
{
  std::vector<pcl::PCLPointField> fields;
  int field_idx = pcl::getFieldIndex (cloud, "label", fields);
  if (field_idx == -1)
    return (false);
  const size_t offset = fields[field_idx].offset;

  switch (color_mode_)
  {
    case COLORS_MONO:
    {
      img.encoding = "mono16";
      img.width = cloud.width;
      img.height = cloud.height;
      img.step = img.width * sizeof (unsigned short);
      img.data.resize (img.step * img.height);
      unsigned short* data = reinterpret_cast<unsigned short*>(&img.data[0]);
      for (size_t i = 0; i < cloud.points.size (); ++i)
      {
        uint32_t val;
        pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], offset, val);
        data[i] = static_cast<unsigned short>(val);
      }
      break;
    }
    case COLORS_RGB_RANDOM:
    {
      img.encoding = "rgb8";
      img.width = cloud.width;
      img.height = cloud.height;
      img.step = img.width * sizeof (unsigned char) * 3;
      img.data.resize (img.step * img.height);

      std::srand(std::time(0));
      std::map<uint32_t, size_t> colormap;

      for (size_t i = 0; i < cloud.points.size (); ++i)
      {
        uint32_t val;
        pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], offset, val);
        if (colormap.count (val) == 0)
        {
          colormap[val] = i * 3;
          img.data[i * 3 + 0] = static_cast<uint8_t> ((std::rand () % 256));
          img.data[i * 3 + 1] = static_cast<uint8_t> ((std::rand () % 256));
          img.data[i * 3 + 2] = static_cast<uint8_t> ((std::rand () % 256));
        }
        else
        {
          memcpy (&img.data[i * 3], &img.data[colormap[val]], 3);
        }
      }
      break;
    }
    case COLORS_RGB_GLASBEY:
    {
      img.encoding = "rgb8";
      img.width = cloud.width;
      img.height = cloud.height;
      img.step = img.width * sizeof (unsigned char) * 3;
      img.data.resize (img.step * img.height);

      std::srand(std::time(0));
      std::set<uint32_t> labels;
      std::map<uint32_t, size_t> colormap;

      // First pass: find unique labels
      for (size_t i = 0; i < cloud.points.size (); ++i)
      {
        // If we need to paint NaN points with black do not waste colors on them
        if (paint_nans_with_black_ && !pcl::isFinite (cloud.points[i]))
          continue;
        uint32_t val;
        pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], offset, val);
        labels.insert (val);
      }

      // Assign Glasbey colors in ascending order of labels
      // Note: the color LUT has a finite size (256 colors), therefore when
      // there are more labels the colors will repeat
      size_t color = 0;
      for (std::set<uint32_t>::iterator iter = labels.begin (); iter != labels.end (); ++iter)
      {
        colormap[*iter] = color % GlasbeyLUT::size ();
        ++color;
      }

      // Second pass: copy colors from the LUT
      for (size_t i = 0; i < cloud.points.size (); ++i)
      {
        uint32_t val;
        pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], offset, val);
        memcpy (&img.data[i * 3], GlasbeyLUT::data () + colormap[val] * 3, 3);
      }

      break;
    }
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::PointCloudImageExtractorWithScaling<PointT>::extractImpl (const PointCloud& cloud, pcl::PCLImage& img) const
{
  std::vector<pcl::PCLPointField> fields;
  int field_idx = pcl::getFieldIndex (cloud, field_name_, fields);
  if (field_idx == -1)
    return (false);
  const size_t offset = fields[field_idx].offset;

  img.encoding = "mono16";
  img.width = cloud.width;
  img.height = cloud.height;
  img.step = img.width * sizeof (unsigned short);
  img.data.resize (img.step * img.height);
  unsigned short* data = reinterpret_cast<unsigned short*>(&img.data[0]);

  float scaling_factor = scaling_factor_;
  float data_min = 0.0f;
  if (scaling_method_ == SCALING_FULL_RANGE)
  {
    float min = std::numeric_limits<float>::infinity();
    float max = -std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      float val;
      pcl::getFieldValue<PointT, float> (cloud.points[i], offset, val);
      if (val < min)
        min = val;
      if (val > max)
        max = val;
    }
    scaling_factor = min == max ? 0 : std::numeric_limits<unsigned short>::max() / (max - min);
    data_min = min;
  }

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    float val;
    pcl::getFieldValue<PointT, float> (cloud.points[i], offset, val);
    if (scaling_method_ == SCALING_NO)
    {
      data[i] = val;
    }
    else if (scaling_method_ == SCALING_FULL_RANGE)
    {
      data[i] = (val - data_min) * scaling_factor;
    }
    else if (scaling_method_ == SCALING_FIXED_FACTOR)
    {
      data[i] = val * scaling_factor;
    }
  }

  return (true);
}

#endif      // PCL_POINT_CLOUD_IMAGE_EXTRACTORS_IMPL_HPP_


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
 * $Id$
 *
 */

#pragma once

#include <pcl/conversions.h> // for FieldAdder
#include <pcl/common/concatenate.h>
#include <pcl/common/copy_point.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>


namespace pcl
{

template <typename PointT> int
getFieldIndex (const pcl::PointCloud<PointT> &,
               const std::string &field_name,
               std::vector<pcl::PCLPointField> &fields)
{
  return getFieldIndex<PointT>(field_name, fields);
}


template <typename PointT> int
getFieldIndex (const std::string &field_name,
               std::vector<pcl::PCLPointField> &fields)
{
  fields = getFields<PointT> ();
  const auto& ref = fields;
  return pcl::getFieldIndex<PointT> (field_name, ref);
}


template <typename PointT> int
getFieldIndex (const std::string &field_name,
               const std::vector<pcl::PCLPointField> &fields)
{
  const auto result = std::find_if(fields.begin (), fields.end (),
      [&field_name](const auto& field) { return field.name == field_name; });
  if (result == fields.end ())
    return -1;
  return std::distance(fields.begin (), result);
}


template <typename PointT> void
getFields (const pcl::PointCloud<PointT> &, std::vector<pcl::PCLPointField> &fields)
{
  fields = getFields<PointT> ();
}


template <typename PointT> void
getFields (std::vector<pcl::PCLPointField> &fields)
{
  fields = getFields<PointT> ();
}


template <typename PointT> std::vector<pcl::PCLPointField>
getFields ()
{
  std::vector<pcl::PCLPointField> fields;
  // Get the fields list
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));
  return fields;
}


template <typename PointT> std::string
getFieldsList (const pcl::PointCloud<PointT> &)
{
  // Get the fields list
  const auto fields = getFields<PointT>();
  std::string result;
  for (std::size_t i = 0; i < fields.size () - 1; ++i)
    result += fields[i].name + " ";
  result += fields[fields.size () - 1].name;
  return (result);
}

namespace detail
{

  template <typename PointInT, typename PointOutT> void
  copyPointCloudMemcpy (const pcl::PointCloud<PointInT> &cloud_in,
                        pcl::PointCloud<PointOutT> &cloud_out)
  {
    // Iterate over each point, if the point types of two clouds are different
    for (std::size_t i = 0; i < cloud_in.size (); ++i)
      copyPoint (cloud_in[i], cloud_out[i]);
  }


  template <typename PointT> void
  copyPointCloudMemcpy (const pcl::PointCloud<PointT> &cloud_in,
                        pcl::PointCloud<PointT> &cloud_out)
  {
    // Use std::copy directly, if the point types of two clouds are same
    std::copy (&cloud_in[0], (&cloud_in[0]) + cloud_in.size (), &cloud_out[0]);
  }

} // namespace detail

template <typename PointInT, typename PointOutT> void
copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                pcl::PointCloud<PointOutT> &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = cloud_in.width;
  cloud_out.height   = cloud_in.height;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
  cloud_out.resize (cloud_in.size ());

  if (!cloud_in.empty ())
    detail::copyPointCloudMemcpy (cloud_in, cloud_out);
}


template <typename PointT, typename IndicesVectorAllocator> void
copyPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                const IndicesAllocator< IndicesVectorAllocator> &indices,
                pcl::PointCloud<PointT> &cloud_out)
{
  // Do we want to copy everything?
  if (indices.size () == cloud_in.size ())
  {
    cloud_out = cloud_in;
    return;
  }

  // Allocate enough space and copy the basics
  cloud_out.clear ();
  cloud_out.reserve (indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = indices.size ();
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each point
  for (const auto& index : indices)
    cloud_out.transient_push_back (cloud_in[index]);
}


template <typename PointInT, typename PointOutT, typename IndicesVectorAllocator> void
copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                const IndicesAllocator< IndicesVectorAllocator> &indices,
                pcl::PointCloud<PointOutT> &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out.resize (indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = indices.size ();
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each point
  for (std::size_t i = 0; i < indices.size (); ++i)
    copyPoint (cloud_in[indices[i]], cloud_out[i]);
}


template <typename PointT> void
copyPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                const pcl::PointIndices &indices,
                     pcl::PointCloud<PointT> &cloud_out)
{
  copyPointCloud (cloud_in, indices.indices, cloud_out);
}


template <typename PointInT, typename PointOutT> void
copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                const pcl::PointIndices &indices,
                pcl::PointCloud<PointOutT> &cloud_out)
{
  copyPointCloud (cloud_in, indices.indices, cloud_out);
}


template <typename PointT> void
copyPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                const std::vector<pcl::PointIndices> &indices,
                pcl::PointCloud<PointT> &cloud_out)
{
  std::size_t nr_p = 0;
  for (const auto &index : indices)
    nr_p += index.indices.size ();

  // Do we want to copy everything? Remember we assume UNIQUE indices
  if (nr_p == cloud_in.size ())
  {
    cloud_out = cloud_in;
    return;
  }

  // Allocate enough space and copy the basics
  cloud_out.clear ();
  cloud_out.reserve (nr_p);
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = nr_p;
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each cluster
  for (const auto &cluster_index : indices)
  {
    // Iterate over each idx
    for (const auto &index : cluster_index.indices)
    {
      // Iterate over each dimension
      cloud_out.transient_push_back (cloud_in[index]);
    }
  }
}


template <typename PointInT, typename PointOutT> void
copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                const std::vector<pcl::PointIndices> &indices,
                pcl::PointCloud<PointOutT> &cloud_out)
{
  const auto nr_p = std::accumulate(indices.begin (), indices.end (), 0,
      [](const auto& acc, const auto& index) { return index.indices.size() + acc; });

  // Do we want to copy everything? Remember we assume UNIQUE indices
  if (nr_p == cloud_in.size ())
  {
    copyPointCloud (cloud_in, cloud_out);
    return;
  }

  // Allocate enough space and copy the basics
  cloud_out.resize (nr_p);
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = nr_p;
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each cluster
  std::size_t cp = 0;
  for (const auto &cluster_index : indices)
  {
    // Iterate over each idx
    for (const auto &index : cluster_index.indices)
    {
      copyPoint (cloud_in[index], cloud_out[cp]);
      ++cp;
    }
  }
}


template <typename PointIn1T, typename PointIn2T, typename PointOutT> void
concatenateFields (const pcl::PointCloud<PointIn1T> &cloud1_in,
                   const pcl::PointCloud<PointIn2T> &cloud2_in,
                   pcl::PointCloud<PointOutT> &cloud_out)
{
  using FieldList1 = typename pcl::traits::fieldList<PointIn1T>::type;
  using FieldList2 = typename pcl::traits::fieldList<PointIn2T>::type;

  if (cloud1_in.size () != cloud2_in.size ())
  {
    PCL_ERROR ("[pcl::concatenateFields] The number of points in the two input datasets differs!\n");
    return;
  }

  // Resize the output dataset
  cloud_out.resize (cloud1_in.size ());
  cloud_out.header   = cloud1_in.header;
  cloud_out.width    = cloud1_in.width;
  cloud_out.height   = cloud1_in.height;
  if (!cloud1_in.is_dense || !cloud2_in.is_dense)
    cloud_out.is_dense = false;
  else
    cloud_out.is_dense = true;

  // Iterate over each point
  for (std::size_t i = 0; i < cloud_out.size (); ++i)
  {
    // Iterate over each dimension
    pcl::for_each_type <FieldList1> (pcl::NdConcatenateFunctor <PointIn1T, PointOutT> (cloud1_in[i], cloud_out[i]));
    pcl::for_each_type <FieldList2> (pcl::NdConcatenateFunctor <PointIn2T, PointOutT> (cloud2_in[i], cloud_out[i]));
  }
}


template <typename PointT> void
copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
                int top, int bottom, int left, int right, pcl::InterpolationType border_type, const PointT& value)
{
  if (top < 0 || left < 0 || bottom < 0 || right < 0)
  {
    std::string faulty = (top < 0) ? "top" : (left < 0) ? "left" : (bottom < 0) ? "bottom" : "right";
    PCL_THROW_EXCEPTION (pcl::BadArgumentException, "[pcl::copyPointCloud] error: " << faulty << " must be positive!");
    return;
  }

  if (top == 0 && left == 0 && bottom == 0 && right == 0)
   cloud_out = cloud_in;
  else
  {
    // Allocate enough space and copy the basics
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width + left + right;
    cloud_out.height   = cloud_in.height + top + bottom;
    if (cloud_out.size () != cloud_out.width * cloud_out.height)
      cloud_out.resize (cloud_out.width * cloud_out.height);
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

    if (border_type == pcl::BORDER_TRANSPARENT)
    {
      const PointT* in = &(cloud_in[0]);
      PointT* out = &(cloud_out[0]);
      PointT* out_inner = out + cloud_out.width*top + left;
      for (std::uint32_t i = 0; i < cloud_in.height; i++, out_inner += cloud_out.width, in += cloud_in.width)
      {
        if (out_inner != in)
          memcpy (out_inner, in, cloud_in.width * sizeof (PointT));
      }
    }
    else
    {
      // Copy the data
      if (border_type != pcl::BORDER_CONSTANT)
      {
        try
        {
          std::vector<int> padding (cloud_out.width - cloud_in.width);
          int right = cloud_out.width - cloud_in.width - left;
          int bottom = cloud_out.height - cloud_in.height - top;

          for (int i = 0; i < left; i++)
            padding[i] = pcl::interpolatePointIndex (i-left, cloud_in.width, border_type);

          for (int i = 0; i < right; i++)
            padding[i+left] = pcl::interpolatePointIndex (cloud_in.width+i, cloud_in.width, border_type);

          const PointT* in = &(cloud_in[0]);
          PointT* out = &(cloud_out[0]);
          PointT* out_inner = out + cloud_out.width*top + left;

          for (std::uint32_t i = 0; i < cloud_in.height; i++, out_inner += cloud_out.width, in += cloud_in.width)
          {
            if (out_inner != in)
              memcpy (out_inner, in, cloud_in.width * sizeof (PointT));

            for (int j = 0; j < left; j++)
              out_inner[j - left] = in[padding[j]];

            for (int j = 0; j < right; j++)
              out_inner[j + cloud_in.width] = in[padding[j + left]];
          }

          for (int i = 0; i < top; i++)
          {
            int j = pcl::interpolatePointIndex (i - top, cloud_in.height, border_type);
            memcpy (out + i*cloud_out.width,
                    out + (j+top) * cloud_out.width,
                    sizeof (PointT) * cloud_out.width);
          }

          for (int i = 0; i < bottom; i++)
          {
            int j = pcl::interpolatePointIndex (i + cloud_in.height, cloud_in.height, border_type);
            memcpy (out + (i + cloud_in.height + top)*cloud_out.width,
                    out + (j+top)*cloud_out.width,
                    cloud_out.width * sizeof (PointT));
          }
        }
        catch (pcl::BadArgumentException&)
        {
          PCL_ERROR ("[pcl::copyPointCloud] Unhandled interpolation type %d!\n", border_type);
        }
      }
      else
      {
        int right = cloud_out.width - cloud_in.width - left;
        int bottom = cloud_out.height - cloud_in.height - top;
        std::vector<PointT> buff (cloud_out.width, value);
        PointT* buff_ptr = &(buff[0]);
        const PointT* in = &(cloud_in[0]);
        PointT* out = &(cloud_out[0]);
        PointT* out_inner = out + cloud_out.width*top + left;

        for (std::uint32_t i = 0; i < cloud_in.height; i++, out_inner += cloud_out.width, in += cloud_in.width)
        {
          if (out_inner != in)
            memcpy (out_inner, in, cloud_in.width * sizeof (PointT));

          memcpy (out_inner - left, buff_ptr, left  * sizeof (PointT));
          memcpy (out_inner + cloud_in.width, buff_ptr, right * sizeof (PointT));
        }

        for (int i = 0; i < top; i++)
        {
          memcpy (out + i*cloud_out.width, buff_ptr, cloud_out.width * sizeof (PointT));
        }

        for (int i = 0; i < bottom; i++)
        {
          memcpy (out + (i + cloud_in.height + top)*cloud_out.width,
                  buff_ptr,
                  cloud_out.width * sizeof (PointT));
        }
      }
    }
  }
}

} // namespace pcl


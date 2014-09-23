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

#ifndef PCL_IO_IMPL_IO_HPP_
#define PCL_IO_IMPL_IO_HPP_

#include <pcl/common/concatenate.h>
#include <pcl/common/copy_point.h>
#include <pcl/point_types.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::getFieldIndex (const pcl::PointCloud<PointT> &, 
                    const std::string &field_name, 
                    std::vector<pcl::PCLPointField> &fields)
{
  fields.clear ();
  // Get the fields list
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));
  for (size_t d = 0; d < fields.size (); ++d)
    if (fields[d].name == field_name)
      return (static_cast<int>(d));
  return (-1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::getFieldIndex (const std::string &field_name, 
                    std::vector<pcl::PCLPointField> &fields)
{
  fields.clear ();
  // Get the fields list
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));
  for (size_t d = 0; d < fields.size (); ++d)
    if (fields[d].name == field_name)
      return (static_cast<int>(d));
  return (-1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::getFields (const pcl::PointCloud<PointT> &, std::vector<pcl::PCLPointField> &fields)
{
  fields.clear ();
  // Get the fields list
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::getFields (std::vector<pcl::PCLPointField> &fields)
{
  fields.clear ();
  // Get the fields list
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::string
pcl::getFieldsList (const pcl::PointCloud<PointT> &)
{
  // Get the fields list
  std::vector<pcl::PCLPointField> fields;
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));
  std::string result;
  for (size_t i = 0; i < fields.size () - 1; ++i)
    result += fields[i].name + " ";
  result += fields[fields.size () - 1].name;
  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                     pcl::PointCloud<PointOutT> &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = cloud_in.width;
  cloud_out.height   = cloud_in.height;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
  cloud_out.points.resize (cloud_in.points.size ());

  if (isSamePointType<PointInT, PointOutT> ())
    // Copy the whole memory block
    memcpy (&cloud_out.points[0], &cloud_in.points[0], cloud_in.points.size () * sizeof (PointInT));
  else
    // Iterate over each point
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
      copyPoint (cloud_in.points[i], cloud_out.points[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                     const std::vector<int> &indices,
                     pcl::PointCloud<PointT> &cloud_out)
{
  // Do we want to copy everything?
  if (indices.size () == cloud_in.points.size ())
  {
    cloud_out = cloud_in;
    return;
  }

  // Allocate enough space and copy the basics
  cloud_out.points.resize (indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = static_cast<uint32_t>(indices.size ());
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each point
  for (size_t i = 0; i < indices.size (); ++i)
    cloud_out.points[i] = cloud_in.points[indices[i]];
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                     const std::vector<int, Eigen::aligned_allocator<int> > &indices,
                     pcl::PointCloud<PointT> &cloud_out)
{
  // Do we want to copy everything?
  if (indices.size () == cloud_in.points.size ())
  {
    cloud_out = cloud_in;
    return;
  }

  // Allocate enough space and copy the basics
  cloud_out.points.resize (indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = static_cast<uint32_t> (indices.size ());
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each point
  for (size_t i = 0; i < indices.size (); ++i)
    cloud_out.points[i] = cloud_in.points[indices[i]];
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                     const std::vector<int> &indices,
                     pcl::PointCloud<PointOutT> &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out.points.resize (indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = uint32_t (indices.size ());
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each point
  for (size_t i = 0; i < indices.size (); ++i)
    copyPoint (cloud_in.points[indices[i]], cloud_out.points[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                     const std::vector<int, Eigen::aligned_allocator<int> > &indices,
                     pcl::PointCloud<PointOutT> &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out.points.resize (indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = static_cast<uint32_t> (indices.size ());
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each point
  for (size_t i = 0; i < indices.size (); ++i)
    copyPoint (cloud_in.points[indices[i]], cloud_out.points[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                     const pcl::PointIndices &indices,
                     pcl::PointCloud<PointT> &cloud_out)
{
  // Do we want to copy everything?
  if (indices.indices.size () == cloud_in.points.size ())
  {
    cloud_out = cloud_in;
    return;
  }

  // Allocate enough space and copy the basics
  cloud_out.points.resize (indices.indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = indices.indices.size ();
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each point
  for (size_t i = 0; i < indices.indices.size (); ++i)
    cloud_out.points[i] = cloud_in.points[indices.indices[i]];
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                     const pcl::PointIndices &indices,
                     pcl::PointCloud<PointOutT> &cloud_out)
{
  copyPointCloud (cloud_in, indices.indices, cloud_out);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                     const std::vector<pcl::PointIndices> &indices,
                     pcl::PointCloud<PointT> &cloud_out)
{
  int nr_p = 0;
  for (size_t i = 0; i < indices.size (); ++i)
    nr_p += indices[i].indices.size ();

  // Do we want to copy everything? Remember we assume UNIQUE indices
  if (nr_p == cloud_in.points.size ())
  {
    cloud_out = cloud_in;
    return;
  }

  // Allocate enough space and copy the basics
  cloud_out.points.resize (nr_p);
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = nr_p;
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each cluster
  int cp = 0;
  for (size_t cc = 0; cc < indices.size (); ++cc)
  {
    // Iterate over each idx
    for (size_t i = 0; i < indices[cc].indices.size (); ++i)
    {
      // Iterate over each dimension
      cloud_out.points[cp] = cloud_in.points[indices[cc].indices[i]];
      cp++;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in, 
                     const std::vector<pcl::PointIndices> &indices,
                     pcl::PointCloud<PointOutT> &cloud_out)
{
  int nr_p = 0;
  for (size_t i = 0; i < indices.size (); ++i)
    nr_p += indices[i].indices.size ();

  // Do we want to copy everything? Remember we assume UNIQUE indices
  if (nr_p == cloud_in.points.size ())
  {
    copyPointCloud<PointInT, PointOutT> (cloud_in, cloud_out);
    return;
  }

  // Allocate enough space and copy the basics
  cloud_out.points.resize (nr_p);
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = nr_p;
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each cluster
  int cp = 0;
  for (size_t cc = 0; cc < indices.size (); ++cc)
  {
    // Iterate over each idx
    for (size_t i = 0; i < indices[cc].indices.size (); ++i)
    {
      copyPoint (cloud_in.points[indices[cc].indices[i]], cloud_out.points[cp]);
      ++cp;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointIn1T, typename PointIn2T, typename PointOutT> void
pcl::concatenateFields (const pcl::PointCloud<PointIn1T> &cloud1_in,
                        const pcl::PointCloud<PointIn2T> &cloud2_in,
                        pcl::PointCloud<PointOutT> &cloud_out)
{
  typedef typename pcl::traits::fieldList<PointIn1T>::type FieldList1;
  typedef typename pcl::traits::fieldList<PointIn2T>::type FieldList2;

  if (cloud1_in.points.size () != cloud2_in.points.size ())
  {
    PCL_ERROR ("[pcl::concatenateFields] The number of points in the two input datasets differs!\n");
    return;
  }

  // Resize the output dataset
  cloud_out.points.resize (cloud1_in.points.size ());
  cloud_out.header   = cloud1_in.header;
  cloud_out.width    = cloud1_in.width;
  cloud_out.height   = cloud1_in.height;
  if (!cloud1_in.is_dense || !cloud2_in.is_dense)
    cloud_out.is_dense = false;
  else
    cloud_out.is_dense = true;

  // Iterate over each point
  for (size_t i = 0; i < cloud_out.points.size (); ++i)
  {
    // Iterate over each dimension
    pcl::for_each_type <FieldList1> (pcl::NdConcatenateFunctor <PointIn1T, PointOutT> (cloud1_in.points[i], cloud_out.points[i]));
    pcl::for_each_type <FieldList2> (pcl::NdConcatenateFunctor <PointIn2T, PointOutT> (cloud2_in.points[i], cloud_out.points[i]));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
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
      const PointT* in = &(cloud_in.points[0]);
      PointT* out = &(cloud_out.points[0]);
      PointT* out_inner = out + cloud_out.width*top + left;
      for (uint32_t i = 0; i < cloud_in.height; i++, out_inner += cloud_out.width, in += cloud_in.width)
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

          const PointT* in = &(cloud_in.points[0]);
          PointT* out = &(cloud_out.points[0]);
          PointT* out_inner = out + cloud_out.width*top + left;

          for (uint32_t i = 0; i < cloud_in.height; i++, out_inner += cloud_out.width, in += cloud_in.width)
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
        catch (pcl::BadArgumentException &e)
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
        const PointT* in = &(cloud_in.points[0]);
        PointT* out = &(cloud_out.points[0]);
        PointT* out_inner = out + cloud_out.width*top + left;

        for (uint32_t i = 0; i < cloud_in.height; i++, out_inner += cloud_out.width, in += cloud_in.width)
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

#endif // PCL_IO_IMPL_IO_H_


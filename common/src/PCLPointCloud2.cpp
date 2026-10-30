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

#include <vector>

#include <pcl/common/io.h>
#include <pcl/pcl_macros.h>
#include <pcl/exceptions.h>
#include <pcl/PCLPointCloud2.h>

bool
pcl::PCLPointCloud2::concatenate (pcl::PCLPointCloud2 &cloud1, const pcl::PCLPointCloud2 &cloud2)
{
  if (cloud1.is_bigendian != cloud2.is_bigendian)
  {
    // In future, it might be possible to convert based on pcl::getFieldSize(fields.datatype)
    PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Endianness of clouds does not match\n");
    return (false);
  }

  const auto size1 = cloud1.width * cloud1.height;
  const auto size2 = cloud2.width * cloud2.height;
  //if one input cloud has no points, but the other input does, just select the cloud with points
  if ((size1 == 0) && (size2 != 0)) {
    cloud1 = cloud2;
  }

  if ((size1 == 0) || (size2 == 0)) {
    cloud1.header.stamp = std::max (cloud1.header.stamp, cloud2.header.stamp);
    return true;
  }

  // The data of each cloud has to hold width * height points of point_step bytes
  auto data_holds_points = [](const pcl::PCLPointCloud2& cloud)
  {
    const auto nr_points = static_cast<std::size_t> (cloud.width) * cloud.height;
    return (cloud.point_step == 0 || cloud.data.size () / cloud.point_step >= nr_points);
  };
  if (!data_holds_points (cloud1) || !data_holds_points (cloud2))
  {
    PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Size of data in cloud1 (%zu) or cloud2 (%zu) is smaller than width * height * point_step\n", cloud1.data.size (), cloud2.data.size ());
    return (false);
  }

  // Ideally this should be in PCLPointField class since this is global behavior
  auto field_eq = [](const auto& field1, const auto& field2)
  {
    // We're fine with the special RGB vs RGBA use case
    return ((field1.name == field2.name) ||
            (field1.name == "rgb" && field2.name == "rgba") ||
            (field1.name == "rgba" && field2.name == "rgb"));
  };

  // The data of a field is copied as is, so both fields need the same number of bytes
  auto field_size_eq = [](const auto& field1, const auto& field2)
  {
    return ((field1.count == field2.count) &&
            (pcl::getFieldSize (field1.datatype) == pcl::getFieldSize (field2.datatype)));
  };

  auto layout_eq = [&](const auto& field1, const auto& field2)
  {
    return (field_eq (field1, field2) &&
            (field1.offset == field2.offset) &&
            field_size_eq (field1, field2));
  };

  // A simple memcpy is possible if layout (name, order, offset and size of fields) is same for both clouds
  bool simple_layout = (cloud1.point_step == cloud2.point_step) &&
                       std::equal(cloud1.fields.begin (),
                                  cloud1.fields.end (),
                                  cloud2.fields.begin (),
                                  cloud2.fields.end (),
                                  layout_eq);

  // A field has to lie within one point of its cloud
  auto field_in_point = [](const pcl::PCLPointField& field, const pcl::uindex_t point_step)
  {
    const auto field_size = static_cast<pcl::uindex_t> (pcl::getFieldSize (field.datatype));
    return ((field.offset <= point_step) &&
            (field_size == 0 || field.count <= (point_step - field.offset) / field_size));
  };

  struct FieldDetails
  {
    std::size_t idx1, idx2;
    std::uint16_t size;
    FieldDetails (std::size_t idx1_, std::size_t idx2_, std::uint16_t size_): idx1 (idx1_), idx2 (idx2_), size (size_)
    {}
  };
  std::vector<FieldDetails> valid_fields;
  const auto max_field_size = std::max (cloud1.fields.size (), cloud2.fields.size ());
  valid_fields.reserve (max_field_size);

  // @TODO: Refactor to return std::optional<std::vector<FieldDetails>>
  // Store the details of fields with common data in both cloud, exit early if any errors are found
  if (!simple_layout)
  {
    std::size_t i = 0, j = 0;
    while (i < cloud1.fields.size () && j < cloud2.fields.size ())
    {
      if (cloud1.fields[i].name == "_")
      {
        ++i;
        continue;
      }
      if (cloud2.fields[j].name == "_")
      {
        ++j;
        continue;
      }

      if (field_eq(cloud1.fields[i], cloud2.fields[j]))
      {
        if (!field_size_eq (cloud1.fields[i], cloud2.fields[j]))
        {
          PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Size of field %s in cloud1 does not match size of field %s in cloud2\n", cloud1.fields[i].name.c_str (), cloud2.fields[j].name.c_str ());
          return (false);
        }
        if (!field_in_point (cloud1.fields[i], cloud1.point_step) || !field_in_point (cloud2.fields[j], cloud2.point_step))
        {
          PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Field %s does not fit in a point of cloud1 or cloud2\n", cloud1.fields[i].name.c_str ());
          return (false);
        }
        // Fields without data have nothing to copy
        if (cloud2.fields[j].count != 0 && pcl::getFieldSize (cloud2.fields[j].datatype) != 0)
          valid_fields.emplace_back(i, j, pcl::getFieldSize (cloud2.fields[j].datatype));
        ++i;
        ++j;
        continue;
      }
      PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Name of field %zu in cloud1, %s, does not match name of field %zu in cloud2, %s\n", i, cloud1.fields[i].name.c_str (), j, cloud2.fields[j].name.c_str ());
      return (false);
    }
    // Both i and j should have exhausted their respective cloud.fields
    if (i != cloud1.fields.size () || j != cloud2.fields.size ())
    {
      PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Number of fields to copy in cloud1 (%zu) != Number of fields to copy in cloud2 (%zu)\n", i, j);
      return (false);
    }
    // The points of cloud2 are added with the point_step of cloud1
    if (cloud2.point_step == 0)
    {
      PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] point_step of cloud2 is 0\n");
      return (false);
    }
    if (cloud1.point_step != 0 && size2 > (cloud1.data.max_size () - cloud1.data.size ()) / cloud1.point_step)
    {
      PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Too many points in cloud2 for the point_step of cloud1\n");
      return (false);
    }
  }

  // Save the latest timestamp in the destination cloud
  cloud1.header.stamp = std::max (cloud1.header.stamp, cloud2.header.stamp);

  cloud1.is_dense = cloud1.is_dense && cloud2.is_dense;
  cloud1.height = 1;
  cloud1.width = size1 + size2;
  cloud1.row_step = cloud1.width * cloud1.point_step; // changed width

  if (simple_layout)
  {
    cloud1.data.insert (cloud1.data.end (), cloud2.data.begin (), cloud2.data.end ());
    return (true);
  }
  const auto data1_size = cloud1.data.size ();
  cloud1.data.resize(data1_size + static_cast<std::size_t> (size2) * cloud1.point_step);
  for (std::size_t cp = 0; cp < size2; ++cp)
  {
    for (const auto& field_data: valid_fields)
    {
      const auto& i = field_data.idx1;
      const auto& j = field_data.idx2;
      const std::size_t size = field_data.size;
      // Leave the data for the skip fields untouched in cloud1
      // Copy only the required data from cloud2 to the correct location for cloud1
      memcpy (reinterpret_cast<char*> (&cloud1.data[data1_size + cp * cloud1.point_step + cloud1.fields[i].offset]),
              reinterpret_cast<const char*> (&cloud2.data[cp * cloud2.point_step + cloud2.fields[j].offset]),
              cloud2.fields[j].count * size);
    }
  }
  return (true);
}

pcl::PCLPointCloud2&
pcl::PCLPointCloud2::operator += (const PCLPointCloud2& rhs)
{
  if (concatenate((*this), rhs))
  {
    return *this;
  }
  PCL_THROW_EXCEPTION(ComputeFailedException, "Field or Endian mismatch. Please check log for more details");
}

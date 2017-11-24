/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_CONVERSIONS_H_
#define PCL_CONVERSIONS_H_

#ifdef __GNUC__
#pragma GCC system_header
#endif

#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLImage.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/exceptions.h>
#include <pcl/console/print.h>
#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#endif

namespace pcl
{
  namespace detail
  {
    // For converting template point cloud to message.
    template<typename PointT>
    struct FieldAdder
    {
      FieldAdder (std::vector<pcl::PCLPointField>& fields) : fields_ (fields) {};

      template<typename U> void operator() ()
      {
        pcl::PCLPointField f;
        f.name = traits::name<PointT, U>::value;
        f.offset = traits::offset<PointT, U>::value;
        f.datatype = traits::datatype<PointT, U>::value;
        f.count = traits::datatype<PointT, U>::size;
        fields_.push_back (f);
      }

      std::vector<pcl::PCLPointField>& fields_;
    };

    // For converting message to template point cloud.
    template<typename PointT>
    struct FieldMapper
    {
      FieldMapper (const std::vector<pcl::PCLPointField>& fields,
                   std::vector<FieldMapping>& map)
        : fields_ (fields), map_ (map)
      {
      }

      template<typename Tag> void
      operator () ()
      {
        BOOST_FOREACH (const pcl::PCLPointField& field, fields_)
        {
          if (FieldMatches<PointT, Tag>()(field))
          {
            FieldMapping mapping;
            mapping.serialized_offset = field.offset;
            mapping.struct_offset = traits::offset<PointT, Tag>::value;
            mapping.size = sizeof (typename traits::datatype<PointT, Tag>::type);
            map_.push_back (mapping);
            return;
          }
        }
        // Disable thrown exception per #595: http://dev.pointclouds.org/issues/595
        PCL_WARN ("Failed to find match for field '%s'.\n", traits::name<PointT, Tag>::value);
        //throw pcl::InvalidConversionException (ss.str ());
      }

      const std::vector<pcl::PCLPointField>& fields_;
      std::vector<FieldMapping>& map_;
    };

    inline bool
    fieldOrdering (const FieldMapping& a, const FieldMapping& b)
    {
      return (a.serialized_offset < b.serialized_offset);
    }

  } //namespace detail

  template<typename PointT> void
  createMapping (const std::vector<pcl::PCLPointField>& msg_fields, MsgFieldMap& field_map)
  {
    // Create initial 1-1 mapping between serialized data segments and struct fields
    detail::FieldMapper<PointT> mapper (msg_fields, field_map);
    for_each_type< typename traits::fieldList<PointT>::type > (mapper);

    // Coalesce adjacent fields into single memcpy's where possible
    if (field_map.size() > 1)
    {
      std::sort(field_map.begin(), field_map.end(), detail::fieldOrdering);
      MsgFieldMap::iterator i = field_map.begin(), j = i + 1;
      while (j != field_map.end())
      {
        // This check is designed to permit padding between adjacent fields.
        /// @todo One could construct a pathological case where the struct has a
        /// field where the serialized data has padding
        if (j->serialized_offset - i->serialized_offset == j->struct_offset - i->struct_offset)
        {
          i->size += (j->struct_offset + j->size) - (i->struct_offset + i->size);
          j = field_map.erase(j);
        }
        else
        {
          ++i;
          ++j;
        }
      }
    }
  }

  /** \brief Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map.
    * \param[in] msg the PCLPointCloud2 binary blob
    * \param[out] cloud the resultant pcl::PointCloud<T>
    * \param[in] field_map a MsgFieldMap object
    *
    * \note Use fromPCLPointCloud2 (PCLPointCloud2, PointCloud<T>) directly or create you
    * own MsgFieldMap using:
    *
    * \code
    * MsgFieldMap field_map;
    * createMapping<PointT> (msg.fields, field_map);
    * \endcode
    */
  template <typename PointT> void
  fromPCLPointCloud2 (const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
              const MsgFieldMap& field_map)
  {
    // Copy info fields
    cloud.header   = msg.header;
    cloud.width    = msg.width;
    cloud.height   = msg.height;
    cloud.is_dense = msg.is_dense == 1;

    // Copy point data
    uint32_t num_points = msg.width * msg.height;
    cloud.points.resize (num_points);
    uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud.points[0]);

    // Check if we can copy adjacent points in a single memcpy
    if (field_map.size() == 1 &&
        field_map[0].serialized_offset == 0 &&
        field_map[0].struct_offset == 0 &&
        msg.point_step == sizeof(PointT))
    {
      uint32_t cloud_row_step = static_cast<uint32_t> (sizeof (PointT) * cloud.width);
      const uint8_t* msg_data = &msg.data[0];
      // Should usually be able to copy all rows at once
      if (msg.row_step == cloud_row_step)
      {
        memcpy (cloud_data, msg_data, msg.data.size ());
      }
      else
      {
        for (uint32_t i = 0; i < msg.height; ++i, cloud_data += cloud_row_step, msg_data += msg.row_step)
          memcpy (cloud_data, msg_data, cloud_row_step);
      }

    }
    else
    {
      // If not, memcpy each group of contiguous fields separately
      for (uint32_t row = 0; row < msg.height; ++row)
      {
        const uint8_t* row_data = &msg.data[row * msg.row_step];
        for (uint32_t col = 0; col < msg.width; ++col)
        {
          const uint8_t* msg_data = row_data + col * msg.point_step;
          BOOST_FOREACH (const detail::FieldMapping& mapping, field_map)
          {
            memcpy (cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
          }
          cloud_data += sizeof (PointT);
        }
      }
    }
  }

  /** \brief Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object.
    * \param[in] msg the PCLPointCloud2 binary blob
    * \param[out] cloud the resultant pcl::PointCloud<T>
    */
  template<typename PointT> void
  fromPCLPointCloud2 (const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud)
  {
    MsgFieldMap field_map;
    createMapping<PointT> (msg.fields, field_map);
    fromPCLPointCloud2 (msg, cloud, field_map);
  }

  /** \brief Convert a pcl::PointCloud<T> object to a PCLPointCloud2 binary data blob.
    * \param[in] cloud the input pcl::PointCloud<T>
    * \param[out] msg the resultant PCLPointCloud2 binary blob
    */
  template<typename PointT> void
  toPCLPointCloud2 (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)
  {
    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloud.width == 0 && cloud.height == 0)
    {
      msg.width  = static_cast<uint32_t>(cloud.points.size ());
      msg.height = 1;
    }
    else
    {
      assert (cloud.points.size () == cloud.width * cloud.height);
      msg.height = cloud.height;
      msg.width  = cloud.width;
    }

    // Fill point cloud binary data (padding and all)
    size_t data_size = sizeof (PointT) * cloud.points.size ();
    msg.data.resize (data_size);
    if (data_size)
    {
      memcpy(&msg.data[0], &cloud.points[0], data_size);
    }

    // Fill fields metadata
    msg.fields.clear ();
    for_each_type<typename traits::fieldList<PointT>::type> (detail::FieldAdder<PointT>(msg.fields));

    msg.header     = cloud.header;
    msg.point_step = sizeof (PointT);
    msg.row_step   = static_cast<uint32_t> (sizeof (PointT) * msg.width);
    msg.is_dense   = cloud.is_dense;
    /// @todo msg.is_bigendian = ?;
  }

   /** \brief Copy the RGB fields of a PointCloud into pcl::PCLImage format
     * \param[in] cloud the point cloud message
     * \param[out] msg the resultant pcl::PCLImage
     * CloudT cloud type, CloudT should be akin to pcl::PointCloud<pcl::PointXYZRGBA>
     * \note will throw std::runtime_error if there is a problem
     */
  template<typename CloudT> void
  toPCLPointCloud2 (const CloudT& cloud, pcl::PCLImage& msg)
  {
    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloud.width == 0 && cloud.height == 0)
      throw std::runtime_error("Needs to be a dense like cloud!!");
    else
    {
      if (cloud.points.size () != cloud.width * cloud.height)
        throw std::runtime_error("The width and height do not match the cloud size!");
      msg.height = cloud.height;
      msg.width = cloud.width;
    }

    // ensor_msgs::image_encodings::BGR8;
    msg.encoding = "bgr8";
    msg.step = msg.width * sizeof (uint8_t) * 3;
    msg.data.resize (msg.step * msg.height);
    for (size_t y = 0; y < cloud.height; y++)
    {
      for (size_t x = 0; x < cloud.width; x++)
      {
        uint8_t * pixel = &(msg.data[y * msg.step + x * 3]);
        memcpy (pixel, &cloud (x, y).rgb, 3 * sizeof(uint8_t));
      }
    }
  }

  /** \brief Copy the RGB fields of a PCLPointCloud2 msg into pcl::PCLImage format
    * \param cloud the point cloud message
    * \param msg the resultant pcl::PCLImage
    * will throw std::runtime_error if there is a problem
    */
  inline void
  toPCLPointCloud2 (const pcl::PCLPointCloud2& cloud, pcl::PCLImage& msg)
  {
    int rgb_index = -1;
    // Get the index we need
    for (size_t d = 0; d < cloud.fields.size (); ++d)
      if (cloud.fields[d].name == "rgb")
      {
        rgb_index = static_cast<int>(d);
        break;
      }

    if(rgb_index == -1)
      throw std::runtime_error ("No rgb field!!");
    if (cloud.width == 0 && cloud.height == 0)
      throw std::runtime_error ("Needs to be a dense like cloud!!");
    else
    {
      msg.height = cloud.height;
      msg.width = cloud.width;
    }
    int rgb_offset = cloud.fields[rgb_index].offset;
    int point_step = cloud.point_step;

    // pcl::image_encodings::BGR8;
    msg.encoding = "bgr8";
    msg.step = static_cast<uint32_t>(msg.width * sizeof (uint8_t) * 3);
    msg.data.resize (msg.step * msg.height);

    for (size_t y = 0; y < cloud.height; y++)
    {
      for (size_t x = 0; x < cloud.width; x++, rgb_offset += point_step)
      {
        uint8_t * pixel = &(msg.data[y * msg.step + x * 3]);
        memcpy (pixel, &(cloud.data[rgb_offset]), 3 * sizeof (uint8_t));
      }
    }
  }
}

#endif  //#ifndef PCL_CONVERSIONS_H_

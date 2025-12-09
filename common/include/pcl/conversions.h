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

#pragma once

#ifdef __GNUC__
#pragma GCC system_header
#endif

#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLImage.h>
#include <pcl/point_cloud.h>
#include <pcl/type_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/console/print.h>

#include <algorithm>
#include <iterator>
#include <numeric> // for accumulate

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
        f.name = pcl::traits::name<PointT, U>::value;
        f.offset = pcl::traits::offset<PointT, U>::value;
        f.datatype = pcl::traits::datatype<PointT, U>::value;
        f.count = pcl::traits::datatype<PointT, U>::size;
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
        for (const auto& field : fields_)
        {
          if (FieldMatches<PointT, Tag>()(field))
          {
            FieldMapping mapping;
            mapping.serialized_offset = field.offset;
            mapping.struct_offset = pcl::traits::offset<PointT, Tag>::value;
            mapping.size = sizeof (typename pcl::traits::datatype<PointT, Tag>::type);
            map_.push_back (mapping);
            return;
          }
        }
        // Disable thrown exception per #595: http://dev.pointclouds.org/issues/595
        PCL_WARN ("Failed to find exact match for field '%s'.\n", pcl::traits::name<PointT, Tag>::value);
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

    // Helps converting PCLPointCloud2 to templated point cloud. Casts fields if datatype is different.
    template<typename PointT>
    struct FieldCaster
    {
      FieldCaster (const std::vector<pcl::PCLPointField>& fields, const pcl::PCLPointCloud2& msg, const std::uint8_t* msg_data, std::uint8_t* cloud_data)
        : fields_ (fields), msg_(msg), msg_data_(msg_data), cloud_data_(cloud_data)
      {}

      template<typename Tag> void
      operator () ()
      {
        // first check whether any field matches exactly. Then there is nothing to do because the contents are memcpy-ed elsewhere
        for (const auto& field : fields_) {
          if (FieldMatches<PointT, Tag>()(field)) {
            return;
          }
        }
        for (const auto& field : fields_)
        {
          // The following check is similar to FieldMatches, but it tests for different datatypes
          if ((field.name == pcl::traits::name<PointT, Tag>::value) &&
              (field.datatype != pcl::traits::datatype<PointT, Tag>::value) &&
              ((field.count == pcl::traits::datatype<PointT, Tag>::size) ||
               (field.count == 0 && pcl::traits::datatype<PointT, Tag>::size == 1))) {
#define PCL_CAST_POINT_FIELD(TYPE) case ::pcl::traits::asEnum_v<TYPE>: \
            PCL_WARN("Will try to cast field '%s' (original type is " #TYPE "). You may loose precision during casting. Make sure that this is acceptable or choose a different point type.\n", pcl::traits::name<PointT, Tag>::value); \
            for (std::size_t row = 0; row < msg_.height; ++row) { \
              const std::uint8_t* row_data = msg_data_ + row * msg_.row_step; \
              for (std::size_t col = 0; col < msg_.width; ++col) { \
                const std::uint8_t* msg_data = row_data + col * msg_.point_step; \
                for(std::uint32_t i=0; i<pcl::traits::datatype<PointT, Tag>::size; ++i) { \
                  *(reinterpret_cast<typename pcl::traits::datatype<PointT, Tag>::decomposed::type*>(cloud_data + pcl::traits::offset<PointT, Tag>::value) + i) = *(reinterpret_cast<const TYPE*>(msg_data + field.offset) + i); \
                } \
                cloud_data += sizeof (PointT); \
              } \
            } \
            break;
            // end of PCL_CAST_POINT_FIELD definition

            std::uint8_t* cloud_data = cloud_data_;
            switch(field.datatype) {
              PCL_CAST_POINT_FIELD(bool)
              PCL_CAST_POINT_FIELD(std::int8_t)
              PCL_CAST_POINT_FIELD(std::uint8_t)
              PCL_CAST_POINT_FIELD(std::int16_t)
              PCL_CAST_POINT_FIELD(std::uint16_t)
              PCL_CAST_POINT_FIELD(std::int32_t)
              PCL_CAST_POINT_FIELD(std::uint32_t)
              PCL_CAST_POINT_FIELD(std::int64_t)
              PCL_CAST_POINT_FIELD(std::uint64_t)
              PCL_CAST_POINT_FIELD(float)
              PCL_CAST_POINT_FIELD(double)
              default: std::cout << "Unknown datatype: " << field.datatype << std::endl;
            }
            return;
          }
#undef PCL_CAST_POINT_FIELD
        }
      }

      const std::vector<pcl::PCLPointField>& fields_;
      const pcl::PCLPointCloud2& msg_;
      const std::uint8_t* msg_data_;
      std::uint8_t* cloud_data_;
    };
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
    * \param[in] msg the PCLPointCloud2 binary blob (note that the binary point data in msg.data will not be used!)
    * \param[out] cloud the resultant pcl::PointCloud<T>
    * \param[in] field_map a MsgFieldMap object
    * \param[in] msg_data pointer to binary blob data, used instead of msg.data
    *
    * \note Use fromPCLPointCloud2 (PCLPointCloud2, PointCloud<T>) instead, except if you have a binary blob of
    * point data that you do not want to copy into a pcl::PCLPointCloud2 in order to use fromPCLPointCloud2.
    */
  template <typename PointT> void
  fromPCLPointCloud2 (const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
              const MsgFieldMap& field_map, const std::uint8_t* msg_data)
  {
    // Copy info fields
    cloud.header   = msg.header;
    cloud.width    = msg.width;
    cloud.height   = msg.height;
    cloud.is_dense = msg.is_dense == 1;

    // Resize cloud
    cloud.resize (msg.width * msg.height);

    // check if there is data to copy
    if (msg.width * msg.height == 0)
    {
      return;
    }

    // Copy point data
    std::uint8_t* cloud_data = reinterpret_cast<std::uint8_t*>(cloud.data());

    // Check if we can copy adjacent points in a single memcpy.  We can do so if there
    // is exactly one field to copy and it is the same size as the source and destination
    // point types.
    if (field_map.size() == 1 &&
        field_map[0].serialized_offset == 0 &&
        field_map[0].struct_offset == 0 &&
        field_map[0].size == msg.point_step &&
        field_map[0].size == sizeof(PointT))
    {
      const auto cloud_row_step = (sizeof (PointT) * cloud.width);
      // Should usually be able to copy all rows at once
      if (msg.row_step == cloud_row_step)
      {
        memcpy (cloud_data, msg_data, msg.width * msg.height * sizeof(PointT));
      }
      else
      {
        for (uindex_t i = 0; i < msg.height; ++i, cloud_data += cloud_row_step, msg_data += msg.row_step)
          memcpy (cloud_data, msg_data, cloud_row_step);
      }

    }
    else
    {
      // If not, memcpy each group of contiguous fields separately
      for (std::size_t row = 0; row < msg.height; ++row)
      {
        const std::uint8_t* row_data = msg_data + row * msg.row_step;
        for (std::size_t col = 0; col < msg.width; ++col)
        {
          const std::uint8_t* msg_data = row_data + col * msg.point_step;
          for (const detail::FieldMapping& mapping : field_map)
          {
            std::copy(msg_data + mapping.serialized_offset, msg_data + mapping.serialized_offset + mapping.size,
                        cloud_data + mapping.struct_offset);
          }
          cloud_data += sizeof (PointT);
        }
      }
    }
    // if any fields in msg and cloud have different datatypes but the same name, we cast them:
    detail::FieldCaster<PointT> caster (msg.fields, msg, msg_data, reinterpret_cast<std::uint8_t*>(cloud.data()));
    for_each_type< typename traits::fieldList<PointT>::type > (caster);
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
    fromPCLPointCloud2 (msg, cloud, field_map, msg.data.data());
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

  namespace detail {
    /** \brief Used together with `pcl::for_each_type`, copies all point fields from `cloud_data` (respecting each field offset) to `msg_data` (tightly packed).
      */
    template<typename PointT>
    struct FieldCopier {
      FieldCopier(std::uint8_t*& msg_data, const std::uint8_t*& cloud_data) : msg_data_ (msg_data), cloud_data_ (cloud_data) {};

      template<typename U> void operator() () {
        memcpy(msg_data_, cloud_data_ + pcl::traits::offset<PointT, U>::value, sizeof(typename pcl::traits::datatype<PointT, U>::type));
        msg_data_ += sizeof(typename pcl::traits::datatype<PointT, U>::type);
      }

      std::uint8_t*& msg_data_;
      const std::uint8_t*& cloud_data_;
    };

    /** \brief Used together with `pcl::for_each_type`, creates list of all fields, and list of size of each field.
      */
    template<typename PointT>
    struct FieldAdderAdvanced
    {
      FieldAdderAdvanced (std::vector<pcl::PCLPointField>& fields, std::vector<std::size_t>& field_sizes) : fields_ (fields), field_sizes_ (field_sizes) {};

      template<typename U> void operator() ()
      {
        pcl::PCLPointField f;
        f.name = pcl::traits::name<PointT, U>::value;
        f.offset = pcl::traits::offset<PointT, U>::value;
        f.datatype = pcl::traits::datatype<PointT, U>::value;
        f.count = pcl::traits::datatype<PointT, U>::size;
        fields_.push_back (f);
        field_sizes_.push_back (sizeof(typename pcl::traits::datatype<PointT, U>::type)); // If field is an array, then this is the size of all array elements
      }

      std::vector<pcl::PCLPointField>& fields_;
      std::vector<std::size_t>& field_sizes_;
    };
  } // namespace detail

  /** \brief Convert a pcl::PointCloud<T> object to a PCLPointCloud2 binary data blob.
    * \param[in] cloud the input pcl::PointCloud<T>
    * \param[out] msg the resultant PCLPointCloud2 binary blob
    * \param[in] padding Many point types have padding to ensure alignment and SIMD compatibility. Setting this to true will copy the padding to the `PCLPointCloud2` (the default in older PCL versions). Setting this to false will make the data blob in `PCLPointCloud2` smaller, while still keeping all information (useful e.g. when sending msg over network or storing it). The amount of padding depends on the point type, and can in some cases be up to 50 percent.
    */
  template<typename PointT> void
  toPCLPointCloud2 (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg, bool padding)
  {
    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloud.width == 0 && cloud.height == 0)
    {
      msg.width  = cloud.size ();
      msg.height = 1;
    }
    else
    {
      assert (cloud.size () == cloud.width * cloud.height);
      msg.height = cloud.height;
      msg.width  = cloud.width;
    }
    // Fill fields metadata
    msg.fields.clear ();
    std::vector<std::size_t> field_sizes;
    for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdderAdvanced<PointT>(msg.fields, field_sizes));
    // Check if padding should be kept, or if the point type does not contain padding (then the single memcpy is faster)
    if (padding || std::accumulate(field_sizes.begin(), field_sizes.end(), static_cast<std::size_t>(0)) == sizeof (PointT)) {
      // Fill point cloud binary data (padding and all)
      std::size_t data_size = sizeof (PointT) * cloud.size ();
      msg.data.resize (data_size);
      if (data_size)
      {
        memcpy(msg.data.data(), cloud.data(), data_size);
      }

      msg.point_step = sizeof (PointT);
      msg.row_step   = (sizeof (PointT) * msg.width);
    } else {
      std::size_t point_size = 0;
      for(std::size_t i=0; i<msg.fields.size(); ++i) {
        msg.fields[i].offset = point_size; // Adjust offset when padding is removed
        point_size += field_sizes[i];
      }
      msg.data.resize (point_size * cloud.size());
      std::uint8_t* msg_data = &msg.data[0];
      const std::uint8_t* cloud_data=reinterpret_cast<const std::uint8_t*>(&cloud[0]);
      const std::uint8_t* end = cloud_data + sizeof (PointT) * cloud.size ();
      pcl::detail::FieldCopier<PointT> copier(msg_data, cloud_data); // copier takes msg_data and cloud_data as references, so the two are shared
      for (; cloud_data<end; cloud_data+=sizeof(PointT)) {
        for_each_type< typename traits::fieldList<PointT>::type > (copier);
      }

      msg.point_step = point_size;
      msg.row_step   = point_size * msg.width;
    }
    msg.header     = cloud.header;
    msg.is_dense   = cloud.is_dense;
    /// @todo msg.is_bigendian = ?;
  }

  /** \brief Convert a pcl::PointCloud<T> object to a PCLPointCloud2 binary data blob.
    * \param[in] cloud the input pcl::PointCloud<T>
    * \param[out] msg the resultant PCLPointCloud2 binary blob
    */
  template<typename PointT> void
  toPCLPointCloud2 (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)
  {
    toPCLPointCloud2 (cloud, msg, true); // true is the default in older PCL version
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
      if (cloud.size () != cloud.width * cloud.height)
        throw std::runtime_error("The width and height do not match the cloud size!");
      msg.height = cloud.height;
      msg.width = cloud.width;
    }

    // ensor_msgs::image_encodings::BGR8;
    msg.header = cloud.header;
    msg.encoding = "bgr8";
    msg.step = msg.width * sizeof (std::uint8_t) * 3;
    msg.data.resize (msg.step * msg.height);
    for (std::size_t y = 0; y < cloud.height; y++)
    {
      for (std::size_t x = 0; x < cloud.width; x++)
      {
        std::uint8_t * pixel = &(msg.data[y * msg.step + x * 3]);
        memcpy (pixel, &cloud (x, y).rgb, 3 * sizeof(std::uint8_t));
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
    const auto predicate = [](const auto& field) { return field.name == "rgb"; };
    const auto result = std::find_if(cloud.fields.cbegin (), cloud.fields.cend (), predicate);
    if (result == cloud.fields.end ())
      throw std::runtime_error ("No rgb field!!");

    const auto rgb_index = std::distance(cloud.fields.begin (), result);
    if (cloud.width == 0 && cloud.height == 0)
      throw std::runtime_error ("Needs to be a dense like cloud!!");
    else
    {
      msg.height = cloud.height;
      msg.width = cloud.width;
    }
    auto rgb_offset = cloud.fields[rgb_index].offset;
    const auto point_step = cloud.point_step;

    // pcl::image_encodings::BGR8;
    msg.header = cloud.header;
    msg.encoding = "bgr8";
    msg.step = (msg.width * sizeof (std::uint8_t) * 3);
    msg.data.resize (msg.step * msg.height);

    for (std::size_t y = 0; y < cloud.height; y++)
    {
      for (std::size_t x = 0; x < cloud.width; x++, rgb_offset += point_step)
      {
        std::uint8_t * pixel = &(msg.data[y * msg.step + x * 3]);
        std::copy(&cloud.data[rgb_offset], &cloud.data[rgb_offset] + 3, pixel);
      }
    }
  }
}

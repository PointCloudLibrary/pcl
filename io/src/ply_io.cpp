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

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <string>
#include <tuple>

// https://www.boost.org/doc/libs/1_70_0/libs/filesystem/doc/index.htm#Coding-guidelines
#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp> // for split

namespace fs = boost::filesystem;

std::tuple<std::function<void ()>, std::function<void ()> >
pcl::PLYReader::elementDefinitionCallback (const std::string& element_name, std::size_t count)
{
  if (element_name == "vertex")
  {
    cloud_->data.clear ();
    cloud_->fields.clear ();
    // Cloud dimensions may have already been set from obj_info fields
    if (cloud_->width == 0 || cloud_->height == 0)
    {
      cloud_->width = static_cast<std::uint32_t> (count);
      cloud_->height = 1;
    }
    cloud_->is_dense = true;
    cloud_->point_step = 0;
    cloud_->row_step = 0;
    vertex_count_ = 0;
    return (std::tuple<std::function<void ()>, std::function<void ()> > (
              [this] { vertexBeginCallback (); },
              [this] { vertexEndCallback (); }));
  }
  if ((element_name == "face") && polygons_)
  {
    polygons_->reserve (count);
    return (std::tuple<std::function<void ()>, std::function<void ()> > (
            [this] { faceBeginCallback (); },
            [this] { faceEndCallback (); }));
  }
  if (element_name == "camera")
  {
    cloud_->is_dense = true;
    return {};
  }
  if (element_name == "range_grid")
  {
    range_grid_->reserve (count);
    return (std::tuple<std::function<void ()>, std::function<void ()> > (
              [this] { rangeGridBeginCallback (); },
              [this] { rangeGridEndCallback (); }));
  }
  return {};
}

bool
pcl::PLYReader::endHeaderCallback ()
{
  cloud_->data.resize (static_cast<std::size_t>(cloud_->point_step) * cloud_->width * cloud_->height);
  return (true);
}

template<typename Scalar> void
pcl::PLYReader::appendScalarProperty (const std::string& name, const std::size_t& size)
{
  cloud_->fields.emplace_back();
  ::pcl::PCLPointField &current_field = cloud_->fields.back ();
  current_field.name = name;
  current_field.offset = cloud_->point_step;
  current_field.datatype = pcl::traits::asEnum<Scalar>::value;
  current_field.count = static_cast<std::uint32_t> (size);
  cloud_->point_step += static_cast<std::uint32_t> (pcl::getFieldSize (pcl::traits::asEnum<Scalar>::value) * size);
}

bool
pcl::PLYReader::amendProperty (const std::string& old_name, const std::string& new_name, std::uint8_t new_datatype)
{
  const auto fieldIndex = pcl::getFieldIndex(*cloud_, old_name);
  if (fieldIndex == -1) {
    return false;
  }

  auto& field = cloud_->fields[fieldIndex];

  field.name = new_name;
  
  if (new_datatype > 0 && new_datatype != field.datatype)
    field.datatype = new_datatype;

  return true;
}

namespace pcl
{
  template <>
  std::function<void (pcl::io::ply::float32)>
  PLYReader::scalarPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if (element_name == "vertex")
    {
      appendScalarProperty<pcl::io::ply::float32> (property_name, 1);
      return ([this] (pcl::io::ply::float32 value) { vertexScalarPropertyCallback<pcl::io::ply::float32> (value); });
    }
    if (element_name == "camera")
    {
      if (property_name == "view_px")
      {
        return [this] (const float& value) { originXCallback (value); };
      }
      if (property_name == "view_py")
      {
        return [this] (const float& value) { originYCallback (value); };
      }
      if (property_name == "view_pz")
      {
        return [this] (const float& value) { originZCallback (value); };
      }
      if (property_name == "x_axisx")
      {
        return [this] (const float& value) { orientationXaxisXCallback (value); };
      }
      if (property_name == "x_axisy")
      {
        return [this] (const float& value) { orientationXaxisYCallback (value); };
      }
      if (property_name == "x_axisz")
      {
        return [this] (const float& value) { orientationXaxisZCallback (value); };
      }
      if (property_name == "y_axisx")
      {
        return [this] (const float& value) { orientationYaxisXCallback (value); };
      }
      if (property_name == "y_axisy")
      {
        return [this] (const float& value) { orientationYaxisYCallback (value); };
      }
      if (property_name == "y_axisz")
      {
        return [this] (const float& value) { orientationYaxisZCallback (value); };
      }
      if (property_name == "z_axisx")
      {
        return [this] (const float& value) { orientationZaxisXCallback (value); };
      }
      if (property_name == "z_axisy")
      {
        return [this] (const float& value) { orientationZaxisYCallback (value); };
      }
      if (property_name == "z_axisz")
      {
        return [this] (const float& value) { orientationZaxisZCallback (value); };
      }
    }
    return {};
  }

  template <> std::function<void (pcl::io::ply::uint8)>
  PLYReader::scalarPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if (element_name == "vertex")
    {
      if ((property_name == "red") || (property_name == "green") || (property_name == "blue") ||
          (property_name == "diffuse_red") || (property_name == "diffuse_green") || (property_name == "diffuse_blue"))
      {
        if ((property_name == "red") || (property_name == "diffuse_red"))
          appendScalarProperty<pcl::io::ply::float32> ("rgb");
        return [this, property_name] (pcl::io::ply::uint8 color) { vertexColorCallback (property_name, color); };
      }
      if (property_name == "alpha")
      {
        if (!amendProperty("rgb", "rgba", pcl::PCLPointField::UINT32))
        {
          PCL_ERROR("[pcl::PLYReader::scalarPropertyDefinitionCallback] 'rgb' was not "
                    "found in cloud_->fields!,"
                    " can't amend property '%s' to get new type 'rgba' \n",
                    property_name.c_str());
          return {};
        }
        
        return [this] (pcl::io::ply::uint8 alpha) { vertexAlphaCallback (alpha); };
      }
      if (property_name == "intensity")
      {
        appendScalarProperty<pcl::io::ply::float32> (property_name);
        return [this] (pcl::io::ply::uint8 intensity) { vertexIntensityCallback (intensity); };
      }
      appendScalarProperty<pcl::io::ply::uint8> (property_name);
      return ([this] (pcl::io::ply::uint8 value) { vertexScalarPropertyCallback<pcl::io::ply::uint8> (value); });
    }
    return {};
  }

  template <> std::function<void (pcl::io::ply::int32)>
  PLYReader::scalarPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if (element_name == "vertex")
    {
      appendScalarProperty<pcl::io::ply::int32> (property_name, 1);
      return ([this] (pcl::io::ply::uint32 value) { vertexScalarPropertyCallback<pcl::io::ply::uint32> (value); });
    }
    if (element_name == "camera")
    {
      if (property_name == "viewportx")
      {
        return [this] (const int& width) { cloudWidthCallback (width); };
      }
      if (property_name == "viewporty")
      {
        return [this] (const int& height) { cloudHeightCallback (height); };
      }
      return {};
    }
    return {};
  }

  template <typename Scalar> std::function<void (Scalar)>
  PLYReader::scalarPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if (element_name == "vertex")
    {
      appendScalarProperty<Scalar> (property_name, 1);
      return ([this] (Scalar value) { vertexScalarPropertyCallback<Scalar> (value); });
    }
    return {};
  }

  template<typename T> inline
  std::enable_if_t<std::is_floating_point<T>::value>
  unsetDenseFlagIfNotFinite(T value, PCLPointCloud2* cloud)
  {
    //MSVC is missing bool std::isfinite(IntegralType arg); variant, so we implement an own template specialization for this
    if (!std::isfinite(value))
      cloud->is_dense = false;
  }

  template<typename T> inline
  std::enable_if_t<std::is_integral<T>::value>
  unsetDenseFlagIfNotFinite(T /* value */, PCLPointCloud2* /* cloud */)
  {
  }

  template<typename Scalar> void
  PLYReader::vertexScalarPropertyCallback (Scalar value)
  {
    try
    {
      unsetDenseFlagIfNotFinite(value, cloud_);
      cloud_->at<Scalar>(vertex_count_, vertex_offset_before_) = value;
      vertex_offset_before_ += static_cast<int> (sizeof (Scalar));
    }
    catch(const std::out_of_range&)
    {
      PCL_WARN ("[pcl::PLYReader::vertexScalarPropertyCallback] Incorrect data index specified (%lu)!\n", vertex_count_ * cloud_->point_step + vertex_offset_before_);
      assert(false);
    }
  }

  template <typename SizeType> void
  PLYReader::vertexListPropertyBeginCallback (const std::string& name, SizeType size)
  {
    // Adjust size only once
    if (vertex_count_ == 0)
    {
      auto finder = cloud_->fields.rbegin ();
      for (; finder != cloud_->fields.rend (); ++finder)
        if (finder->name == name)
          break;
      assert (finder != cloud_->fields.rend ());
      finder->count = size;
    }
  }

  template<typename ContentType> void
  PLYReader::vertexListPropertyContentCallback (ContentType value)
  {
    try
    {
      unsetDenseFlagIfNotFinite(value, cloud_);
      cloud_->at<ContentType>(vertex_count_, vertex_offset_before_) = value;
      vertex_offset_before_ += static_cast<int> (sizeof (ContentType));
    }
    catch(const std::out_of_range&)
    {
      PCL_WARN ("[pcl::PLYReader::vertexListPropertyContentCallback] Incorrect data index specified (%lu)!\n", vertex_count_ * cloud_->point_step + vertex_offset_before_);
      assert(false);
    }
  }

  template <typename SizeType, typename ContentType>
  std::tuple<std::function<void (SizeType)>, std::function<void (ContentType)>, std::function<void ()> >
  pcl::PLYReader::listPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if ((element_name == "range_grid") && (property_name == "vertex_indices" || property_name == "vertex_index"))
    {
      return std::tuple<std::function<void (SizeType)>, std::function<void (ContentType)>, std::function<void ()> > (
        [this] (SizeType size) { rangeGridVertexIndicesBeginCallback (size); },
        [this] (ContentType vertex_index) { rangeGridVertexIndicesElementCallback (vertex_index); },
        [this] { rangeGridVertexIndicesEndCallback (); }
      );
    }
    if ((element_name == "face") && (property_name == "vertex_indices" || property_name == "vertex_index") && polygons_)
    {
      return std::tuple<std::function<void (SizeType)>, std::function<void (ContentType)>, std::function<void ()> > (
        [this] (SizeType size) { faceVertexIndicesBeginCallback (size); },
        [this] (ContentType vertex_index) { faceVertexIndicesElementCallback (vertex_index); },
        [this] { faceVertexIndicesEndCallback (); }
      );
    }
    if (element_name == "vertex")
    {
      cloud_->fields.emplace_back();
      pcl::PCLPointField &current_field = cloud_->fields.back ();
      current_field.name = property_name;
      current_field.offset = cloud_->point_step;
      current_field.datatype = pcl::traits::asEnum<ContentType>::value;
      current_field.count = 1u; // value will be updated once first vertex is read
      if (sizeof (ContentType) + cloud_->point_step < std::numeric_limits<std::uint32_t>::max ())
        cloud_->point_step += static_cast<std::uint32_t> (sizeof (ContentType));
      else
        cloud_->point_step = static_cast<std::uint32_t> (std::numeric_limits<std::uint32_t>::max ());
      do_resize_ = true;
      return std::tuple<std::function<void (SizeType)>, std::function<void (ContentType)>, std::function<void ()> > (
        std::bind (&pcl::PLYReader::vertexListPropertyBeginCallback<SizeType>, this, property_name, std::placeholders::_1),
        [this] (ContentType value) { vertexListPropertyContentCallback (value); },
        [this] { vertexListPropertyEndCallback (); }
      );
    }
    PCL_WARN("[pcl::PLYReader::listPropertyDefinitionCallback] no fitting callbacks. element_name=%s, property_name=%s\n", element_name.c_str(), property_name.c_str());
    return {};
  }
}

void
pcl::PLYReader::vertexColorCallback (const std::string& color_name, pcl::io::ply::uint8 color)
{
  if ((color_name == "red") || (color_name == "diffuse_red"))
  {
    r_ = std::int32_t (color);
    rgb_offset_before_ = vertex_offset_before_;
  }
  if ((color_name == "green") || (color_name == "diffuse_green"))
  {
    g_ = std::int32_t (color);
  }
  if ((color_name == "blue") || (color_name == "diffuse_blue"))
  {
    b_ = std::int32_t (color);
    std::int32_t rgb = r_ << 16 | g_ << 8 | b_;
    try
    {
      cloud_->at<std::int32_t>(vertex_count_, rgb_offset_before_) = rgb;
      vertex_offset_before_ += static_cast<int> (sizeof (pcl::io::ply::float32));
    }
    catch(const std::out_of_range&)
    {
      PCL_WARN ("[pcl::PLYReader::vertexColorCallback] Incorrect data index specified (%lu)!\n", vertex_count_ * cloud_->point_step + rgb_offset_before_);
      assert(false);
    }
  }
}

void
pcl::PLYReader::vertexAlphaCallback (pcl::io::ply::uint8 alpha)
{
  // get anscient rgb value and store it in rgba
  rgba_ = cloud_->at<std::uint32_t>(vertex_count_, rgb_offset_before_);
  // append alpha
  a_ = std::uint32_t (alpha);
  rgba_ |= a_ << 24;
  // put rgba back
  cloud_->at<std::uint32_t>(vertex_count_, rgb_offset_before_) = rgba_;
}

void
pcl::PLYReader::vertexIntensityCallback (pcl::io::ply::uint8 intensity)
{
  pcl::io::ply::float32 intensity_ (intensity);
  cloud_->at<pcl::io::ply::float32>(vertex_count_, vertex_offset_before_) = intensity_;
  vertex_offset_before_ += static_cast<int> (sizeof (pcl::io::ply::float32));
}

void
pcl::PLYReader::vertexBeginCallback ()
{
  vertex_offset_before_ = 0;
}

void
pcl::PLYReader::vertexEndCallback ()
{
  // Resize data if needed
  if (vertex_count_ == 0 && do_resize_)
  {
    cloud_->point_step = vertex_offset_before_;
    cloud_->row_step = cloud_->point_step * cloud_->width;
    cloud_->data.resize (static_cast<std::size_t>(cloud_->point_step) * cloud_->width * cloud_->height);
  }
  ++vertex_count_;
}

void
pcl::PLYReader::rangeGridBeginCallback ()
{
  range_grid_->push_back (std::vector <int> ());
}

void
pcl::PLYReader::rangeGridVertexIndicesBeginCallback (pcl::io::ply::uint8 size)
{
  range_grid_->back ().reserve (size);
}

void
pcl::PLYReader::rangeGridVertexIndicesElementCallback (pcl::io::ply::int32 vertex_index)
{
  range_grid_->back ().push_back (vertex_index);
}

void
pcl::PLYReader::rangeGridVertexIndicesEndCallback () {}

void
pcl::PLYReader::rangeGridEndCallback () {}

void
pcl::PLYReader::faceBeginCallback ()
{
  polygons_->push_back (pcl::Vertices ());
}

void
pcl::PLYReader::faceVertexIndicesBeginCallback (pcl::io::ply::uint8 size)
{
  polygons_->back ().vertices.reserve (size);
}

void
pcl::PLYReader::faceVertexIndicesElementCallback (pcl::io::ply::int32 vertex_index)
{
  polygons_->back ().vertices.push_back (vertex_index);
}

void
pcl::PLYReader::faceVertexIndicesEndCallback () { }

void
pcl::PLYReader::faceEndCallback () {}

void
pcl::PLYReader::objInfoCallback (const std::string& line)
{
  std::vector<std::string> st;
  boost::split (st, line, boost::is_any_of (std::string ( "\t ")), boost::token_compress_on);
  assert (st[0].substr (0, 8) == "obj_info");
  {
    if (st.size() >= 3)
    {
      if (st[1] == "num_cols")
        cloudWidthCallback (atoi (st[2].c_str ()));
      else if (st[1] == "num_rows")
        cloudHeightCallback (atoi (st[2].c_str ()));
      else if (st[1] == "echo_rgb_offset_x")
        originXCallback (static_cast<float> (atof (st[2].c_str ())));
      else if (st[1] == "echo_rgb_offset_y")
        originYCallback (static_cast<float> (atof (st[2].c_str ())));
      else if (st[1] == "echo_rgb_offset_z")
        originZCallback (static_cast<float> (atof (st[2].c_str ())));
    }
  }
}

void
pcl::PLYReader::vertexListPropertyEndCallback () {}

bool
pcl::PLYReader::parse (const std::string& istream_filename)
{
  pcl::io::ply::ply_parser ply_parser;

  ply_parser.info_callback ([&, this] (std::size_t line_number, const std::string& message) { infoCallback (istream_filename, line_number, message); });
  ply_parser.warning_callback ([&, this] (std::size_t line_number, const std::string& message) { warningCallback (istream_filename, line_number, message); });
  ply_parser.error_callback ([&, this] (std::size_t line_number, const std::string& message) { errorCallback (istream_filename, line_number, message); });

  ply_parser.obj_info_callback ([this] (const std::string& line) { objInfoCallback (line); });
  ply_parser.element_definition_callback ([this] (const std::string& element_name, std::size_t count) { return elementDefinitionCallback (element_name, count); });
  ply_parser.end_header_callback ([this] { return endHeaderCallback (); });

  pcl::io::ply::ply_parser::scalar_property_definition_callbacks_type scalar_property_definition_callbacks;
  pcl::io::ply::ply_parser::at<pcl::io::ply::float64> (scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalarPropertyDefinitionCallback<pcl::io::ply::float64> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::float32> (scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalarPropertyDefinitionCallback<pcl::io::ply::float32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::int8> (scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalarPropertyDefinitionCallback<pcl::io::ply::int8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8> (scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalarPropertyDefinitionCallback<pcl::io::ply::uint8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::int32> (scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalarPropertyDefinitionCallback<pcl::io::ply::int32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32> (scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalarPropertyDefinitionCallback<pcl::io::ply::uint32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::int16> (scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalarPropertyDefinitionCallback<pcl::io::ply::int16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16> (scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalarPropertyDefinitionCallback<pcl::io::ply::uint16> (element_name, property_name); };
  ply_parser.scalar_property_definition_callbacks (scalar_property_definition_callbacks);

  pcl::io::ply::ply_parser::list_property_definition_callbacks_type list_property_definition_callbacks;
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::int32> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint8, pcl::io::ply::int32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::uint32> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint8, pcl::io::ply::int32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::float64> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::float64> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::float32> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::float32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::uint32> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::uint32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::int32> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::int32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::uint16> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::uint16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::int16> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::int16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::uint8> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::uint8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::int8> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::int8> (element_name, property_name); };
  ply_parser.list_property_definition_callbacks (list_property_definition_callbacks);

  return ply_parser.parse (istream_filename);
}

////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PLYReader::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                            int &, int &, unsigned int &, const int)
{
  // Silence compiler warnings
  cloud_ = &cloud;
  range_grid_ = new std::vector<std::vector<int> >;
  cloud_->width = cloud_->height = 0;
  origin = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  if (!parse (file_name))
  {
    PCL_ERROR ("[pcl::PLYReader::read] problem parsing header!\n");
    return (-1);
  }
  cloud_->row_step = cloud_->point_step * cloud_->width;
  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PLYReader::read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &ply_version, const int)
{
  // kept only for backward compatibility
  int data_type;
  unsigned int data_idx;

  if (!fs::exists (file_name))
  {
    PCL_ERROR ("[pcl::PLYReader::read] File (%s) not found!\n",file_name.c_str ());
    return (-1);
  }

  if (this->readHeader (file_name, cloud, origin, orientation, ply_version, data_type, data_idx))
  {
    PCL_ERROR ("[pcl::PLYReader::read] problem parsing header!\n");
    return (-1);
  }

  // a range_grid element was found ?
  std::size_t r_size;
  if ((r_size  = (*range_grid_).size ()) > 0 && r_size != vertex_count_)
  {
    //cloud.header = cloud_->header;
    std::vector<std::uint8_t> data ((*range_grid_).size () * cloud.point_step);
    const static float f_nan = std::numeric_limits <float>::quiet_NaN ();
    const static double d_nan = std::numeric_limits <double>::quiet_NaN ();
    for (std::size_t r = 0; r < r_size; ++r)
    {
      if ((*range_grid_)[r].empty ())
      {
        for (const auto &field : cloud_->fields)
          if (field.datatype == ::pcl::PCLPointField::FLOAT32)
          {
            const auto idx = r * cloud_->point_step + field.offset;
            if (idx + sizeof (float) > data.size())
            {
              PCL_ERROR ("[pcl::PLYReader::read] invalid data index (%lu)!\n", idx);
              return (-1);
            }
            memcpy (&data[idx],
                    reinterpret_cast<const char*> (&f_nan), sizeof (float));
          }
          else if (field.datatype == ::pcl::PCLPointField::FLOAT64)
          {
            const auto idx = r * cloud_->point_step + field.offset;
            if (idx + sizeof (double) > data.size())
            {
              PCL_ERROR ("[pcl::PLYReader::read] invalid data index (%lu)!\n", idx);
              return (-1);
            }
            memcpy (&data[idx],
                    reinterpret_cast<const char*> (&d_nan), sizeof (double));
          }
          else
          {
            const auto idx = r * cloud_->point_step + field.offset;
            if (idx + pcl::getFieldSize (field.datatype) * field.count > data.size())
            {
              PCL_ERROR ("[pcl::PLYReader::read] invalid data index (%lu)!\n", idx);
              return (-1);
            }
            std::fill_n(&data[idx],
                        pcl::getFieldSize (field.datatype) * field.count, 0);
          }
      }
      else
      {
        const auto srcIdx = (*range_grid_)[r][0] * cloud_->point_step;
        if (srcIdx + cloud_->point_step > cloud_->data.size())
        {
          PCL_ERROR ("[pcl::PLYReader::read] invalid data index (%lu)!\n", srcIdx);
          return (-1);
        }
        memcpy (&data[r* cloud_->point_step], &cloud_->data[srcIdx], cloud_->point_step);
      }
    }
    cloud_->data.swap (data);
  }

  orientation_ = Eigen::Quaternionf (orientation);
  origin_ = origin;

  for (auto &field : cloud_->fields)
  {
    if (field.name == "nx")
      field.name = "normal_x";
    if (field.name == "ny")
      field.name = "normal_y";
    if (field.name == "nz")
      field.name = "normal_z";
  }
  return (0);
}

////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PLYReader::read (const std::string &file_name, pcl::PolygonMesh &mesh,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                      int &ply_version, const int offset)
{
  // kept only for backward compatibility
  int data_type;
  unsigned int data_idx;
  polygons_ = &(mesh.polygons);

  if (!fs::exists (file_name))
  {
    PCL_ERROR ("[pcl::PLYReader::read] File (%s) not found!\n",file_name.c_str ());
    return (-1);
  }

  if (this->readHeader (file_name, mesh.cloud, origin, orientation, ply_version, data_type, data_idx, offset))
  {
    PCL_ERROR ("[pcl::PLYReader::read] problem parsing header!\n");
    return (-1);
  }

  // a range_grid element was found ?
  std::size_t r_size;
  if ((r_size  = (*range_grid_).size ()) > 0 && r_size != vertex_count_)
  {
    //cloud.header = cloud_->header;
    std::vector<std::uint8_t> data ((*range_grid_).size () * mesh.cloud.point_step);
    const static float f_nan = std::numeric_limits <float>::quiet_NaN ();
    const static double d_nan = std::numeric_limits <double>::quiet_NaN ();
    for (std::size_t r = 0; r < r_size; ++r)
    {
      if ((*range_grid_)[r].empty ())
      {
        for (const auto &field : cloud_->fields)
          if (field.datatype == ::pcl::PCLPointField::FLOAT32)
          {
            const auto idx = r * cloud_->point_step + field.offset;
            if (idx + sizeof (float) > data.size())
            {
              PCL_ERROR ("[pcl::PLYReader::read] invalid data index (%lu)!\n", idx);
              return (-1);
            }
            memcpy (&data[idx],
                    reinterpret_cast<const char*> (&f_nan), sizeof (float));
          }
          else if (field.datatype == ::pcl::PCLPointField::FLOAT64)
          {
            const auto idx = r * cloud_->point_step + field.offset;
            if (idx + sizeof (double) > data.size())
            {
              PCL_ERROR ("[pcl::PLYReader::read] invalid data index (%lu)!\n", idx);
              return (-1);
            }
            memcpy (&data[idx],
                    reinterpret_cast<const char*> (&d_nan), sizeof (double));
          }
          else
          {
            const auto idx = r * cloud_->point_step + field.offset;
            if (idx + pcl::getFieldSize (field.datatype) * field.count > data.size())
            {
              PCL_ERROR ("[pcl::PLYReader::read] invalid data index (%lu)!\n", idx);
              return (-1);
            }
            std::fill_n(&data[idx],
                        pcl::getFieldSize (field.datatype) * field.count, 0);
          }
      }
      else
      {
        const auto srcIdx = (*range_grid_)[r][0] * cloud_->point_step;
        if (srcIdx + cloud_->point_step > cloud_->data.size())
        {
          PCL_ERROR ("[pcl::PLYReader::read] invalid data index (%lu)!\n", srcIdx);
          return (-1);
        }
        memcpy (&data[r* cloud_->point_step], &cloud_->data[srcIdx], cloud_->point_step);
      }
    }
    cloud_->data.swap (data);
  }

  orientation_ = Eigen::Quaternionf (orientation);
  origin_ = origin;

  for (auto &field : cloud_->fields)
  {
    if (field.name == "nx")
      field.name = "normal_x";
    if (field.name == "ny")
      field.name = "normal_y";
    if (field.name == "nz")
      field.name = "normal_z";
  }
  return (0);
}

////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PLYReader::read (const std::string &file_name, pcl::PolygonMesh &mesh, const int offset)
{
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int ply_version;
  return read (file_name, mesh, origin, orientation, ply_version, offset);
}

////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::PLYWriter::generateHeader (const pcl::PCLPointCloud2 &cloud,
                                const Eigen::Vector4f &origin,
                                const Eigen::Quaternionf &,
                                bool binary,
                                bool use_camera,
                                int valid_points)
{
  std::ostringstream oss;
  // Begin header
  oss << "ply";
  if (!binary)
    oss << "\nformat ascii 1.0";
  else
  {
    if (cloud.is_bigendian)
      oss << "\nformat binary_big_endian 1.0";
    else
      oss << "\nformat binary_little_endian 1.0";
  }
  oss << "\ncomment PCL generated";

  if (!use_camera)
  {
    oss << "\nobj_info is_cyberware_data 0"
      "\nobj_info is_mesh 0"
      "\nobj_info is_warped 0"
      "\nobj_info is_interlaced 0";
    oss << "\nobj_info num_cols " << cloud.width;
    oss << "\nobj_info num_rows " << cloud.height;
    oss << "\nobj_info echo_rgb_offset_x " << origin[0];
    oss << "\nobj_info echo_rgb_offset_y " << origin[1];
    oss << "\nobj_info echo_rgb_offset_z " << origin[2];
    oss << "\nobj_info echo_rgb_frontfocus 0.0"
      "\nobj_info echo_rgb_backfocus 0.0"
      "\nobj_info echo_rgb_pixelsize 0.0"
      "\nobj_info echo_rgb_centerpixel 0"
      "\nobj_info echo_frames 1"
      "\nobj_info echo_lgincr 0.0";
  }

  oss << "\nelement vertex "<< valid_points;

  for (const auto &field : cloud.fields)
  {
    if (field.name == "normal_x")
    {
      oss << "\nproperty float nx";
    }
    else if (field.name == "normal_y")
    {
      oss << "\nproperty float ny";
    }
    else if (field.name == "normal_z")
    {
      oss << "\nproperty float nz";
    }
    else if (field.name == "rgb")
    {
      oss << "\nproperty uchar red"
        "\nproperty uchar green"
        "\nproperty uchar blue";
    }
    else if (field.name == "rgba")
    {
      oss << "\nproperty uchar red"
        "\nproperty uchar green"
        "\nproperty uchar blue"
        "\nproperty uchar alpha";
    }
    else
    {
      oss << "\nproperty";
      if (field.count != 1)
        oss << " list uint";
      switch (field.datatype)
      {
        case pcl::PCLPointField::INT8 : oss << " char "; break;
        case pcl::PCLPointField::UINT8 : oss << " uchar "; break;
        case pcl::PCLPointField::INT16 : oss << " short "; break;
        case pcl::PCLPointField::UINT16 : oss << " ushort "; break;
        case pcl::PCLPointField::INT32 : oss << " int "; break;
        case pcl::PCLPointField::UINT32 : oss << " uint "; break;
        case pcl::PCLPointField::FLOAT32 : oss << " float "; break;
        case pcl::PCLPointField::FLOAT64 : oss << " double "; break;
        default :
        {
          PCL_ERROR ("[pcl::PLYWriter::generateHeader] unknown data field type!\n");
          return ("");
        }
      }
      oss << field.name;
    }
  }

  // vtk requires face entry to load PLY
  oss << "\nelement face 0";

  if (use_camera)
  {
    oss << "\nelement camera 1"
      "\nproperty float view_px"
      "\nproperty float view_py"
      "\nproperty float view_pz"
      "\nproperty float x_axisx"
      "\nproperty float x_axisy"
      "\nproperty float x_axisz"
      "\nproperty float y_axisx"
      "\nproperty float y_axisy"
      "\nproperty float y_axisz"
      "\nproperty float z_axisx"
      "\nproperty float z_axisy"
      "\nproperty float z_axisz"
      "\nproperty float focal"
      "\nproperty float scalex"
      "\nproperty float scaley"
      "\nproperty float centerx"
      "\nproperty float centery"
      "\nproperty int viewportx"
      "\nproperty int viewporty"
      "\nproperty float k1"
      "\nproperty float k2";
  }
  else if (cloud.height > 1)
  {
    oss << "\nelement range_grid " << cloud.width * cloud.height;
    oss << "\nproperty list uchar int vertex_indices";
  }

  // End header
  oss << "\nend_header\n";
  return (oss.str ());
}

////////////////////////////////////////////////////////////////////////////////////////

int
pcl::PLYWriter::writeASCII (const std::string &file_name,
                            const pcl::PCLPointCloud2 &cloud,
                            const Eigen::Vector4f &origin,
                            const Eigen::Quaternionf &orientation,
                            int precision,
                            bool use_camera)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PLYWriter::writeASCII] Input point cloud has no data!\n");
    return (-1);
  }

  std::ofstream fs;
  fs.precision (precision);
  // Open file
  fs.open (file_name.c_str ());
  if (!fs)
  {
    PCL_ERROR ("[pcl::PLYWriter::writeASCII] Error during opening (%s)!\n", file_name.c_str ());
    return (-1);
  }

  unsigned int nr_points  = cloud.width * cloud.height;

  // Write the header information if available
  if (use_camera)
  {
    fs << generateHeader (cloud, origin, orientation, false, use_camera, nr_points);
    writeContentWithCameraASCII (nr_points, cloud, origin, orientation, fs);
  }
  else
  {
    std::ostringstream os;
    int nr_valid_points;
    writeContentWithRangeGridASCII (nr_points, cloud, os, nr_valid_points);
    fs << generateHeader (cloud, origin, orientation, false, use_camera, nr_valid_points);
    fs << os.str ();
  }

  // Close file
  fs.close ();
  return (0);
}

void
pcl::PLYWriter::writeContentWithCameraASCII (int nr_points,
                                             const pcl::PCLPointCloud2 &cloud,
                                             const Eigen::Vector4f &origin,
                                             const Eigen::Quaternionf &orientation,
                                             std::ofstream& fs)
{
  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    for (std::size_t d = 0; d < cloud.fields.size (); ++d)
    {
      int count = cloud.fields[d].count;
      if (count == 0)
        count = 1; //workaround

      if (count > 1)
        fs << count << " ";
      for (int c = 0; c < count; ++c)
      {
        switch (cloud.fields[d].datatype)
        {
          case pcl::PCLPointField::INT8:
          {
            fs << boost::numeric_cast<int> (cloud.at<char>(i, cloud.fields[d].offset + c * sizeof (char)));
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            fs << boost::numeric_cast<int> (cloud.at<unsigned char>(i, cloud.fields[d].offset + c * sizeof (unsigned char)));
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            fs << boost::numeric_cast<int> (cloud.at<short>(i, cloud.fields[d].offset + c * sizeof (short)));
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            fs << boost::numeric_cast<int> (cloud.at<unsigned short>(i, cloud.fields[d].offset + c * sizeof (unsigned short)));
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            fs << cloud.at<int>(i, cloud.fields[d].offset + c * sizeof (int));
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            if (cloud.fields[d].name.find ("rgba") == std::string::npos)
            {
              fs << cloud.at<unsigned int>(i, cloud.fields[d].offset + c * sizeof (unsigned int));
            }
            else
            {
              const auto& color = cloud.at<pcl::RGB>(i, cloud.fields[d].offset + c * sizeof (pcl::RGB));
              fs << static_cast<int>(color.r) << " " << static_cast<int>(color.g) << " " << static_cast<int>(color.b) << " " << static_cast<int>(color.a);
            }
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            if (cloud.fields[d].name.find ("rgb") == std::string::npos)
            {
              fs << cloud.at<float>(i, cloud.fields[d].offset + c * sizeof (float));
            }
            else
            {
              const auto& color = cloud.at<pcl::RGB>(i, cloud.fields[d].offset + c * sizeof (pcl::RGB));
              fs << static_cast<int>(color.r) << " " << static_cast<int>(color.g) << " " << static_cast<int>(color.b);
            }
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            fs << cloud.at<double>(i, cloud.fields[d].offset + c * sizeof (double));
            break;
          }
          default:
            PCL_WARN ("[pcl::PLYWriter::writeASCII] Incorrect field data type specified (%d)!\n", cloud.fields[d].datatype);
            break;
        }

        if (d < cloud.fields.size () - 1 || c < static_cast<int> (cloud.fields[d].count) - 1)
          fs << " ";
      }
    }
    fs << '\n';
  }
  // Append sensor information
  if (origin[3] != 0)
    fs << origin[0]/origin[3] << " " << origin[1]/origin[3] << " " << origin[2]/origin[3] << " ";
  else
    fs << origin[0] << " " << origin[1] << " " << origin[2] << " ";

  Eigen::Matrix3f R = orientation.toRotationMatrix ();
  fs << R (0,0) << " " << R (0,1) << " " << R (0,2) << " ";
  fs << R (1,0) << " " << R (1,1) << " " << R (1,2) << " ";
  fs << R (2,0) << " " << R (2,1) << " " << R (2,2) << " ";
  // No focal
  fs << 0 << " ";
  // No scale
  fs << 0 << " " << 0 << " ";
  // No center
  fs << 0 << " " << 0 << " ";
  // Viewport set to width x height
  fs << cloud.width << " " << cloud.height << " ";
  // No corrections
  fs << 0 << " " << 0;
  fs << std::endl;
  fs.flush ();
}

void
pcl::PLYWriter::writeContentWithRangeGridASCII (int nr_points,
                                                const pcl::PCLPointCloud2 &cloud,
                                                std::ostringstream& fs,
                                                int& valid_points)
{
  valid_points = 0;
  std::vector<std::vector <int> > grids (nr_points);
  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    std::ostringstream line;
    bool is_valid_line = true;
    for (std::size_t d = 0; d < cloud.fields.size (); ++d)
    {
      int count = cloud.fields[d].count;
      if (count == 0)
        count = 1; //workaround
      if (count > 1)
        fs << count << " ";
      for (int c = 0; c < count; ++c)
      {
        switch (cloud.fields[d].datatype)
        {
          case pcl::PCLPointField::INT8:
          {
            line << boost::numeric_cast<int> (cloud.at<char>(i, cloud.fields[d].offset + c * sizeof (char)));
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            line << boost::numeric_cast<int> (cloud.at<unsigned char>(i, cloud.fields[d].offset + c * sizeof (unsigned char)));
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            line << boost::numeric_cast<int> (cloud.at<short>(i, cloud.fields[d].offset + c * sizeof (short)));
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            line << boost::numeric_cast<int> (cloud.at<unsigned short>(i, cloud.fields[d].offset + c * sizeof (unsigned short)));
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            line << cloud.at<int>(i, cloud.fields[d].offset + c * sizeof (int));
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            if (cloud.fields[d].name.find ("rgba") == std::string::npos)
            {
              line << cloud.at<unsigned int>(i, cloud.fields[d].offset + c * sizeof (unsigned int));
            }
            else
            {
              const auto& color = cloud.at<pcl::RGB>(i, cloud.fields[d].offset + c * sizeof (pcl::RGB));
              line << static_cast<int>(color.r) << " " << static_cast<int>(color.g) << " " << static_cast<int>(color.b) << " " << static_cast<int>(color.a);
            }
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            if (cloud.fields[d].name.find ("rgb") == std::string::npos)
            {
              const float& value = cloud.at<float>(i, cloud.fields[d].offset + c * sizeof (float));
              // Test if x-coordinate is NaN, thus an invalid point
              if ("x" == cloud.fields[d].name)
              {
                if (!std::isfinite(value))
                  is_valid_line = false;
              }
              line << value;
            }
            else
            {
              const auto& color = cloud.at<pcl::RGB>(i, cloud.fields[d].offset + c * sizeof (pcl::RGB));
              line << static_cast<int>(color.r) << " " << static_cast<int>(color.g) << " " << static_cast<int>(color.b);
            }
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            line << cloud.at<double>(i, cloud.fields[d].offset + c * sizeof (double));
            break;
          }
          default:
            PCL_WARN ("[pcl::PLYWriter::writeASCII] Incorrect field data type specified (%d)!\n", cloud.fields[d].datatype);
            break;
        }

        if (d < cloud.fields.size () - 1 || c < static_cast<int> (cloud.fields[d].count) - 1)
          line << " ";
      }
    }

    if (is_valid_line)
    {
      grids[i].push_back (valid_points);
      fs << line.str () << '\n';
      ++valid_points;
    }
  }

  // If point cloud is organized, then append range grid
  if (cloud.height > 1)
  {
    for (int i = 0; i < nr_points; ++i)
    {
      fs << grids [i].size ();
      for (const auto& grid : grids [i]) {
        fs << " " << grid;
      }
      fs << '\n';
    }
  }

  fs.flush ();
}

////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PLYWriter::writeBinary (const std::string &file_name,
                             const pcl::PCLPointCloud2 &cloud,
                             const Eigen::Vector4f &origin,
                             const Eigen::Quaternionf &orientation,
                             bool use_camera)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PLYWriter::writeBinary] Input point cloud has no data!\n");
    return (-1);
  }

  std::ofstream fs;
  fs.open (file_name.c_str ());      // Open file
  if (!fs)
  {
    PCL_ERROR ("[pcl::PLYWriter::writeBinary] Error during opening (%s)!\n", file_name.c_str ());
    return (-1);
  }

  unsigned int nr_points  = cloud.width * cloud.height;

  // Compute the range_grid, if necessary, and then write out the PLY header
  bool doRangeGrid = !use_camera && cloud.height > 1;
  std::vector<pcl::io::ply::int32> rangegrid (nr_points);
  if (doRangeGrid)
  {
    unsigned int valid_points = 0;

    // Determine the field containing the x-coordinate
    int xfield = pcl::getFieldIndex (cloud, "x");
    if (xfield >= 0 && cloud.fields[xfield].datatype != pcl::PCLPointField::FLOAT32)
      xfield = -1;

    // If no x-coordinate field exists, then assume all points are valid
    if (xfield < 0)
    {
      for (unsigned int i=0; i < nr_points; ++i)
        rangegrid[i] = i;
      valid_points = nr_points;
    }
    // Otherwise, look at their x-coordinates to determine if points are valid
    else
    {
      for (std::size_t i=0; i < nr_points; ++i)
      {
        const float& value = cloud.at<float>(i, cloud.fields[xfield].offset);
        if (std::isfinite(value))
        {
          rangegrid[i] = valid_points;
          ++valid_points;
        }
        else
          rangegrid[i] = -1;
      }
    }
    fs << generateHeader (cloud, origin, orientation, true, use_camera, valid_points);
  }
  else
  {
    fs << generateHeader (cloud, origin, orientation, true, use_camera, nr_points);
  }

  // Close the file
  fs.close ();
  // Open file in binary appendable
  std::ofstream fpout (file_name.c_str (), std::ios::app | std::ios::binary);
  if (!fpout)
  {
    PCL_ERROR ("[pcl::PLYWriter::writeBinary] Error during reopening (%s)!\n", file_name.c_str ());
    return (-1);
  }

  // Iterate through the points
  for (unsigned int i = 0; i < nr_points; ++i)
  {
    // Skip writing any invalid points from range_grid
    if (doRangeGrid && rangegrid[i] < 0)
      continue;

    std::size_t total = 0;
    for (std::size_t d = 0; d < cloud.fields.size (); ++d)
    {
      int count = cloud.fields[d].count;
      if (count == 0)
        count = 1; //workaround
      if (count > 1)
      {
        static unsigned int ucount (count);
        fpout.write (reinterpret_cast<const char*> (&ucount), sizeof (unsigned int));
      }
      // Ignore invalid padded dimensions that are inherited from binary data
      if (cloud.fields[d].name == "_")
      {
        total += cloud.fields[d].count; // jump over this many elements in the string token
        continue;
      }

      for (int c = 0; c < count; ++c)
      {
        switch (cloud.fields[d].datatype)
        {
          case pcl::PCLPointField::INT8:
          {
            fpout.write (&cloud.at<char>(i, cloud.fields[d].offset + (total + c) * sizeof (char)), sizeof (char));
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            fpout.write (reinterpret_cast<const char*> (&cloud.at<unsigned char>(i, cloud.fields[d].offset + (total + c) * sizeof (unsigned char))), sizeof (unsigned char));
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            fpout.write (reinterpret_cast<const char*> (&cloud.at<short>(i, cloud.fields[d].offset + (total + c) * sizeof (short))), sizeof (short));
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            fpout.write (reinterpret_cast<const char*> (&cloud.at<unsigned short>(i, cloud.fields[d].offset + (total + c) * sizeof (unsigned short))), sizeof (unsigned short));
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            fpout.write (reinterpret_cast<const char*> (&cloud.at<int>(i, cloud.fields[d].offset + (total + c) * sizeof (int))), sizeof (int));
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            if (cloud.fields[d].name.find ("rgba") == std::string::npos)
            {
              fpout.write (reinterpret_cast<const char*> (&cloud.at<unsigned int>(i, cloud.fields[d].offset + (total + c) * sizeof (unsigned int))), sizeof (unsigned int));
            }
            else
            {
              const auto& color = cloud.at<pcl::RGB>(i, cloud.fields[d].offset + (total + c) * sizeof (pcl::RGB));
              fpout.write (reinterpret_cast<const char*> (&color.r), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&color.g), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&color.b), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&color.a), sizeof (unsigned char));
            }
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            if (cloud.fields[d].name.find ("rgb") == std::string::npos)
            {
              fpout.write (reinterpret_cast<const char*> (&cloud.at<float>(i, cloud.fields[d].offset + (total + c) * sizeof (float))), sizeof (float));
            }
            else
            {
              const auto& color = cloud.at<pcl::RGB>(i, cloud.fields[d].offset + (total + c) * sizeof (pcl::RGB));
              fpout.write (reinterpret_cast<const char*> (&color.r), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&color.g), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&color.b), sizeof (unsigned char));
            }
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            fpout.write (reinterpret_cast<const char*> (&cloud.at<double>(i, cloud.fields[d].offset + (total + c) * sizeof (double))), sizeof (double));
            break;
          }
          default:
            PCL_WARN ("[pcl::PLYWriter::writeBinary] Incorrect field data type specified (%d)!\n", cloud.fields[d].datatype);
            break;
        }
      }
    }
  }

  if (use_camera)
  {
    // Append sensor information
    float t;
    for (int i = 0; i < 3; ++i)
    {
      if (origin[3] != 0)
        t = origin[i]/origin[3];
      else
        t = origin[i];
      fpout.write (reinterpret_cast<const char*> (&t), sizeof (float));
    }
    Eigen::Matrix3f R = orientation.toRotationMatrix ();
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
    {
      fpout.write (reinterpret_cast<const char*> (&R (i, j)),sizeof (float));
    }

    /////////////////////////////////////////////////////
    // Append those properties directly.               //
    // They are for perspective cameras so just put 0  //
    //                                                 //
    // property float focal                            //
    // property float scalex                           //
    // property float scaley                           //
    // property float centerx                          //
    // property float centery                          //
    // and later on                                    //
    // property float k1                               //
    // property float k2                               //
    /////////////////////////////////////////////////////

    const float zerof = 0;
    for (int i = 0; i < 5; ++i)
      fpout.write (reinterpret_cast<const char*> (&zerof), sizeof (float));

    // width and height
    int width = cloud.width;
    fpout.write (reinterpret_cast<const char*> (&width), sizeof (int));

    int height = cloud.height;
    fpout.write (reinterpret_cast<const char*> (&height), sizeof (int));

    for (int i = 0; i < 2; ++i)
      fpout.write (reinterpret_cast<const char*> (&zerof), sizeof (float));
  }
  else if (doRangeGrid)
  {
    // Write out range_grid
    for (std::size_t i=0; i < nr_points; ++i)
    {
      pcl::io::ply::uint8 listlen;

      if (rangegrid[i] >= 0)
      {
        listlen = 1;
        fpout.write (reinterpret_cast<const char*> (&listlen), sizeof (pcl::io::ply::uint8));
        fpout.write (reinterpret_cast<const char*> (&rangegrid[i]), sizeof (pcl::io::ply::int32));
      }
      else
      {
        listlen = 0;
        fpout.write (reinterpret_cast<const char*> (&listlen), sizeof (pcl::io::ply::uint8));
      }
    }
  }

  // Close file
  fpout.close ();
  return (0);
}

namespace pcl {
namespace io {
void writePLYHeader (std::ostream& fs, const pcl::PolygonMesh& mesh, const std::string& format) {
  // Write header
  fs << "ply";
  fs << "\nformat " << format;
  fs << "\ncomment PCL generated";
  // Vertices
  fs << "\nelement vertex "<< mesh.cloud.width * mesh.cloud.height;
  for(const pcl::PCLPointField& field : mesh.cloud.fields) {
    if(field.name == "x")
      fs << "\nproperty float x";
    else if(field.name == "y")
      fs << "\nproperty float y";
    else if(field.name == "z")
      fs << "\nproperty float z";
    else if(field.name == "rgb")
      fs << "\nproperty uchar red"
            "\nproperty uchar green"
            "\nproperty uchar blue";
    else if(field.name == "rgba")
      fs << "\nproperty uchar red"
            "\nproperty uchar green"
            "\nproperty uchar blue"
            "\nproperty uchar alpha";
    else if(field.name == "normal_x")
      fs << "\nproperty float nx";
    else if(field.name == "normal_y")
      fs << "\nproperty float ny";
    else if(field.name == "normal_z")
      fs << "\nproperty float nz";
    else if(field.name == "curvature")
      fs << "\nproperty float curvature";
    else
      PCL_WARN("[pcl::io::writePLYHeader] unknown field: %s\n", field.name.c_str());
  }
  // Faces
  fs << "\nelement face "<< mesh.polygons.size ();
  fs << "\nproperty list uchar int vertex_indices";
  fs << "\nend_header\n";
}
} // namespace io
} // namespace pcl

////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::savePLYFile (const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision)
{
  if (mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::savePLYFile] Input point cloud has no data!\n");
    return (-1);
  }
  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());
  if (!fs)
  {
    PCL_ERROR ("[pcl::io::savePLYFile] Error during opening (%s)!\n", file_name.c_str ());
    return (-1);
  }

  // number of points
  std::size_t nr_points  = mesh.cloud.width * mesh.cloud.height;

  pcl::io::writePLYHeader (fs, mesh, "ascii 1.0");

  // Write down vertices
  for (std::size_t i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (std::size_t d = 0; d < mesh.cloud.fields.size (); ++d)
    {
      // adding vertex
      if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
          mesh.cloud.fields[d].name == "x" ||
          mesh.cloud.fields[d].name == "y" ||
          mesh.cloud.fields[d].name == "z"))
      {
        fs << mesh.cloud.at<float>(i, mesh.cloud.fields[d].offset) << " ";
        // if (++xyz == 3)
        //   break;
        ++xyz;
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) &&
                (mesh.cloud.fields[d].name == "rgb"))

      {
        const auto& color = mesh.cloud.at<RGB>(i, mesh.cloud.fields[d].offset);
        fs << int (color.r) << " " << int (color.g) << " " << int (color.b) << " ";
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::UINT32) &&
               (mesh.cloud.fields[d].name == "rgba"))
      {
        const auto& color = mesh.cloud.at<RGB>(i, mesh.cloud.fields[d].offset);
        fs << int (color.r) << " " << int (color.g) << " " << int (color.b) << " " << int (color.a) << " ";
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                mesh.cloud.fields[d].name == "normal_x" ||
                mesh.cloud.fields[d].name == "normal_y" ||
                mesh.cloud.fields[d].name == "normal_z"))
      {
        fs << mesh.cloud.at<float>(i, mesh.cloud.fields[d].offset) << " ";
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                mesh.cloud.fields[d].name == "curvature"))
      {
        fs << mesh.cloud.at<float>(i, mesh.cloud.fields[d].offset) << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::savePLYFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << '\n';
  }

  // Write down faces
  PCL_DEBUG ("[pcl::io::savePLYFile] Saving %zu polygons/faces\n", mesh.polygons.size());
  for (const pcl::Vertices& polygon : mesh.polygons)
  {
    fs << polygon.vertices.size ();
    for (const auto& vertex : polygon.vertices)
      fs << " " << vertex;
    fs << '\n';
  }

  // Close file
  fs.close ();
  return (0);
}

////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::savePLYFileBinary (const std::string &file_name, const pcl::PolygonMesh &mesh)
{
  if (mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::savePLYFile] Input point cloud has no data!\n");
    return (-1);
  }
  // Open file
  std::ofstream fs;
  fs.open (file_name.c_str ());
  if (!fs)
  {
    PCL_ERROR ("[pcl::io::savePLYFile] Error during opening (%s)!\n", file_name.c_str ());
    return (-1);
  }

  // number of points
  std::size_t nr_points  = mesh.cloud.width * mesh.cloud.height;

  pcl::io::writePLYHeader(fs, mesh, (mesh.cloud.is_bigendian ? "binary_big_endian 1.0" : "binary_little_endian 1.0"));

  // Close the file
  fs.close ();
  // Open file in binary appendable
  std::ofstream fpout (file_name.c_str (), std::ios::app | std::ios::binary);
  if (!fpout)
  {
    PCL_ERROR ("[pcl::io::writePLYFileBinary] Error during reopening (%s)!\n", file_name.c_str ());
    return (-1);
  }

  // Write down vertices
  for (std::size_t i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (std::size_t d = 0; d < mesh.cloud.fields.size (); ++d)
    {
      // adding vertex
      if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
          mesh.cloud.fields[d].name == "x" ||
          mesh.cloud.fields[d].name == "y" ||
          mesh.cloud.fields[d].name == "z"))
      {
        fpout.write (reinterpret_cast<const char*> (&mesh.cloud.at<float>(i, mesh.cloud.fields[d].offset)), sizeof (float));
        // if (++xyz == 3)
        //   break;
        ++xyz;
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) &&
                (mesh.cloud.fields[d].name == "rgb"))

      {
        const auto& color = mesh.cloud.at<RGB>(i, mesh.cloud.fields[d].offset);
        fpout.write (reinterpret_cast<const char*> (&color.r), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.g), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.b), sizeof (unsigned char));
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::UINT32) &&
               (mesh.cloud.fields[d].name == "rgba"))
      {
        const auto& color = mesh.cloud.at<RGB>(i, mesh.cloud.fields[d].offset);
        fpout.write (reinterpret_cast<const char*> (&color.r), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.g), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.b), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.a), sizeof (unsigned char));
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
               mesh.cloud.fields[d].name == "normal_x" ||
               mesh.cloud.fields[d].name == "normal_y" ||
               mesh.cloud.fields[d].name == "normal_z"))
      {
        fpout.write (reinterpret_cast<const char*> (&mesh.cloud.at<float>(i, mesh.cloud.fields[d].offset)), sizeof (float));
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && 
               (mesh.cloud.fields[d].name == "curvature"))
      {
        fpout.write (reinterpret_cast<const char*> (&mesh.cloud.at<float>(i, mesh.cloud.fields[d].offset)), sizeof (float));
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::savePLYFileBinary] Input point cloud has no XYZ data!\n");
      return (-2);
    }
  }

  // Write down faces
  for (const pcl::Vertices& polygon : mesh.polygons)
  {
    auto value = static_cast<unsigned char> (polygon.vertices.size ());
    fpout.write (reinterpret_cast<const char*> (&value), sizeof (unsigned char));
    for (const int value : polygon.vertices)
    {
      //fs << mesh.polygons[i].vertices[j] << " ";
      fpout.write (reinterpret_cast<const char*> (&value), sizeof (int));
    }
  }

  // Close file
  fpout.close ();
  return (0);
}

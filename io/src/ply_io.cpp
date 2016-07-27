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

#include <fstream>
#include <fcntl.h>
#include <string>
#include <map>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/boost.h>
#include <sstream>

boost::tuple<boost::function<void ()>, boost::function<void ()> >
pcl::PLYReader::elementDefinitionCallback (const std::string& element_name, std::size_t count)
{
  if (element_name == "vertex")
  {
    cloud_->data.clear ();
    cloud_->fields.clear ();
    // Cloud dimensions may have already been set from obj_info fields
    if (cloud_->width == 0 || cloud_->height == 0)
    {
      cloud_->width = static_cast<uint32_t> (count);
      cloud_->height = 1;
    }
    cloud_->is_dense = false;
    cloud_->point_step = 0;
    cloud_->row_step = 0;
    vertex_count_ = 0;
    return (boost::tuple<boost::function<void ()>, boost::function<void ()> > (
              boost::bind (&pcl::PLYReader::vertexBeginCallback, this),
              boost::bind (&pcl::PLYReader::vertexEndCallback, this)));
  }
  else if ((element_name == "face") && polygons_)
  {
    polygons_->reserve (count);
    return (boost::tuple<boost::function<void ()>, boost::function<void ()> > (
            boost::bind (&pcl::PLYReader::faceBeginCallback, this),
            boost::bind (&pcl::PLYReader::faceEndCallback, this)));
  }
  else if (element_name == "camera")
  {
    cloud_->is_dense = true;
    return (boost::tuple<boost::function<void ()>, boost::function<void ()> > (0, 0));
  }
  else if (element_name == "range_grid")
  {
    range_grid_->reserve (count);
    return (boost::tuple<boost::function<void ()>, boost::function<void ()> > (
              boost::bind (&pcl::PLYReader::rangeGridBeginCallback, this),
              boost::bind (&pcl::PLYReader::rangeGridEndCallback, this)));
  }
  else
  {
    return (boost::tuple<boost::function<void ()>, boost::function<void ()> > (0, 0));
  }
}

bool
pcl::PLYReader::endHeaderCallback ()
{
  cloud_->data.resize (cloud_->point_step * cloud_->width * cloud_->height);
  return (cloud_->data.size () == cloud_->point_step * cloud_->width * cloud_->height);
}

template<typename Scalar> void
pcl::PLYReader::appendScalarProperty (const std::string& name, const size_t& size)
{
  cloud_->fields.push_back (::pcl::PCLPointField ());
  ::pcl::PCLPointField &current_field = cloud_->fields.back ();
  current_field.name = name;
  current_field.offset = cloud_->point_step;
  current_field.datatype = pcl::traits::asEnum<Scalar>::value;
  current_field.count = static_cast<uint32_t> (size);
  cloud_->point_step += static_cast<uint32_t> (pcl::getFieldSize (pcl::traits::asEnum<Scalar>::value) * size);
}

void
pcl::PLYReader::amendProperty (const std::string& old_name, const std::string& new_name, uint8_t new_datatype)
{
  std::vector< ::pcl::PCLPointField>::reverse_iterator finder = cloud_->fields.rbegin ();
  for (; finder != cloud_->fields.rend (); ++finder)
    if (finder->name == old_name)
      break;
  assert (finder != cloud_->fields.rend ());
  finder->name = new_name;
  if (new_datatype > 0 && new_datatype != finder->datatype)
    finder->datatype = new_datatype;
}

namespace pcl
{
  template <>
  boost::function<void (pcl::io::ply::float32)>
  PLYReader::scalarPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if (element_name == "vertex")
    {
      appendScalarProperty<pcl::io::ply::float32> (property_name, 1);
      return (boost::bind (&pcl::PLYReader::vertexScalarPropertyCallback<pcl::io::ply::float32>, this, _1));
    }
    else if (element_name == "camera")
    {
      if (property_name == "view_px")
      {
        return boost::bind (&pcl::PLYReader::originXCallback, this, _1);
      }
      else if (property_name == "view_py")
      {
        return boost::bind (&pcl::PLYReader::originYCallback, this, _1);
      }
      else if (property_name == "view_pz")
      {
        return boost::bind (&pcl::PLYReader::originZCallback, this, _1);
      }
      else if (property_name == "x_axisx")
      {
        return boost::bind (&pcl::PLYReader::orientationXaxisXCallback, this, _1);
      }
      else if (property_name == "x_axisy")
      {
        return boost::bind (&pcl::PLYReader::orientationXaxisYCallback, this, _1);
      }
      else if (property_name == "x_axisz")
      {
        return boost::bind (&pcl::PLYReader::orientationXaxisZCallback, this, _1);
      }
      else if (property_name == "y_axisx")
      {
        return boost::bind (&pcl::PLYReader::orientationYaxisXCallback, this, _1);
      }
      else if (property_name == "y_axisy")
      {
        return boost::bind (&pcl::PLYReader::orientationYaxisYCallback, this, _1);
      }
      else if (property_name == "y_axisz")
      {
        return boost::bind (&pcl::PLYReader::orientationYaxisZCallback, this, _1);
      }
      else if (property_name == "z_axisx")
      {
        return boost::bind (&pcl::PLYReader::orientationZaxisXCallback, this, _1);
      }
      else if (property_name == "z_axisy")
      {
        return boost::bind (&pcl::PLYReader::orientationZaxisYCallback, this, _1);
      }
      else if (property_name == "z_axisz")
      {
        return boost::bind (&pcl::PLYReader::orientationZaxisZCallback, this, _1);
      }
      else
      {
        return (0);
      }
    }
    else
    {
      return (0);
    }
  }

  template <> boost::function<void (pcl::io::ply::uint8)>
  PLYReader::scalarPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if (element_name == "vertex")
    {
      if ((property_name == "red") || (property_name == "green") || (property_name == "blue") ||
          (property_name == "diffuse_red") || (property_name == "diffuse_green") || (property_name == "diffuse_blue"))
      {
        if ((property_name == "red") || (property_name == "diffuse_red"))
          appendScalarProperty<pcl::io::ply::float32> ("rgb");
        return boost::bind (&pcl::PLYReader::vertexColorCallback, this, property_name, _1);
      }
      else if (property_name == "alpha")
      {
        amendProperty ("rgb", "rgba", pcl::PCLPointField::UINT32);
        return boost::bind (&pcl::PLYReader::vertexAlphaCallback, this, _1);
      }
      else if (property_name == "intensity")
      {
        appendScalarProperty<pcl::io::ply::float32> (property_name);
        return boost::bind (&pcl::PLYReader::vertexIntensityCallback, this, _1);
      }
      else
      {
        appendScalarProperty<pcl::io::ply::uint8> (property_name);
        return boost::bind (&pcl::PLYReader::vertexScalarPropertyCallback<pcl::io::ply::uint8>, this, _1);
      }
    }
    else
      return (0);
  }

  template <> boost::function<void (pcl::io::ply::int32)>
  PLYReader::scalarPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if (element_name == "vertex")
    {
      appendScalarProperty<pcl::io::ply::int32> (property_name, 1);
      return (boost::bind (&pcl::PLYReader::vertexScalarPropertyCallback<pcl::io::ply::int32>, this, _1));
    }
    if (element_name == "camera")
    {
      if (property_name == "viewportx")
      {
        return boost::bind (&pcl::PLYReader::cloudWidthCallback, this, _1);
      }
      else if (property_name == "viewporty")
      {
        return boost::bind (&pcl::PLYReader::cloudHeightCallback, this, _1);
      }
      else
        return (0);
    }
    else
      return (0);
  }

  template <typename Scalar> boost::function<void (Scalar)>
  PLYReader::scalarPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if (element_name == "vertex")
    {
      appendScalarProperty<Scalar> (property_name, 1);
      return (boost::bind (&pcl::PLYReader::vertexScalarPropertyCallback<Scalar>, this, _1));
    }
    return (0);
  }

  template<typename Scalar> void
  PLYReader::vertexScalarPropertyCallback (Scalar value)
  {
    memcpy (&cloud_->data[vertex_count_ * cloud_->point_step + vertex_offset_before_],
            &value,
            sizeof (Scalar));
    vertex_offset_before_ += static_cast<int> (sizeof (Scalar));
  }

  template <typename SizeType> void
  PLYReader::vertexListPropertyBeginCallback (const std::string& name, SizeType size)
  {
    // Adjust size only once
    if (vertex_count_ == 0)
    {
      std::vector< pcl::PCLPointField>::reverse_iterator finder = cloud_->fields.rbegin ();
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
    memcpy (&cloud_->data[vertex_count_ * cloud_->point_step + vertex_offset_before_],
            &value,
            sizeof (ContentType));
    vertex_offset_before_ += static_cast<int> (sizeof (ContentType));
  }

  template <>
  boost::tuple<boost::function<void (pcl::io::ply::uint8)>, boost::function<void (pcl::io::ply::int32)>, boost::function<void ()> >
  pcl::PLYReader::listPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if ((element_name == "range_grid") && (property_name == "vertex_indices"))
    {
      return boost::tuple<boost::function<void (pcl::io::ply::uint8)>, boost::function<void (pcl::io::ply::int32)>, boost::function<void ()> > (
        boost::bind (&pcl::PLYReader::rangeGridVertexIndicesBeginCallback, this, _1),
        boost::bind (&pcl::PLYReader::rangeGridVertexIndicesElementCallback, this, _1),
        boost::bind (&pcl::PLYReader::rangeGridVertexIndicesEndCallback, this)
      );
    }
    else if ((element_name == "face") && (property_name == "vertex_indices") && polygons_)
    {
      return boost::tuple<boost::function<void (pcl::io::ply::uint8)>, boost::function<void (pcl::io::ply::int32)>, boost::function<void ()> > (
        boost::bind (&pcl::PLYReader::faceVertexIndicesBeginCallback, this, _1),
        boost::bind (&pcl::PLYReader::faceVertexIndicesElementCallback, this, _1),
        boost::bind (&pcl::PLYReader::faceVertexIndicesEndCallback, this)
      );
    }
    else if (element_name == "vertex")
    {
      cloud_->fields.push_back (pcl::PCLPointField ());
      pcl::PCLPointField &current_field = cloud_->fields.back ();
      current_field.name = property_name;
      current_field.offset = cloud_->point_step;
      current_field.datatype = pcl::traits::asEnum<pcl::io::ply::int32>::value;
      current_field.count = std::numeric_limits<pcl::io::ply::uint8>::max ();
      if (current_field.count * sizeof (pcl::io::ply::int32) + cloud_->point_step < std::numeric_limits<uint32_t>::max ())
          cloud_->point_step += static_cast<uint32_t> (current_field.count * sizeof (pcl::io::ply::int32));
      else
        cloud_->point_step = static_cast<uint32_t> (std::numeric_limits<uint32_t>::max ());
      do_resize_ = true;
      return boost::tuple<boost::function<void (pcl::io::ply::uint8)>, boost::function<void (pcl::io::ply::int32)>, boost::function<void ()> > (
                                                                                                      boost::bind (&pcl::PLYReader::vertexListPropertyBeginCallback<pcl::io::ply::uint8>, this, property_name, _1),
        boost::bind (&pcl::PLYReader::vertexListPropertyContentCallback<pcl::io::ply::int32>, this, _1),
        boost::bind (&pcl::PLYReader::vertexListPropertyEndCallback, this)
      );
    }
    else
    {
      return boost::tuple<boost::function<void (pcl::io::ply::uint8)>, boost::function<void (pcl::io::ply::int32)>, boost::function<void ()> > (0, 0, 0);
    }
  }

  template <typename SizeType, typename ContentType>
  boost::tuple<boost::function<void (SizeType)>, boost::function<void (ContentType)>, boost::function<void ()> >
  pcl::PLYReader::listPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name)
  {
    if (element_name == "vertex")
    {
      cloud_->fields.push_back (pcl::PCLPointField ());
      pcl::PCLPointField &current_field = cloud_->fields.back ();
      current_field.name = property_name;
      current_field.offset = cloud_->point_step;
      current_field.datatype = pcl::traits::asEnum<ContentType>::value;
      current_field.count = std::numeric_limits<SizeType>::max ();
      if (current_field.count * sizeof (ContentType) + cloud_->point_step < std::numeric_limits<uint32_t>::max ())
        cloud_->point_step += static_cast<uint32_t> (current_field.count * sizeof (ContentType));
      else
        cloud_->point_step = static_cast<uint32_t> (std::numeric_limits<uint32_t>::max ());
      do_resize_ = true;
      return boost::tuple<boost::function<void (SizeType)>, boost::function<void (ContentType)>, boost::function<void ()> > (
        boost::bind (&pcl::PLYReader::vertexListPropertyBeginCallback<SizeType>, this, property_name, _1),
        boost::bind (&pcl::PLYReader::vertexListPropertyContentCallback<ContentType>, this, _1),
        boost::bind (&pcl::PLYReader::vertexListPropertyEndCallback, this)
      );
    }
    else
    {
      return boost::tuple<boost::function<void (SizeType)>, boost::function<void (ContentType)>, boost::function<void ()> > (0, 0, 0);
    }
  }
}

void
pcl::PLYReader::vertexColorCallback (const std::string& color_name, pcl::io::ply::uint8 color)
{
  if ((color_name == "red") || (color_name == "diffuse_red"))
  {
    r_ = int32_t (color);
    rgb_offset_before_ = vertex_offset_before_;
  }
  if ((color_name == "green") || (color_name == "diffuse_green"))
  {
    g_ = int32_t (color);
  }
  if ((color_name == "blue") || (color_name == "diffuse_blue"))
  {
    b_ = int32_t (color);
    int32_t rgb = r_ << 16 | g_ << 8 | b_;
    memcpy (&cloud_->data[vertex_count_ * cloud_->point_step + rgb_offset_before_],
            &rgb,
            sizeof (pcl::io::ply::float32));
    vertex_offset_before_ += static_cast<int> (sizeof (pcl::io::ply::float32));
  }
}

void
pcl::PLYReader::vertexAlphaCallback (pcl::io::ply::uint8 alpha)
{
  a_ = uint32_t (alpha);
  // get anscient rgb value and store it in rgba
  memcpy (&rgba_, 
          &cloud_->data[vertex_count_ * cloud_->point_step + rgb_offset_before_], 
          sizeof (pcl::io::ply::float32));
  // append alpha
  rgba_ = rgba_ | a_ << 24;
  // put rgba back
  memcpy (&cloud_->data[vertex_count_ * cloud_->point_step + rgb_offset_before_], 
          &rgba_, 
          sizeof (uint32_t));
}

void
pcl::PLYReader::vertexIntensityCallback (pcl::io::ply::uint8 intensity)
{
  pcl::io::ply::float32 intensity_ (intensity);
  memcpy (&cloud_->data[vertex_count_ * cloud_->point_step + vertex_offset_before_],
          &intensity_,
          sizeof (pcl::io::ply::float32));
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
    cloud_->data.resize (cloud_->point_step * cloud_->width * cloud_->height);
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
    assert (st.size () == 3);
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
  pcl::io::ply::ply_parser::flags_type ply_parser_flags = 0;
  pcl::io::ply::ply_parser ply_parser (ply_parser_flags);

  ply_parser.info_callback (boost::bind (&pcl::PLYReader::infoCallback, this, boost::ref (istream_filename), _1, _2));
  ply_parser.warning_callback (boost::bind (&pcl::PLYReader::warningCallback, this, boost::ref (istream_filename), _1, _2));
  ply_parser.error_callback (boost::bind (&pcl::PLYReader::errorCallback, this, boost::ref (istream_filename), _1, _2));

  ply_parser.obj_info_callback (boost::bind (&pcl::PLYReader::objInfoCallback, this, _1));
  ply_parser.element_definition_callback (boost::bind (&pcl::PLYReader::elementDefinitionCallback, this, _1, _2));
  ply_parser.end_header_callback (boost::bind (&pcl::PLYReader::endHeaderCallback, this));

  pcl::io::ply::ply_parser::scalar_property_definition_callbacks_type scalar_property_definition_callbacks;
  pcl::io::ply::ply_parser::at<pcl::io::ply::float64> (scalar_property_definition_callbacks) = boost::bind (&pcl::PLYReader::scalarPropertyDefinitionCallback<pcl::io::ply::float64>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::float32> (scalar_property_definition_callbacks) = boost::bind (&pcl::PLYReader::scalarPropertyDefinitionCallback<pcl::io::ply::float32>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::int8> (scalar_property_definition_callbacks) = boost::bind (&pcl::PLYReader::scalarPropertyDefinitionCallback<pcl::io::ply::int8>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8> (scalar_property_definition_callbacks) = boost::bind (&pcl::PLYReader::scalarPropertyDefinitionCallback<pcl::io::ply::uint8>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::int32> (scalar_property_definition_callbacks) = boost::bind (&pcl::PLYReader::scalarPropertyDefinitionCallback<pcl::io::ply::int32>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32> (scalar_property_definition_callbacks) = boost::bind (&pcl::PLYReader::scalarPropertyDefinitionCallback<pcl::io::ply::uint32>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::int16> (scalar_property_definition_callbacks) = boost::bind (&pcl::PLYReader::scalarPropertyDefinitionCallback<pcl::io::ply::int16>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16> (scalar_property_definition_callbacks) = boost::bind (&pcl::PLYReader::scalarPropertyDefinitionCallback<pcl::io::ply::uint16>, this, _1, _2);
  ply_parser.scalar_property_definition_callbacks (scalar_property_definition_callbacks);

  pcl::io::ply::ply_parser::list_property_definition_callbacks_type list_property_definition_callbacks;
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::int32> (list_property_definition_callbacks) = boost::bind (&pcl::PLYReader::listPropertyDefinitionCallback<pcl::io::ply::uint8, pcl::io::ply::int32>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::float64> (list_property_definition_callbacks) = boost::bind (&pcl::PLYReader::listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::float64>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::float32> (list_property_definition_callbacks) = boost::bind (&pcl::PLYReader::listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::float32>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::uint32> (list_property_definition_callbacks) = boost::bind (&pcl::PLYReader::listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::uint32>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::int32> (list_property_definition_callbacks) = boost::bind (&pcl::PLYReader::listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::int32>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::uint16> (list_property_definition_callbacks) = boost::bind (&pcl::PLYReader::listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::uint16>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::int16> (list_property_definition_callbacks) = boost::bind (&pcl::PLYReader::listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::int16>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::uint8> (list_property_definition_callbacks) = boost::bind (&pcl::PLYReader::listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::uint8>, this, _1, _2);
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::int8> (list_property_definition_callbacks) = boost::bind (&pcl::PLYReader::listPropertyDefinitionCallback<pcl::io::ply::uint32, pcl::io::ply::int8>, this, _1, _2);
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

  if (this->readHeader (file_name, cloud, origin, orientation, ply_version, data_type, data_idx))
  {
    PCL_ERROR ("[pcl::PLYReader::read] problem parsing header!\n");
    return (-1);
  }

  // a range_grid element was found ?
  size_t r_size;
  if ((r_size  = (*range_grid_).size ()) > 0 && r_size != vertex_count_)
  {
    //cloud.header = cloud_->header;
    std::vector<pcl::uint8_t> data ((*range_grid_).size () * cloud.point_step);
    const static float f_nan = std::numeric_limits <float>::quiet_NaN ();
    const static double d_nan = std::numeric_limits <double>::quiet_NaN ();
    for (size_t r = 0; r < r_size; ++r)
    {
      if ((*range_grid_)[r].size () == 0)
      {
        for (size_t f = 0; f < cloud_->fields.size (); ++f)
          if (cloud_->fields[f].datatype == ::pcl::PCLPointField::FLOAT32)
            memcpy (&data[r * cloud_->point_step + cloud_->fields[f].offset],
                    reinterpret_cast<const char*> (&f_nan), sizeof (float));
          else if (cloud_->fields[f].datatype == ::pcl::PCLPointField::FLOAT64)
            memcpy (&data[r * cloud_->point_step + cloud_->fields[f].offset],
                    reinterpret_cast<const char*> (&d_nan), sizeof (double));
          else
            memset (&data[r * cloud_->point_step + cloud_->fields[f].offset], 0,
                    pcl::getFieldSize (cloud_->fields[f].datatype) * cloud_->fields[f].count);
      }
      else
        memcpy (&data[r* cloud_->point_step], &cloud_->data[(*range_grid_)[r][0] * cloud_->point_step], cloud_->point_step);
    }
    cloud_->data.swap (data);
  }

  orientation_ = Eigen::Quaternionf (orientation);
  origin_ = origin;

  for (size_t i = 0; i < cloud_->fields.size (); ++i)
  {
    if (cloud_->fields[i].name == "nx")
      cloud_->fields[i].name = "normal_x";
    if (cloud_->fields[i].name == "ny")
      cloud_->fields[i].name = "normal_y";
    if (cloud_->fields[i].name == "nz")
      cloud_->fields[i].name = "normal_z";
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
  if (this->readHeader (file_name, mesh.cloud, origin, orientation, ply_version, data_type, data_idx, offset))
  {
    PCL_ERROR ("[pcl::PLYReader::read] problem parsing header!\n");
    return (-1);
  }

  // a range_grid element was found ?
  size_t r_size;
  if ((r_size  = (*range_grid_).size ()) > 0 && r_size != vertex_count_)
  {
    //cloud.header = cloud_->header;
    std::vector<pcl::uint8_t> data ((*range_grid_).size () * mesh.cloud.point_step);
    const static float f_nan = std::numeric_limits <float>::quiet_NaN ();
    const static double d_nan = std::numeric_limits <double>::quiet_NaN ();
    for (size_t r = 0; r < r_size; ++r)
    {
      if ((*range_grid_)[r].size () == 0)
      {
        for (size_t f = 0; f < cloud_->fields.size (); ++f)
          if (cloud_->fields[f].datatype == ::pcl::PCLPointField::FLOAT32)
            memcpy (&data[r * cloud_->point_step + cloud_->fields[f].offset],
                    reinterpret_cast<const char*> (&f_nan), sizeof (float));
          else if (cloud_->fields[f].datatype == ::pcl::PCLPointField::FLOAT64)
            memcpy (&data[r * cloud_->point_step + cloud_->fields[f].offset],
                    reinterpret_cast<const char*> (&d_nan), sizeof (double));
          else
            memset (&data[r * cloud_->point_step + cloud_->fields[f].offset], 0,
                    pcl::getFieldSize (cloud_->fields[f].datatype) * cloud_->fields[f].count);
      }
      else
        memcpy (&data[r* cloud_->point_step], &cloud_->data[(*range_grid_)[r][0] * cloud_->point_step], cloud_->point_step);
    }
    cloud_->data.swap (data);
  }

  orientation_ = Eigen::Quaternionf (orientation);
  origin_ = origin;

  for (size_t i = 0; i < cloud_->fields.size (); ++i)
  {
    if (cloud_->fields[i].name == "nx")
      cloud_->fields[i].name = "normal_x";
    if (cloud_->fields[i].name == "ny")
      cloud_->fields[i].name = "normal_y";
    if (cloud_->fields[i].name == "nz")
      cloud_->fields[i].name = "normal_z";
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

  for (std::size_t i = 0; i < cloud.fields.size (); ++i)
  {
    if (cloud.fields[i].name == "normal_x")
    {
      oss << "\nproperty float nx";
    }
    else if (cloud.fields[i].name == "normal_y")
    {
      oss << "\nproperty float ny";
    }
    else if (cloud.fields[i].name == "normal_z")
    {
      oss << "\nproperty float nz";
    }
    else if (cloud.fields[i].name == "rgb")
    {
      oss << "\nproperty uchar red"
        "\nproperty uchar green"
        "\nproperty uchar blue";
    }
    else if (cloud.fields[i].name == "rgba")
    {
      oss << "\nproperty uchar red"
        "\nproperty uchar green"
        "\nproperty uchar blue"
        "\nproperty uchar alpha";
    }
    else
    {
      oss << "\nproperty";
      if (cloud.fields[i].count != 1)
        oss << " list uint";
      switch (cloud.fields[i].datatype)
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
          PCL_ERROR ("[pcl::PLYWriter::generateHeader] unknown data field type!");
          return ("");
        }
      }
      oss << cloud.fields[i].name;
    }
  }

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
  unsigned int point_size = static_cast<unsigned int> (cloud.data.size () / nr_points);

  // Write the header information if available
  if (use_camera)
  {
    fs << generateHeader (cloud, origin, orientation, false, use_camera, nr_points);
    writeContentWithCameraASCII (nr_points, point_size, cloud, origin, orientation, fs);
  }
  else
  {
    std::ostringstream os;
    int nr_valid_points;
    writeContentWithRangeGridASCII (nr_points, point_size, cloud, os, nr_valid_points);
    fs << generateHeader (cloud, origin, orientation, false, use_camera, nr_valid_points);
    fs << os.str ();
  }

  // Close file
  fs.close ();
  return (0);
}

void
pcl::PLYWriter::writeContentWithCameraASCII (int nr_points,
                                             int point_size,
                                             const pcl::PCLPointCloud2 &cloud,
                                             const Eigen::Vector4f &origin,
                                             const Eigen::Quaternionf &orientation,
                                             std::ofstream& fs)
{
  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    for (size_t d = 0; d < cloud.fields.size (); ++d)
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
            char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (char)], sizeof (char));
            fs << boost::numeric_cast<int> (value);
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            unsigned char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned char)], sizeof (unsigned char));
            fs << boost::numeric_cast<int> (value);
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (short)], sizeof (short));
            fs << boost::numeric_cast<int> (value);
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            unsigned short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned short)], sizeof (unsigned short));
            fs << boost::numeric_cast<int> (value);
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            int value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (int)], sizeof (int));
            fs << value;
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            if (cloud.fields[d].name.find ("rgba") == std::string::npos)
            {
              unsigned int value;
              memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned int)], sizeof (unsigned int));
              fs << value;
            }
            else
            {
              pcl::RGB color;
              memcpy (&color, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned int)], sizeof (pcl::RGB));
              int r = color.r;
              int g = color.g;
              int b = color.b;
              int a = color.a;
              fs << r << " " << g << " " << b << " " << a;
            }
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            if (cloud.fields[d].name.find ("rgb") == std::string::npos)
            {
              float value;
              memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
              fs << value;
            }
            else
            {
              pcl::RGB color;
              memcpy (&color, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (float)], sizeof (pcl::RGB));
              int r = color.r;
              int g = color.g;
              int b = color.b;
              fs << r << " " << g << " " << b;
            }
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            double value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (double)], sizeof (double));
            fs << value;
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
                                                int point_size,
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
    for (size_t d = 0; d < cloud.fields.size (); ++d)
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
            char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (char)], sizeof (char));
            line << boost::numeric_cast<int> (value);
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            unsigned char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned char)], sizeof (unsigned char));
            line << boost::numeric_cast<int> (value);
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (short)], sizeof (short));
            line << boost::numeric_cast<int> (value);
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            unsigned short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned short)], sizeof (unsigned short));
            line << boost::numeric_cast<int> (value);
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            int value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (int)], sizeof (int));
            line << value;
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            if (cloud.fields[d].name.find ("rgba") == std::string::npos)
            {
              unsigned int value;
              memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned int)], sizeof (unsigned int));
              line << value;
            }
            else
            {
              pcl::RGB color;
              memcpy (&color, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned int)], sizeof (pcl::RGB));
              int r = color.r;
              int g = color.g;
              int b = color.b;
              int a = color.a;
              line << r << " " << g << " " << b << " " << a;
            }
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            if (cloud.fields[d].name.find ("rgb") == std::string::npos)
            {
              float value;
              memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
              // Test if x-coordinate is NaN, thus an invalid point
              if ("x" == cloud.fields[d].name)
              {
                if (!pcl_isfinite(value))
                  is_valid_line = false;
              }
              line << value;
            }
            else
            {
              pcl::RGB color;
              memcpy (&color, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (float)], sizeof (pcl::RGB));
              int r = color.r;
              int g = color.g;
              int b = color.b;
              line << r << " " << g << " " << b;
            }
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            double value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (double)], sizeof (double));
            line << value;
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
      for (std::vector <int>::const_iterator it = grids [i].begin ();
           it != grids [i].end ();
           ++it)
        fs << " " << *it;
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
  unsigned int point_size = static_cast<unsigned int> (cloud.data.size () / nr_points);

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
      for (size_t i=0; i < nr_points; ++i)
      {
        float value;
        memcpy(&value, &cloud.data[i * point_size + cloud.fields[xfield].offset], sizeof(float));
        if (pcl_isfinite(value))
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

    size_t total = 0;
    for (size_t d = 0; d < cloud.fields.size (); ++d)
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
            char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (char)], sizeof (char));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (char));
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            unsigned char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (unsigned char)], sizeof (unsigned char));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (unsigned char));
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (short)], sizeof (short));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (short));
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            unsigned short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (unsigned short)], sizeof (unsigned short));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (unsigned short));
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            int value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (int)], sizeof (int));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (int));
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            if (cloud.fields[d].name.find ("rgba") == std::string::npos)
            {
              unsigned int value;
              memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (unsigned int)], sizeof (unsigned int));
              fpout.write (reinterpret_cast<const char*> (&value), sizeof (unsigned int));
            }
            else
            {
              pcl::RGB color;
              memcpy (&color, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (unsigned int)], sizeof (pcl::RGB));
              unsigned char r = color.r;
              unsigned char g = color.g;
              unsigned char b = color.b;
              unsigned char a = color.a;
              fpout.write (reinterpret_cast<const char*> (&r), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&g), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&b), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&a), sizeof (unsigned char));
            }
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            if (cloud.fields[d].name.find ("rgb") == std::string::npos)
            {
              float value;
              memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (float)], sizeof (float));
              fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));
            }
            else
            {
              pcl::RGB color;
              memcpy (&color, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (float)], sizeof (pcl::RGB));
              unsigned char r = color.r;
              unsigned char g = color.g;
              unsigned char b = color.b;
              fpout.write (reinterpret_cast<const char*> (&r), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&g), sizeof (unsigned char));
              fpout.write (reinterpret_cast<const char*> (&b), sizeof (unsigned char));
            }
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            double value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + (total + c) * sizeof (double)], sizeof (double));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (double));
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
    for (size_t i=0; i < nr_points; ++i)
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
  size_t nr_points  = mesh.cloud.width * mesh.cloud.height;
  size_t point_size = mesh.cloud.data.size () / nr_points;

  // number of faces
  size_t nr_faces = mesh.polygons.size ();

  // Write header
  fs << "ply";
  fs << "\nformat ascii 1.0";
  fs << "\ncomment PCL generated";
  // Vertices
  fs << "\nelement vertex "<< mesh.cloud.width * mesh.cloud.height;
  fs << "\nproperty float x"
        "\nproperty float y"
        "\nproperty float z";
  // Check if we have color on vertices
  int rgba_index = getFieldIndex (mesh.cloud, "rgba"),
  rgb_index = getFieldIndex (mesh.cloud, "rgb");
  if (rgba_index != -1)
  {
    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue"
          "\nproperty uchar alpha";
  }
  else if (rgb_index != -1)
  {
    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue";
  }
  // Check if we have normal on vertices
  int normal_x_index = getFieldIndex(mesh.cloud, "normal_x");
  int normal_y_index = getFieldIndex(mesh.cloud, "normal_y");
  int normal_z_index = getFieldIndex(mesh.cloud, "normal_z");
  if (normal_x_index != -1 && normal_y_index != -1 && normal_z_index != -1)
  {
      fs << "\nproperty float nx"
            "\nproperty float ny"
            "\nproperty float nz";
  }
  // Check if we have curvature on vertices
  int curvature_index = getFieldIndex(mesh.cloud, "curvature");
  if ( curvature_index != -1)
  {
      fs << "\nproperty float curvature";
  }
  // Faces
  fs << "\nelement face "<< nr_faces;
  fs << "\nproperty list uchar int vertex_indices";
  fs << "\nend_header\n";

  // Write down vertices
  for (size_t i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
    {
      int count = mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;

      // adding vertex
      if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
          mesh.cloud.fields[d].name == "x" ||
          mesh.cloud.fields[d].name == "y" ||
          mesh.cloud.fields[d].name == "z"))
      {
        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        // if (++xyz == 3)
        //   break;
        ++xyz;
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) &&
                (mesh.cloud.fields[d].name == "rgb"))

      {
        pcl::RGB color;
        memcpy (&color, &mesh.cloud.data[i * point_size + mesh.cloud.fields[rgb_index].offset + c * sizeof (float)], sizeof (RGB));
        fs << int (color.r) << " " << int (color.g) << " " << int (color.b);
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::UINT32) &&
               (mesh.cloud.fields[d].name == "rgba"))
      {
        pcl::RGB color;
        memcpy (&color, &mesh.cloud.data[i * point_size + mesh.cloud.fields[rgba_index].offset + c * sizeof (uint32_t)], sizeof (RGB));
        fs << int (color.r) << " " << int (color.g) << " " << int (color.b) << " " << int (color.a);
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                mesh.cloud.fields[d].name == "normal_x" ||
                mesh.cloud.fields[d].name == "normal_y" ||
                mesh.cloud.fields[d].name == "normal_z"))
      {
        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof(float)], sizeof(float));
        fs << value;
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                mesh.cloud.fields[d].name == "curvature"))
      {
        float value;
        memcpy(&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof(float)], sizeof(float));
        fs << value;
      }
      fs << " ";
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::savePLYFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << '\n';
  }

  // Write down faces
  for (size_t i = 0; i < nr_faces; i++)
  {
    fs << mesh.polygons[i].vertices.size () << " ";
    size_t j = 0;
    for (j = 0; j < mesh.polygons[i].vertices.size () - 1; ++j)
      fs << mesh.polygons[i].vertices[j] << " ";
    fs << mesh.polygons[i].vertices[j] << '\n';
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
  size_t nr_points  = mesh.cloud.width * mesh.cloud.height;
  size_t point_size = mesh.cloud.data.size () / nr_points;

  // number of faces
  size_t nr_faces = mesh.polygons.size ();

  // Write header
  fs << "ply";
  fs << "\nformat " << (mesh.cloud.is_bigendian ? "binary_big_endian" : "binary_little_endian") << " 1.0";
  fs << "\ncomment PCL generated";
  // Vertices
  fs << "\nelement vertex "<< mesh.cloud.width * mesh.cloud.height;
  fs << "\nproperty float x"
        "\nproperty float y"
        "\nproperty float z";
  // Check if we have color on vertices
  int rgba_index = getFieldIndex (mesh.cloud, "rgba"),
  rgb_index = getFieldIndex (mesh.cloud, "rgb");
  if (rgba_index != -1)
  {
    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue"
          "\nproperty uchar alpha";
  }
  else if (rgb_index != -1)
  {
    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue";
  }
  // Check if we have normal on vertices
  int normal_x_index = getFieldIndex(mesh.cloud, "normal_x");
  int normal_y_index = getFieldIndex(mesh.cloud, "normal_y");
  int normal_z_index = getFieldIndex(mesh.cloud, "normal_z");
  if (normal_x_index != -1 && normal_y_index != -1 && normal_z_index != -1)
  {
	  fs << "\nproperty float nx"
		  "\nproperty float ny"
		  "\nproperty float nz";
  }
  // Check if we have curvature on vertices
  int curvature_index = getFieldIndex(mesh.cloud, "curvature");
  if ( curvature_index != -1)
  {
	  fs << "\nproperty float curvature";
  }
  // Faces
  fs << "\nelement face "<< nr_faces;
  fs << "\nproperty list uchar int vertex_indices";
  fs << "\nend_header\n";

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
  for (size_t i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
    {
      int count = mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;

      // adding vertex
      if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
          mesh.cloud.fields[d].name == "x" ||
          mesh.cloud.fields[d].name == "y" ||
          mesh.cloud.fields[d].name == "z"))
      {
        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));
        // if (++xyz == 3)
        //   break;
        ++xyz;
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) &&
                (mesh.cloud.fields[d].name == "rgb"))

      {
        pcl::RGB color;
        memcpy (&color, &mesh.cloud.data[i * point_size + mesh.cloud.fields[rgb_index].offset + c * sizeof (float)], sizeof (RGB));
        fpout.write (reinterpret_cast<const char*> (&color.r), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.g), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.b), sizeof (unsigned char));
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::UINT32) &&
               (mesh.cloud.fields[d].name == "rgba"))
      {
        pcl::RGB color;
        memcpy (&color, &mesh.cloud.data[i * point_size + mesh.cloud.fields[rgba_index].offset + c * sizeof (uint32_t)], sizeof (RGB));
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
        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && 
               (mesh.cloud.fields[d].name == "curvature"))
      {
        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));        
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::savePLYFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
  }

  // Write down faces
  for (size_t i = 0; i < nr_faces; i++)
  {
    unsigned char value = static_cast<unsigned char> (mesh.polygons[i].vertices.size ());
    fpout.write (reinterpret_cast<const char*> (&value), sizeof (unsigned char));
    size_t j = 0;
    for (j = 0; j < mesh.polygons[i].vertices.size (); ++j)
    {
      //fs << mesh.polygons[i].vertices[j] << " ";
      int value = mesh.polygons[i].vertices[j];
      fpout.write (reinterpret_cast<const char*> (&value), sizeof (int));
    }
  }

  // Close file
  fs.close ();
  return (0);
}

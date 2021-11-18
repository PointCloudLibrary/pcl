/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2007-2012, Ares Lagae
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/io/ply/ply_parser.h>

#include <fstream> // for ifstream
#include <sstream> // for istringstream

bool pcl::io::ply::ply_parser::parse (const std::string& filename)
{
  std::ifstream istream (filename.c_str (), std::ios::in | std::ios::binary);

  std::string line;
  line_number_ = 0;

  std::size_t number_of_format_statements = 0;
  std::size_t number_of_element_statements = 0;
  std::size_t number_of_property_statements = 0;
  std::size_t number_of_obj_info_statements = 0;
  std::size_t number_of_comment_statements = 0;

  format_type format = pcl::io::ply::unknown;
  std::vector<std::shared_ptr<element>> elements;

  char line_delim = '\n';
  int char_ignore_count = 0;

  // magic
  char magic[4];
  istream.read (magic, 4);

  // Check if CR/LF, setup delim and char skip
  if (magic[3] == '\r')
  {
    istream.ignore (1);
    line_delim = '\r';
    char_ignore_count = 1;
  }

  ++line_number_;
  if (!istream)
  {
    error_callback_ (line_number_, "parse error: couldn't read the magic string");
    return false;
  }

  if ((magic[0] != 'p') || (magic[1] != 'l') || (magic[2] != 'y'))
  {
    error_callback_ (line_number_, "parse error: wrong magic string");
    return false;
  }

  magic_callback_ ();

  // parse header
  while (std::getline (istream, line, line_delim))
  {
    istream.ignore (char_ignore_count);
    ++line_number_;
    std::istringstream stringstream (line);
    stringstream.unsetf (std::ios_base::skipws);

    stringstream >> std::ws;
    if (stringstream.eof ())
    {
      warning_callback_ (line_number_, "ignoring line '" + line + "'");
      continue;
    }

    std::string keyword;
    stringstream >> keyword;

    // format
    if (keyword == "format")
    {
      std::string format_string, version;
      char space_format_format_string, space_format_string_version;
      stringstream >> space_format_format_string >> std::ws >> format_string >> space_format_string_version >> std::ws >> version;
      if (!stringstream.eof ())
      {
        stringstream >> std::ws;
        warning_callback_ (line_number_, "parse warning: trailing whitespaces in the header");
      }
      if (!stringstream ||
          !stringstream.eof () ||
          !isspace (space_format_format_string) ||
          !isspace (space_format_string_version))
      {
        error_callback_ (line_number_, "parse error: invalid format statement");
        return false;
      }
      if (format_string == "ascii")
      {
        format = ascii_format;
      }
      else if (format_string == "binary_big_endian")
      {
        format = binary_big_endian_format;
      }
      else if (format_string == "binary_little_endian")
      {
        format = binary_little_endian_format;
      }
      else
      {
        error_callback_ (line_number_, "parse error: unknown format");
        return false;
      }
      if (version != "1.0")
      {
        error_callback_ (line_number_, "version '" + version + "' is not supported");
        return false;
      }
      if (number_of_format_statements > 0)
      {
        error_callback_ (line_number_, "parse error: more than 1 format statement");
        return false;
      }
      ++number_of_format_statements;
      format_callback_ (format, version);
    }
    // element
    else if (keyword == "element")
    {
      std::string name;
      std::size_t count;
      char space_element_name, space_name_count;
      stringstream >> space_element_name >> std::ws >> name >> space_name_count >> std::ws >> count;
      if (!stringstream.eof ())
      {
        stringstream >> std::ws;
        warning_callback_ (line_number_, "parse warning: trailing whitespaces in the header");
      }
      if (!stringstream ||
          !stringstream.eof () ||
          !isspace (space_element_name) ||
          !isspace (space_name_count))
      {
        error_callback_ (line_number_, "parse error: invalid element statement");
        return false;
      }
      const auto iterator = std::find_if (elements.cbegin (), elements.cend (),
            [&name](const auto& ptr) { return ptr->name == name;});
      if (iterator != elements.cend ())
      {
        error_callback_ (line_number_, "parse error: invalid elements");
        return false;
      }
      ++number_of_element_statements;
      const element_callbacks_type element_callbacks =
          element_definition_callbacks_ (name, count);
      elements.emplace_back(new element (name, count, std::get<0>(element_callbacks), std::get<1>(element_callbacks)));
      current_element_ = elements.back ().get ();
    }

    // property
    else if (keyword == "property")
    {
      if (number_of_element_statements == 0)
      {
        error_callback_ (line_number_, "parse error: property specified without any element declaration");
        return false;
      }
      std::string type_or_list;
      char space_property_type_or_list;
      stringstream >> space_property_type_or_list >> std::ws >> type_or_list;
      if (!stringstream || !isspace (space_property_type_or_list))
      {
        error_callback_ (line_number_, "parse error: invalid property statement");
        return false;
      }
      if (type_or_list != "list") {
        std::string name;
        std::string& type = type_or_list;
        char space_type_name;
        stringstream >> space_type_name >> std::ws >> name;
        if (!stringstream || !isspace (space_type_name))
        {
          error_callback_ (line_number_, "parse error: invalid variable property statement");
          return false;
        }
        const auto iterator =
            std::find_if (current_element_->properties.cbegin (),
                          current_element_->properties.cend (),
                          [&name](const auto& ptr) { return ptr->name == name;});
        if (iterator != current_element_->properties.cend ())
        {
          error_callback_ (line_number_, "parse error: duplicate property found");
          return false;
        }
        if ((type == type_traits<int8>::name ()) || (type == type_traits<int8>::old_name ()))
        {
          parse_scalar_property_definition<int8>(name);
        }
        else if ((type == type_traits<int16>::name ()) || (type == type_traits<int16>::old_name ()))
        {
          parse_scalar_property_definition<int16>(name);
        }
        else if ((type == type_traits<int32>::name ()) || (type == type_traits<int32>::old_name ()))
        {
          parse_scalar_property_definition<int32>(name);
        }
        else if ((type == type_traits<uint8>::name ()) || (type == type_traits<uint8>::old_name ()))
        {
          parse_scalar_property_definition<uint8>(name);
        }
        else if ((type == type_traits<uint16>::name ()) || (type == type_traits<uint16>::old_name ()))
        {
          parse_scalar_property_definition<uint16>(name);
        }
        else if ((type == type_traits<uint32>::name ()) || (type == type_traits<uint32>::old_name ()))
        {
          parse_scalar_property_definition<uint32>(name);
        }
        else if ((type == type_traits<float32>::name ()) || (type == type_traits<float32>::old_name ()))
        {
          parse_scalar_property_definition<float32>(name);
        }
        else if ((type == type_traits<float64>::name ()) || (type == type_traits<float64>::old_name ()))
        {
          parse_scalar_property_definition<float64>(name);
        }
        else
        {
          error_callback_ (line_number_, "parse error: unknown type");
          return false;
        }
        ++number_of_property_statements;
      }
      else
      {
        std::string name;
        std::string size_type_string, scalar_type_string;
        char space_list_size_type, space_size_type_scalar_type, space_scalar_type_name;
        stringstream >> space_list_size_type >> std::ws >> size_type_string >> space_size_type_scalar_type >> std::ws >> scalar_type_string >> space_scalar_type_name >> std::ws >> name;
        if (!stringstream ||
            !isspace (space_list_size_type) ||
            !isspace (space_size_type_scalar_type) ||
            !isspace (space_scalar_type_name))
        {
          error_callback_ (line_number_, "parse error: invalid list statement");
          return false;
        }
        const auto iterator =
            std::find_if (current_element_->properties.cbegin (),
                          current_element_->properties.cend (),
                          [&name](const auto& ptr) { return ptr->name == name;});
        if (iterator != current_element_->properties.cend ())
        {
          error_callback_ (line_number_, "parse error: duplicate property found");
          return false;
        }
        if ((size_type_string == type_traits<uint8>::name ()) || (size_type_string == type_traits<uint8>::old_name ()))
        {
          using size_type = uint8;
          if ((scalar_type_string == type_traits<int8>::name ()) || (scalar_type_string == type_traits<int8>::old_name ()))
          {
            parse_list_property_definition<size_type, int8>(name);
          }
          else if ((scalar_type_string == type_traits<int16>::name ()) || (scalar_type_string == type_traits<int16>::old_name ()))
          {
            parse_list_property_definition<size_type, int16>(name);
          }
          else if ((scalar_type_string == type_traits<int32>::name ()) || (scalar_type_string == type_traits<int32>::old_name ()))
          {
            parse_list_property_definition<size_type, int32>(name);
          }
          else if ((scalar_type_string == type_traits<uint8>::name ()) || (scalar_type_string == type_traits<uint8>::old_name ()))
          {
            parse_list_property_definition<size_type, uint8>(name);
          }
          else if ((scalar_type_string == type_traits<uint16>::name ()) || (scalar_type_string == type_traits<uint16>::old_name ()))
          {
            parse_list_property_definition<size_type, uint16>(name);
          }
          else if ((scalar_type_string == type_traits<uint32>::name ()) || (scalar_type_string == type_traits<uint32>::old_name ()))
          {
            parse_list_property_definition<size_type, uint32>(name);
          }
          else if ((scalar_type_string == type_traits<float32>::name ()) || (scalar_type_string == type_traits<float32>::old_name ()))
          {
            parse_list_property_definition<size_type, float32>(name);
          }
          else if ((scalar_type_string == type_traits<float64>::name ()) || (scalar_type_string == type_traits<float64>::old_name ()))
          {
            parse_list_property_definition<size_type, float64>(name);
          }
          else
          {
            error_callback_ (line_number_, "parse error: unknown scalar type");
            return false;
          }
        }
        else if ((size_type_string == type_traits<uint16>::name ()) || (size_type_string == type_traits<uint16>::old_name ()))
        {
          using size_type = uint16;
          if ((scalar_type_string == type_traits<int8>::name ()) || (scalar_type_string == type_traits<int8>::old_name ()))
          {
            parse_list_property_definition<size_type, int8>(name);
          }
          else if ((scalar_type_string == type_traits<int16>::name ()) || (scalar_type_string == type_traits<int16>::old_name ()))
          {
            parse_list_property_definition<size_type, int16>(name);
          }
          else if ((scalar_type_string == type_traits<int32>::name ()) || (scalar_type_string == type_traits<int32>::old_name ()))
          {
            parse_list_property_definition<size_type, int32>(name);
          }
          else if ((scalar_type_string == type_traits<uint8>::name ()) || (scalar_type_string == type_traits<uint8>::old_name ()))
          {
            parse_list_property_definition<size_type, uint8>(name);
          }
          else if ((scalar_type_string == type_traits<uint16>::name ()) || (scalar_type_string == type_traits<uint16>::old_name ()))
          {
            parse_list_property_definition<size_type, uint16>(name);
          }
          else if ((scalar_type_string == type_traits<uint32>::name ()) || (scalar_type_string == type_traits<uint32>::old_name ()))
          {
            parse_list_property_definition<size_type, uint32>(name);
          }
          else if ((scalar_type_string == type_traits<float32>::name ()) || (scalar_type_string == type_traits<float32>::old_name ()))
          {
            parse_list_property_definition<size_type, float32>(name);
          }
          else if ((scalar_type_string == type_traits<float64>::name ()) || (scalar_type_string == type_traits<float64>::old_name ()))
          {
            parse_list_property_definition<size_type, float64>(name);
          }
          else
          {
            error_callback_ (line_number_, "parse error: unknown scalar type");
            return false;
          }
        }
        // It is safe to use size_type = uint32 here even if it is actually int32, because the size/number of list entries is never negative,
        // uint32 and int32 have the same width, and all allowed (non-negative) values have the same binary encoding in int32 and uint32.
        else if ((size_type_string == type_traits<uint32>::name ()) || (size_type_string == type_traits<uint32>::old_name ()) ||
                 (size_type_string == type_traits< int32>::name ()) || (size_type_string == type_traits< int32>::old_name ()))
        {
          using size_type = uint32;
          if ((scalar_type_string == type_traits<int8>::name ()) || (scalar_type_string == type_traits<int8>::old_name ()))
          {
            parse_list_property_definition<size_type, int8>(name);
          }
          else if ((scalar_type_string == type_traits<int16>::name ()) || (scalar_type_string == type_traits<int16>::old_name ()))
          {
            parse_list_property_definition<size_type, int16>(name);
          }
          else if ((scalar_type_string == type_traits<int32>::name ()) || (scalar_type_string == type_traits<int32>::old_name ()))
          {
            parse_list_property_definition<size_type, int32>(name);
          }
          else if ((scalar_type_string == type_traits<uint8>::name ()) || (scalar_type_string == type_traits<uint8>::old_name ()))
          {
            parse_list_property_definition<size_type, uint8>(name);
          }
          else if ((scalar_type_string == type_traits<uint16>::name ()) || (scalar_type_string == type_traits<uint16>::old_name ()))
          {
            parse_list_property_definition<size_type, uint16>(name);
          }
          else if ((scalar_type_string == type_traits<uint32>::name ()) || (scalar_type_string == type_traits<uint32>::old_name ()))
          {
            parse_list_property_definition<size_type, uint32>(name);
          }
          else if ((scalar_type_string == type_traits<float32>::name ()) || (scalar_type_string == type_traits<float32>::old_name ()))
          {
            parse_list_property_definition<size_type, float32>(name);
          }
          else if ((scalar_type_string == type_traits<float64>::name ()) || (scalar_type_string == type_traits<float64>::old_name ()))
          {
            parse_list_property_definition<size_type, float64>(name);
          }
          else
          {
            error_callback_ (line_number_, "parse error: unknown scalar type");
            return false;
          }
        }
        else
        {
          error_callback_ (line_number_, "parse error: unknown list size type");
          return false;
        }
        ++number_of_property_statements;
      }
    }

    // comment
    else if (keyword == "comment")
    {
      comment_callback_ (line);
      ++number_of_comment_statements;
    }

    // obj_info
    else if (keyword == "obj_info")
    {
      obj_info_callback_ (line);
      ++number_of_obj_info_statements;
    }

    // end_header
    else if (keyword == "end_header")
    {
      if (!end_header_callback_ ())
          return true;
      break;
    }
    // unknown keyword
    else
    {
      warning_callback_ (line_number_, "ignoring line '" + line + "'");
    }
  }

  if (number_of_format_statements == 0)
  {
    error_callback_ (line_number_, "parse error: a format statement is required");
    return false;
  }

  // ascii
  if (format == ascii_format)
  {
    for (const auto &element_ptr: elements)
    {
      auto& element = *(element_ptr.get ());
      for (std::size_t element_index = 0; element_index < element.count; ++element_index)
      {
        if (element.begin_element_callback)
          element.begin_element_callback ();
        if (!std::getline (istream, line, line_delim))
        {
          error_callback_ (line_number_, "parse error: found less elements than declared in the header");
          return false;
        }
        istream.ignore (char_ignore_count);
        ++line_number_;
        std::istringstream stringstream (line);
        stringstream.unsetf (std::ios_base::skipws);
        stringstream >> std::ws;

        for (const auto &property_ptr: element.properties)
        {
          auto& property = *(property_ptr.get ());
          if (!property.parse (*this, format, stringstream))
          {
            error_callback_ (line_number_, "parse error: element property count doesn't match the declaration in the header");
            return false;
          }
        }
        if (!stringstream.eof ())
        {
          error_callback_ (line_number_, "parse error: element contains more properties than declared");
          return false;
        }
        if (element.end_element_callback)
          element.end_element_callback ();
      }
    }
    istream >> std::ws;
    if (istream.fail ())
    {
      warning_callback_ (line_number_, "no newline at the end of file");
    }
    if (!istream.eof ())
    {
      warning_callback_ (line_number_, "ignoring extra data at the end of ascii stream");
    }
    return true;
  }

  // binary
  std::streampos data_start = istream.tellg ();
  istream.close ();
  istream.open (filename.c_str (), std::ios::in | std::ios::binary);
  istream.seekg (data_start);

  for (const auto &element_ptr: elements)
  {
    auto& element = *(element_ptr.get ());
    for (std::size_t element_index = 0; element_index < element.count; ++element_index)
    {
      if (element.begin_element_callback)
        element.begin_element_callback ();
      for (const auto &property_ptr: element.properties)
      {
        auto& property = *(property_ptr.get ());
        if (!property.parse (*this, format, istream))
        {
          return false;
        }
      }
      if (element.end_element_callback)
        element.end_element_callback ();
    }
  }
  if (istream.fail () || istream.bad ())
  {
    error_callback_ (line_number_, "parse error: failed to read from the binary stream");
    return false;
  }
  if (istream.rdbuf ()->sgetc () != std::char_traits<char>::eof ())
  {
    warning_callback_ (line_number_, "ignoring extra data at the end of binary stream");
  }
  return true;
}

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2007-2012, Ares Lagae
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <tuple>

/** \class ply_to_ply_converter
  * Converts a PLY file with format FORMAT_IN to PLY file with format FORMAT_OUT.
  * Format may be one of the following: ascii, binary, binary_big_endian, binary_little_endian.
  * If no format is given, the format of input is kept.
  *
  * \author Ares Lagae
  * \ingroup io
  */
class ply_to_ply_converter
{
  public:
    using format_type = int;
    enum format
    {
      same_format,
      ascii_format,
      binary_format,
      binary_big_endian_format,
      binary_little_endian_format
    };
  
    ply_to_ply_converter(format_type format) : 
      format_(format), input_format_(), output_format_(), 
      bol_ (), ostream_ () {}

    bool 
    convert (const std::string &filename, std::istream& istream, std::ostream& ostream);

  private:
    void
    info_callback(const std::string& filename, std::size_t line_number, const std::string& message);
    
    void
    warning_callback(const std::string& filename, std::size_t line_number, const std::string& message);
    
    void
    error_callback(const std::string& filename, std::size_t line_number, const std::string& message);
    
    void
    magic_callback();
    
    void
    format_callback(pcl::io::ply::format_type format, const std::string& version);
    
    void
    element_begin_callback();
    
    void
    element_end_callback();

    std::tuple<std::function<void()>, std::function<void()> > 
    element_definition_callback(const std::string& element_name, std::size_t count);

    template <typename ScalarType> void
    scalar_property_callback(ScalarType scalar);

    template <typename ScalarType> std::function<void (ScalarType)> 
    scalar_property_definition_callback(const std::string& element_name, const std::string& property_name);

    template <typename SizeType, typename ScalarType> void
    list_property_begin_callback(SizeType size);

    template <typename SizeType, typename ScalarType> void
    list_property_element_callback(ScalarType scalar);

    template <typename SizeType, typename ScalarType> void
    list_property_end_callback();

    template <typename SizeType, typename ScalarType> std::tuple<std::function<void (SizeType)>, 
                                                                      std::function<void (ScalarType)>, 
                                                                      std::function<void ()> > 
    list_property_definition_callback(const std::string& element_name, const std::string& property_name);
    
    void
    comment_callback(const std::string& comment);
    
    void
    obj_info_callback(const std::string& obj_info);

    bool 
    end_header_callback();

    format_type format_;
    pcl::io::ply::format_type input_format_, output_format_;
    bool bol_;
    std::ostream* ostream_;
};

void
ply_to_ply_converter::info_callback(const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ": " << line_number << ": " << "info: " << message << std::endl;
}

void
ply_to_ply_converter::warning_callback(const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ": " << line_number << ": " << "warning: " << message << std::endl;
}

void
ply_to_ply_converter::error_callback(const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ": " << line_number << ": " << "error: " << message << std::endl;
}

void
ply_to_ply_converter::magic_callback()
{
  (*ostream_) << "ply" << "\n";
}

void
ply_to_ply_converter::format_callback(pcl::io::ply::format_type format, const std::string& version)
{
  input_format_ = format;

  switch (format_) {
    case same_format:
      output_format_ = input_format_;
      break;
    case ascii_format:
      output_format_ = pcl::io::ply::ascii_format;
      break;
    case binary_format:
      output_format_ = pcl::io::ply::host_byte_order == pcl::io::ply::little_endian_byte_order ? pcl::io::ply::binary_little_endian_format : pcl::io::ply::binary_big_endian_format;
      break;
    case binary_big_endian_format:
      output_format_ = pcl::io::ply::binary_big_endian_format;
      break;
    case binary_little_endian_format:
      output_format_ = pcl::io::ply::binary_little_endian_format;
      break;
  };

  (*ostream_) << "format ";
  switch (output_format_) {
    case pcl::io::ply::ascii_format:
      (*ostream_) << "ascii";
      break;
    case pcl::io::ply::binary_little_endian_format:
      (*ostream_) << "binary_little_endian";
      break;
    case pcl::io::ply::binary_big_endian_format:
      (*ostream_) << "binary_big_endian";
      break;
  }
  (*ostream_) << " " << version << "\n";
}

void
ply_to_ply_converter::element_begin_callback()
{
  if (output_format_ == pcl::io::ply::ascii_format) {
    bol_ = true;
  }
}

void
ply_to_ply_converter::element_end_callback()
{
  if (output_format_ == pcl::io::ply::ascii_format) {
    (*ostream_) << "\n";
  }
}

std::tuple<std::function<void()>, std::function<void()> > ply_to_ply_converter::element_definition_callback(const std::string& element_name, std::size_t count)
{
  (*ostream_) << "element " << element_name << " " << count << "\n";
  return std::tuple<std::function<void()>, std::function<void()> >(
    [this] { element_begin_callback (); },
    [this] { element_end_callback (); }
  );
}

template <typename ScalarType>
void
ply_to_ply_converter::scalar_property_callback(ScalarType scalar)
{
  if (output_format_ == pcl::io::ply::ascii_format) {
    using namespace pcl::io::ply::io_operators;
    if (bol_) {
      bol_ = false;
      (*ostream_) << scalar;
    }
    else {
      (*ostream_) << " " << scalar;
    }
  }
  else {
    if (((pcl::io::ply::host_byte_order == pcl::io::ply::little_endian_byte_order) && (output_format_ == pcl::io::ply::binary_big_endian_format))
      || ((pcl::io::ply::host_byte_order == pcl::io::ply::big_endian_byte_order) && (output_format_ == pcl::io::ply::binary_little_endian_format))) {
      pcl::io::ply::swap_byte_order(scalar);
    }
    ostream_->write(reinterpret_cast<char*>(&scalar), sizeof(scalar));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename ScalarType> std::function<void (ScalarType)> 
ply_to_ply_converter::scalar_property_definition_callback (const std::string&, const std::string& property_name)
{
  (*ostream_) << "property " << pcl::io::ply::type_traits<ScalarType>::old_name() << " " << property_name << "\n";
  return [this] (ScalarType scalar) { scalar_property_callback<ScalarType> (scalar); };
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename SizeType, typename ScalarType> void
ply_to_ply_converter::list_property_begin_callback (SizeType size)
{
  if (output_format_ == pcl::io::ply::ascii_format) 
  {
    using namespace pcl::io::ply::io_operators;
    if (bol_) 
    {
      bol_ = false;
      (*ostream_) << size;
    }
    else 
      (*ostream_) << " " << size;
  }
  else 
  {
    if (((pcl::io::ply::host_byte_order == pcl::io::ply::little_endian_byte_order) && (output_format_ == pcl::io::ply::binary_big_endian_format))
      || ((pcl::io::ply::host_byte_order == pcl::io::ply::big_endian_byte_order) && (output_format_ == pcl::io::ply::binary_little_endian_format))) {
      pcl::io::ply::swap_byte_order(size);
    }
    ostream_->write(reinterpret_cast<char*>(&size), sizeof(size));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename SizeType, typename ScalarType> void
ply_to_ply_converter::list_property_element_callback (ScalarType scalar)
{
  if (output_format_ == pcl::io::ply::ascii_format) 
  {
    using namespace pcl::io::ply::io_operators;
    (*ostream_) << " " << scalar;
  }
  else 
  {
    if (((pcl::io::ply::host_byte_order == pcl::io::ply::little_endian_byte_order) && (output_format_ == pcl::io::ply::binary_big_endian_format)) || 
        ((pcl::io::ply::host_byte_order == pcl::io::ply::big_endian_byte_order) && (output_format_ == pcl::io::ply::binary_little_endian_format)))
      pcl::io::ply::swap_byte_order(scalar);

    ostream_->write(reinterpret_cast<char*>(&scalar), sizeof(scalar));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename SizeType, typename ScalarType> void
ply_to_ply_converter::list_property_end_callback() {}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename SizeType, typename ScalarType> std::tuple<std::function<void (SizeType)>, 
                                                                  std::function<void (ScalarType)>, 
                                                                  std::function<void ()> > 
ply_to_ply_converter::list_property_definition_callback (const std::string&, const std::string& property_name)
{
  (*ostream_) << "property list " << pcl::io::ply::type_traits<SizeType>::old_name() << " " << pcl::io::ply::type_traits<ScalarType>::old_name() << " " << property_name << "\n";
  return std::tuple<std::function<void (SizeType)>, std::function<void (ScalarType)>, std::function<void ()> >(
    [this] (SizeType size) { list_property_begin_callback<SizeType, ScalarType> (size); },
    [this] (ScalarType scalar) { list_property_element_callback<SizeType, ScalarType> (scalar); },
    [this] { list_property_end_callback<SizeType, ScalarType> (); }
  );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ply_to_ply_converter::comment_callback(const std::string& comment)
{
  (*ostream_) << comment << "\n";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ply_to_ply_converter::obj_info_callback(const std::string& obj_info)
{
  (*ostream_) << obj_info << "\n";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
ply_to_ply_converter::end_header_callback()
{
  (*ostream_) << "end_header" << "\n";
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
ply_to_ply_converter::convert (const std::string &ifilename, std::istream&, std::ostream& ostream)
{
  pcl::io::ply::ply_parser ply_parser;

  ply_parser.info_callback ([&, this] (std::size_t line_number, const std::string& message) { info_callback (ifilename, line_number, message); });
  ply_parser.warning_callback ([&, this] (std::size_t line_number, const std::string& message) { warning_callback (ifilename, line_number, message); });
  ply_parser.error_callback ([&, this] (std::size_t line_number, const std::string& message) { error_callback (ifilename, line_number, message); });

  ply_parser.magic_callback ([this] { magic_callback (); });
  ply_parser.format_callback ([this] (pcl::io::ply::format_type format, const std::string& version) { format_callback (format, version); });
  ply_parser.element_definition_callback ([this] (const std::string& element_name, std::size_t count) { return element_definition_callback (element_name, count); });

  pcl::io::ply::ply_parser::scalar_property_definition_callbacks_type scalar_property_definition_callbacks;

  pcl::io::ply::ply_parser::at<pcl::io::ply::int8>(scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalar_property_definition_callback<pcl::io::ply::int8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::int16>(scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalar_property_definition_callback<pcl::io::ply::int16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::int32>(scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalar_property_definition_callback<pcl::io::ply::int32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8>(scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalar_property_definition_callback<pcl::io::ply::uint8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16>(scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalar_property_definition_callback<pcl::io::ply::uint16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32>(scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalar_property_definition_callback<pcl::io::ply::uint32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::float32>(scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalar_property_definition_callback<pcl::io::ply::float32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::float64>(scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return scalar_property_definition_callback<pcl::io::ply::float64> (element_name, property_name); };

  ply_parser.scalar_property_definition_callbacks(scalar_property_definition_callbacks);

  pcl::io::ply::ply_parser::list_property_definition_callbacks_type list_property_definition_callbacks;

  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::int8>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::int8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::int16>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::int16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::int32>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::int32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::uint8>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::uint8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::uint16>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::uint16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::uint32>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::uint32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::float32>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::float32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::float64>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::float64> (element_name, property_name); };

  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16, pcl::io::ply::int8>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint16, pcl::io::ply::int8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16, pcl::io::ply::int16>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint16, pcl::io::ply::int16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16, pcl::io::ply::int32>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint16, pcl::io::ply::int32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16, pcl::io::ply::uint8>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint16, pcl::io::ply::uint8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16, pcl::io::ply::uint16>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint16, pcl::io::ply::uint16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16, pcl::io::ply::uint32>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint16, pcl::io::ply::uint32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16, pcl::io::ply::float32>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint16, pcl::io::ply::float32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint16, pcl::io::ply::float64>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint16, pcl::io::ply::float64> (element_name, property_name); };

  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::int8>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint32, pcl::io::ply::int8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::int16>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint32, pcl::io::ply::int16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::int32>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint32, pcl::io::ply::int32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::uint8>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint32, pcl::io::ply::uint8> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::uint16>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint32, pcl::io::ply::uint16> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::uint32>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint32, pcl::io::ply::uint32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::float32>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint32, pcl::io::ply::float32> (element_name, property_name); };
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint32, pcl::io::ply::float64>(list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name) { return list_property_definition_callback<pcl::io::ply::uint32, pcl::io::ply::float64> (element_name, property_name); };

  ply_parser.list_property_definition_callbacks(list_property_definition_callbacks);

  ply_parser.comment_callback([this] (const std::string& comment) { comment_callback (comment); });
  ply_parser.obj_info_callback([this] (const std::string& obj_info) { obj_info_callback (obj_info); });
  ply_parser.end_header_callback( [this] { return end_header_callback (); });

  ostream_ = &ostream;
  return ply_parser.parse(ifilename);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int 
main(int argc, char* argv[])
{
  ply_to_ply_converter::format_type ply_to_ply_converter_format = ply_to_ply_converter::same_format;

  int argi;
  for (argi = 1; argi < argc; ++argi) {

    if (argv[argi][0] != '-') {
      break;
    }
    if (argv[argi][1] == 0) {
      ++argi;
      break;
    }
    char short_opt, *long_opt, *opt_arg;
    if (argv[argi][1] != '-') {
      short_opt = argv[argi][1];
      opt_arg = &argv[argi][2];
      long_opt = &argv[argi][2];
      while (*long_opt != '\0') {
        ++long_opt;
      }
    }
    else {
      short_opt = 0;
      long_opt = &argv[argi][2];
      opt_arg = long_opt;
      while ((*opt_arg != '=') && (*opt_arg != '\0')) {
        ++opt_arg;
      }
      if (*opt_arg == '=') {
        *opt_arg++ = '\0';
      }
    }

    if ((short_opt == 'h') || (std::strcmp(long_opt, "help") == 0)) {
      std::cout << "Usage: ply2ply [OPTION] [[INFILE] OUTFILE]\n";
      std::cout << "Parse an PLY file.\n";
      std::cout << "\n";
      std::cout << "  -h, --help           display this help and exit\n";
      std::cout << "  -v, --version        output version information and exit\n";
      std::cout << "  -f, --format=FORMAT  set format\n";
      std::cout << "\n";
      std::cout << "FORMAT may be one of the following: ascii, binary, binary_big_endian,\n";
      std::cout << "binary_little_endian.\n";
      std::cout << "If no format is given, the format of INFILE is kept.\n";
      std::cout << "\n";
      std::cout << "With no INFILE/OUTFILE, or when INFILE/OUTFILE is -, read standard input/output.\n";
      std::cout << "\n";
      std::cout << "Report bugs to <www.pointclouds.org/issues>.\n";
      return EXIT_SUCCESS;
    }

    if ((short_opt == 'v') || (std::strcmp(long_opt, "version") == 0)) {
      std::cout << "ply2ply\n";
      std::cout << " Point Cloud Library (PCL) - www.pointclouds.org\n";
      std::cout << " Copyright (c) 2007-2012, Ares Lagae\n";
      std::cout << " Copyright (c) 2012, Willow Garage, Inc.\n";
      std::cout << " All rights reserved.\n";
      std::cout << " Redistribution and use in source and binary forms, with or without\n";
      std::cout << " modification, are permitted provided that the following conditions\n";
      std::cout << " are met:\n";
      std::cout << "  * Redistributions of source code must retain the above copyright\n";
      std::cout << "    notice, this list of conditions and the following disclaimer.\n";
      std::cout << "  * Redistributions in binary form must reproduce the above\n";
      std::cout << "    copyright notice, this list of conditions and the following\n";
      std::cout << "    disclaimer in the documentation and/or other materials provided\n";
      std::cout << "    with the distribution.\n";
      std::cout << "  * Neither the name of Willow Garage, Inc. nor the names of its\n";
      std::cout << "    contributors may be used to endorse or promote products derived\n";
      std::cout << "    from this software without specific prior written permission.\n";
      std::cout << " THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n";
      std::cout << " \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n";
      std::cout << " LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS\n";
      std::cout << " FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE\n";
      std::cout << " COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,\n";
      std::cout << " INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,\n";
      std::cout << " BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;\n";
      std::cout << " LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER\n";
      std::cout << " CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT\n";
      std::cout << " LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN\n";
      std::cout << " ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE\n";
      std::cout << " POSSIBILITY OF SUCH DAMAGE.\n";
      return EXIT_SUCCESS;
    }

    if ((short_opt == 'f') || (std::strcmp(long_opt, "format") == 0)) {
      if (strcmp(opt_arg, "ascii") == 0) {
        ply_to_ply_converter_format = ply_to_ply_converter::ascii_format;
      }
      else if (strcmp(opt_arg, "binary") == 0) {
        ply_to_ply_converter_format = ply_to_ply_converter::binary_format;
      }
      else if (strcmp(opt_arg, "binary_little_endian") == 0) {
        ply_to_ply_converter_format = ply_to_ply_converter::binary_little_endian_format;
      }
      else if (strcmp(opt_arg, "binary_big_endian") == 0) {
        ply_to_ply_converter_format = ply_to_ply_converter::binary_big_endian_format;
      }
      else {
        std::cerr << "ply2ply: " << "invalid option `" << argv[argi] << "'" << "\n";
        std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
        return EXIT_FAILURE;
      }
    }

    else {
      std::cerr << "ply2ply: " << "invalid option `" << argv[argi] << "'" << "\n";
      std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
      return EXIT_FAILURE;
    }
  }

  int parc = argc - argi;
  char** parv = argv + argi;
  if (parc > 2) {
    std::cerr << "ply2ply: " << "too many parameters" << "\n";
    std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
    return EXIT_FAILURE;
  }

  std::ifstream ifstream;
  const char* ifilename = "";
  if (parc > 0) {
    ifilename = parv[0];
    if (std::strcmp(ifilename, "-") != 0) {
      ifstream.open(ifilename);
      if (!ifstream.is_open()) {
        std::cerr << "ply2ply: " << ifilename << ": " << "no such file or directory" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::ofstream ofstream;
  if (parc > 1) {
    const char* ofilename = parv[1];
    if (std::strcmp(ofilename, "-") != 0) {
      ofstream.open(ofilename);
      if (!ofstream.is_open()) {
        std::cerr << "ply2ply: " << ofilename << ": " << "could not open file" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::istream& istream = ifstream.is_open() ? ifstream : std::cin;
  std::ostream& ostream = ofstream.is_open() ? ofstream : std::cout;

  class ply_to_ply_converter ply_to_ply_converter(ply_to_ply_converter_format);
  return ply_to_ply_converter.convert (ifilename, istream, ostream);
}

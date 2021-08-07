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

/** Class ply_to_raw_converter converts a PLY file to a povray (www.povray.org) RAW file
  * The following PLY elements and properties are supported.
  *   element vertex
  *     property float32 x
  *     property float32 y
  *     property float32 z
  *   element face
  *     property list uint8 int32 vertex_indices.
  * 
  * \author Ares Lagae
  * \ingroup io
  */

class ply_to_raw_converter
{
  public:
    ply_to_raw_converter () : 
      ostream_ (), vertex_x_ (0), vertex_y_ (0), vertex_z_ (0), 
      face_vertex_indices_element_index_ (),
      face_vertex_indices_first_element_ (), 
      face_vertex_indices_previous_element_ ()
    {}

    ply_to_raw_converter (const ply_to_raw_converter &f) :
      ostream_ (), vertex_x_ (0), vertex_y_ (0), vertex_z_ (0), 
      face_vertex_indices_element_index_ (),
      face_vertex_indices_first_element_ (), 
      face_vertex_indices_previous_element_ ()
    {
      *this = f;
    }

    ply_to_raw_converter&
    operator = (const ply_to_raw_converter &f)
    {
      ostream_ = f.ostream_;
      vertex_x_ = f.vertex_x_;
      vertex_y_ = f.vertex_y_;
      vertex_z_ = f.vertex_z_;
      face_vertex_indices_element_index_ = f.face_vertex_indices_element_index_;
      face_vertex_indices_first_element_ = f.face_vertex_indices_first_element_;
      face_vertex_indices_previous_element_ = f.face_vertex_indices_previous_element_;
      return (*this);
    }

    bool 
    convert (std::istream& istream, const std::string& istream_filename, std::ostream& ostream, const std::string& ostream_filename);

  private:
    void
    info_callback (const std::string& filename, std::size_t line_number, const std::string& message);

    void
    warning_callback (const std::string& filename, std::size_t line_number, const std::string& message);

    void
    error_callback (const std::string& filename, std::size_t line_number, const std::string& message);

    std::tuple<std::function<void ()>, std::function<void ()> > 
    element_definition_callback (const std::string& element_name, std::size_t count);

    template <typename ScalarType> std::function<void (ScalarType)> 
    scalar_property_definition_callback (const std::string& element_name, const std::string& property_name);

    template <typename SizeType, typename ScalarType>  std::tuple<std::function<void (SizeType)>, 
                                                                       std::function<void (ScalarType)>, 
                                                                       std::function<void ()> > 
    list_property_definition_callback (const std::string& element_name, const std::string& property_name);

    void
    vertex_begin ();

    void
    vertex_x (pcl::io::ply::float32 x);

    void
    vertex_y (pcl::io::ply::float32 y);

    void
    vertex_z (pcl::io::ply::float32 z);

    void
    vertex_end ();

    void
    face_begin ();

    void
    face_vertex_indices_begin (pcl::io::ply::uint8 size);

    void
    face_vertex_indices_element (pcl::io::ply::int32 vertex_index);

    void
    face_vertex_indices_end ();

    void
    face_end ();

    std::ostream* ostream_;
    pcl::io::ply::float32 vertex_x_, vertex_y_, vertex_z_;
    pcl::io::ply::int32 face_vertex_indices_element_index_, face_vertex_indices_first_element_, face_vertex_indices_previous_element_;
    std::vector<std::tuple<pcl::io::ply::float32, pcl::io::ply::float32, pcl::io::ply::float32> > vertices_;
};

void
ply_to_raw_converter::info_callback (const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ":" << line_number << ": " << "info: " << message << std::endl;
}

void
ply_to_raw_converter::warning_callback (const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ":" << line_number << ": " << "warning: " << message << std::endl;
}

void
ply_to_raw_converter::error_callback (const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ":" << line_number << ": " << "error: " << message << std::endl;
}

std::tuple<std::function<void ()>, std::function<void ()> > 
ply_to_raw_converter::element_definition_callback (const std::string& element_name, std::size_t)
{
  if (element_name == "vertex") {
    return std::tuple<std::function<void ()>, std::function<void ()> > (
      [this] { vertex_begin (); },
      [this] { vertex_end (); }
    );
  }
  if (element_name == "face") {
    return std::tuple<std::function<void ()>, std::function<void ()> > (
      [this] { face_begin (); },
      [this] { face_end (); }
    );
  }
  return {};
}

template <> std::function<void (pcl::io::ply::float32)> 
ply_to_raw_converter::scalar_property_definition_callback (const std::string& element_name, const std::string& property_name)
{
  if (element_name == "vertex") {
    if (property_name == "x") {
      return [this] (pcl::io::ply::float32 x) { vertex_x (x); };
    }
    if (property_name == "y") {
      return [this] (pcl::io::ply::float32 y) { vertex_y (y); };
    }
    if (property_name == "z") {
      return [this] (pcl::io::ply::float32 z) { vertex_z (z); };
    }
  }
  return {};
}

template <> std::tuple<std::function<void (pcl::io::ply::uint8)>, 
                            std::function<void (pcl::io::ply::int32)>, 
                            std::function<void ()> > 
ply_to_raw_converter::list_property_definition_callback (const std::string& element_name, const std::string& property_name)
{
  if ((element_name == "face") && (property_name == "vertex_indices")) 
  {
    return std::tuple<std::function<void (pcl::io::ply::uint8)>, 
      std::function<void (pcl::io::ply::int32)>, 
      std::function<void ()> > (
        [this] (pcl::io::ply::uint8 p){ face_vertex_indices_begin (p); },
        [this] (pcl::io::ply::int32 vertex_index) { face_vertex_indices_element (vertex_index); },
        [this] { face_vertex_indices_end (); }
    );
  }
  return {};
}

void
ply_to_raw_converter::vertex_begin () {}

void
ply_to_raw_converter::vertex_x (pcl::io::ply::float32 x)
{
  vertex_x_ = x;
}

void
ply_to_raw_converter::vertex_y (pcl::io::ply::float32 y)
{
  vertex_y_ = y;
}

void
ply_to_raw_converter::vertex_z (pcl::io::ply::float32 z)
{
  vertex_z_ = z;
}

void
ply_to_raw_converter::vertex_end ()
{
  vertices_.emplace_back(vertex_x_, vertex_y_, vertex_z_);
}

void
ply_to_raw_converter::face_begin () {}

void
ply_to_raw_converter::face_vertex_indices_begin (pcl::io::ply::uint8)
{
  face_vertex_indices_element_index_ = 0;
}

void
ply_to_raw_converter::face_vertex_indices_element (pcl::io::ply::int32 vertex_index)
{
  if (face_vertex_indices_element_index_ == 0) {
    face_vertex_indices_first_element_ = vertex_index;
  }
  else if (face_vertex_indices_element_index_ == 1) {
    face_vertex_indices_previous_element_ = vertex_index;
  }
  else {
    (*ostream_) << std::get<0> (vertices_[   face_vertex_indices_first_element_])
         << " " << std::get<1> (vertices_[   face_vertex_indices_first_element_])
         << " " << std::get<2> (vertices_[   face_vertex_indices_first_element_])
         << " " << std::get<0> (vertices_[face_vertex_indices_previous_element_])
         << " " << std::get<1> (vertices_[face_vertex_indices_previous_element_])
         << " " << std::get<2> (vertices_[face_vertex_indices_previous_element_])
         << " " << std::get<0> (vertices_[                         vertex_index])
         << " " << std::get<1> (vertices_[                         vertex_index])
         << " " << std::get<2> (vertices_[                         vertex_index]) << "\n";
    face_vertex_indices_previous_element_ = vertex_index;
  }
  ++face_vertex_indices_element_index_;
}

void
ply_to_raw_converter::face_vertex_indices_end () {}

void
ply_to_raw_converter::face_end () {}

bool 
ply_to_raw_converter::convert (std::istream&, const std::string& istream_filename, std::ostream& ostream, const std::string&)
{
  pcl::io::ply::ply_parser ply_parser;

  ply_parser.info_callback ([&, this] (std::size_t line_number, const std::string& message) { info_callback (istream_filename, line_number, message); });
  ply_parser.warning_callback ([&, this] (std::size_t line_number, const std::string& message) { warning_callback (istream_filename, line_number, message); });
  ply_parser.error_callback ([&, this] (std::size_t line_number, const std::string& message) { error_callback (istream_filename, line_number, message); });

  ply_parser.element_definition_callback ([this] (const std::string& element_name, std::size_t count) { return element_definition_callback (element_name, count); });

  pcl::io::ply::ply_parser::scalar_property_definition_callbacks_type scalar_property_definition_callbacks;
  pcl::io::ply::ply_parser::at<pcl::io::ply::float32> (scalar_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name)
  {
    return scalar_property_definition_callback<pcl::io::ply::float32> (element_name, property_name);
  };
  ply_parser.scalar_property_definition_callbacks (scalar_property_definition_callbacks);

  pcl::io::ply::ply_parser::list_property_definition_callbacks_type list_property_definition_callbacks;
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::int32> (list_property_definition_callbacks) = [this] (const std::string& element_name, const std::string& property_name)
  {
    return list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::int32> (element_name, property_name);
  };
  ply_parser.list_property_definition_callbacks (list_property_definition_callbacks);

  ostream_ = &ostream;

  return ply_parser.parse (istream_filename);
}

int main (int argc, char* argv[])
{
  int argi;
  for (argi = 1; argi < argc; ++argi) {

    if (argv[argi][0] != '-') {
      break;
    }
    if (argv[argi][1] == 0) {
      ++argi;
      break;
    }
    char short_opt, *long_opt;
    if (argv[argi][1] != '-') {
      short_opt = argv[argi][1];
      long_opt = &argv[argi][2];
      while (*long_opt != '\0') {
        ++long_opt;
      }
    }
    else {
      short_opt = 0;
      long_opt = &argv[argi][2];
      char *opt_arg = long_opt;
      while ((*opt_arg != '=') && (*opt_arg != '\0')) {
        ++opt_arg;
      }
      if (*opt_arg == '=') {
        *opt_arg++ = '\0';
      }
    }

    if ((short_opt == 'h') || (std::strcmp (long_opt, "help") == 0)) {
      std::cout << "Usage: ply2raw [OPTION] [[INFILE] OUTFILE]\n";
      std::cout << "Convert from PLY to POV-Ray RAW triangle format.\n";
      std::cout << "\n";
      std::cout << "  -h, --help       display this help and exit\n";
      std::cout << "  -v, --version    output version information and exit\n";
      std::cout << "\n";
      std::cout << "With no INFILE/OUTFILE, or when INFILE/OUTFILE is -, read standard input/output.\n";
      std::cout << "\n";
      std::cout << "The following PLY elements and properties are supported.\n";
      std::cout << "  element vertex\n";
      std::cout << "    property float32 x\n";
      std::cout << "    property float32 y\n";
      std::cout << "    property float32 z\n";
      std::cout << "  element face\n";
      std::cout << "    property list uint8 int32 vertex_indices.\n";
      std::cout << "\n";
      std::cout << "Report bugs to <www.pointclouds.org/issues>.\n";
      return EXIT_SUCCESS;
    }

    if ((short_opt == 'v') || (std::strcmp (long_opt, "version") == 0)) {
      std::cout << "ply2raw \n";
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

    std::cerr << "ply2raw: " << "invalid option `" << argv[argi] << "'" << "\n";
    std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
    return EXIT_FAILURE;
  }

  int parc = argc - argi;
  char** parv = argv + argi;
  if (parc > 2) {
    std::cerr << "ply2raw: " << "too many parameters" << "\n";
    std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
    return EXIT_FAILURE;
  }

  std::ifstream ifstream;
  const char* istream_filename = "";
  if (parc > 0) {
    istream_filename = parv[0];
    if (std::strcmp (istream_filename, "-") != 0) {
      ifstream.open (istream_filename);
      if (!ifstream.is_open ()) {
        std::cerr << "ply2raw: " << istream_filename << ": " << "no such file or directory" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::ofstream ofstream;
  const char* ostream_filename = "";
  if (parc > 1) {
    ostream_filename = parv[1];
    if (std::strcmp (ostream_filename, "-") != 0) {
      ofstream.open (ostream_filename);
      if (!ofstream.is_open ()) {
        std::cerr << "ply2raw: " << ostream_filename << ": " << "could not open file" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::istream& istream = ifstream.is_open () ? ifstream : std::cin;
  std::ostream& ostream = ofstream.is_open () ? ofstream : std::cout;

  class ply_to_raw_converter ply_to_raw_converter;
  return ply_to_raw_converter.convert (istream, istream_filename, ostream, ostream_filename);
}

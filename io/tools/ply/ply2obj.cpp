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

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include <pcl/io/boost.h>
#include <pcl/io/ply/ply_parser.h>

/** \class ply_to_obj_converter
  * Convert a PLY file, optionally meshed to an OBJ file.
  * The following PLY elements and properties are supported.
  *  element vertex
  *    property float32 x
  *    property float32 y
  *    property float32 z
  *  element face
  *    property list uint8 int32 vertex_indices.
  *
  * \author Ares Lagae
  * \ingroup io
  */ 
class ply_to_obj_converter
{
  public:
    typedef int flags_type;
    enum { triangulate = 1 << 0 };

    ply_to_obj_converter (flags_type flags = 0);

    bool 
    convert (std::istream& istream, const std::string& istream_filename, std::ostream& ostream, const std::string& ostream_filename);

  private:

    void
    info_callback (const std::string& filename, std::size_t line_number, const std::string& message);

    void
    warning_callback (const std::string& filename, std::size_t line_number, const std::string& message);

    void
    error_callback (const std::string& filename, std::size_t line_number, const std::string& message);

    boost::tuple<boost::function<void ()>, boost::function<void ()> > 
    element_definition_callback (const std::string& element_name, std::size_t count);

    template <typename ScalarType> boost::function<void (ScalarType)> 
    scalar_property_definition_callback (const std::string& element_name, const std::string& property_name);

    template <typename SizeType, typename ScalarType> boost::tuple<boost::function<void (SizeType)>, 
                                                                      boost::function<void (ScalarType)>, 
                                                                      boost::function<void ()> > 
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

    flags_type flags_;
    std::ostream* ostream_;
    double vertex_x_, vertex_y_, vertex_z_;
    std::size_t face_vertex_indices_element_index_, face_vertex_indices_first_element_, face_vertex_indices_previous_element_;
};

ply_to_obj_converter::ply_to_obj_converter (flags_type flags)
  : flags_ (flags), ostream_ (), 
  vertex_x_ (0), vertex_y_ (0), vertex_z_ (0),
  face_vertex_indices_element_index_ (), 
  face_vertex_indices_first_element_ (),
  face_vertex_indices_previous_element_ ()
{
}

void 
ply_to_obj_converter::info_callback (const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ":" << line_number << ": " << "info: " << message << std::endl;
}

void 
ply_to_obj_converter::warning_callback (const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ":" << line_number << ": " << "warning: " << message << std::endl;
}

void 
ply_to_obj_converter::error_callback (const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ":" << line_number << ": " << "error: " << message << std::endl;
}

boost::tuple<boost::function<void ()>, boost::function<void ()> > 
ply_to_obj_converter::element_definition_callback (const std::string& element_name, std::size_t)
{
  if (element_name == "vertex") 
  {
    return boost::tuple<boost::function<void ()>, boost::function<void ()> > (
      boost::bind (&ply_to_obj_converter::vertex_begin, this),
      boost::bind (&ply_to_obj_converter::vertex_end, this)
    );
  }
  else if (element_name == "face") 
  {
    return boost::tuple<boost::function<void ()>, boost::function<void ()> > (
      boost::bind (&ply_to_obj_converter::face_begin, this),
      boost::bind (&ply_to_obj_converter::face_end, this)
    );
  }
  else {
    return boost::tuple<boost::function<void ()>, boost::function<void ()> > (0, 0);
  }
}

template <> boost::function<void (pcl::io::ply::float32)> 
ply_to_obj_converter::scalar_property_definition_callback (const std::string& element_name, const std::string& property_name)
{
  if (element_name == "vertex") {
    if (property_name == "x") {
      return boost::bind (&ply_to_obj_converter::vertex_x, this, _1);
    }
    else if (property_name == "y") {
      return boost::bind (&ply_to_obj_converter::vertex_y, this, _1);
    }
    else if (property_name == "z") {
      return boost::bind (&ply_to_obj_converter::vertex_z, this, _1);
    }
    else {
      return 0;
    }
  }
  else {
    return 0;
  }
}

template <> boost::tuple<boost::function<void (pcl::io::ply::uint8)>, boost::function<void (pcl::io::ply::int32)>, boost::function<void ()> > 
ply_to_obj_converter::list_property_definition_callback (const std::string& element_name, const std::string& property_name)
{
  if ((element_name == "face") && (property_name == "vertex_indices")) {
    return boost::tuple<boost::function<void (pcl::io::ply::uint8)>, boost::function<void (pcl::io::ply::int32)>, boost::function<void ()> > (
      boost::bind (&ply_to_obj_converter::face_vertex_indices_begin, this, _1),
      boost::bind (&ply_to_obj_converter::face_vertex_indices_element, this, _1),
      boost::bind (&ply_to_obj_converter::face_vertex_indices_end, this)
    );
  }
  else {
    return boost::tuple<boost::function<void (pcl::io::ply::uint8)>, boost::function<void (pcl::io::ply::int32)>, boost::function<void ()> > (0, 0, 0);
  }
}

void 
ply_to_obj_converter::vertex_begin ()
{
}

void 
ply_to_obj_converter::vertex_x (pcl::io::ply::float32 x)
{
  vertex_x_ = x;
}

void 
ply_to_obj_converter::vertex_y (pcl::io::ply::float32 y)
{
  vertex_y_ = y;
}

void 
ply_to_obj_converter::vertex_z (pcl::io::ply::float32 z)
{
  vertex_z_ = z;
}

void 
ply_to_obj_converter::vertex_end ()
{
  (*ostream_) << "v " << vertex_x_ << " " << vertex_y_ << " " << vertex_z_ << "\n";
}

void 
ply_to_obj_converter::face_begin ()
{
  if (!(flags_ & triangulate)) {
    (*ostream_) << "f";
  }
}

void 
ply_to_obj_converter::face_vertex_indices_begin (pcl::io::ply::uint8)
{
  face_vertex_indices_element_index_ = 0;
}

void 
ply_to_obj_converter::face_vertex_indices_element (pcl::io::ply::int32 vertex_index)
{
  if (flags_ & triangulate) {
    if (face_vertex_indices_element_index_ == 0) {
      face_vertex_indices_first_element_ = vertex_index;
    }
    else if (face_vertex_indices_element_index_ == 1) {
      face_vertex_indices_previous_element_ = vertex_index;
    }
    else {
      (*ostream_) << "f " << (face_vertex_indices_first_element_ + 1) << " " << (face_vertex_indices_previous_element_ + 1) << " " << (vertex_index + 1) << "\n";
      face_vertex_indices_previous_element_ = vertex_index;
    }
    ++face_vertex_indices_element_index_;
  }
  else {
    (*ostream_) << " " << (vertex_index + 1);
  }
}

void 
ply_to_obj_converter::face_vertex_indices_end ()
{
  if (!(flags_ & triangulate)) {
    (*ostream_) << "\n";
  }
}

void 
ply_to_obj_converter::face_end ()
{
}

bool 
ply_to_obj_converter::convert (std::istream&, const std::string& istream_filename, std::ostream& ostream, const std::string&)
{
  pcl::io::ply::ply_parser::flags_type ply_parser_flags = 0;
  pcl::io::ply::ply_parser ply_parser (ply_parser_flags);

  ply_parser.info_callback (boost::bind (&ply_to_obj_converter::info_callback, this, boost::ref (istream_filename), _1, _2));
  ply_parser.warning_callback (boost::bind (&ply_to_obj_converter::warning_callback, this, boost::ref (istream_filename), _1, _2));
  ply_parser.error_callback (boost::bind (&ply_to_obj_converter::error_callback, this, boost::ref (istream_filename), _1, _2)); 

  ply_parser.element_definition_callback (boost::bind (&ply_to_obj_converter::element_definition_callback, this, _1, _2));

  pcl::io::ply::ply_parser::scalar_property_definition_callbacks_type scalar_property_definition_callbacks;
  pcl::io::ply::ply_parser::at<pcl::io::ply::float32> (scalar_property_definition_callbacks) = boost::bind (&ply_to_obj_converter::scalar_property_definition_callback<pcl::io::ply::float32>, this, _1, _2);
  ply_parser.scalar_property_definition_callbacks (scalar_property_definition_callbacks);

  pcl::io::ply::ply_parser::list_property_definition_callbacks_type list_property_definition_callbacks;
  pcl::io::ply::ply_parser::at<pcl::io::ply::uint8, pcl::io::ply::int32> (list_property_definition_callbacks) = boost::bind (&ply_to_obj_converter::list_property_definition_callback<pcl::io::ply::uint8, pcl::io::ply::int32>, this, _1, _2);
  ply_parser.list_property_definition_callbacks (list_property_definition_callbacks);

  ostream_ = &ostream;

  return ply_parser.parse (istream_filename);
}

int main (int argc, char* argv[])
{
  ply_to_obj_converter::flags_type ply_to_obj_converter_flags = 0;

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

    if ((short_opt == 'h') || (std::strcmp (long_opt, "help") == 0)) {
      std::cout << "Usage: ply2obj [OPTION] [[INFILE] OUTFILE]\n";
      std::cout << "Convert a PLY file to an OBJ file.\n";
      std::cout << "\n";
      std::cout << "  -h, --help       display this help and exit\n";
      std::cout << "  -v, --version    output version information and exit\n";
      std::cout << "  -f, --flag=FLAG  set flag\n";
      std::cout << "\n";
      std::cout << "FLAG may be one of the following: triangulate.\n";
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

    else if ((short_opt == 'v') || (std::strcmp (long_opt, "version") == 0)) {
      std::cout << "ply2obj \n";
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

    else if ((short_opt == 'f') || (std::strcmp (long_opt, "flag") == 0)) {
      if (strcmp (opt_arg, "triangulate") == 0) {
        ply_to_obj_converter_flags |= ply_to_obj_converter::triangulate;
      }
      else {
        std::cerr << "ply2obj : " << "invalid option `" << argv[argi] << "'" << "\n";
        std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
        return EXIT_FAILURE;
      }
    }

    else {
      std::cerr << "ply2obj: " << "invalid option `" << argv[argi] << "'" << "\n";
      std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
      return EXIT_FAILURE;
    }
  }

  int parc = argc - argi;
  char** parv = argv + argi;
  if (parc > 2) {
    std::cerr << "ply2obj: " << "too many parameters" << "\n";
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
        std::cerr << "ply2obj: " << istream_filename << ": " << "no such file or directory" << "\n";
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
        std::cerr << "ply2obj: " << ostream_filename << ": " << "could not open file" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::istream& istream = ifstream.is_open () ? ifstream : std::cin;
  std::ostream& ostream = ofstream.is_open () ? ofstream : std::cout;

  class ply_to_obj_converter ply_to_obj_converter (ply_to_obj_converter_flags);
  return ply_to_obj_converter.convert (istream, istream_filename, ostream, ostream_filename);
}

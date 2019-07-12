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

#include <fstream>
#include <iostream>
#include <cstring>
#include <string>
#include <cstdlib>

/** \file plheader extracts and prints out the header of a PLY file
  * 
  * \author Ares Lagae
  * \ingroup io
  */
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

    if ((short_opt == 'h') || (strcmp (long_opt, "help") == 0)) {
      std::cout << "Usage: plyheader [OPTION] [[INFILE] OUTFILE]\n";
      std::cout << "Extract the header from a PLY file.\n";
      std::cout << "\n";
      std::cout << "  -h, --help       display this help and exit\n";
      std::cout << "  -v, --version    output version information and exit\n";
      std::cout << "\n";
      std::cout << "With no INFILE/OUTFILE, or when INFILE/OUTFILE is -, read standard input/output.\n";
      std::cout << "\n";
      std::cout << "Report bugs to <www.pointclouds.org/issues>.\n";
      return EXIT_SUCCESS;
    }

    if ((short_opt == 'v') || (strcmp (long_opt, "version") == 0)) {
      std::cout << "plyheader \n";
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

    std::cerr << "plyheader: " << "invalid option `" << argv[argi] << "'" << "\n";
    std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
    return EXIT_FAILURE;
  }

  int parc = argc - argi;
  char** parv = argv + argi;
  if (parc > 2) {
    std::cerr << "plyheader: " << "too many parameters" << "\n";
    std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
    return EXIT_FAILURE;
  }

  std::ifstream ifstream;
  if (parc > 0) {
    const char* ifilename = parv[0];
    if (strcmp (ifilename, "-") != 0) {
      ifstream.open (ifilename);
      if (!ifstream.is_open ()) {
        std::cerr << "plyheader: " << ifilename << ": " << "no such file or directory" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::ofstream ofstream;
  if (parc > 1) {
    const char* ofilename = parv[1];
    if (strcmp (ofilename, "-") != 0) {
      ofstream.open (ofilename);
      if (!ofstream.is_open ()) {
        std::cerr << "plyheader: " << ofilename << ": " << "could not open file" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::istream& istream = ifstream.is_open () ? ifstream : std::cin;
  std::ostream& ostream = ofstream.is_open () ? ofstream : std::cout;

  char magic[3];
  istream.read (magic, 3);
  if (!istream || (magic[0] != 'p') || (magic[1] != 'l') || (magic[2] != 'y')){
    return EXIT_FAILURE;
  }
  istream.ignore (1);
  ostream << magic[0] << magic[1] << magic[2] << "\n";
  std::string line;
  while (std::getline (istream, line)) {
    ostream << line << "\n";
    if (line == "end_header") {
      break;
    }
  }
  return istream ? EXIT_SUCCESS : EXIT_FAILURE;
}

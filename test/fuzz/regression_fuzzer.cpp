/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

void
test_ply_reader(const std::string& filename);

namespace fs = boost::filesystem;

int
main(int argc, char** argv)
{

  if (argc < 2) {
    std::cerr << "No folder for corpus was provided. Please add the path to corpus "
                 "folder to this test."
              << std::endl;
    return -1;
  }
  else if (argc > 2) {
    std::cout << "Ignoring all but the first argument\n";
  }

  std::string corpus_dir = argv[1];
  std::vector<std::string> corpus_filenames;

  fs::directory_iterator end_itr;
  for (fs::directory_iterator itr(corpus_dir); itr != end_itr; ++itr) {
    if (fs::is_regular_file(itr->status()) &&
        boost::algorithm::to_upper_copy(fs::extension(itr->path())) == ".PLY") {
      corpus_filenames.push_back(itr->path().string());
    }
  }
  std::cout << "Total PLY files found: " << corpus_filenames.size() << "\n";

  for (const auto& name : corpus_filenames) {
    std::cout << "Running: " << name << "\n";
    test_ply_reader(name);
  }

  return 0;
}

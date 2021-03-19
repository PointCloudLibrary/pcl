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

/*
extern "C" int
LLVMFuzzerTestOneInput(const unsigned char* data, size_t size);

__attribute__((weak)) extern "C" int
LLVMFuzzerInitialize(int* argc, char*** argv);
*/

namespace fs = boost::filesystem;

int
main(int argc, char** argv)
{
  /*
  if (LLVMFuzzerInitialize) {
    LLVMFuzzerInitialize(&argc, &argv);
  }
  */

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
      std::cout << "adding: " << corpus_filenames.back() << "\n";
    }
  }

  for (const auto& name : corpus_filenames) {
    test_ply_reader(name);
  }

  /*
  for (int i = 1; i < argc; i++) {
    fprintf(stderr, "Running: %s\n", argv[i]);
    FILE* f = fopen(argv[i], "r");
    assert(f);
    fseek(f, 0, SEEK_END);
    size_t len = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char* buf = (unsigned char*)malloc(len);
    size_t n_read = fread(buf, 1, len, f);
    fclose(f);
    assert(n_read == len);
    LLVMFuzzerTestOneInput(buf, len);
    free(buf);
    fprintf(stderr, "Done:    %s: (%zd bytes)\n", argv[i], n_read);
  }
  */
  return 0;
}

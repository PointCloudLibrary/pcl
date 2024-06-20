/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <cstdio>  // for snprintf, f{open, write, close}
#include <string>  // for std::string

#include <unistd.h>  // for unlink

void test_ply_reader(const std::string& filename);

extern "C" int
LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
  char filename[256];
  std::snprintf(filename, 256, "/tmp/libfuzzer.%d", getpid());

  FILE* fp = std::fopen(filename, "wb");
  if (!fp)
    return 0;
  std::fwrite(data, size, 1, fp);
  std::fclose(fp);

  test_ply_reader(filename);

  unlink(filename);
  return 0;
}

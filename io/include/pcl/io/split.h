/*
* SPDX-License-Identifier: BSD-3-Clause
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014-, Open Perception Inc.
*
*  All rights reserved
*/

#pragma once
#include <string>

namespace pcl {

/** \brief Lightweight tokenization function
 * This function can be used as a boost::split substitute. When benchmarked against
 * boost, this function will create much less alocations and hence it is much better
 * suited for quick line tokenization.
 *
 * Cool thing is this function will work with SequenceSequenceT =
 * std::vector<std::string> and std::vector<std::string_view>
 */
template <typename SequenceSequenceT>
void
split(SequenceSequenceT& result, std::string const& in, const char* const delimiters)
{
  using StringSizeT = std::string::size_type;

  const auto len = in.length();
  StringSizeT token_start = 0;

  result.clear();
  while (token_start < len) {
    // eat leading whitespace
    token_start = in.find_first_not_of(delimiters, token_start);
    if (token_start == std::string::npos) {
      return; // nothing left but white space
    }

    // find the end of the token
    const auto token_end = in.find_first_of(delimiters, token_start);

    // push token
    if (token_end == std::string::npos) {
      result.emplace_back(in.data() + token_start, len - token_start);
      return;
    }
    else {
      result.emplace_back(in.data() + token_start, token_end - token_start);
    }

    // set up for next loop
    token_start = token_end + 1;
  }
}
} // namespace pcl

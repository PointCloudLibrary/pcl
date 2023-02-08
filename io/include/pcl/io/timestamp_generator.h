/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2023-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/pcl_exports.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

namespace pcl {
/**
 * @brief Returns a timestamp as string formatted like boosts to_iso_string see https://www.boost.org/doc/libs/1_81_0/doc/html/date_time/posix_time.html#ptime_to_string
 * Example: 19750101T235959.123456
 * @param time std::chrono::timepoint to convert, defaults to now
 * @return std::string containing the timestamp
*/
PCL_EXPORTS inline std::string
getTimestamp(const std::chrono::time_point<std::chrono::system_clock>& time =
                 std::chrono::system_clock::now())
{
  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch());

  const auto s = std::chrono::duration_cast<std::chrono::seconds>(us);
  std::time_t tt = s.count();
  std::size_t fractional_seconds = us.count() % 1000000;

  std::tm tm = *std::gmtime(&tt); // GMT (UTC)
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y%m%dT%H%M%S");

  if (fractional_seconds > 0) {
    ss << "." << std::setw(6) << std::setfill('0') << fractional_seconds;
  }

  return ss.str();
}
} // namespace pcl

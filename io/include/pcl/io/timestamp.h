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
 * @brief Returns a timestamp in local time as string formatted like boosts to_iso_string see https://www.boost.org/doc/libs/1_81_0/doc/html/date_time/posix_time.html#ptime_to_string
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

  std::tm tm = *std::localtime(&tt); // local time
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y%m%dT%H%M%S");

  if (fractional_seconds > 0) {
    ss << "." << std::setw(6) << std::setfill('0') << fractional_seconds;
  }

  return ss.str();
}

/**
 * @brief Parses a iso timestring (see https://www.boost.org/doc/libs/1_81_0/doc/html/date_time/posix_time.html#ptime_to_string) and returns a timepoint
 * @param timestamp as string formatted like boost iso date
 * @return std::chrono::time_point with system_clock
*/
PCL_EXPORTS inline std::chrono::time_point<std::chrono::system_clock>
parseTimestamp(std::string timestamp)
{
  std::istringstream ss;

  std::tm tm = {};

  std::size_t fractional_seconds = 0;

  ss.str(timestamp);
  ss >> std::get_time(&tm, "%Y%m%dT%H%M%S");

  auto timepoint = std::chrono::system_clock::from_time_t(std::mktime(&tm));

  const auto pos = timestamp.find('.');

  if (pos != std::string::npos) {
    const auto frac_text = timestamp.substr(pos+1);  
    ss.str(frac_text);
    ss >> fractional_seconds;
    timepoint += std::chrono::microseconds(fractional_seconds);
  }

  return timepoint;
}

} // namespace pcl

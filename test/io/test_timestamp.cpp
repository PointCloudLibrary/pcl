#include <pcl/io/timestamp.h>
#include <pcl/test/gtest.h>

std::string
getTimeOffset()
{
  // local offset
  auto offset_hour = std::localtime(new time_t(0))->tm_hour;
  std::ostringstream ss;
  ss << std::setfill('0') << std::setw(2) << offset_hour;

  return ss.str();
}

TEST(PCL, TestTimestampGeneratorZeroFraction)
{
  const std::chrono::time_point<std::chrono::system_clock> time;

  const auto timestamp = pcl::getTimestamp(time);

  EXPECT_EQ(timestamp, "19700101T" + getTimeOffset() + "0000");
}

TEST(PCL, TestTimestampGeneratorWithFraction)
{
  const std::chrono::microseconds dur(123456);
  const std::chrono::time_point<std::chrono::system_clock> dt(dur);

  const auto timestamp = pcl::getTimestamp(dt);

  EXPECT_EQ(timestamp, "19700101T" + getTimeOffset() + "0000.123456");
}

TEST(PCL, TestTimestampGeneratorWithSmallFraction)
{
  const std::chrono::microseconds dur(123);
  const std::chrono::time_point<std::chrono::system_clock> dt(dur);

  const auto timestamp = pcl::getTimestamp(dt);

  EXPECT_EQ(timestamp, "19700101T" + getTimeOffset() + "0000.000123");
}

TEST(PCL, TestParseTimestamp)
{
  const std::chrono::time_point<std::chrono::system_clock> timepoint(std::chrono::system_clock::now());

  const auto timestamp = pcl::getTimestamp(timepoint);

  const auto parsedTimepoint = pcl::parseTimestamp(timestamp);

  const auto diff = std::chrono::duration<double,std::milli>(timepoint - parsedTimepoint).count();

  EXPECT_LT(diff, 1e-3);
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */

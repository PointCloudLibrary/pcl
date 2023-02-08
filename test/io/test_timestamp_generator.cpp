#include <pcl/io/timestamp_generator.h>
#include <pcl/test/gtest.h>

TEST(PCL, TestTimestampGeneratorZeroFraction)
{
  const std::chrono::time_point<std::chrono::system_clock> time;

  const auto filename = pcl::getTimestamp(time);

  EXPECT_EQ(filename, "19700101T000000");
}

TEST(PCL, TestTimestampGeneratorWithFraction)
{
  const std::chrono::microseconds dur(123456);
  const std::chrono::time_point<std::chrono::system_clock> dt(dur);

  const auto filename = pcl::getTimestamp(dt);

  EXPECT_EQ(filename, "19700101T000000.123456");
}

TEST(PCL, TestTimestampGeneratorWithSmallFraction)
{
  const std::chrono::microseconds dur(123);
  const std::chrono::time_point<std::chrono::system_clock> dt(dur);

  const auto filename = pcl::getTimestamp(dt);

  EXPECT_EQ(filename, "19700101T000000.000123");
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */

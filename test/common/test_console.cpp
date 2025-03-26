#include <pcl/test/gtest.h>

#include <pcl/console/print.h>
#include <pcl/point_types.h>

TEST(PCL, printStreamsMacros) {
  pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);

  PCL_ALWAYS_STREAM("Always_stream\n");
  PCL_ERROR_STREAM("Error_stream\n")
  PCL_WARN_STREAM("Warn_stream\n");
  PCL_INFO_STREAM("Info_stream: this is a point: " << pcl::PointXYZ(1.0, 2.0, 3.0)
                                                   << "\n");
  PCL_DEBUG_STREAM("Debug_stream\n");
  PCL_VERBOSE_STREAM("Verbose_stream: an Eigen vector: "
                     << "\n"
                     << Eigen::Vector3f(1.0, 2.0, 3.0) << "\n");
}

TEST(PCL, printMacros)
{
  pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);

  PCL_ALWAYS("Always\n");
  PCL_ERROR("Error\n");
  PCL_WARN("Warn\n");
  PCL_INFO("Info\n");
  PCL_DEBUG("Debug\n");
  PCL_VERBOSE("Verbose\n");
  PCL_HIGH("High\n");
}

TEST(PCL, print)
{
  pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);

  pcl::console::print(pcl::console::L_ALWAYS, "Print always\n");
  
  pcl::console::print_error("Error\n");
  pcl::console::print_warn("Warn\n");
  pcl::console::print_info("Info\n");
  pcl::console::print_debug("Debug\n");
  pcl::console::print(pcl::console::L_VERBOSE, "Print verbose\n");

  pcl::console::print_highlight("Highlight\n");
  pcl::console::print_value("%g\n", 0.001235);
}

void
callPrints()
{
  pcl::console::print(pcl::console::L_ALWAYS, "Print always\n");
  pcl::console::print_error("Error\n");
  pcl::console::print_warn("Warn\n");
  pcl::console::print_info("Info\n");
  pcl::console::print_debug("Debug\n");
  pcl::console::print(pcl::console::L_VERBOSE, "Print verbose\n");
}

TEST(PCL, checkLogLevel)
{
  int callbackCount = 0;
  
  pcl::console::setCallback(
      [&callbackCount](const pcl::console::LogRecord& rec) {
        std::cout << "Received logrecord: level:" << rec.level << " with message: " << rec.message;
        callbackCount++;
       });

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  callPrints();
  EXPECT_EQ(callbackCount, 1);

  callbackCount = 0;
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  callPrints();
  EXPECT_EQ(callbackCount, 2);

  callbackCount = 0;
  pcl::console::setVerbosityLevel(pcl::console::L_WARN);
  callPrints();
  EXPECT_EQ(callbackCount, 3);

  callbackCount = 0;
  pcl::console::setVerbosityLevel(pcl::console::L_INFO);
  callPrints();
  EXPECT_EQ(callbackCount, 4);

  callbackCount = 0;
  pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
  callPrints();
  EXPECT_EQ(callbackCount, 5);

  callbackCount = 0;
  pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
  callPrints();
  EXPECT_EQ(callbackCount, 6);

}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */

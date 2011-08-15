#include <iostream>
#include <map>
#include <fstream>

#include <boost/lexical_cast.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/passthrough.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <Eigen/Core>

#include <pcl/features/feature_evaluation/feature_evaluation_framework.h>

namespace co = pcl::console;

/*
 * Use this executable to run multiple trials of feature evaluation tests, with a single independant variable.
 * All parameters should be set from the commandline.
 *
 * The necessary parameters are:
 *
 * -pointtype <type of the input clouds>
 * -feature <name of the feature algorithm to be tested>
 * -source <path to source pcd file>
 * -target <path to target pcd file>
 * -datalabel <optional label for the dataset, useful for organizing test results>
 * -groundtruth <path to file from which groundtruhs are to be read, as Eigen::Matrix4f>
 * -threshold <threshold for nearest neighbour search in the feature descriptor space>
 * (or) -lthreshold, -uthreshold, -dthreshold <specify threshold range with lower, upper and delta values>
 * -params <string containing required parameters for the Feature algorithm, given as param1=value1,param2=value2...>
 * -paramslabel <optional label for the parameters, useful for organizing test results>
 * -downsample <provide this if input clouds need to be downsampled>
 * -leafsize <voxelgrid leafsize for downsampling>
 * (or) -leafx, -leafy, leafz <for asymmetric leaf dimensions>
 * -verbose <provide this argument to see test execution status on console>
 * -logfile <path to file for logging test results in CSV format>
 * -single <to run a single feature evaluation trial: must provide all neccessary parameters>
 * -multiple <provide an independant variable name (feature/clouds/parameters/leaf) and set all remaining parameters through above options>
 * -filename <in case of multiple, path to file containing list of values for the independant variable, one on each line>
 *
 */


template <typename PointIn>
int runTests (int argc, char** argv)
{
  pcl::FeatureEvaluationFramework<PointIn> test_features;

  std::string feature_name = "";
  if (co::find_argument (argc, argv, "-feature") != -1) {
    co::parse_argument (argc, argv, "-feature", feature_name);
    test_features.setFeatureTest (feature_name);
  }

  std::string source_file = "", target_file = "", dataset_label = "";
  if (co::find_argument (argc, argv, "-datalabel") != -1)
      co::parse_argument (argc, argv, "-datalabel", dataset_label);

  if (co::find_argument (argc, argv, "-source") != -1 &&
      co::find_argument (argc, argv, "-target") != -1) {
    co::parse_argument (argc, argv, "-source", source_file);
    co::parse_argument (argc, argv, "-target", target_file);
    test_features.setInputClouds (source_file, target_file, dataset_label);
  }

  std::string ground_truth = "";
  if (co::find_argument (argc, argv, "-groundtruth") != -1) {
    co::parse_argument (argc, argv, "-groundtruth", ground_truth);
    test_features.setGroundTruth (ground_truth);
  }

  std::string threshold, l_threshold, u_threshold, d_threshold;
  if (co::find_argument (argc, argv, "-threshold") != -1) {
    co::parse_argument (argc, argv, "-threshold", threshold);
    float th = boost::lexical_cast<float> (threshold);
    test_features.setThreshold (th);
  }
  else if (co::find_argument (argc, argv, "-lthreshold") != -1 &&
           co::find_argument (argc, argv, "-uthreshold") != -1 &&
           co::find_argument (argc, argv, "-dthreshold") != -1) {
    co::parse_argument (argc, argv, "-lthreshold", l_threshold);
    co::parse_argument (argc, argv, "-uthreshold", u_threshold);
    co::parse_argument (argc, argv, "-dthreshold", d_threshold);
    float l_t = boost::lexical_cast<float> (l_threshold);
    float u_t = boost::lexical_cast<float> (u_threshold);
    float d_t = boost::lexical_cast<float> (d_threshold);
    test_features.setThreshold (l_t, u_t, d_t);
  }

  std::string parameters = "", params_label = "";
  if (co::find_argument (argc, argv, "-paramslabel") != -1)
      co::parse_argument (argc, argv, "-paramslabel", params_label);

  if (co::find_argument (argc, argv, "-params") != -1) {
    co::parse_argument (argc, argv, "-params", parameters);
    test_features.setParameters(parameters, params_label);
  }

  if (co::find_argument (argc, argv, "-downsample") != -1) {
    test_features.setDownsampling (true);
  }

  std::string leaf_s, leaf_x, leaf_y, leaf_z;
  if (co::find_argument (argc, argv, "-leafsize") != -1) {
    co::parse_argument (argc, argv, "-leafsize", leaf_s);
    float l_s = boost::lexical_cast<float> (leaf_s);
    test_features.setLeafSize(l_s);
  }
  else if (co::find_argument (argc, argv, "-leafx") != -1 &&
           co::find_argument (argc, argv, "-leafy") != -1 &&
           co::find_argument (argc, argv, "-leafz") != -1) {
    co::parse_argument (argc, argv, "-leafx", leaf_x);
    co::parse_argument (argc, argv, "-leafy", leaf_y);
    co::parse_argument (argc, argv, "-leafz", leaf_z);
    float l_x = boost::lexical_cast<float> (leaf_x);
    float l_y = boost::lexical_cast<float> (leaf_y);
    float l_z = boost::lexical_cast<float> (leaf_z);
    test_features.setLeafSize (l_x, l_y, l_z);
  }

  if (co::find_argument (argc, argv, "-verbose") != -1) {
    test_features.setVerbose (true);
  }

  std::string logfile = "";
  if (co::find_argument (argc, argv, "-logfile") != -1) {
    co::parse_argument (argc, argv, "-logfile", logfile);
    test_features.setLogFile(logfile);
  }

  if (co::find_argument (argc, argv, "-single") != -1) {
    test_features.runSingleTest ();
    return 0;
  }

  if (co::find_argument (argc, argv, "-multiple") == -1) {
    co::print_error ("Specify test mode: -single or -multiple\n");
    //print_help ();
    return -1;
  }

  std::string ind_variable = "", file_name = "";
  co::parse_argument (argc, argv, "-multiple", ind_variable);
  co::parse_argument (argc, argv, "-filename", file_name);

  if (ind_variable == "" || file_name == "") {
    co::print_error ("Specify an independent variable and filename containing input values, with\n");
    co::print_error ("-multiple <ind_variable> -filename <file_name>\n");
    //print_help ();
    return -1;
  }

  if (ind_variable == "feature") test_features.runMultipleFeatures (file_name);
  else if (ind_variable == "clouds") test_features.runMultipleClouds (file_name);
  else if (ind_variable == "parameters") test_features.runMultipleParameters (file_name);
  else if (ind_variable == "leaf") test_features.runMultipleLeafSizes (file_name);
  else {
    co::print_error ("Select correct independent variable for multiple tests: feature, clouds, parameters or leaf\n");
    //print_help();
    return -1;
  }

  test_features.clearData ();
  return 0;
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (co::find_argument (argc, argv, "-pointtype") == -1) {
    co::print_error ("Specify point type of input clouds with -pointtype\n");
    //print_help();
    return -1;
  }

  std::string pointtype;
  co::parse_argument (argc, argv, "-pointtype", pointtype);

  if (pointtype == "PointXYZ") {
    return runTests<pcl::PointXYZ> (argc, argv);
  }
  else if (pointtype == "PointXYZRGB") {
    return runTests<pcl::PointXYZRGB> (argc, argv);
  }
  else if (pointtype == "PointNormal") {
    return runTests<pcl::PointNormal> (argc, argv);
  }
  else if (pointtype == "PointXYZRGBNormal") {
    return runTests<pcl::PointXYZRGBNormal> (argc, argv);
  }
  //et cetera..
  else {
    co::print_error ("Specify valid point cloud type with -pointtype\n");
    //print_help();
    return -1;
  }

  return 0;
}
/* ]--- */

/*

int main()
{
  pcl::FeatureEvaluationFramework<pcl::PointXYZRGB> test_features;
  std::string parameters = "searchradius=0.05";

  test_features.setFeatureTest ("FPFHTest");
  test_features.setInputClouds ("../conference_room/cloud_000.pcd", "../conference_room/cloud_000.pcd", "cloud_000.pcd");
  //test_features.setGroundTruths();
  test_features.setThreshold (0.01,0.1,0.01);
  test_features.setParameters (parameters);
  test_features.setDownsampling (true);
  test_features.setLeafSize(0.01,0.01,0.01);
  test_features.setVerbose (true);
  test_features.setLogFile ("variation-with-leaf-sizes-new.txt");

  test_features.runMultipleLeafSizes("leafsizes.txt");

  return 0;
}

*/

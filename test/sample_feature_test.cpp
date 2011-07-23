#include <iostream>
#include <map>
#include <fstream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Core>

#include "feature_evaluation_framework.h"

/*
 * This is a sample program to illustrate the use of the Feature Evaluation Framework.
 *
 * Basically one can run multiple trials on a single independant variable.
 * The list of values for that variable will be read from a file. For the input format,
 * see the source files for FeatureEvaluationFramework.
 *
 * All other parameters for the trial, including the output logfile, should be set by
 * calling appropriate functions of the framework class, else they will assume default
 * values.
 *
 * Here, the trials are run on varying leaf-sizes of the Voxel Grid filter.
 *
 */


int main()
{
  pcl::FeatureEvaluationFramework<pcl::PointXYZRGB> test_features;
  std::string parameters = "searchradius=0.05";

  test_features.setFeatureTest ("FPFHTest");
  test_features.setInputClouds ("./cloud_000.pcd", "./cloud_000.pcd", "cloud_000.pcd");
  //test_features.setGroundTruths();
  test_features.setThreshold (0.01,0.1,0.01);
  test_features.setParameters (parameters);
  test_features.setDownsampling (true);
  test_features.setVerbose (true);
  test_features.setLogFile ("variation-with-leaf-sizes.txt");

  test_features.runMultipleLeafSizes("leaf-sizes.txt");

  return 0;
}


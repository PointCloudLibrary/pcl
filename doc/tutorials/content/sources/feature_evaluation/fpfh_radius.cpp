#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/feature_evaluation/feature_evaluation_framework.h>

int main (int argc, char** argv)
{
  if (argc < 4) {
    std::cout << "Specify the input cloud, ground truth and parameter files:\n";
    std::cout << "   " << argv[0] << " input_cloud.pcd ground_truth.txt parameters.txt\n";
    exit(0);
  }
  pcl::FeatureEvaluationFramework<pcl::PointXYZRGB> test_features;

  test_features.setFeatureTest ("FPFHTest");
  test_features.setGroundTruth (argv[2]);

  //If "" is passed for the target-cloud file, source-cloud will be transformed by ground_truth to get the target cloud
  test_features.setInputClouds (argv[1], "", argv[1]);
  test_features.setThreshold (0.1f,1.0f,0.1f);

  //The independent variable need not be set separately, its values will be read from the file
  //std::string parameters = "searchradius=0.05";
  //test_features.setParameters (parameters);

  test_features.setDownsampling (true);
  test_features.setLeafSize (0.01f);
  test_features.setVerbose (true);
  test_features.setLogFile ("fpfh-radius-variation.txt");

  test_features.runMultipleParameters (argv[3]);

  test_features.clearData ();
  return 0;
}

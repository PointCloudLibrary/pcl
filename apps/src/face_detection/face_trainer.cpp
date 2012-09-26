/*
 * face_trainer.cpp
 *
 *  Created on: 22 Sep 2012
 *      Author: Aitor Aldoma
 */

#include "pcl/recognition/face_detection/rf_face_detector_trainer.h"
#include <pcl/console/parse.h>

int main(int argc, char ** argv)
{
  int ntrees = 10;
  std::string forest_fn = "forest.txt";
  int n_features = 1000;
  std::string directory = "";
  int use_normals = 0;
  int num_images = 3000;

  pcl::console::parse_argument (argc, argv, "-ntrees", ntrees);
  pcl::console::parse_argument (argc, argv, "-forest_fn", forest_fn);
  pcl::console::parse_argument (argc, argv, "-n_features", n_features);
  pcl::console::parse_argument (argc, argv, "-directory", directory);
  pcl::console::parse_argument (argc, argv, "-use_normals", use_normals);
  pcl::console::parse_argument (argc, argv, "-num_images", num_images);

  pcl::RFFaceDetectorTrainer fdrf;
  fdrf.setForestFilename (forest_fn);
  fdrf.setDirectory (directory);
  fdrf.setWSize (80);
  fdrf.setNumTrees (ntrees);
  fdrf.setNumFeatures (n_features);
  fdrf.setUseNormals (static_cast<bool> (use_normals));
  fdrf.setNumTrainingImages (num_images);

  //TODO: do some checks here..., fn file exists, directory exists, etc...
  fdrf.trainWithDataProvider ();
}


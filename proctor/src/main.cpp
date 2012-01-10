#include "proctor/detector.h"
#include "proctor/proctor.h"
#include "proctor/scanning_model_source.h"
#include "proctor/basic_proposer.h"

using pcl::proctor::Detector;
//using pcl::proctor::ProctorMPI;
using pcl::proctor::Proctor;
using pcl::proctor::ScanningModelSource;
using pcl::proctor::BasicProposer;

int main(int argc, char **argv) {
  unsigned int model_seed = 2;
  unsigned int test_seed = 0; //time(NULL);

  if (argc >= 2)
  {
    model_seed = atoi(argv[1]);
  }

  if (argc >= 3)
  {
    test_seed = atoi(argv[2]);
  }

  Detector detector;
  Proctor proctor;

  BasicProposer::Ptr proposer(new BasicProposer);
  detector.setProposer(proposer);

  ScanningModelSource model_source("Princeton", "/home/justin/Documents/benchmark/db");
  model_source.loadModels();

  //detector.enableVisualization();
  proctor.setModelSource(&model_source);
  proctor.train(detector);
  proctor.test(detector, test_seed);
  proctor.printResults(detector);

  sleep(100000);

  return 0;
}


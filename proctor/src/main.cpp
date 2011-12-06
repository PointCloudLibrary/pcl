#include "proctor/detector.h"
#include "proctor/proctor_mpi.h"
#include <omp.h>
#include <boost/mpi.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/serialization/string.hpp>
#include <boost/unordered_map.hpp>
namespace mpi = boost::mpi;

using pcl::proctor::Detector;
using pcl::proctor::ProctorMPI;
using pcl::proctor::Proctor;

Detector detector;
ProctorMPI proctor;

int main(int argc, char **argv) {

  unsigned int model_seed = 2;
  unsigned int test_seed =  0; //time(NULL);
  if (argc >= 2) model_seed = atoi(argv[1]);
  if (argc >= 3) test_seed = atoi(argv[2]);

  Proctor::readModels("/home/justin/Documents/benchmark/db", 1814, model_seed);
  //detector.enableVisualization();
  proctor.train(detector);
  proctor.test(detector, test_seed);
  proctor.printResults(detector);

  while (true) {}

  return 0;
}


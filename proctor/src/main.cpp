#include "proctor/detector.h"
#include "proctor/proctor.h"
#include <omp.h>

pcl::proctor::Detector detector;
pcl::proctor::Proctor proctor;

int main(int argc, char **argv) {
  unsigned int model_seed = 2;
  unsigned int test_seed = time(NULL);
  if (argc >= 2) model_seed = atoi(argv[1]);
  if (argc >= 3) test_seed = atoi(argv[2]);

  pcl::proctor::Proctor::readModels("/home/pabbeel/wh/benchmark/db", 1814, model_seed);
  detector.enableVisualization();
  proctor.train(detector);
  proctor.test(detector, test_seed);
  proctor.printResults(detector);

  return 0;
}

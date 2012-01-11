#include <boost/thread.hpp>

#include <QApplication>

#include "proctor/detector.h"
#include "proctor/detector_visualizer.h"
#include "proctor/proctor.h"
#include "proctor/scanning_model_source.h"
#include "proctor/basic_proposer.h"

using pcl::proctor::Detector;
//using pcl::proctor::ProctorMPI;
using pcl::proctor::Proctor;
using pcl::proctor::ScanningModelSource;
using pcl::proctor::BasicProposer;

struct run_proctor
{
  DetectorVisualizer *vis_;

  run_proctor(DetectorVisualizer &vis)
  {
    vis_ = &vis;
  }

  void operator()()
  {
    unsigned int model_seed = 2;
    unsigned int test_seed = 0; //time(NULL);

    Detector detector;
    Proctor proctor;

    BasicProposer::Ptr proposer(new BasicProposer);
    detector.setProposer(proposer);

    cout << "Secondary: " << boost::this_thread::get_id() << endl;
    ScanningModelSource model_source("Princeton", "/home/justin/Documents/benchmark/db");
    model_source.loadModels();

    detector.enableVisualization(vis_);

    proctor.setModelSource(&model_source);
    proctor.train(detector);
    proctor.test(detector, test_seed);
    proctor.printResults(detector);

    sleep(100000);
  }
};

int main(int argc, char **argv) {

  //if (argc >= 2)
  //{
    //model_seed = atoi(argv[1]);
  //}

  //if (argc >= 3)
  //{
    //test_seed = atoi(argv[2]);
  //}
  QApplication app (argc, argv); 

  DetectorVisualizer v;
  v.show ();

  cout << "Main: " << boost::this_thread::get_id() << endl;
  run_proctor x(v);
  boost::thread proctor_thread(x);

  return (app.exec ());
}


#include <boost/thread.hpp>

#include <QApplication>

#include "proctor/detector.h"
#include "proctor/detector_visualizer.h"
#include "proctor/proctor.h"
#include "proctor/scanning_model_source.h"
#include "proctor/primitive_model_source.h"
#include "proctor/basic_proposer.h"
#include "proctor/registration_proposer.h"

#include "proctor/uniform_sampling_wrapper.h"
#include "proctor/harris_wrapper.h"

#include "proctor/fpfh_wrapper.h"
#include "proctor/shot_wrapper.h"
#include "proctor/feature_wrapper.h"

#include <pcl/features/feature.h>
#include <pcl/features/fpfh.h>

#include <Eigen/Dense>

using pcl::proctor::Detector;
//using pcl::proctor::ProctorMPI;
using pcl::proctor::Proctor;
using pcl::proctor::ScanningModelSource;
using pcl::proctor::PrimitiveModelSource;
using pcl::proctor::Proposer;
using pcl::proctor::BasicProposer;
using pcl::proctor::RegistrationProposer;
using pcl::proctor::UniformSamplingWrapper;
using pcl::proctor::HarrisWrapper;
using pcl::proctor::KeypointWrapper;
using pcl::proctor::FeatureWrapper;
using pcl::proctor::FPFHWrapper;
using pcl::proctor::SHOTWrapper;

struct run_proctor
{
  DetectorVisualizer *vis_;

  run_proctor(DetectorVisualizer &vis)
  {
    vis_ = &vis;
  }

  void operator()()
  {
    //unsigned int model_seed = 2;
    unsigned int test_seed = 0; //time(NULL);

    /* Configure Detector */
    Detector detector;

    std::vector<Proposer::Ptr> proposers;
    proposers.push_back(BasicProposer::Ptr(new BasicProposer));
    proposers.push_back(RegistrationProposer::Ptr(new RegistrationProposer));
    detector.setProposers(proposers);

    KeypointWrapper::Ptr us_wrap (new UniformSamplingWrapper);
    KeypointWrapper::Ptr harris_wrap (new HarrisWrapper);
    detector.setKeypointWrapper(us_wrap);

    FeatureWrapper::Ptr fpfh_wrap (new FPFHWrapper);
    FeatureWrapper::Ptr shot_wrap (new SHOTWrapper);
    detector.setFeatureEstimator(shot_wrap);

    detector.enableVisualization(vis_);

    // Configure Proctor
    Proctor proctor;

    ScanningModelSource model_source("princeton", "/home/justin/Documents/benchmark/db");
    model_source.loadModels();

    //PrimitiveModelSource model_source("primitive");
    //model_source.loadModels();

    proctor.setModelSource(&model_source);
    proctor.train(detector);
    proctor.test(detector, test_seed);
    proctor.printResults(detector);

    sleep(100000);
  }
};

int main(int argc, char **argv)
{

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

  run_proctor x(v);
  boost::thread proctor_thread(x);

  return (app.exec ());
}


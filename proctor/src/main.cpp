#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <QApplication>

#include "proctor/detector.h"
#include "proctor/detector_visualizer.h"
#include "proctor/proctor.h"
#include "proctor/scanning_model_source.h"
#include "proctor/primitive_model_source.h"

#include "proctor/basic_proposer.h"
#include "proctor/inverse_bag_proposer.h"
#include "proctor/threshold_bag_proposer.h"
#include "proctor/registration_proposer.h"
#include "proctor/hough_proposer.h"
#include "proctor/soft_hough_proposer.h"

#include "proctor/uniform_sampling_wrapper.h"
#include "proctor/harris_wrapper.h"

#include "proctor/fpfh_wrapper.h"
#include "proctor/shot_wrapper.h"
#include "proctor/si_wrapper.h"
#include "proctor/3dsc_wrapper.h"
#include "proctor/feature_wrapper.h"

#include <pcl/features/feature.h>
#include <pcl/features/fpfh.h>

#include <Eigen/Dense>

using pcl::proctor::Detector;
using pcl::proctor::Proctor;
using pcl::proctor::ScanningModelSource;
using pcl::proctor::PrimitiveModelSource;

using pcl::proctor::Proposer;
using pcl::proctor::BasicProposer;
using pcl::proctor::InverseBagProposer;
using pcl::proctor::ThresholdBagProposer;
using pcl::proctor::RegistrationProposer;
using pcl::proctor::HoughProposer;
using pcl::proctor::SoftHoughProposer;

using pcl::proctor::KeypointWrapper;
using pcl::proctor::UniformSamplingWrapper;
using pcl::proctor::HarrisWrapper;

using pcl::proctor::FeatureWrapper;
using pcl::proctor::FPFHWrapper;
using pcl::proctor::SHOTWrapper;
using pcl::proctor::SIWrapper;
using pcl::proctor::ShapeContextWrapper;

struct run_proctor
{
  DetectorVisualizer *vis_;

  run_proctor()
  {
    vis_ = NULL;
  }

  run_proctor(DetectorVisualizer &vis)
  {
    vis_ = &vis;
  }

  void operator()()
  {
    //unsigned int model_seed = 2;
    unsigned int test_seed = 0; //time(NULL);

    // Gets rid of warnings
    //pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::vector<FeatureWrapper::Ptr> features;
    FeatureWrapper::Ptr fpfh_wrap (new FPFHWrapper);
    features.push_back(fpfh_wrap);
    FeatureWrapper::Ptr shot_wrap (new SHOTWrapper);
    //features.push_back(shot_wrap);
    FeatureWrapper::Ptr si_wrap (new SIWrapper);
    //features.push_back(si_wrap);
    FeatureWrapper::Ptr shape_context_wrapper (new ShapeContextWrapper);
    //features.push_back(shape_context_wrapper);

    std::vector<KeypointWrapper::Ptr> keypoints;
    KeypointWrapper::Ptr us_wrap (new UniformSamplingWrapper);
    keypoints.push_back(us_wrap);
    KeypointWrapper::Ptr harris_wrap (new HarrisWrapper);
    //keypoints.push_back(harris_wrap);

    std::vector<Proposer::Ptr> proposers;
    //proposers.push_back(BasicProposer::Ptr(new BasicProposer));
    //proposers.push_back(InverseBagProposer::Ptr(new InverseBagProposer));
    ThresholdBagProposer::Ptr threshold_proposer(new ThresholdBagProposer);
    threshold_proposer->setThreshold(5);
    proposers.push_back(threshold_proposer);
    //proposers.push_back(RegistrationProposer::Ptr(new RegistrationProposer));

    //HoughProposer::Ptr hough(new HoughProposer());
    //hough->num_angles_ = 60;
    //hough->bins_ = 10;
    //proposers.push_back(hough);
    //SoftHoughProposer::Ptr soft_hough(new SoftHoughProposer());
    //soft_hough->num_angles_ = 60;
    //soft_hough->bins_ = 10;
    //proposers.push_back(soft_hough);

    for (unsigned int i = 0; i < features.size(); i++)
    {
      for (unsigned int j = 0; j < keypoints.size(); j++)
      {
        for (unsigned int k = 0; k < proposers.size(); k++)
        {

          // Configure Detector
          Detector detector;

          // Enable visualization if available
          if (vis_)
            detector.enableVisualization(vis_);

          // Get current parameters
          FeatureWrapper::Ptr feature = features[i];
          KeypointWrapper::Ptr keypoint = keypoints[j];
          Proposer::Ptr proposer = proposers[k];

          std::cout << boost::format("Evaluating with feature: %|30t| %s") % feature->name_ << std::endl;
          std::cout << boost::format("Evaluating with keypoint: %|30t| %s") % keypoint->name_ << std::endl;

          std::vector<Proposer::Ptr> input_proposers;
          input_proposers.push_back(proposer);

          detector.setProposers(input_proposers);

          detector.setKeypointWrapper(keypoint);

          detector.setFeatureEstimator(feature);

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
        }
      }
    }
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

  bool enable_vis = false;
  if (enable_vis)
  {
    QApplication app (argc, argv); 

    DetectorVisualizer v;
    v.show ();

    run_proctor x(v);
    boost::thread proctor_thread(x);

    return (app.exec ());
  }
  else
  {
    run_proctor no_vis;
    boost::thread proctor_thread(no_vis);
    proctor_thread.join();

    return 0;
  }
}


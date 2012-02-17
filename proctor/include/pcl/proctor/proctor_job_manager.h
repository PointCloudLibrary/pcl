#ifndef PROCTOR_JOB_MANAGER_H_
#define PROCTOR_JOB_MANAGER_H_

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include "proctor/timer.h"

namespace pcl
{

  namespace proctor
  {

    class ScanningModelSource;

    class ConfusionMatrix;

    class Detector;

    struct Scene {
      std::string id;
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud;

      Scene(std::string id, pcl::PointCloud<pcl::PointNormal>::Ptr cloud) : id(id), cloud(cloud) {};
    };

    class Proctor {
      public:
        /** a model from the dataset plus metadata */

        enum TimerBin {
          OBTAIN_CLOUD_TRAINING,
          OBTAIN_CLOUD_TESTING,
          DETECTOR_TRAIN,
          DETECTOR_TEST,
          NUM_BINS
        };

        /** return a random indices list; use srand() to influence the output */
        static IndicesPtr randomSubset(int n, int r);


        /** load/generate training data and pass to detector */
        virtual void
        train(Detector &detector);

        /** generate testing data and pass to detector; this populates scenes, confusion, and distance */
        virtual double
        test(Detector &detector, unsigned int seed);

        /** compute and print the precision and recall data */
        void
        printPrecisionRecall();

        /** compute and print classifier performance data */
        void
        printClassifierStats();

        /** print the timing data */
        void
        printTimer();

        void
        printConfusionMatrix(ConfusionMatrix &matrix);

        /** print the results of testing */
        virtual void
        printResults(Detector &detector);

        /** the timer */
        Timer<NUM_BINS> timer;

        void
        setModelSource(ScanningModelSource *source)
        {
          this->source_ = source;
        }

        void
        setNumModels(int num_models)
        {
          num_models_ = num_models;
        }

        int
        getNumModels()
        {
          return num_models_;
        }

        void
        setNumTrials(int num_trials)
        {
          num_trials_ = num_trials;
        }

        int
        getNumTrials()
        {
          return num_trials_;
        }

      protected:
        ScanningModelSource *source_;

        std::vector<std::string> model_ids_;

        int num_models_;
        int num_trials_;
    };

  }

}

#endif

#ifndef PROCTOR_H_
#define PROCTOR_H_

#include <pcl/pcl_base.h>

#include <vtkPolyData.h>

#include "proctor/config.h"
#include "proctor/detector.h"
#include "proctor/scanner.h"
#include "proctor/timer.h"

namespace pcl {

  namespace proctor {

    class ScanningModelSource;

    struct Scene {
      std::string id;
      PointCloud<PointNormal>::Ptr cloud;

      Scene(std::string id, PointCloud<PointNormal>::Ptr cloud) : id(id), cloud(cloud) {};
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
        virtual void train(Detector &detector);

        /** generate testing data and pass to detector; this populates scenes, confusion, and distance */
        virtual void test(Detector &detector, unsigned int seed);

        /** compute and print the precision and recall data */
        void printPrecisionRecall();

        /** compute and print classifier performance data */
        void printClassifierStats();

        /** print the timing data */
        void printTimer();

        void printConfusionMatrix(std::map<std::string, std::map<std::string, int> > &guesses);

        /** print the results of testing */
        virtual void printResults(Detector &detector);

        // parameters for creating registered point cloud
        static const float theta_start;
        static const float theta_step;
        static const int theta_count;
        static const float phi_start;
        static const float phi_step;
        static const int phi_count;

        // parameters for test scans
        static const float theta_min;
        static const float theta_max;
        static const float phi_min;
        static const float phi_max;

        /** database of models and metadata */
        static Model models[Config::num_models];

        /** the exact parameters used during test() */
        Scanner::Scan scenes[Config::num_trials];

        /** histogram of [scene model][detector guess] */
        int confusion[Config::num_models][Config::num_models];

        /** detector's classification similarity ratings [trial][model candidate] */
        float classifier[Config::num_trials][Config::num_models];

        /** detector's registration distance ratings [trial][model candidate] */
        double registration[Config::num_trials][Config::num_models];

        /** total number of correct guesses */
        int trace;

        /** the timer */
        Timer<NUM_BINS> timer;

        void setModelSource(ScanningModelSource *source) {
          this->source_ = source;
        }

      protected:
        ScanningModelSource *source_;

        std::vector<std::string> model_ids;
    };

  }

}

#endif

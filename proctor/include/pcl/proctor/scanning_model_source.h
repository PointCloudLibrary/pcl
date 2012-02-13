#ifndef SCANNING_MODEL_SOURCE_H
#define SCANNING_MODEL_SOURCE_H

#include <vtkPolyData.h>

#include <boost/random.hpp>

#include "proctor/model_source.h"
#include "proctor/scanner.h"

#include <pcl/pcl_base.h>

namespace pcl
{

  namespace proctor
  {

    class ScanningModelSource : public ModelSource {
      public:
        ScanningModelSource(std::string name, std::string dir) : ModelSource(name, dir)
        {
          const float theta_start = M_PI / 12;
          const float theta_step = 0.0f;
          const int theta_count = 1;
          const float phi_start = 0.0f;
          const float phi_step = M_PI / 6;
          const int phi_count = 12;
          const float theta_min = 0.0f;
          const float theta_max = M_PI / 6;
          const float phi_min = 0.0f;
          const float phi_max = M_PI * 2;
          rng_.seed(0);

          resetTestGenerator();
        }

        IndicesPtr
        randomSubset(int n, int r);

        virtual void
        loadModels();

        virtual void
        getModelIDs(std::vector<std::string> &output);

        virtual
        PointCloud<PointNormal>::Ptr getTrainingModel(std::string model_id);

        virtual
        PointCloud<PointNormal>::Ptr getTestModel(std::string model_id);

        void resetTestGenerator();

      protected:
        std::map<std::string, Model> models_;
        boost::mt19937 rng_;
        boost::uniform_real<float> theta_u;
        boost::uniform_real<float> phi_u;

        boost::variate_generator<boost::mt19937&, boost::uniform_real<float> >* theta_gen;
        boost::variate_generator<boost::mt19937&, boost::uniform_real<float> >* phi_gen;
    };

  }
}

#endif

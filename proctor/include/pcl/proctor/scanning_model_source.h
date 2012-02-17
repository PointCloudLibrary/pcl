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
          rng_.seed(0);

          resetTestGenerator();
        }

        IndicesPtr
        randomSubset(int n, int r);

        void
        pickUniqueModels(std::vector<int>& ids);

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

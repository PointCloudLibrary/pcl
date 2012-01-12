#ifndef SCANNING_MODEL_SOURCE_H
#define SCANNING_MODEL_SOURCE_H

#include <vtkPolyData.h>

#include <boost/random.hpp>

#include "proctor/model_source.h"
#include "proctor/scanner.h"

namespace pcl {

  namespace proctor {

    class ScanningModelSource : public ModelSource {
      public:
        ScanningModelSource(std::string name, std::string dir) : ModelSource(name, dir)
        {}

        virtual void loadModels();

        virtual void getModelIDs(std::vector<std::string> &output);

        virtual PointCloud<PointNormal>::Ptr getTrainingModel(std::string model_id);

        virtual PointCloud<PointNormal>::Ptr getTestModel(std::string model_id);

      private:
        std::map<std::string, Model> models;
        boost::mt19937 rng;
    };

  }
}

#endif

#ifndef PRIMITIVE_MODEL_SOURCE_H
#define PRIMITIVE_MODEL_SOURCE_H

#include <vtkPolyData.h>

#include "proctor/scanning_model_source.h"

namespace pcl
{

  namespace proctor
  {

    class PrimitiveModelSource : public ScanningModelSource {
      public:
        PrimitiveModelSource(std::string name) : ScanningModelSource(name, "")
        {}

        void
        addDefaultModel(std::string name, vtkAlgorithmOutput *mesh);

        virtual void
        loadModels();

        //virtual void
        //getModelIDs(std::vector<std::string> &output);

        //virtual
        //PointCloud<PointNormal>::Ptr getTrainingModel(std::string model_id);

        //virtual
        //PointCloud<PointNormal>::Ptr getTestModel(std::string model_id);

      private:
        //std::map<std::string, Model> models_;
        //boost::mt19937 rng_;
    };

  }
}

#endif

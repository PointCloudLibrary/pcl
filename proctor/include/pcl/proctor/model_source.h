#ifndef MODEL_SOURCE_H
#define MODEL_SOURCE_H

#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl {

  namespace proctor {

    class ModelSource {
      public:
        ModelSource(std::string name, std::string dir) : name(name), dir(dir)
        {}

        virtual void loadModels() = 0;

        virtual void getModelIDs(std::vector<std::string> &output) = 0;

        virtual PointCloud<PointNormal>::Ptr getTrainingModel(std::string model_id) = 0;

        virtual PointCloud<PointNormal>::Ptr getTestModel(std::string model_id) = 0;

      protected:
        std::string name;
        std::string dir;
      private:
    };
  }
}

#endif


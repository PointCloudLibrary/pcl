#ifndef PROCTOR_WORKER_H_
#define PROCTOR_WORKER_H_

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include "proctor/timer.h"
#include "proctor/proctor.h"

namespace pcl
{

  namespace proctor
  {

    class ProctorWorker
    {
      public:
        void
        start();

        std::string
        receivedJob(std::string message);

        std::string
        handleTrain(std::string model_id, Detector &detector);

        std::string
        handleTest(std::string model_id, Detector &detector, std::vector<std::string> &database_ids);
    };

  }

}

#endif

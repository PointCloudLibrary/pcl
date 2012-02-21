#include "proctor/proctor_job_manager.h"

#include <cstdio>
#include <sstream>
#include <algorithm>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>

#include "proctor/detector.h"
#include "proctor/scanning_model_source.h"
#include "proctor/confusion_matrix.h"

#include "proctor/fpfh_wrapper.h"
#include "proctor/shot_wrapper.h"
#include "proctor/si_wrapper.h"
#include "proctor/3dsc_wrapper.h"
#include "proctor/feature_wrapper.h"

#include "proctor/uniform_sampling_wrapper.h"
#include "proctor/harris_wrapper.h"
#include "proctor/proposer.h"


#ifdef _MSC_VER
# define snprintf _snprintf
#endif

namespace pcl
{
  namespace proctor
  {
    void
    ProctorJobManager::train(Detector &detector)
    {
      source_->getModelIDs(model_ids_);

      cout << "Proctor beginning training" << endl;
      cout << "[models]" << endl;

      const int num_model_ids = std::min((int) model_ids_.size(), num_models_);

      // Because of the LRU broker, we can fire off all our requests
      // then wait for all the responses separately.
      for (int mi = 0; mi < num_model_ids; mi++) {
        std::string model_id = model_ids_[mi];

        JobRequest req;
        req.type = JobRequest::TRAIN;
        req.model_id = model_id;
        req.detector = &detector;

        // Serialize job request
        std::stringstream serialized;
        {
          boost::archive::text_oarchive oa(serialized);
          oa << req;
        }

        s_sendmore(sender_, ""); // Required because we are using DEALER, not REQ
        s_send(sender_, serialized.str());
      }

      for (int i = 0; i < num_model_ids; i++)
      {
        std::string empty = s_recv(sender_);
        assert(empty == "");
        std::string message = s_recv(sender_);
        cout << "Received: " << message << endl;
      }
      cout << "Proctor finished training" << endl;
    }

    double
    ProctorJobManager::test(Detector &detector, unsigned int seed)
    {
      source_->resetTestGenerator();
      source_->getModelIDs(model_ids_);

      ConfusionMatrix confusion_matrix;

      int num_model_ids = std::min((int) model_ids_.size(), num_models_);
      if (num_model_ids == 0)
        assert(false);
      for (int ni = 0; ni < num_trials_; ni++) {
        std::string truth_id = model_ids_[ni % num_model_ids];

        JobRequest req;
        req.type = JobRequest::TEST;
        req.model_id = truth_id;
        req.detector = &detector;
        req.database_ids.resize(num_model_ids);
        std::copy(model_ids_.begin(), model_ids_.begin() + num_model_ids, req.database_ids.begin());

        // Serialize job request
        std::stringstream serialized;
        {
          boost::archive::text_oarchive oa(serialized);
          oa << req;
        }

        s_sendmore(sender_, ""); // Required because we are using DEALER, not REQ
        s_send(sender_, serialized.str());

      }

      // Receive results
      for (int i = 0; i < num_model_ids; i++)
      {
        std::string empty = s_recv(sender_);
        assert(empty == "");
        std::string message = s_recv(sender_);

        // Deserialize response
        std::stringstream serialized(message);
        QueryResponse res;
        {
          boost::archive::text_iarchive ia(serialized);
          ia >> res;
        }

        confusion_matrix.increment(res.truth_id, res.guessed_id);
        cout << "Received: " << boost::trim_copy( message) << endl;
      }
      cout << "Proctor finished testing" << endl;

      printConfusionMatrix(confusion_matrix);

      return confusion_matrix.trace();
    }
  }
}

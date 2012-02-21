#include "proctor/proctor_worker.h"
#include "proctor/proctor_job_manager.h"
#include "proctor/scanning_model_source.h"
#include "proctor/detector.h"
#include "proctor/proposer.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "proctor/zhelpers.hpp"
#include <sstream>

namespace pcl
{
  namespace proctor
  {
    void
    ProctorWorker::start()
    {
      zmq::context_t context(1);

      zmq::socket_t receiver(context, ZMQ_REQ);
      receiver.connect("tcp://localhost:5557");

      s_send (receiver, "READY");
      while (1)
      {
        zmq::message_t message;
        std::cout << "Begin Receiving..." << std::endl;

        std::string address = s_recv (receiver);
        {
          std::string empty = s_recv (receiver);
          assert (empty.size() == 0);
        }

        std::string a = s_recv(receiver);

        std::string response = receivedJob(a);

        s_sendmore (receiver, address);
        s_sendmore (receiver, "");
        s_send     (receiver, response);
      }

    }

    std::string
    ProctorWorker::receivedJob(std::string message)
    {
      JobRequest req;
      {
        std::stringstream test;
        test.str(message);
        boost::archive::text_iarchive ia(test);
        ia >> req;
      }
      std::string model_id = req.model_id;
      Detector *detector = req.detector;

      if (req.type == JobRequest::TRAIN)
      {
        std::cout << "Received Train Request" << std::endl;
        return handleTrain(model_id, *detector);
      }
      else if (req.type == JobRequest::TEST)
      {
        std::cout << "Received Test Request" << std::endl;
        return handleTest(model_id, *detector, req.database_ids);
      }
      else
        assert(false);
    }

    std::string
    ProctorWorker::handleTrain(std::string model_id, Detector &detector)
    {
      std::cout << "Model ID: " << model_id << std::endl;
      std::cout << detector << std::endl;

      ScanningModelSource model_source("princeton", "/home/justin/Documents/benchmark");
      model_source.loadModels();

      std::cout << "Begin scanning model: " << model_id << std::endl;
      Scene *scene = new Scene(model_id, model_source.getTrainingModel(model_id));
      std::cout << "Finished scanning model: " << model_id << std::endl;

      std::cout << "Begin training model: " << model_id << std::endl;
      detector.train(*scene);
      std::cout << "Finished training model: " << model_id << std::endl;
      std::cout << std::endl;

      return "TRAIN_DONE";
    }

    std::string
    ProctorWorker::handleTest(std::string model_id, Detector &detector, std::vector<std::string> &database_ids)
    {
      ScanningModelSource model_source("princeton", "/home/justin/Documents/benchmark");
      model_source.loadModels();

      for (size_t i = 0; i < database_ids.size(); i++)
      {
        Scene *scene = new Scene(database_ids[i], model_source.getTrainingModel(database_ids[i]));
        detector.train(*scene);
      }

      std::cout << "Done loading database" << std::endl;
      PointCloud<PointNormal>::Ptr test_cloud = model_source.getTestModel(model_id);
      Scene test_scene(model_id, test_cloud);

      QueryResponse res;
      res.truth_id = model_id;
      res.guessed_id = detector.query(test_scene);

      std::stringstream serialized;
      {
        boost::archive::text_oarchive oa(serialized);
        oa << res;
      }

      return serialized.str();
    }
  }
}

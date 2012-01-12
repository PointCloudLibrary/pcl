#include <string>
#include <sstream>
#include <boost/serialization/string.hpp>
#include <boost/unordered_map.hpp>

#include "proctor/proctor_mpi.h"
#include "proctor/detector.h"
#include "proctor/scanning_model_source.h"

namespace mpi = boost::mpi;

namespace pcl
{
  namespace proctor
  {
    int REQ_FEATURE = 0;
    int FIN_FEATURE = 1;


    int Job::next_job_id = 0;

    struct request_equal_to : std::binary_function<mpi::request, mpi::request, bool> {
      bool operator()(mpi::request const& x, mpi::request const&y) const
      {
        return x.m_data.get() == y.m_data.get();
      }
    };

    struct request_hash : std::unary_function<mpi::request, int> {
      std::size_t operator()(mpi::request const &x) const
      {
        return (std::size_t) x.m_data.get();
      }
    };

    void range(int range, std::deque<int>& output) {
      output.clear();
      for (int i = 0; i < range; i++) {
        output.push_back(i);
      }
    }

    void ProctorMPI::train(Detector &detector) {
      cout << "ProctorMPI beginning training" << endl;
      cout << "[models]" << endl;
      broadcast_jobs(TRAIN, TRAIN_FINISHED, detector, &ProctorMPI::generate_feature, &ProctorMPI::update_nothing);
    }

    void ProctorMPI::test(Detector &detector, unsigned int seed) {
      cout << "Loading all models into all nodes" << endl;
      for (int mi = 0; mi < Config::num_models; mi++) {
        Scene *scene = new Scene(mi, source_->getTrainingModel(mi));
        detector.train(*scene);
      }

      srand(seed);
      const float theta_scale = (theta_max - theta_min) / RAND_MAX;
      const float phi_scale = (phi_max - phi_min) / RAND_MAX;

      // prepare test vectors in advance
      for (int ni = 0; ni < Config::num_trials; ni++) {
        scenes[ni].mi = rand() % Config::num_models;
        scenes[ni].theta = theta_min + rand() * theta_scale;
        scenes[ni].phi = phi_min + rand() * phi_scale;
      }

      // run the tests
      trace = 0;
      memset(confusion, 0, sizeof(confusion));

      broadcast_jobs(TEST, TEST_FINISHED, detector, &ProctorMPI::run_test, &ProctorMPI::update_results);
    }

    int
    ProctorMPI::generate_feature(int model_index, Detector &detector) {
      cout << "Begin scanning model " << model_index << " (" << models[model_index].id << ")" << endl;
      Scene *scene = new Scene(model_index, source_->getTrainingModel(model_index));
      cout << "Finished scanning model " << model_index << " (" << scene->id << ")" << endl;

      cout << "Begin training model " << model_index << " (" << scene->id << ")" << endl;
      // TODO Time the training phase
      detector.train(*scene);
      cout << "Finished training model " << model_index << " (" << scene->id << ")" << endl;
      cout << endl;

      return 0;
    }

    int
    ProctorMPI::run_test(int scene_index, Detector &detector) {
      cout << "Running Test for Scene Index: " << scene_index << endl;

      cout << "[test " << scene_index << "]" << endl;
      timer.start();
      PointCloud<PointNormal>::Ptr scene = Scanner::getCloud(scenes[scene_index], models[scenes[scene_index].mi]);
      timer.stop(OBTAIN_CLOUD_TESTING);
      cout << "scanned model " << scenes[scene_index].mi << endl;

      timer.start();
      int guess;
      try {
        guess = detector.query(scene, classifier[scene_index], registration[scene_index]);
      } catch (exception &e) {
        cout << "Detector exception" << endl;
        cout << e.what() << endl;
        guess = 0;
        memset(classifier[scene_index], 0, sizeof(classifier[scene_index]));
        memset(registration[scene_index], 0, sizeof(registration[scene_index]));
      }
      timer.stop(DETECTOR_TEST);
      cout << "detector guessed " << guess << endl;

      return guess;
    }

    int
    ProctorMPI::update_results(Job &job) {
      confusion[scenes[job.model_index].mi][job.guess]++;
      if (job.guess == scenes[job.model_index].mi) trace++;
      return 0;
    }

    int
    ProctorMPI::update_nothing(Job &job) {
      return 0;
    }

    void ProctorMPI::broadcast_jobs(JobType type, JobType type_finished, Detector &detector, JobHandlerFn handler, PostJobHandlerFn post_handler) {
      if (world.rank() == 0) {
        // Master
        std::deque<int> model_indices;
        range(Config::num_models, model_indices);

        boost::unordered_map<int, Job*> requests_values;
        std::vector<mpi::request> requests;
        for (int node = 1; node < world.size(); node++) {
          if (!(model_indices.empty())) {
            cout << "SENDING to " << node << "\t Model: " << model_indices.front() << endl;

            Job j(type, model_indices.front());

            int job_id = Job::next_job_id++;
            world.isend(node, job_id, j);
            model_indices.pop_front();

            Job* ret_value = new Job();
            mpi::request req = world.irecv(node, job_id, *ret_value);
            requests_values[job_id] =  ret_value;
            requests.push_back(req);
          }
          else {
            break;
          }
        }

        while(!(requests.empty())) {
          std::pair< mpi::status, vector<mpi::request>::iterator > done = mpi::wait_any(requests.begin(), requests.end());
          mpi::status st = done.first;
          vector<mpi::request>::iterator it = done.second;
          requests.erase(it);

          int sending_node = st.source();

          Job *data = requests_values[st.tag()];

          CALL_MEMBER_FN(*this, post_handler)(*data);

          cout << "RECEIVED FINISH from: " << sending_node << endl;
          if (!(model_indices.empty())) {
            cout << "RE-SENDING to " << sending_node << "\tModel: " << model_indices.front() << endl;

            Job j(type, model_indices.front());

            int job_id = Job::next_job_id++;
            world.isend(sending_node, job_id, j);

            model_indices.pop_front();

            Job* ret_value = new Job();
            mpi::request req = world.irecv(sending_node, job_id, *ret_value);
            requests_values[job_id] =  ret_value;
            requests.push_back(req);
          }
        }

        // Next Phase
        for (int node = 1; node < world.size(); node++) {
          Job j(type_finished, 0);
          world.send(node, REQ_FEATURE, j);
        }
      } else {
        // Slave
        while (true) {
          Job job;
          mpi::status st = world.recv(0, mpi::any_tag, job);
          if (job.type == type) {
            cout << "Received feature request for model: " << job.model_index << endl;
            int value = CALL_MEMBER_FN(*this, handler)(job.model_index, detector);
            Job response;
            response.type = job.type;
            response.model_index = job.model_index;
            response.guess = value;
            world.send(0, st.tag(), response);
          } else if (job.type == type_finished) {
            break;
          } else {
            assert(false);
          }
        }
      }
    }

    void ProctorMPI::printResults(Detector &detector) {
      if (world.rank() == 0) {
        Proctor::printResults(detector);
      }
    }

  }
}

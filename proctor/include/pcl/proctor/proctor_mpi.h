#ifndef PROCTOR_MPI_H_
#define PROCTOR_MPI_H_

#include "proctor/proctor.h"
#include <boost/mpi.hpp>

#define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(ptrToMember)) 

namespace pcl {

  namespace proctor {

    enum JobType {
      TRAIN = 1337,
      TRAIN_FINISHED,
      TEST,
      TEST_RESPONSE,
      TEST_FINISHED
    };

    struct Job {
      JobType type;
      int model_index;
      int guess;

      Job() { }
      Job(JobType type, int model_index) : type(type), model_index(model_index) { }


      static int next_job_id;
    private:

      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
          ar & type;
          ar & model_index;
          ar & guess;
        }
    };

    class ProctorMPI : public Proctor {
    public:
      typedef int (ProctorMPI::*JobHandlerFn)(int model_index, Detector &detector);

      typedef int (ProctorMPI::*PostJobHandlerFn)(Job &job);

      virtual void train(Detector &detector);

      virtual void test(Detector &detector, unsigned int seed);

      int generate_feature(int model_index, Detector &detector);

      int run_test(int scene_index, Detector &detector);

      int update_results(Job &job);

      int update_nothing(Job &job);

      virtual void broadcast_jobs(JobType type, JobType type_finished, Detector &detector, JobHandlerFn handler, PostJobHandlerFn post_handler);

      virtual void printResults(Detector &detector);
    private:
      boost::mpi::environment env;
      boost::mpi::communicator world;
    };

  }
}

#endif

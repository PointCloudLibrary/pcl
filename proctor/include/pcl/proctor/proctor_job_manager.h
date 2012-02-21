#ifndef PROCTOR_JOB_MANAGER_H_
#define PROCTOR_JOB_MANAGER_H_

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include "proctor/timer.h"
#include "proctor/proctor.h"

#include <queue>

#include "zhelpers.hpp"
#include <boost/thread.hpp>

namespace pcl
{
  namespace proctor
  {
    struct JobRequest
    {
      enum Type {
        TRAIN,
        TEST
      } type;
      std::string model_id;
      Detector *detector;
      std::vector<std::string> database_ids;

      template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
      {
        ar & model_id;
        ar & detector;
        ar & type;
        ar & database_ids;
      }
    };

    struct QueryResponse
    {
      std::string truth_id;
      std::string guessed_id;

      template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
      {
        ar & truth_id;
        ar & guessed_id;
      }
    };

    struct startBroker
    {
      void operator()()
      {
        zmq::context_t context(1);
        zmq::socket_t frontend (context, ZMQ_ROUTER);
        zmq::socket_t backend (context, ZMQ_ROUTER);
        frontend.bind("ipc://frontend.ipc");
        backend.bind("tcp://*:5557");

        //  Logic of LRU loop
        //  - Poll backend always, frontend only if 1+ worker ready
        //  - If worker replies, queue worker as ready and forward reply
        //    to client if necessary
        //  - If client requests, pop next worker and send request to it
        //
        //  A very simple queue structure with known max size
        std::queue<std::string> worker_queue;

        while (1) {

          //  Initialize poll set
          zmq::pollitem_t items [] = {
            //  Always poll for worker activity on backend
            { backend,  0, ZMQ_POLLIN, 0 },
            //  Poll front-end only if we have available workers
            { frontend, 0, ZMQ_POLLIN, 0 }
          };
          if (worker_queue.size())
            zmq::poll (&items [0], 2, -1);
          else
            zmq::poll (&items [0], 1, -1);

          //  Handle worker activity on backend
          if (items [0].revents & ZMQ_POLLIN) {

            //  Queue worker address for LRU routing
            worker_queue.push(s_recv (backend));

            {
              //  Second frame is empty
              std::string empty = s_recv (backend);
              assert (empty.size() == 0);
            }

            //  Third frame is READY or else a client reply address
            std::string client_addr = s_recv (backend);

            //  If client reply, send rest back to frontend
            if (client_addr.compare("READY") != 0) {

              {
                std::string empty = s_recv (backend);
                assert (empty.size() == 0);
              }

              std::string reply = s_recv (backend);
              s_sendmore (frontend, client_addr);
              s_sendmore (frontend, "");
              s_send     (frontend, reply);
            }
          }
          if (items [1].revents & ZMQ_POLLIN) {

            //  Now get next client request, route to LRU worker
            //  Client request is [address][empty][request]
            std::string client_addr = s_recv (frontend);

            {
              std::string empty = s_recv (frontend);
              assert (empty.size() == 0);
            }

            std::string request = s_recv (frontend);

            std::string worker_addr = worker_queue.front();//worker_queue [0];
            worker_queue.pop();

            s_sendmore (backend, worker_addr);
            s_sendmore (backend, "");
            s_sendmore (backend, client_addr);
            s_sendmore (backend, "");
            s_send     (backend, request);
          }
        }
      }
    };

    class ProctorJobManager : public Proctor {
      public:
        ProctorJobManager () : context_(1), sender_(context_, ZMQ_DEALER)
        {
          boost::thread broker = startBroker();

          //zmq::context_t context(1);

          //// Socket to send messages on
          //zmq::socket_t sender(context, ZMQ_DEALER);
          sender_.connect("ipc://frontend.ipc");
        }

        void
        train(Detector &detector);

        double
        test(Detector &detector, unsigned int seed = 0);

      protected:

      private:
        zmq::context_t context_;
        zmq::socket_t sender_; //(context, ZMQ_DEALER);
    };

  }

}

#endif

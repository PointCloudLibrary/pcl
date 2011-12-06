#include "proctor/detector.h"
#include "proctor/proctor.h"
#include <omp.h>
#include <boost/mpi.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/serialization/string.hpp>
#include <boost/unordered_map.hpp>
namespace mpi = boost::mpi;
using namespace std;
//using boost:unordered_map;

pcl::proctor::Detector detector;
pcl::proctor::Proctor proctor;

void generate_features(std::vector<int>& model_indices);
void split_indices(int range, std::vector< std::vector<int> >& indices);
void range(int range, std::deque<int>& output);
pcl::PointCloud<pcl::proctor::Detector::Signature>::Ptr generate_feature(int model_index);

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

int REQ_FEATURE = 0;
int FIN_FEATURE = 1;

int main(int argc, char **argv) {
  mpi::environment env(argc, argv);
  mpi::communicator world;

  if (world.rank() == 0) {
    // Master
    std::deque<int> model_indices;
    range(Config::num_models, model_indices);

    boost::unordered_map<mpi::request, int*, request_hash, request_equal_to> requests_values;
    std::vector<mpi::request> requests;
    for (int node = 1; node < 3; node++) {
      if (!(model_indices.empty())) {
        cout << "SENDING to " << node << "\t Model: " << model_indices.front() << endl;
        world.isend(node, REQ_FEATURE, model_indices.front());
        model_indices.pop_front();

        int* ret_value = new int;
        *ret_value = 10;
        mpi::request req = world.irecv(node, FIN_FEATURE, *ret_value);
        requests_values[req] =  ret_value;
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

      int *data = requests_values[*it];

      cout << "RECEIVED FINISH from: " << sending_node << endl;
      cout << "Data: " << *data << endl;
      if (!(model_indices.empty())) {
        cout << "RE-SENDING to " << sending_node << "\tModel: " << model_indices.front() << endl;
        world.isend(sending_node, REQ_FEATURE, model_indices.front());

        model_indices.pop_front();

        int* ret_value = new int;
        *ret_value = 10;
        mpi::request req = world.irecv(sending_node, FIN_FEATURE, *ret_value);
        requests_values[req] =  ret_value;
        requests.push_back(req);
      }
    }

    //std::vector< std::vector<int> > vec;
    //split_indices(world.size(), vec);

    //std::vector<int> model_indices;
    //scatter(world, vec, model_indices, 0);
    //generate_features(model_indices);
  } else {
    // Slave
    while (true) {
      int model_index;
      world.recv(0, REQ_FEATURE, model_index);
      cout << "Received feature request for model: " << model_index << endl;
      pcl::PointCloud<pcl::proctor::Detector::Signature>::Ptr feature_cloud = generate_feature(model_index);
      int value = 1337;
      world.send(0, FIN_FEATURE, 1337);
    }
  }
  return 0;

  //unsigned int model_seed = 2;
  //unsigned int test_seed = time(NULL);
  //if (argc >= 2) model_seed = atoi(argv[1]);
  //if (argc >= 3) test_seed = atoi(argv[2]);

  //pcl::proctor::Proctor::readModels("/home/justin/Documents/benchmark/db", 1814, model_seed);
  ////detector.enableVisualization();
  //proctor.train(detector);
  //proctor.test(detector, test_seed);
  //proctor.printResults(detector);

  //return 0;
}

void range(int range, std::deque<int>& output) {
  output.clear();
  for (int i = 0; i < range; i++) {
    output.push_back(i);
  }
}

void split_indices(int range, std::vector< std::vector<int> >& indices) {
  /* Clear the output vector */
  indices.clear();

  for (int i = 0; i < range; i++) {
    std::vector<int>* vecForP = new std::vector<int>();
    indices.push_back(*vecForP);
  }

  for (int i = 0; i < Config::num_models; i++) {
    indices[i % range].push_back(i);
  }
}

void generate_features(std::vector<int>& model_indices) {
  unsigned int model_seed = 2;
  unsigned int test_seed = time(NULL);
  //if (argc >= 2) model_seed = atoi(argv[1]);
  //if (argc >= 3) test_seed = atoi(argv[2]);

  pcl::proctor::Proctor::readModels("/home/justin/Documents/benchmark/db", 1814, model_seed);
  //detector.enableVisualization();
  //proctor.train(detector);
  //proctor.test(detector, test_seed);
  //proctor.printResults(detector);
  for (unsigned int model_index = 0; model_index < model_indices.size(); model_index++) {
    std::cout << model_indices[model_index] << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud =  proctor.getFullPointCloud(model_index);
    pcl::IndicesPtr indices = detector.computeKeypoints(cloud);
    pcl::PointCloud<pcl::proctor::Detector::Signature>::Ptr features = detector.obtainFeatures(model_index, cloud, indices);
  }
}

pcl::PointCloud<pcl::proctor::Detector::Signature>::Ptr generate_feature(int model_index) {
  cout << "Generating features for model: " << model_index << endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud =  proctor.getFullPointCloud(model_index);
  pcl::IndicesPtr indices = detector.computeKeypoints(cloud);
  pcl::PointCloud<pcl::proctor::Detector::Signature>::Ptr features = detector.obtainFeatures(model_index, cloud, indices);
  cout << "Finished features for model: " << model_index << endl;
  return features;
}


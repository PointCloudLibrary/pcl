#ifndef PROPOSER_H
#define PROPOSER_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>
#include "proctor/detector.h"

namespace pcl {

  namespace proctor {

    class Proposer {
      public:
        typedef boost::shared_ptr<Proposer> Ptr;
        typedef boost::shared_ptr<const Proposer> ConstPtr;

        Proposer()
        {}

        virtual void
        getProposed(int max_num, Detector::Entry &query, std::map<std::string, Detector::Entry> &database, std::vector<std::string> &output) = 0;

      private:
    };

  }
}

#endif

#ifndef BASIC_PROPOSER_H
#define BASIC_PROPOSER_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include "proctor/detector.h"
#include "proctor/proposer.h"

namespace pcl {

  namespace proctor {

    class BasicProposer : public Proposer {
      public:
        typedef boost::shared_ptr<BasicProposer> Ptr;
        typedef boost::shared_ptr<const BasicProposer> ConstPtr;

        BasicProposer()
        {}

        void
        getProposed(int max_num, Detector::Entry &query, std::map<std::string, Detector::Entry> &database, std::vector<std::string> &output);

        double
        getVotes(Detector::Entry &query, Detector::Entry &match);

      private:
    };

  }
}

#endif

#ifndef BASIC_PROPOSER_H
#define BASIC_PROPOSER_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include "proctor/detector.h"
#include "proctor/proposer.h"

namespace pcl
{
  namespace proctor
  {

    class BasicProposer : public Proposer {
      public:
        typedef boost::shared_ptr<BasicProposer> Ptr;
        typedef boost::shared_ptr<const BasicProposer> ConstPtr;

        BasicProposer()
        {}

        void
        getProposed(int max_num, Entry &query, std::vector<std::string> &input, std::vector<std::string> &output);

        virtual double
        getVotes(Entry &query, Entry &match);

      private:
    };

  }
}

#endif

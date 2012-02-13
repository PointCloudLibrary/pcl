#ifndef INVERSE_BAG_PROPOSER_H
#define INVERSE_BAG_PROPOSER_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include "proctor/detector.h"
#include "proctor/basic_proposer.h"

namespace pcl
{
  namespace proctor
  {

    class InverseBagProposer : public BasicProposer {
      public:
        typedef boost::shared_ptr<InverseBagProposer> Ptr;
        typedef boost::shared_ptr<const InverseBagProposer> ConstPtr;

        InverseBagProposer()
        {}

        double
        getVotes(Entry &query, Entry &match);

      private:
    };

  }
}

#endif

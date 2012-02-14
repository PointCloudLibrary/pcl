#ifndef THRESHOLD_BAG_PROPOSER_H
#define THRESHOLD_BAG_PROPOSER_H

#include <boost/shared_ptr.hpp>
#include "proctor/detector.h"
#include "proctor/basic_proposer.h"

#include <vector>

namespace pcl
{
  namespace proctor
  {

    class ThresholdBagProposer : public BasicProposer {
      public:
        typedef boost::shared_ptr<ThresholdBagProposer> Ptr;
        typedef boost::shared_ptr<const ThresholdBagProposer> ConstPtr;

        ThresholdBagProposer()
        {}

        void
        preGetProposed(Entry &query, std::vector<std::string> &input);

        double
        getVotes(Entry &query, Entry &match);

        void
        setThreshold(double threshold)
        {
          threshold_ = threshold;
        }

      private:
        std::map< std::string, std::vector<float> > feature_distances;
        double mean;
        double std_dev;

        double threshold_;
    };

  }
}

#endif

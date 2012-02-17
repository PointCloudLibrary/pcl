#ifndef NORMAL_HOUGH_PROPOSER_H
#define NORMAL_HOUGH_PROPOSER_H

#include "proctor/hough_proposer.h"

#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/features/principal_curvatures.h"

namespace pcl
{
  namespace proctor
  {

    class NormalHoughProposer : public HoughProposer {
      public:
        typedef boost::shared_ptr<NormalHoughProposer> Ptr;
        typedef boost::shared_ptr<const NormalHoughProposer> ConstPtr;

        NormalHoughProposer(Detector *detector = NULL) : HoughProposer(detector)
        {}

        virtual void
        houghVote(Entry &query, Entry &target, bin_t& bins);
    };

  }
}

#endif

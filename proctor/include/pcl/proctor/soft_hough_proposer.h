#ifndef SOFT_HOUGH_PROPOSER_H
#define SOFT_HOUGH_PROPOSER_H

#include "proctor/hough_proposer.h"

namespace pcl
{
  namespace proctor
  {

    class SoftHoughProposer : public HoughProposer {
      public:
        typedef boost::shared_ptr<SoftHoughProposer> Ptr;
        typedef boost::shared_ptr<const SoftHoughProposer> ConstPtr;

        //typedef boost::multi_array<double, 3> bin_t;

        SoftHoughProposer(Detector *detector = NULL) : HoughProposer(detector)
        {}

        bool
        castVotes(Eigen::Vector3f& indices, bin_t& bins);
    };

  }
}

#endif

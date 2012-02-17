#ifndef SOFT_HOUGH_PROPOSER_H
#define SOFT_HOUGH_PROPOSER_H

#include "proctor/hough_proposer.h"

#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/features/principal_curvatures.h"

namespace pcl
{
  namespace proctor
  {

    class RadiusHoughProposer : public HoughProposer {
      public:
        typedef boost::shared_ptr<RadiusHoughProposer> Ptr;
        typedef boost::shared_ptr<const RadiusHoughProposer> ConstPtr;

        RadiusHoughProposer(Detector *detector = NULL) : HoughProposer(detector)
        {}

        virtual void
        houghVote(Entry &query, Entry &target, bin_t& bins);
    };

  }
}

#endif

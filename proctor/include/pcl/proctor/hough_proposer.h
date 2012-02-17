#ifndef HOUGH_PROPOSER_H
#define HOUGH_PROPOSER_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include "proctor/detector.h"
#include "proctor/proposer.h"

#include "boost/multi_array.hpp"

namespace pcl
{
  namespace proctor
  {

    class HoughProposer : public Proposer {
      public:
        typedef boost::shared_ptr<HoughProposer> Ptr;
        typedef boost::shared_ptr<const HoughProposer> ConstPtr;

        typedef boost::multi_array<double, 3> bin_t;

        HoughProposer(Detector *detector = NULL) : Proposer(detector), is_soft_vote_(false)
        {}

        void
        getProposed(int max_num, Entry &query, std::vector<std::string> &input, std::vector<std::string> &output);

        virtual void
        houghVote(Entry &query, Entry &target, bin_t& bins) = 0;

        virtual bool
        castVotes(Eigen::Vector3f& indices, bin_t& bins);

        bool
        softCastVotes(Eigen::Vector3f& indices, bin_t& bins);

        bool
        hardCastVotes(Eigen::Vector3f& indices, bin_t& bins);

        double
        angleBetween(Eigen::Vector3f a, Eigen::Vector3f b);

        void
        setSoftVote(bool is_soft_vote)
        {
          is_soft_vote_ = is_soft_vote_;
        }

      public:

        int bins_, num_angles_;

        // The number of features correspondences to vote for
        int num_correspondences;

      private:
        bool is_soft_vote_;
    };

  }
}

#endif

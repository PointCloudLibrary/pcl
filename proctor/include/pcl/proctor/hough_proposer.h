#ifndef HOUGH_PROPOSER_H
#define HOUGH_PROPOSER_H

#include <boost/shared_ptr.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "boost/multi_array.hpp"

#include <vector>

#include "proctor/detector.h"
#include "proctor/proposer.h"


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
        setSoftVote (bool is_soft_vote)
        {
          is_soft_vote_ = is_soft_vote;
        }

      public:

        int bins_, num_angles_;

        // The number of features correspondences to vote for
        int num_correspondences;

      private:
        bool is_soft_vote_;

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
          ar & boost::serialization::base_object<Proposer>( *this );
          ar & bins_;
          ar & num_angles_;
          ar & num_correspondences;
        }
    };

  }
}

//BOOST_CLASS_EXPORT(pcl::proctor::HoughProposer);

#endif

#include "proctor/hough_proposer.h"

#include <pcl/common/common.h>

#include <assert.h>
#include <algorithm>
#include "boost/array.hpp"
#include "boost/cstdlib.hpp"

namespace pcl
{

  namespace proctor
  {
    template <typename Array>
    void print(std::ostream& os, const Array& A)
    {
      typename Array::const_iterator i;
      os << "[";
      for (i = A.begin(); i != A.end(); ++i) {
        print(os, *i);
        if (boost::next(i) != A.end())
          os << ',';
      }
      os << "]";
    }

    void
    HoughProposer::getProposed(int max_num, Entry &query, std::vector<std::string> &input, std::vector<std::string> &output)
    {
      std::vector<std::string>::iterator database_it;

      std::map<std::string, bin_t*> bins_map;

      for ( database_it = input.begin() ; database_it != input.end(); database_it++ ) {
        std::string target_id = (*database_it);

        bins_map[target_id] = new bin_t(boost::extents[bins_][bins_][bins_]);
      }

      // Conduct hough voting
      for ( database_it = input.begin() ; database_it != input.end(); database_it++ ) {
        std::string target_id = (*database_it);
        std::cout << "Voting for: " << target_id << endl;

        bin_t *target_bin = bins_map[target_id];

        // Reset all bins to 0 votes
        std::fill(target_bin->data(), target_bin->data() + target_bin->num_elements(), 0);

        houghVote(query, (*database_)[target_id], *(bins_map[target_id]));
      }

      // Use the highest number of votes in a bin as the vote for the object
      vector<Candidate> ballot;
      for ( database_it = input.begin() ; database_it != input.end(); database_it++ ) {
        std::string target_id = (*database_it);
        Entry target = (*database_)[target_id];

        Candidate* candidate = new Candidate;
        candidate->id = target_id;


        bin_t *target_bin = bins_map[target_id];
        double *max_votes_it = std::max_element(target_bin->data(), target_bin->data() + target_bin->num_elements());

        double sum = 0;
        for (double* i =  target_bin->data(); i < target_bin->data() + target_bin->num_elements(); ++i)
        {
          sum += *i;
        }

        candidate->votes = *max_votes_it;
        ballot.push_back(*candidate);

        std::cout << target_id << "\t" << candidate->votes << endl;
      }

      selectBestCandidates(max_num, ballot, output);
    }

    bool
    HoughProposer::castVotes(Eigen::Vector3f& indices, bin_t& bins)
    {
      if (is_soft_vote_)
      {
        return softCastVotes(indices, bins);
      }
      else
      {
        return hardCastVotes(indices, bins);
      }
    }

    bool
    HoughProposer::softCastVotes(Eigen::Vector3f& indices, bin_t& bins)
    {
      if ((indices.array() >= 0).all() && (indices.array() < bins_).all())
      {
        Eigen::Vector3i indices_i = indices.cast<int>();

        Eigen::Vector3f unit_distance = indices - indices_i.cast<float>();

        float vote = 1.0 / num_angles_;

        bins[indices_i[0]][indices_i[1]][indices_i[2]] += vote;

        for (int b_x = 0; b_x < 2; b_x++)
        {
          for (int b_y = 0; b_y < 2; b_y++)
          {
            for (int b_z = 0; b_z < 2; b_z++)
            {
              float soft_bin_votes = vote;
              if (b_x == 0)
                soft_bin_votes *= (unit_distance[0]);
              else
                soft_bin_votes *= (1 - unit_distance[0]);

              if (b_y == 0)
                soft_bin_votes *= (unit_distance[1]);
              else
                soft_bin_votes *= (1 - unit_distance[1]);

              if (b_z == 0)
                soft_bin_votes *= (unit_distance[2]);
              else
                soft_bin_votes *= (1 - unit_distance[2]);

              int bin_x = std::min(indices_i[0] + b_x, bins_ - 1);
              int bin_y = std::min(indices_i[1] + b_y, bins_ - 1);
              int bin_z = std::min(indices_i[2] + b_z, bins_ - 1);

              bins[bin_x][bin_y][bin_z] += soft_bin_votes;
            }
          }
        }

        return true;
      }

      return false;
    }

    bool
    HoughProposer::hardCastVotes(Eigen::Vector3f& indices, bin_t& bins)
    {
      if ((indices.array() >= 0).all() && (indices.array() < bins_).all())
      {
        Eigen::Vector3i indices_i = indices.cast<int>();

        float vote = 1.0 / num_angles_;

        bins[indices_i[0]][indices_i[1]][indices_i[2]] += vote;

        return true;
      }

      return false;
    }


    double
    HoughProposer::angleBetween(Eigen::Vector3f a, Eigen::Vector3f b)
    {
      return acos( a.dot(b) / (a.norm() * b.norm()) );
    }

  }

}

#include "proctor/soft_hough_proposer.h"

namespace pcl
{
  namespace proctor
  {
    bool
    SoftHoughProposer::castVotes(Eigen::Vector3f& indices, bin_t& bins)
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
  }
}


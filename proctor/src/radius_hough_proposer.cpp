#include "proctor/radius_hough_proposer.h"

#include <cmath>

#include <boost/format.hpp>

namespace pcl
{
  namespace proctor
  {
    void
    RadiusHoughProposer::houghVote(Entry &query, Entry &target, bin_t& bins)
    {
      // Compute the reference point for the R-table
      Eigen::Vector4f centroid4;
      compute3DCentroid(*(target.cloud), centroid4);
      Eigen::Vector3f centroid(centroid4[0], centroid4[1], centroid4[2]);

      assert(query.keypoints->size() == query.features->size());
      assert(target.keypoints->size() == target.features->size());

      // Figure out bin dimension
      Eigen::Vector4f query_min4, query_max4;
      getMinMax3D(*query.cloud, query_min4, query_max4);
      Eigen::Vector3f query_min(query_min4[0], query_min4[1], query_min4[2]);
      Eigen::Vector3f query_max(query_max4[0], query_max4[1], query_max4[2]);

      Eigen::Vector3f region = query_max - query_min;
      Eigen::Vector3f bin_size = region / bins_;

      for (unsigned int i = 0; i < query.keypoints->size(); i++)
      {
        std::vector<int> feature_indices;
        std::vector<float> sqr_distances;

        int num_correspondences = 2;
        if (!pcl_isfinite (query.features->points.row(i)(0)))
          continue;
        int num_found = target.tree->nearestKSearch(*query.features, i, num_correspondences, feature_indices, sqr_distances);

        for (int j = 0; j < num_found; j++)
        {
          // For each one of the feature correspondences
          int feature_index = feature_indices[j];

          Eigen::Vector3f query_keypoint = query.keypoints->at(i).getVector3fMap();
          Eigen::Vector3f target_keypoint = target.keypoints->at(feature_index).getVector3fMap();

          // Get corresponding target keypoint, and calculate its r to its centroid
          PointNormal correspondence = target.keypoints->at(feature_index); // Since features correspond to the keypoints
          Eigen::Vector3f r = correspondence.getVector3fMap() - centroid;

          float dist_from_target = r.norm();

          int hits = 0;
          int misses = 0;
          for (int x = 0; x < bins_; x++)
          {
            for (int y = 0; y < bins_; y++)
            {
              for (int z = 0; z < bins_; z++)
              {
                Eigen::Vector3f indices (x, y, z);

                Eigen::Vector3f test = indices.array() * bin_size.array();

                Eigen::Vector3f world_coordinates = indices.array() * bin_size.array() + query_min.array();

                float dist_from_query = (world_coordinates - query_keypoint).norm();
                float diff = std::abs(dist_from_query - dist_from_target);

                if (diff < bin_size.norm())
                {
                  // TODO Normalize by how many actually get in?
                  bins[x][y][z] += 1;
                  hits++;
                }
                else
                {
                  misses++;
                }
              }
            }
          }
          //cout << boost::format("Hits: %-10d Misses: %-10d") % hits % misses << endl;
        }
      }
    }
  }
}


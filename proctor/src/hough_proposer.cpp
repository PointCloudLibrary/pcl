#include "proctor/hough_proposer.h"

#include "pcl/common/centroid.h"
#include "pcl/common/common.h"

#include <assert.h>
#include "boost/array.hpp"
#include "boost/cstdlib.hpp"
#include <algorithm>
#include <set>

//#define bin_x 5
//#define bins_angle 1

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

      for ( database_it = input.begin() ; database_it != input.end(); database_it++ ) {
        std::string target_id = (*database_it);
        std::cout << "Voting for: " << target_id << endl;

        bin_t *target_bin = bins_map[target_id];

        // Reset all bins to 0 votes
        std::fill(target_bin->data(), target_bin->data() + target_bin->num_elements(), 0);

        houghVote(query, (*database_)[target_id], *(bins_map[target_id]));
      }

      vector<Candidate> ballot;
      for ( database_it = input.begin() ; database_it != input.end(); database_it++ ) {
        std::string target_id = (*database_it);
        Entry target = (*database_)[target_id];

        Candidate* candidate = new Candidate;
        candidate->id = target_id;


        bin_t *target_bin = bins_map[target_id];
        // TODO fix auto
        auto max_votes_it = std::max_element(target_bin->data(), target_bin->data() + target_bin->num_elements());

        double sum = 0;
        for (auto i =  target_bin->data(); i < target_bin->data() + target_bin->num_elements(); ++i)
        {
          //cout << *i << "\t";
          sum += *i;
        }

        candidate->votes = *max_votes_it;
        ballot.push_back(*candidate);

        std::cout << target_id << "\t" << candidate->votes << "\tAverage: " << sum / target_bin->num_elements() << endl;
      }

      selectBestCandidates(max_num, ballot, output);
    }

    void
    HoughProposer::houghVote(Entry &query, Entry &target, bin_t& bins)
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

      //Eigen::Vector3f target_min, target_max;
      //getMinMax3D(*target.cloud, target_min, target_max);

      //Eigen::Vector4f extreme_min, extreme_max;
      //extreme_min = query_min.cwiseMin(target_min);
      //extreme_max = query_max.cwiseMax(target_max);

      Eigen::Affine3f t;
      getTransformation(0, 0, 0, M_PI, 0.5, 1.5, t);

      int correctly_matched = 0;
      int hits = 0;
      int misses = 0;
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

          target_keypoint = t * target_keypoint;
          if ((query_keypoint - target_keypoint).norm() < 0.05)
          {
            //cout << sqr_distances[j] << endl;
            correctly_matched++;
          }

          // Get corresponding target keypoint, and calculate its r to its centroid
          PointNormal correspondence = target.keypoints->at(feature_index); // Since features correspond to the keypoints
          Eigen::Vector3f r = correspondence.getVector3fMap() - centroid;

          // Calculate the rotation transformation from the target normal to the query normal
          Eigen::Vector3f target_normal = correspondence.getNormalVector3fMap();
          target_normal.normalize();
          Eigen::Vector3f query_normal = query.keypoints->at(i).getNormalVector3fMap();
          query_normal.normalize();
          double angle = acos( target_normal.dot(query_normal) / (target_normal.norm() * query_normal.norm()) );
          Eigen::Vector3f axis = target_normal.normalized().cross(query_normal.normalized());
          axis.normalize();
          Eigen::Affine3f rot_transform;
          rot_transform = Eigen::AngleAxisf(angle, axis);
          //if ((query_keypoint - target_keypoint).norm() < 0.002)
            //cout << "Rotation Matrix: " << angle << "\t" << cos(angle) << endl;
            //
          // Check that the rotation matrix is correct
          Eigen::Vector3f projected = rot_transform * target_normal;
          projected.normalize();
          //assert(projected.isApprox(query_normal, 0.05));

          // Transform r based on the difference between the normals
          Eigen::Vector3f transformed_r = rot_transform * r;

          for (int k = 0; k < num_angles_; k++)
          {
            float query_angle = (float(k) / num_angles_) * 2 * M_PI;
            Eigen::Affine3f query_rot;
            query_rot = Eigen::AngleAxisf(query_angle, query_normal.normalized());
            
            Eigen::Vector3f guess_r = query_rot * transformed_r;

            Eigen::Vector3f centroid_est = query.keypoints->at(i).getVector3fMap() - guess_r;

            Eigen::Vector3f region = query_max - query_min;
            Eigen::Vector3f bin_size = region / bins_;
            Eigen::Vector3f diff = (centroid_est - query_min);
            Eigen::Vector3f indices = diff.cwiseQuotient(bin_size);

            if (castVotes(indices, bins))
            {
              hits++;
            }
            else
            {
              misses++;
            }
          }

        }

      }
      cout << "  Correctly Matched: " << correctly_matched << endl;
      cout << "  Hit: " << hits << endl;
      cout << "  Misses: " << misses << endl;
    }


    bool
    HoughProposer::castVotes(Eigen::Vector3f& indices, bin_t& bins)
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

  }

}

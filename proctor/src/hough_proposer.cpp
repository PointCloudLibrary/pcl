#include "proctor/hough_proposer.h"

#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/features/principal_curvatures.h"

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
        auto max_votes_it = std::max_element(target_bin->data(), target_bin->data() + target_bin->num_elements());

        double sum = 0;
        for (auto i =  target_bin->data(); i < target_bin->data() + target_bin->num_elements(); ++i)
        {
          sum += *i;
        }

        candidate->votes = *max_votes_it;
        ballot.push_back(*candidate);

        std::cout << target_id << "\t" << candidate->votes << endl;
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

      PointCloud<PrincipalCurvatures>::Ptr target_curvatures = computeCurvatures(target);
      PointCloud<PrincipalCurvatures>::Ptr query_curvatures = computeCurvatures(query);

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

          // Get corresponding target keypoint, and calculate its r to its centroid in world coordinates
          PointNormal correspondence = target.keypoints->at(feature_index); // Since features correspond to the keypoints
          Eigen::Vector3f r = correspondence.getVector3fMap() - centroid;

          // Calculate a local frame using principal curvature
          Eigen::Vector3f target_normal = correspondence.getNormalVector3fMap();
          target_normal.normalize();
          Eigen::Vector3f query_normal = query.keypoints->at(i).getNormalVector3fMap();
          query_normal.normalize();

          Eigen::Vector3f query_pc = getVectorCurvatureMap(query_curvatures->at(i));
          Eigen::Vector3f query_perp = query_pc - (query_pc.dot(query_normal.normalized())) * query_normal.normalized();
          query_perp.normalize();

          Eigen::Vector3f target_pc = getVectorCurvatureMap(target_curvatures->at(feature_index));
          Eigen::Vector3f target_perp = target_pc - (target_pc.dot(target_normal.normalized())) * target_normal.normalized();
          target_perp.normalize();

          // In case the PrincipalCurvaturesEstimation outputs NaNs,
          // skip this iteration
          if (query_perp != query_perp || target_perp != target_perp)
          {
            continue;
          }

          Eigen::Vector3f world_x(1, 0, 0);
          Eigen::Vector3f world_y(0, 1, 0);
          Eigen::Vector3f world_z(0, 0, 1);
          Eigen::Matrix3f A = getTransformationBetweenFrames(world_x, world_y, target_normal, target_perp);
          Eigen::Matrix3f B = getTransformationBetweenFrames(world_x, world_y, query_normal, query_perp);
          Eigen::Matrix3f rot_transform = B.transpose() * A;

          assert(query_normal.isApprox(rot_transform * target_normal, 0.001));
          assert(query_perp.isApprox(rot_transform * target_perp, 0.001));

          // Transform r based on the difference between the normals
          Eigen::Vector3f transformed_r = rot_transform * r;

          Eigen::Vector3f centroid_est = query.keypoints->at(i).getVector3fMap() - transformed_r;

          Eigen::Vector3f region = query_max - query_min;
          Eigen::Vector3f bin_size = region / bins_;
          Eigen::Vector3f diff = (centroid_est - query_min);
          Eigen::Vector3f indices = diff.cwiseQuotient(bin_size);

          castVotes(indices, bins);

        }

      }
      //cout << "  Correctly Matched: " << correctly_matched << endl;
      //cout << "  Hit: " << hits << endl;
      //cout << "  Misses: " << misses << endl;
    }

    void
    HoughProposer::referenceFrameHoughVote()
    {

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

    Eigen::Vector3f
    HoughProposer::getVectorCurvatureMap(PrincipalCurvatures p)
    {
      Eigen::Vector3f principal_curvature;
      principal_curvature(0) = p.principal_curvature_x;
      principal_curvature(1) = p.principal_curvature_y;
      principal_curvature(2) = p.principal_curvature_z;
      return principal_curvature;
    }

    PointCloud<PrincipalCurvatures>::Ptr
    HoughProposer::computeCurvatures(Entry &e)
    {
      PointCloud<PrincipalCurvatures>::Ptr curvatures (new PointCloud<PrincipalCurvatures>());

      PrincipalCurvaturesEstimation<PointNormal, PointNormal, pcl::PrincipalCurvatures> curvature_estimation;
      curvature_estimation.setInputCloud(e.keypoints);
      curvature_estimation.setInputNormals(e.cloud);
      curvature_estimation.setSearchSurface(e.cloud);
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
      curvature_estimation.setSearchMethod(tree);
      curvature_estimation.setRadiusSearch(0.01);
      curvature_estimation.compute(*curvatures);

      return curvatures;
    }

    /* Builds a matrix that converts a coordinate in "from" coordinates to "to" coordinates */
    Eigen::Matrix3f
    HoughProposer::getTransformationBetweenFrames(Eigen::Vector3f x_from, Eigen::Vector3f y_from, Eigen::Vector3f x_to, Eigen::Vector3f y_to)
    {
      assert(abs(x_from.norm() - 1) < 0.0001);
      assert(abs(x_to.norm() - 1) < 0.0001);
      assert(abs(y_from.norm() - 1) < 0.0001);
      assert(abs(y_to.norm() - 1) < 0.0001);

      assert(abs(x_from.dot(y_from)) < 0.0001);
      assert(abs(x_to.dot(y_to)) < 0.0001);
      Eigen::Vector3f z_from = x_from.cross(y_from);

      Eigen::Vector3f z_to = x_to.cross(y_to);

      Eigen::Matrix3f rot;
      rot << x_from.dot(x_to),  y_from.dot(x_to) , z_from.dot(x_to),
             x_from.dot(y_to),  y_from.dot(y_to) , z_from.dot(y_to),
             x_from.dot(z_to),  y_from.dot(z_to) , z_from.dot(z_to);

      return rot;
    }

    double
    HoughProposer::angleBetween(Eigen::Vector3f a, Eigen::Vector3f b)
    {
      return acos( a.dot(b) / (a.norm() * b.norm()) );
    }

  }

}

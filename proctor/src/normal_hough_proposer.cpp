#include "proctor/normal_hough_proposer.h"

namespace pcl
{
  namespace proctor
  {
    void
    NormalHoughProposer::houghVote(Entry &query, Entry &target, bin_t& bins)
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

      Eigen::Affine3f t;
      getTransformation(0, 0, 0, M_PI, 0.5, 1.5, t);

      int correctly_matched = 0;
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

          // Check that the rotation matrix is correct
          Eigen::Vector3f projected = rot_transform * target_normal;
          projected.normalize();

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

            castVotes(indices, bins);
          }

        }

      }
    }
  }
}


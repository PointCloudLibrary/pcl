#include "proctor/frame_hough_proposer.h"

namespace pcl
{
  namespace proctor
  {
    void
    FrameHoughProposer::houghVote(Entry &query, Entry &target, bin_t& bins)
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
          Eigen::Matrix3f rot_transform (B.transpose() * A);
#ifndef NDEBUG
          Eigen::Matrix3f other = getTransformationBetweenFrames(target_normal, target_perp, query_normal, query_perp);
#endif

          // Test that the target_normal goes to (1, 0, 0)
          assert((A * target_normal).isApprox(world_x, 0.01));

          // Test that the target_normal goes to (1, 0, 0) in A space,
          // then when moved to B space, and returned back to world space,
          // it stays the same
          assert((B.transpose() * other * A * target_normal).isApprox(target_normal, 0.01));

          // Test that the target_normal goes to (1, 0, 0) in A space,
          // then when rotated in A space by the difference between B and A,
          // then returned back to world space, is equal to the query_normal
          //Eigen::Vector3f test = (A.transpose() * other.transpose() * A * target_normal);
          assert((A.transpose() * other.transpose() * A * target_normal).isApprox(query_normal, 0.01));

          assert(query_normal.isApprox(rot_transform * target_normal, 0.01));
          assert(query_perp.isApprox(rot_transform * target_perp, 0.01));

          // Transform r based on the difference between the normals
          Eigen::Vector3f transformed_r (rot_transform * r);

          Eigen::Vector3f centroid_est = query.keypoints->at(i).getVector3fMap() - transformed_r;

          Eigen::Vector3f region = query_max - query_min;
          Eigen::Vector3f bin_size = region / float (bins_);
          Eigen::Vector3f diff = (centroid_est - query_min);
          Eigen::Vector3f indices = diff.cwiseQuotient(bin_size);

          castVotes(indices, bins);
        }
      }
    }

    Eigen::Vector3f
    FrameHoughProposer::getVectorCurvatureMap(PrincipalCurvatures p)
    {
      Eigen::Vector3f principal_curvature;
      principal_curvature(0) = p.principal_curvature_x;
      principal_curvature(1) = p.principal_curvature_y;
      principal_curvature(2) = p.principal_curvature_z;
      return principal_curvature;
    }

    PointCloud<PrincipalCurvatures>::Ptr
    FrameHoughProposer::computeCurvatures(Entry &e)
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
    FrameHoughProposer::getTransformationBetweenFrames(Eigen::Vector3f x_from, Eigen::Vector3f y_from, Eigen::Vector3f x_to, Eigen::Vector3f y_to)
    {
      assert(abs(x_from.norm() - 1) < 0.0001);
      assert(abs(x_to.norm() - 1) < 0.0001);
      assert(abs(y_from.norm() - 1) < 0.0001);
      assert(abs(y_to.norm() - 1) < 0.0001);
      x_from.normalize();
      y_from.normalize();
      x_to.normalize();
      y_to.normalize();

      assert(abs(x_from.dot(y_from)) < 0.0001);
      assert(abs(x_to.dot(y_to)) < 0.0001);
      Eigen::Vector3f z_from = x_from.cross(y_from);
      z_from.normalize();

      Eigen::Vector3f z_to = x_to.cross(y_to);
      z_to.normalize();

      Eigen::Matrix3f rot;
      rot << x_from.dot(x_to),  y_from.dot(x_to) , z_from.dot(x_to),
             x_from.dot(y_to),  y_from.dot(y_to) , z_from.dot(y_to),
             x_from.dot(z_to),  y_from.dot(z_to) , z_from.dot(z_to);

      return rot;
    }
  }
}


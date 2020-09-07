#pragma once

#include "pcl/features/normal_3d.h"
#include "pcl/Vertices.h"

namespace pcl
{
  namespace features
  {

    /** \brief Compute approximate surface normals on a mesh.
     * \param[in] cloud Point cloud containing the XYZ coordinates.
     * \param[in] polygons Polygons from the mesh.
     * \param[out] normals Point cloud with computed surface normals
     */
    template <typename PointT, typename PointNT> inline void
    computeApproximateNormals(const pcl::PointCloud<PointT>& cloud, const std::vector<pcl::Vertices>& polygons, pcl::PointCloud<PointNT>& normals)
    {
      const auto nr_points = cloud.size();

      normals.header = cloud.header;
      normals.width = cloud.width;
      normals.height = cloud.height;
      normals.resize(nr_points);

      for (auto& point: normals.points)
        point.getNormalVector3fMap() = Eigen::Vector3f::Zero();

      // NOTE: for efficiency the weight is computed implicitly by using the
      // cross product, this causes inaccurate normals for meshes containing
      // non-triangle polygons (quads or other types)
      for (const auto& polygon: polygons)
      {
        if (polygon.vertices.size() < 3) continue;

        // compute normal for triangle
        Eigen::Vector3f vec_a_b = cloud[polygon.vertices[0]].getVector3fMap() - cloud[polygon.vertices[1]].getVector3fMap();
        Eigen::Vector3f vec_a_c = cloud[polygon.vertices[0]].getVector3fMap() - cloud[polygon.vertices[2]].getVector3fMap();
        Eigen::Vector3f normal = vec_a_b.cross(vec_a_c);
        pcl::flipNormalTowardsViewpoint(cloud[polygon.vertices[0]], 0.0f, 0.0f, 0.0f, normal(0), normal(1), normal(2));

        // add normal to all points in polygon
        for (const auto& vertex: polygon.vertices)
          normals[vertex].getNormalVector3fMap() += normal;
      }

      for (std::size_t i = 0; i < nr_points; ++i)
      {
        normals[i].getNormalVector3fMap().normalize();
        pcl::flipNormalTowardsViewpoint(cloud[i], 0.0f, 0.0f, 0.0f, normals[i].normal_x, normals[i].normal_y, normals[i].normal_z);
      }
    }


    /** \brief Compute GICP-style covariance matrices given a point cloud and
     * the corresponding surface normals.
     * \param[in] cloud Point cloud containing the XYZ coordinates,
     * \param[in] normals Point cloud containing the corresponding surface normals.
     * \param[out] covariances Vector of computed covariances.
     * \param[in] epsilon Optional: Epsilon for the expected noise along the surface normal (default: 0.001)
     */
    template <typename PointT, typename PointNT> inline void
    computeApproximateCovariances(const pcl::PointCloud<PointT>& cloud,
                                  const pcl::PointCloud<PointNT>& normals,
                                  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >& covariances,
                                  double epsilon = 0.001)
    {
      assert(cloud.size() == normals.size());

      const auto nr_points = cloud.size();
      covariances.clear ();
      covariances.reserve (nr_points);
      for (const auto& point: normals.points)
      {
        Eigen::Vector3d normal (point.normal_x,
                                point.normal_y,
                                point.normal_z);

        // compute rotation matrix
        Eigen::Matrix3d rot;
        Eigen::Vector3d y;
        y << 0, 1, 0;
        rot.row(2) = normal;
        y -= normal(1) * normal;
        y.normalize();
        rot.row(1) = y;
        rot.row(0) = normal.cross(rot.row(1));

        // comnpute approximate covariance
        Eigen::Matrix3d cov;
        cov << 1, 0, 0,
               0, 1, 0,
               0, 0, epsilon;
        covariances.emplace_back (rot.transpose()*cov*rot);
      }
    }

  }
}

#define PCL_INSTANTIATE_computeApproximateCovariances(T,NT) template PCL_EXPORTS void pcl::features::computeApproximateCovariances<T,NT> \
    (const pcl::PointCloud<T>&, const pcl::PointCloud<NT>&, std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>&, double);

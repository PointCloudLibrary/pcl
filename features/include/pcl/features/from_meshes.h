#ifndef PCL_FEATURES_FROM_MESHES_H_
#define PCL_FEATURES_FROM_MESHES_H_

#include <pcl/features/normal_3d.h>

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
      int nr_points = static_cast<int>(cloud.points.size());
      int nr_polygons = static_cast<int>(polygons.size());

      normals.header = cloud.header;
      normals.width = cloud.width;
      normals.height = cloud.height;
      normals.points.resize(nr_points);

      for ( int i = 0; i < nr_points; ++i )
        normals.points[i].getNormalVector3fMap() = Eigen::Vector3f::Zero();

      // NOTE: for efficiency the weight is computed implicitly by using the
      // cross product, this causes inaccurate normals for meshes containing
      // non-triangle polygons (quads or other types)
      for ( int i = 0; i < nr_polygons; ++i )
      {
        const int nr_points_polygon = (int)polygons[i].vertices.size();
        if (nr_points_polygon < 3) continue;

        // compute normal for triangle
        Eigen::Vector3f vec_a_b = cloud.points[polygons[i].vertices[0]].getVector3fMap() - cloud.points[polygons[i].vertices[1]].getVector3fMap();
        Eigen::Vector3f vec_a_c = cloud.points[polygons[i].vertices[0]].getVector3fMap() - cloud.points[polygons[i].vertices[2]].getVector3fMap();
        Eigen::Vector3f normal = vec_a_b.cross(vec_a_c);
        pcl::flipNormalTowardsViewpoint(cloud.points[polygons[i].vertices[0]], 0.0f, 0.0f, 0.0f, normal(0), normal(1), normal(2));

        // add normal to all points in polygon
        for ( int j = 0; j < nr_points_polygon; ++j )
          normals.points[polygons[i].vertices[j]].getNormalVector3fMap() += normal;
      }

      for ( int i = 0; i < nr_points; ++i )
      {
        normals.points[i].getNormalVector3fMap().normalize();
        pcl::flipNormalTowardsViewpoint(cloud.points[i], 0.0f, 0.0f, 0.0f, normals.points[i].normal_x, normals.points[i].normal_y, normals.points[i].normal_z);
      }
    }


    /** \brief Compute GICP-style covariance matrices given a point cloud and 
     * the corresponding surface normals.
     * \param[in] cloud Point cloud containing the XYZ coordinates,
     * \param[in] normals Point cloud containing the corresponding surface normals.
     * \param[out] covariances Vector of computed covariances.
     * \param[in] Optional: Epsilon for the expected noise along the surface normal (default: 0.001)
     */
    template <typename PointT, typename PointNT> inline void
    computeApproximateCovariances(const pcl::PointCloud<PointT>& cloud, 
                                  const pcl::PointCloud<PointNT>& normals,
                                  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >& covariances,
                                  double epsilon = 0.001)
    {
      assert(cloud.points.size() == normals.points.size());

      int nr_points = static_cast<int>(cloud.points.size());
      covariances.resize(nr_points);
      for (int i = 0; i < nr_points; ++i)
      {
        Eigen::Vector3d normal(normals.points[i].normal_x, 
                               normals.points[i].normal_y, 
                               normals.points[i].normal_z);

        // compute rotation matrix
        Eigen::Matrix3d rot;
        Eigen::Vector3d y;
        y << 0, 1, 0;
        rot.row(2) = normal;
        y = y - normal(1) * normal;
        y.normalize();
        rot.row(1) = y;
        rot.row(0) = normal.cross(rot.row(1));
        
        // comnpute approximate covariance
        Eigen::Matrix3d cov;
        cov << 1, 0, 0,
               0, 1, 0,
               0, 0, epsilon;
        covariances[i] = rot.transpose()*cov*rot;
      }
    }

  }
}


#endif // PCL_FEATURES_FROM_MESHES_H_



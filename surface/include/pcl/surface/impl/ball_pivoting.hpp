/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_BALL_PIVOTING_HPP_
#define PCL_BALL_PIVOTING_HPP_

#include <algorithm>
#include <cmath>

#include <boost/make_shared.hpp>
#include <boost/range/algorithm/random_shuffle.hpp>

#include <pcl/surface/ball_pivoting.h>
  
namespace pcl
{
  /**
   * @brief get_plane_between returns the plane between two points,
   *        it is perpendicular to v0-v1 and crosses their mid-point
   * @param point0
   * @param point1
   * @return
   */
  Eigen::Vector4f
  get_plane_between (const Eigen::Vector3f &point0, const Eigen::Vector3f &point1)
  {
    const Eigen::Vector3f normal = (point1 - point0).normalized ();
    Eigen::Vector4f plane;
    // (v1+v0)/2 is on plane
    plane << normal, -(point1 + point0).dot (normal) * 0.5f;
    return plane;
  }
  
  /**
   * @brief get_id_point_in_sphere returns the index of points which are in sphere
   *        with center and radius as given
   * @param kdtree kdtree containing the point cloud
   * @param center
   * @param radius
   * @return
   */
  template<typename PointNT>
  std::vector<int>
  get_id_point_in_sphere (const typename PCLSurfaceBase<PointNT>::KdTreePtr &kdtree, 
                          const Eigen::Vector3f &center, const double radius)
  {
    PointNT center_point;
    std::vector<int> indices;
    std::vector<float> sqr_distances;
    center_point.getVector3fMap () = center;
    kdtree->radiusSearch (center_point, radius, indices, sqr_distances);
    return indices;
  }
  
  /**
   * @brief num_point_in_sphere returns the number of points in sphere with given center and radius
   * @param center
   * @param radius
   * @param kdtree
   * @return
   */
  template<typename PointNT>
  size_t
  num_point_in_sphere (const Eigen::Vector3f &center, const float radius, 
                       const typename PCLSurfaceBase<PointNT>::KdTreePtr &kdtree)
  {
    return get_id_point_in_sphere<PointNT> (kdtree, center, radius).size ();
  }
   
  /**
   * checks whether normal is consistent with the normal vectors of points with indexes
   * @tparam PointNT
   * @param normal
   * @param index
   * @param cloud
   * @return
   */
  template<typename PointNT>
  bool
  is_normal_consistent (const Eigen::Vector3f &normal, const std::vector<uint32_t> &indexes,
                        const typename pcl::PointCloud<PointNT>::ConstPtr &cloud)
  {
    assert(indexes.size () == 3);
    int count_consistent = 0;
    for (size_t id = 0; id < 3; ++id)
    {
      if (normal.dot (cloud->at (indexes.at (id)).getNormalVector3fMap ()) > 0.0f)
      {
        ++count_consistent;
      }
    }
    return count_consistent >= 2;
  }
  
  /**
   * get the center of circle where three point are on
   * @param point0
   * @param point1
   * @param point2
   * @return
   */
  Eigen::Vector3f
  get_circle_center (const Eigen::Vector3f &point0, const Eigen::Vector3f &point1, 
                     const Eigen::Vector3f &point2)
  {
    // https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_from_cross-_and_dot-products
    const Eigen::Vector3f vec2 = point0 - point1;
    const Eigen::Vector3f vec0 = point1 - point2;
    const Eigen::Vector3f vec1 = point2 - point0;
    const float area = vec0.cross (vec1).norm ();
    const float determinator = 2.0f * area * area;
  
    const float alpha = vec0.dot (vec0) * vec2.dot (-vec1) / determinator;
    const float beta = vec1.dot (vec1) * -vec2.dot (vec0) / determinator;
    const float gamma = vec2.dot (vec2) * vec1.dot (-vec0) / determinator;
  
    return alpha * point0 + beta * point1 + gamma * point2;
  }
  
  /**
   * get the normal vector of a triangle, (p1-p0)x(p2-p0)
   * @tparam PointNT
   * @param cloud
   * @param index
   * @return
   */
  template<typename PointNT>
  Eigen::Vector3f
  get_normal_triangle (const typename pcl::PointCloud<PointNT>::ConstPtr &cloud, 
                       const std::vector<uint32_t> &indexes)
  {
    assert(indexes.size () == 3);
    const Eigen::Vector3f p0 = cloud->at (indexes.at (0)).getVector3fMap ();
    return (cloud->at (indexes.at (1)).getVector3fMap () - p0)
      .cross (cloud->at (indexes.at (2)).getVector3fMap () - p0)
      .normalized ();
  }
  
  /**
   * reorder the vertices of triangle so the normal vector of triangle is consistent with normals of vertices
   * @tparam PointNT
   * @param cloud
   * @param triangle
   */
  template<typename PointNT>
  void
  reorder (const typename pcl::PointCloud<PointNT>::ConstPtr &cloud, pcl::Vertices &triangle)
  {
    if (!is_normal_consistent (get_normal_triangle<PointNT> (cloud, triangle.vertices), 
                               triangle.vertices, cloud))
    {
      // if the order (0,1,2) is not consistent, (0,2,1) should be consistent
      std::swap (triangle.vertices.at (1), triangle.vertices.at (2));
    }
  }
  
  /**
   * get the distance from point to plane
   * @tparam PointNT
   * @param plane
   * @param point
   * @return
   */
  template<typename PointNT>
  float
  get_distance_point_plane (const Eigen::Vector4f &plane, const PointNT &point)
  {
    // plane is (a,b,c,d), point is (x,y,z), then distance is ax+by+cz+d
    return point.getVector3fMap ().dot (plane.segment (0, 3)) + plane (3);
  }
  
  /**
   * get the signed rotation angle from (point0-center) to (point1-center) on plane
   * @param point0
   * @param point1
   * @param center
   * @param plane the rotation is along the normal vector of plane
   * @return
   */
  float
  get_angle_rotation (const Eigen::Vector3f &point0, const Eigen::Vector3f &point1, 
                      const Eigen::Vector3f &center, const Eigen::Vector4f &plane)
  {
    const Eigen::Vector3f vc0 = (point0 - center).normalized ();
    const Eigen::Vector3f vc1 = (point1 - center).normalized ();
  
    const float sin_val = vc0.cross (-Eigen::Vector3f (plane.segment (0, 3))).dot (vc1);
    const float cos_val = vc0.dot (vc1);
    float angle = atan2 (sin_val, cos_val);
    if (angle < 0.0f) // -pi~pi -> 0~2pi
    {
      angle += (float) (2.0 * M_PI);
    }
    return angle;
  }
  
  /**
   * estimate a radius for the ball pivoting algorithm. the returned estimation is the minimal value, so that
   * at least min_success_rate of the sample points have at least num_in_radius neighbors
   * @tparam PointNT
   * @param kdtree
   * @param num_sample_point
   * @param num_in_radius
   * @param min_success_rate
   * @return
   */
  template<typename PointNT>
  double
  guess_radius (const typename PCLSurfaceBase<PointNT>::KdTreePtr &kdtree,
                const int num_sample_point = 500,
                const int num_in_radius = 5, const float min_success_rate = 0.95f)
  {
    std::vector<float> farthest_distances;
    const typename pcl::PointCloud<PointNT>::ConstPtr &cloud = kdtree->getInputCloud ();
  
    // sample num_sample_point points in cloud per index
    std::vector<int> index_samples;
    index_samples.reserve (cloud->size ());
    for (size_t id = 0; id < cloud->size (); ++id)
    {
      index_samples.push_back ((int) id);
    }
    boost::range::random_shuffle (index_samples);
    index_samples.resize (num_sample_point);
  
    farthest_distances.reserve (num_sample_point);
    for (int id = 0; id < num_sample_point; ++id)
    {
      std::vector<int> indices;
      std::vector<float> sqr_distances;
      kdtree->nearestKSearch (cloud->at (index_samples.at (id)), num_in_radius, 
                              indices, sqr_distances);
      farthest_distances.push_back (sqr_distances.back ());
    }
  
    // ascending summary of num_in_radius-th nearest neighbor
    std::sort (farthest_distances.begin (), farthest_distances.end ());
  
    return sqrt (farthest_distances.at (
          (int) floor (min_success_rate * (float) num_sample_point)));
  }
  
  template<typename PointNT>
  BallPivoting<PointNT>::BallPivoting ():
    radius_ (-1.0), 
    is_allow_back_ball_ (false), 
    is_allow_flip_ (false),
    threshold_collinear_cos_ (cos (10.0 * M_PI / 180.0)), 
    threshold_distance_near_ (1e-6)
  {
  }
  
  template<typename PointNT>
  BallPivoting<PointNT>::~BallPivoting ()
  {
  }
  
  template<typename PointNT>
  boost::shared_ptr<Eigen::Vector3f>
  BallPivoting<PointNT>::getBallCenter (const bool is_back_first, 
                                        std::vector<uint32_t> &index, 
                                        bool &is_back_ball) const
  {
    boost::shared_ptr<Eigen::Vector3f> center;
    // for checking whether three points are collinear
    const Eigen::Vector3f pos0 (input_->at (index.at (0)).getVector3fMap ());
    const Eigen::Vector3f pos1 (input_->at (index.at (1)).getVector3fMap ());
    const Eigen::Vector3f pos2 (input_->at (index.at (2)).getVector3fMap ());
  
    const Eigen::Vector3f vec0 = (pos1 - pos0).normalized ();
    const Eigen::Vector3f vec1 = (pos2 - pos0).normalized ();
  
    // the three points should not be too near or collinear
    if ((pos0 - pos1).norm () > threshold_distance_near_ && 
        (pos1 - pos2).norm () > threshold_distance_near_ && 
        (pos2 - pos0).norm () > threshold_distance_near_ && 
        fabs (vec0.dot (vec1)) < threshold_collinear_cos_)
    {
      Eigen::Vector3f center_circle = get_circle_center (pos0, pos1, pos2);
  
      // move to distance radius_ along normal direction
      float radius_planar = (center_circle - pos0).norm ();
      if (radius_planar < radius_)
      {
        Eigen::Vector3f normal = vec0.cross (vec1).normalized ();
        Eigen::Vector3f center_candidate;
        const float dist_normal = sqrt ((float) radius_ * radius_ - radius_planar * radius_planar);
        if (!is_normal_consistent<PointNT> (normal, index, input_))
        {
          // reorder the vertices of triangle and reverse the normal vector
          normal = -normal;
          std::swap (index.at (0), index.at (2));
        }
        normal *= dist_normal;
  
        if (is_back_first)
        {
          center_candidate = center_circle - normal;
          if (num_point_in_sphere<PointNT> (center_candidate, radius_, tree_) <= 3)
          {
            center = boost::make_shared<Eigen::Vector3f> (center_candidate);
            is_back_ball = true;
          }
          else if (is_allow_flip_)
          {
            center_candidate = center_circle + normal;
            if (num_point_in_sphere<PointNT> (center_candidate, radius_, tree_) <= 3)
            {
              center = boost::make_shared<Eigen::Vector3f> (center_candidate);
              is_back_ball = false;
            }
          }
        }
        else
        {
          center_candidate = center_circle + normal;
          if (num_point_in_sphere<PointNT> (center_candidate, radius_, tree_) <= 3)
          {
            center = boost::make_shared<Eigen::Vector3f> (center_candidate);
            is_back_ball = false;
          }
          else if (is_allow_flip_)
          {
            center_candidate = center_circle - normal;
            if (num_point_in_sphere<PointNT> (center_candidate, radius_, tree_) <= 3)
            {
              center = boost::make_shared<Eigen::Vector3f> (center_candidate);
              is_back_ball = true;
            }
          }
        }
      }
    }
  
    return center;
  }
  
  template<typename PointNT>
  bool
  BallPivoting<PointNT>::pivot (const ball_pivoting::BallPivotingFront::Edge &edge, uint32_t &id_extended, 
                                Eigen::Vector3f &center_new, bool &is_back_ball) const
  {
    const uint32_t id0 = edge.getIdVertice (0);
    const uint32_t id1 = edge.getIdVertice (1);
    const uint32_t id_op = edge.getIdOpposite ();
    const Eigen::Vector3f center = edge.getCenter ();
    const Eigen::Vector3f v0 = input_->at (id0).getVector3fMap ();
    const Eigen::Vector3f v1 = input_->at (id1).getVector3fMap ();
    const Eigen::Vector3f mid = (v0 + v1) * 0.5f;
    // pivot opposite to normal direction, change direction for angle
    const Eigen::Vector4f plane = edge.isBackBall () ? get_plane_between (v0, v1) 
                                                     : get_plane_between (v1, v0);
    std::vector<Eigen::Vector3f> center_candidates; // only store, no need to align
    std::vector<float> dot_candidates;
    std::vector<uint32_t> id_candidates;
    std::vector<bool> is_back_candidates;
  
    const double search_radius = sqrt (radius_ * radius_ - (v0 - mid).dot (Eigen::Vector3f (v0 - mid))) 
                               + radius_;
    std::vector<uint32_t> point3 (3, 0);
    std::vector<int> indices = get_id_point_in_sphere<PointNT> (tree_, mid, search_radius);
  
    center_candidates.reserve (indices.size ());
    dot_candidates.reserve (indices.size ());
    id_candidates.reserve (indices.size ());
    is_back_candidates.reserve (indices.size ());
    for (std::vector<int>::iterator it = indices.begin (); it != indices.end (); ++it)
    {
      point3.at (0) = id0;
      point3.at (1) = id1;
      point3.at (2) = (uint32_t) (*it);
  
      if (point3.at (2) == id0 || point3.at (2) == id1 || point3.at (2) == id_op ||
          !is_normal_consistent<PointNT> (get_normal_triangle<PointNT> (input_, point3), point3, input_) ||
          fabs (get_distance_point_plane<PointNT> (plane, input_->at (*it))) > radius_)
      {
        continue;
      }
  
      // the three points are different, the normal of triangle is consistent to the normal vectors or vertices
      // and the cloud[point3[2]] has distance to plane smaller than radius
      bool is_back_bool;
      boost::shared_ptr<Eigen::Vector3f> center_jr = getBallCenter (edge.isBackBall (), point3, is_back_bool);
  
      if (center_jr)
      {
        center_candidates.push_back (*center_jr);
        dot_candidates.push_back (get_angle_rotation (center, *center_jr, mid, plane));
        id_candidates.push_back ((uint32_t) *it);
        is_back_candidates.push_back (is_back_bool);
      }
    }
  
    // get the first hit point
    if (!center_candidates.empty ())
    {
      int id_min =
        std::distance (dot_candidates.begin (), std::min_element (dot_candidates.begin (), 
                                                                  dot_candidates.end ()));
      id_extended = id_candidates.at (id_min);
      center_new = center_candidates.at (id_min);
      is_back_ball = is_back_candidates.at (id_min);
      return true;
    }
    else
    {
      return false;
    }
  }
  
  template<typename PointNT>
  void
  BallPivoting<PointNT>::setEstimatedRadius (const int num_sample_point, 
                                             const int num_point_in_radius,
                                             const float ratio_success)
  {
    if (input_ && tree_ && tree_->getInputCloud ())
    {
      radius_ = guess_radius<PointNT> (tree_, num_sample_point, num_point_in_radius, ratio_success);
    }
    else
    {
      PCL_ERROR ("InputCloud or KdTree is empty!\n");
    }
  }
  
  template<typename PointNT>
  void
  BallPivoting<PointNT>::proceedFront (std::vector<pcl::Vertices> &polygons)
  {
    ball_pivoting::BallPivotingFront::Edge::Ptr edge = front_.getActiveEdge ();
    while (edge)
    {
      uint32_t id_ext;
      Eigen::Vector3f center_new;
      bool is_back_ball;
      if (!front_.isEdgeFinished (*edge) && pivot (*edge, id_ext, center_new, is_back_ball))
      {
        const uint32_t id0 = edge->getIdVertice (0);
        const uint32_t id1 = edge->getIdVertice (1);
        pcl::Vertices triangle;
        // add to polygons
        triangle.vertices.reserve (3);
        triangle.vertices.push_back (id0);
        triangle.vertices.push_back (id1);
        triangle.vertices.push_back (id_ext);
        polygons.push_back (triangle);
  
        is_used_.at (id_ext) = true;
        front_.addPoint (*edge, id_ext, center_new, is_back_ball);
      }

      edge = front_.getActiveEdge ();
    }
  }
  
  template<typename PointNT>
  void
  BallPivoting<PointNT>::performReconstruction (pcl::PointCloud<PointNT> &points,
                                                std::vector<pcl::Vertices> &polygons)
  {
    if (!(input_ && tree_ && tree_->getInputCloud ()))
    {
      PCL_ERROR ("InputCloud or KdTree is empty!\n");
      points.clear ();
      polygons.clear ();
      return;
    }

    is_used_.clear ();
    is_used_.resize (input_->size (), false);
    front_.clear ();
  
    // if radius is not valid, guess with default configuration (see default parameters of guess_radius)
    if (radius_ < 0.0)
    {
      radius_ = guess_radius<PointNT> (tree_);
    }
  
    // proceed until not seed can bt found
    while (true)
    {
      proceedFront (polygons);
  
      pcl::Vertices::Ptr seed;
      Eigen::Vector3f center;
      bool is_back_ball;
      if (findSeed (seed, center, is_back_ball))
      {
        // add to mesh
        polygons.push_back (*seed);
        // add for pivoting
        front_.addTriangle (seed, center, is_back_ball);
      }
      else
      {
        // cannot find proper seed
        break;
      }
    }

    points = *input_;
  }
  
  template<typename PointNT>
  void
  BallPivoting<PointNT>::performReconstruction (pcl::PolygonMesh &output)
  {
    typename pcl::PointCloud<PointNT> points;

    performReconstruction (points, output.polygons);

    pcl::toPCLPointCloud2 (points, output.cloud);
  }
  
  template<typename PointNT>
  bool
  BallPivoting<PointNT>::findSeed (pcl::Vertices::Ptr &seed, Eigen::Vector3f &center, bool &is_back_ball)
  {
    const double search_radius = radius_ * 2.0;
  
    // search in all points
    for (size_t id_search = 0; id_search < is_used_.size (); ++id_search)
    {
      if (is_used_.at (id_search))
      {
        // actually ignore used or obsolete points
        continue;
      }
  
      std::vector<uint32_t> index3 (3, 0);
      std::vector<int> indices = get_id_point_in_sphere<PointNT> (
          tree_, input_->at (id_search).getVector3fMap (), search_radius);
      if (indices.size () < 3)
      {
        continue;
      }
  
      for (size_t idn1 = 1; idn1 < indices.size (); ++idn1)
      {
        const uint32_t index1 = indices.at (idn1);
        if (is_used_.at (index1) || index1 == (uint32_t) id_search)
        {
          continue;
        }
  
        for (size_t idn2 = 0; idn2 < idn1; ++idn2)
        {
          const uint32_t index2 = indices.at (idn2);
  
          if (is_used_.at (index2) || index1 == index2 || index2 == (uint32_t) id_search)
          {
            continue;
          }
  
          index3.at (0) = (uint32_t) id_search;
          index3.at (1) = index1;
          index3.at (2) = index2;
  
          bool is_back_ball_local;
          boost::shared_ptr<Eigen::Vector3f> center_ = getBallCenter (false, index3, is_back_ball_local);
          if (!center_ && is_allow_back_ball_)
          {
            center_ = getBallCenter (true, index3, is_back_ball_local);
          }
          if (center_)
          {
            seed = pcl::Vertices::Ptr (new pcl::Vertices ());
            seed->vertices = index3;
            center = *center_;
            is_back_ball = is_back_ball_local;
            is_used_.at (index3.at (0)) = true;
            is_used_.at (index3.at (1)) = true;
            is_used_.at (index3.at (2)) = true;
  
            return true;
          }
        }
      }
  
      is_used_.at (id_search) = true;
    }
    return false;
  }
} // namespace pcl

#define PCL_INSTANTIATE_BallPivoting(T) template class PCL_EXPORTS pcl::BallPivoting<T>;

#endif // PCL_BALL_PIVOTING_HPP_

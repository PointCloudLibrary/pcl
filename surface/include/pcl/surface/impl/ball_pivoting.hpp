/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2017-, Open Perception, Inc.
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
  template<typename PointNT>
  Eigen::Vector4f
  BallPivoting<PointNT>::getPlaneBetweenPoints (const Eigen::Vector3f &point0, 
                                                const Eigen::Vector3f &point1)
  {
    const Eigen::Vector3f normal = (point1 - point0).normalized ();
    Eigen::Vector4f plane;
    // (point1+point0)/2 is on plane
    plane << normal, -(point1 + point0).dot (normal) * 0.5f;
    return plane;
  }
  
  template<typename PointNT>
  std::vector<int>
  BallPivoting<PointNT>::getIdPointsInSphere (const Eigen::Vector3f &center, const double radius) const
  {
    std::vector<int> indices;
    std::vector<float> sqr_distances;
    PointNT center_point;
    center_point.getVector3fMap () = center;
    tree_->radiusSearch (center_point, radius, indices, sqr_distances);
    return indices;
  }
  
  template<typename PointNT>
  size_t
  BallPivoting<PointNT>::getNumPointsInSphere (const Eigen::Vector3f &center, const float radius) const
  {
    return getIdPointsInSphere (center, radius).size ();
  }
   
  template<typename PointNT>
  bool
  BallPivoting<PointNT>::isNormalConsistent (const Eigen::Vector3f &normal, 
                                             const std::vector<uint32_t> &indexes) const
  {
    if (indexes.size () != 3)
    {
      PCL_ERROR ("indexes.size () != 3 in isNormalConsistent");
      return false;
    }
    int count_consistent = 0;
    for (size_t id = 0; id < 3; ++id)
    {
      if (normal.dot (input_->at (indexes.at (id)).getNormalVector3fMap ()) >= 0.0f)
      {
        ++count_consistent;
      }
    }
    return count_consistent >= 2;
  }
   
  template<typename PointNT>
  Eigen::Vector3f
  BallPivoting<PointNT>::getCircleCenter (const Eigen::Vector3f &point0, 
                                          const Eigen::Vector3f &point1, 
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

  template<typename PointNT>
  Eigen::Vector3f
  BallPivoting<PointNT>::getNormalTriangle (const std::vector<uint32_t> &indexes) const
  {
    if (indexes.size () != 3)
    {
      PCL_ERROR ("indexes.size () != 3 in getNormalTriangle");
      return Eigen::Vector3f::Zero ();
    }
    const Eigen::Vector3f p0 = input_->at (indexes.at (0)).getVector3fMap ();
    return (input_->at (indexes.at (1)).getVector3fMap () - p0)
      .cross (input_->at (indexes.at (2)).getVector3fMap () - p0)
      .normalized ();
  }
  
  template<typename PointNT>
  float
  BallPivoting<PointNT>::getRotationAngle (const Eigen::Vector3f &point0, 
                                           const Eigen::Vector3f &point1, 
                                           const Eigen::Vector3f &center, 
                                           const Eigen::Vector4f &plane)
  {
    const Eigen::Vector3f vc0 = (point0 - center).normalized ();
    const Eigen::Vector3f vc1 = (point1 - center).normalized ();
  
    const float sin_val = vc0.cross (-Eigen::Vector3f (plane.segment (0, 3))).dot (vc1);
    const float cos_val = vc0.dot (vc1);
    float angle = std::atan2 (sin_val, cos_val);
    if (angle < 0.0f) // -pi~pi -> 0~2pi
    {
      angle += (float) (2.0 * M_PI);
    }
    return angle;
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
      Eigen::Vector3f center_circle = getCircleCenter (pos0, pos1, pos2);
  
      // move to distance radius_ along normal direction
      float radius_planar = (center_circle - pos0).norm ();
      if (radius_planar < radius_)
      {
        Eigen::Vector3f normal = vec0.cross (vec1).normalized ();
        Eigen::Vector3f center_candidate;
        const float dist_normal = sqrt ((float) radius_ * radius_ - radius_planar * radius_planar);
        if (!isNormalConsistent (normal, index))
        {
          // reorder the vertices of triangle and reverse the normal vector
          normal = -normal;
          std::swap (index.at (0), index.at (2));
        }
        normal *= dist_normal;
  
        if (is_back_first)
        {
          center_candidate = center_circle - normal;
          if (getNumPointsInSphere (center_candidate, radius_) <= 3)
          {
            center = boost::make_shared<Eigen::Vector3f> (center_candidate);
            is_back_ball = true;
          }
          else if (is_allow_flip_)
          {
            center_candidate = center_circle + normal;
            if (getNumPointsInSphere (center_candidate, radius_) <= 3)
            {
              center = boost::make_shared<Eigen::Vector3f> (center_candidate);
              is_back_ball = false;
            }
          }
        }
        else
        {
          center_candidate = center_circle + normal;
          if (getNumPointsInSphere (center_candidate, radius_) <= 3)
          {
            center = boost::make_shared<Eigen::Vector3f> (center_candidate);
            is_back_ball = false;
          }
          else if (is_allow_flip_)
          {
            center_candidate = center_circle - normal;
            if (getNumPointsInSphere (center_candidate, radius_) <= 3)
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
    const uint32_t id0 = edge.id_point_start_;
    const uint32_t id1 = edge.id_point_end_;
    const uint32_t id_op = edge.id_point_opposite_;
    const Eigen::Vector3f &center = edge.center_;
    const Eigen::Vector3f v0 = input_->at (id0).getVector3fMap ();
    const Eigen::Vector3f v1 = input_->at (id1).getVector3fMap ();
    const Eigen::Vector3f mid = (v0 + v1) * 0.5f;
    // pivot opposite to normal direction, change direction for angle
    const Eigen::Vector4f plane = edge.is_back_ball_ ? getPlaneBetweenPoints (v0, v1) 
                                                     : getPlaneBetweenPoints (v1, v0);
    std::vector<Eigen::Vector3f> center_candidates; // only store, no need to align
    std::vector<float> dot_candidates;
    std::vector<uint32_t> id_candidates;
    std::vector<bool> is_back_candidates;
  
    const double search_radius = sqrt (radius_ * radius_ - (v0 - mid).dot (Eigen::Vector3f (v0 - mid))) 
                               + radius_;
    std::vector<uint32_t> point3 (3, 0);
    std::vector<int> indices = getIdPointsInSphere (mid, search_radius);
  
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
          !isNormalConsistent (getNormalTriangle (point3), point3) ||
          std::fabs (plane.segment (0, 3).dot (input_->at (*it).getVector3fMap ()) + plane[3]) > radius_)
      {
        continue;
      }
  
      // the three points are different, the normal of triangle is consistent to the normal vectors or vertices
      // and the cloud[point3[2]] has distance to plane smaller than radius
      bool is_back_bool;
      boost::shared_ptr<Eigen::Vector3f> center_jr = getBallCenter (edge.is_back_ball_, point3, is_back_bool);
  
      if (center_jr)
      {
        center_candidates.push_back (*center_jr);
        dot_candidates.push_back (getRotationAngle (center, *center_jr, mid, plane));
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
  BallPivoting<PointNT>::setSearchRadiusAutomatically (const int num_sample_point, 
                                                       const int num_point_in_radius,
                                                       const float ratio_success)
  {
    if (!(input_ && tree_ && tree_->getInputCloud ()))
    {
      PCL_ERROR ("InputCloud or KdTree is empty!\n");
      return;
    }

    std::vector<float> farthest_distances;
    int num_sample_point_real = num_sample_point <= 0 ? input_->size () / 5 
                                                      : num_sample_point;
  
    // sample num_sample_point_real points in input cloud per index
    std::vector<int> index_samples;
    index_samples.reserve (input_->size ());
    for (size_t id = 0; id < input_->size (); ++id)
    {
      index_samples.push_back ((int) id);
    }
    boost::range::random_shuffle (index_samples);
    index_samples.resize (num_sample_point_real);
  
    farthest_distances.reserve (num_sample_point_real);
    for (int id = 0; id < num_sample_point_real; ++id)
    {
      std::vector<int> indices;
      std::vector<float> sqr_distances;
      tree_->nearestKSearch (input_->at (index_samples.at (id)), 
                             num_point_in_radius, indices, sqr_distances);
      farthest_distances.push_back (sqr_distances.back ());
    }
  
    // ascending summary of num_in_radius-th nearest neighbor
    std::sort (farthest_distances.begin (), farthest_distances.end ());
  
    // find the thresholding value
    radius_ = std::sqrt (farthest_distances.at (
          (int) floor (ratio_success * (float) num_sample_point_real)));
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
        const uint32_t id0 = edge->id_point_start_;
        const uint32_t id1 = edge->id_point_end_;
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

      front_.setEdgeAsFinished (*edge);
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
      setSearchRadiusAutomatically ();
    }
  
    // proceed until not seed can bt found
    while (true)
    {
      proceedFront (polygons);
  
      pcl::Vertices seed;
      Eigen::Vector3f center;
      bool is_back_ball;
      if (findSeed (seed, center, is_back_ball))
      {
        // add to mesh
        polygons.push_back (seed);
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
  BallPivoting<PointNT>::findSeed (pcl::Vertices &seed, Eigen::Vector3f &center, bool &is_back_ball)
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
      std::vector<int> indices = getIdPointsInSphere (
          input_->at (id_search).getVector3fMap (), search_radius);
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
  
          boost::shared_ptr<Eigen::Vector3f> center_new = getBallCenter (false, index3, is_back_ball);
          if (!center_new && is_allow_back_ball_)
          {
            center_new = getBallCenter (true, index3, is_back_ball);
          }
          if (center_new)
          {
            seed.vertices = index3;
            center = *center_new;
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

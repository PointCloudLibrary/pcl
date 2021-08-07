/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef PCL_GEOMETRY_POLYGON_OPERATIONS_HPP_
#define PCL_GEOMETRY_POLYGON_OPERATIONS_HPP_

#include <pcl/geometry/polygon_operations.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::approximatePolygon(const PlanarPolygon<PointT>& polygon,
                        PlanarPolygon<PointT>& approx_polygon,
                        float threshold,
                        bool refine,
                        bool closed)
{
  const Eigen::Vector4f& coefficients = polygon.getCoefficients();
  const typename pcl::PointCloud<PointT>::VectorType& contour = polygon.getContour();

  Eigen::Vector3f rotation_axis(coefficients[1], -coefficients[0], 0.0f);
  rotation_axis.normalize();

  float rotation_angle = acosf(coefficients[2]);
  Eigen::Affine3f transformation = Eigen::Translation3f(0, 0, coefficients[3]) *
                                   Eigen::AngleAxisf(rotation_angle, rotation_axis);

  typename pcl::PointCloud<PointT>::VectorType polygon2D(contour.size());
  for (std::size_t pIdx = 0; pIdx < polygon2D.size(); ++pIdx)
    polygon2D[pIdx].getVector3fMap() = transformation * contour[pIdx].getVector3fMap();

  typename pcl::PointCloud<PointT>::VectorType approx_polygon2D;
  approximatePolygon2D<PointT>(polygon2D, approx_polygon2D, threshold, refine, closed);

  typename pcl::PointCloud<PointT>::VectorType& approx_contour =
      approx_polygon.getContour();
  approx_contour.resize(approx_polygon2D.size());

  Eigen::Affine3f inv_transformation = transformation.inverse();
  for (std::size_t pIdx = 0; pIdx < approx_polygon2D.size(); ++pIdx)
    approx_contour[pIdx].getVector3fMap() =
        inv_transformation * approx_polygon2D[pIdx].getVector3fMap();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::approximatePolygon2D(const typename pcl::PointCloud<PointT>::VectorType& polygon,
                          typename pcl::PointCloud<PointT>::VectorType& approx_polygon,
                          float threshold,
                          bool refine,
                          bool closed)
{
  approx_polygon.clear();
  if (polygon.size() < 3)
    return;

  std::vector<std::pair<unsigned, unsigned>> intervals;
  std::pair<unsigned, unsigned> interval(0, 0);

  if (closed) {
    float max_distance = .0f;
    for (std::size_t idx = 1; idx < polygon.size(); ++idx) {
      float distance =
          (polygon[0].x - polygon[idx].x) * (polygon[0].x - polygon[idx].x) +
          (polygon[0].y - polygon[idx].y) * (polygon[0].y - polygon[idx].y);

      if (distance > max_distance) {
        max_distance = distance;
        interval.second = idx;
      }
    }

    for (std::size_t idx = 1; idx < polygon.size(); ++idx) {
      float distance = (polygon[interval.second].x - polygon[idx].x) *
                           (polygon[interval.second].x - polygon[idx].x) +
                       (polygon[interval.second].y - polygon[idx].y) *
                           (polygon[interval.second].y - polygon[idx].y);

      if (distance > max_distance) {
        max_distance = distance;
        interval.first = idx;
      }
    }

    if (max_distance < threshold * threshold)
      return;

    intervals.push_back(interval);
    std::swap(interval.first, interval.second);
    intervals.push_back(interval);
  }
  else {
    interval.first = 0;
    interval.second = static_cast<unsigned int>(polygon.size()) - 1;
    intervals.push_back(interval);
  }

  std::vector<unsigned> result;
  // recursively refine
  while (!intervals.empty()) {
    std::pair<unsigned, unsigned>& currentInterval = intervals.back();
    float line_x = polygon[currentInterval.first].y - polygon[currentInterval.second].y;
    float line_y = polygon[currentInterval.second].x - polygon[currentInterval.first].x;
    float line_d =
        polygon[currentInterval.first].x * polygon[currentInterval.second].y -
        polygon[currentInterval.first].y * polygon[currentInterval.second].x;

    float linelen = 1.0f / std::sqrt(line_x * line_x + line_y * line_y);

    line_x *= linelen;
    line_y *= linelen;
    line_d *= linelen;

    float max_distance = 0.0;
    unsigned first_index = currentInterval.first + 1;
    unsigned max_index = 0;

    // => 0-crossing
    if (currentInterval.first > currentInterval.second) {
      for (std::size_t idx = first_index; idx < polygon.size(); idx++) {
        float distance =
            std::abs(line_x * polygon[idx].x + line_y * polygon[idx].y + line_d);
        if (distance > max_distance) {
          max_distance = distance;
          max_index = idx;
        }
      }
      first_index = 0;
    }

    for (unsigned int idx = first_index; idx < currentInterval.second; idx++) {
      float distance =
          std::abs(line_x * polygon[idx].x + line_y * polygon[idx].y + line_d);
      if (distance > max_distance) {
        max_distance = distance;
        max_index = idx;
      }
    }

    if (max_distance > threshold) {
      std::pair<unsigned, unsigned> interval(max_index, currentInterval.second);
      currentInterval.second = max_index;
      intervals.push_back(interval);
    }
    else {
      result.push_back(currentInterval.second);
      intervals.pop_back();
    }
  }

  approx_polygon.reserve(result.size());
  if (refine) {
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> lines(
        result.size());
    std::reverse(result.begin(), result.end());
    for (std::size_t rIdx = 0; rIdx < result.size(); ++rIdx) {
      std::size_t nIdx = rIdx + 1;
      if (nIdx == result.size())
        nIdx = 0;

      Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
      Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
      std::size_t pIdx = result[rIdx];
      unsigned num_points = 0;
      if (pIdx > result[nIdx]) {
        num_points = static_cast<unsigned>(polygon.size()) - pIdx;
        for (; pIdx < polygon.size(); ++pIdx) {
          covariance.coeffRef(0) += polygon[pIdx].x * polygon[pIdx].x;
          covariance.coeffRef(1) += polygon[pIdx].x * polygon[pIdx].y;
          covariance.coeffRef(3) += polygon[pIdx].y * polygon[pIdx].y;
          centroid[0] += polygon[pIdx].x;
          centroid[1] += polygon[pIdx].y;
        }
        pIdx = 0;
      }

      num_points += result[nIdx] - pIdx;
      for (; pIdx < result[nIdx]; ++pIdx) {
        covariance.coeffRef(0) += polygon[pIdx].x * polygon[pIdx].x;
        covariance.coeffRef(1) += polygon[pIdx].x * polygon[pIdx].y;
        covariance.coeffRef(3) += polygon[pIdx].y * polygon[pIdx].y;
        centroid[0] += polygon[pIdx].x;
        centroid[1] += polygon[pIdx].y;
      }

      covariance.coeffRef(2) = covariance.coeff(1);

      float norm = 1.0f / float(num_points);
      centroid *= norm;
      covariance *= norm;
      covariance.coeffRef(0) -= centroid[0] * centroid[0];
      covariance.coeffRef(1) -= centroid[0] * centroid[1];
      covariance.coeffRef(3) -= centroid[1] * centroid[1];

      float eval;
      Eigen::Vector2f normal;
      eigen22(covariance, eval, normal);

      // select the one which is more "parallel" to the original line
      Eigen::Vector2f direction;
      direction[0] = polygon[result[nIdx]].x - polygon[result[rIdx]].x;
      direction[1] = polygon[result[nIdx]].y - polygon[result[rIdx]].y;
      direction.normalize();

      if (std::abs(direction.dot(normal)) > float(M_SQRT1_2)) {
        std::swap(normal[0], normal[1]);
        normal[0] = -normal[0];
      }

      // needs to be on the left side of the edge
      if (direction[0] * normal[1] < direction[1] * normal[0])
        normal *= -1.0;

      lines[rIdx].head<2>().matrix() = normal;
      lines[rIdx][2] = -normal.dot(centroid);
    }

    float threshold2 = threshold * threshold;
    for (std::size_t rIdx = 0; rIdx < lines.size(); ++rIdx) {
      std::size_t nIdx = rIdx + 1;
      if (nIdx == result.size())
        nIdx = 0;

      Eigen::Vector3f vertex = lines[rIdx].cross(lines[nIdx]);
      vertex /= vertex[2];
      vertex[2] = 0.0;

      PointT point;
      // test whether we need another edge since the intersection point is too far away
      // from the original vertex
      Eigen::Vector3f pq = polygon[result[nIdx]].getVector3fMap() - vertex;
      pq[2] = 0.0;

      float distance = pq.squaredNorm();
      if (distance > threshold2) {
        // test whether the old point is inside the new polygon or outside
        if ((pq[0] * lines[rIdx][0] + pq[1] * lines[rIdx][1] < 0.0) &&
            (pq[0] * lines[nIdx][0] + pq[1] * lines[nIdx][1] < 0.0)) {
          float distance1 = lines[rIdx][0] * polygon[result[nIdx]].x +
                            lines[rIdx][1] * polygon[result[nIdx]].y + lines[rIdx][2];
          float distance2 = lines[nIdx][0] * polygon[result[nIdx]].x +
                            lines[nIdx][1] * polygon[result[nIdx]].y + lines[nIdx][2];

          point.x = polygon[result[nIdx]].x - distance1 * lines[rIdx][0];
          point.y = polygon[result[nIdx]].y - distance1 * lines[rIdx][1];

          approx_polygon.push_back(point);

          vertex[0] = polygon[result[nIdx]].x - distance2 * lines[nIdx][0];
          vertex[1] = polygon[result[nIdx]].y - distance2 * lines[nIdx][1];
        }
      }
      point.getVector3fMap() = vertex;
      approx_polygon.push_back(point);
    }
  }
  else {
    // we have a new polygon in results, but inverted (clockwise <-> counter-clockwise)
    for (std::vector<unsigned>::reverse_iterator it = result.rbegin();
         it != result.rend();
         ++it)
      approx_polygon.push_back(polygon[*it]);
  }
}

#endif // PCL_GEOMETRY_POLYGON_OPERATIONS_HPP_

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

#include <set>
#include <vector> // for vector

namespace pcl {

/** K-means clustering.
 *
 * \author Christian Potthast
 * \ingroup ML
 */
class PCL_EXPORTS Kmeans {
public:
  using PointId = unsigned int;   // the id of this point
  using ClusterId = unsigned int; // the id of this cluster

  // using Point = std::vector<Coord>;    // a point (a centroid)

  using SetPoints = std::set<PointId>; // set of points

  using Point = std::vector<float>;

  // ClusterId -> (PointId, PointId, PointId, .... )
  using ClustersToPoints = std::vector<SetPoints>;
  // PointId -> ClusterId
  using PointsToClusters = std::vector<ClusterId>;
  // coll of centroids
  using Centroids = std::vector<Point>;

  /** Empty constructor. */
  Kmeans(unsigned int num_points, unsigned int num_dimensions);

  /** This destructor destroys. */
  ~Kmeans();

  /** This method sets the k-means cluster size.
   *
   * \param[in] k number of clusters
   */
  void
  setClusterSize(unsigned int k)
  {
    num_clusters_ = k;
  };

  /*
        void
        setClusterField (std::string field_name)
        {
          cluster_field_name_ = field_name;
        };
  */

  // void
  // getClusterCentroids (PointT &out);

  // void
  // cluster (std::vector<PointIndices> &clusters);

  void
  kMeans();

  void
  setInputData(std::vector<Point>& data)
  {
    if (num_points_ != data.size())
      std::cout << "Data vector not the same" << std::endl;

    data_ = data;
  }

  void
  addDataPoint(Point& data_point)
  {
    if (num_dimensions_ != data_point.size())
      std::cout << "Dimensions not the same" << std::endl;

    data_.push_back(data_point);
  }

  // Initial partition points among available clusters
  void
  initialClusterPoints();

  void
  computeCentroids();

  // distance between two points
  float
  distance(const Point& x, const Point& y)
  {
    float total = 0.0;
    float diff;

    auto cpy = y.cbegin();
    for (auto cpx = x.cbegin(), cpx_end = x.cend(); cpx != cpx_end; ++cpx, ++cpy) {
      diff = *cpx - *cpy;
      total += (diff * diff);
    }
    return total; // no need to take sqrt, which is monotonic
  }

  Centroids
  get_centroids()
  {
    return centroids_;
  }

protected:
  // Members derived from the base class
  /*
        using BasePCLBase::input_;
        using BasePCLBase::indices_;
        using BasePCLBase::initCompute;
        using BasePCLBase::deinitCompute;
  */

  unsigned int num_points_;
  unsigned int num_dimensions_;

  /** The number of clusters. */
  unsigned int num_clusters_;

  /** The cluster centroids. */
  // std::vector

  // std::string cluster_field_name_;

  // one data point

  // all data points
  std::vector<Point> data_;

  ClustersToPoints clusters_to_points_;
  PointsToClusters points_to_clusters_;
  Centroids centroids_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace pcl

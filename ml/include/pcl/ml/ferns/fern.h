/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl/common/common.h>

#include <istream>
#include <ostream>

namespace pcl {

/** Class representing a Fern. */
template <class FeatureType, class NodeType>
class PCL_EXPORTS Fern {
public:
  /** Constructor. */
  Fern() : num_of_decisions_(0), features_(0), thresholds_(0), nodes_(1) {}

  /** Destructor. */
  virtual ~Fern() {}

  /** Initializes the fern.
   *
   * \param num_of_decisions the number of decisions taken to access the nodes
   */
  void
  initialize(const std::size_t num_of_decisions)
  {
    num_of_decisions_ = num_of_decisions;
    features_.resize(num_of_decisions_);
    thresholds_.resize(num_of_decisions_, std::numeric_limits<float>::quiet_NaN());
    nodes_.resize(0x1 << num_of_decisions_);
  }

  /** Returns the number of nodes the Fern has. */
  inline std::size_t
  getNumOfNodes()
  {
    return 0x1U << num_of_decisions_;
  }

  /** Returns the number of features the Fern has. */
  inline std::size_t
  getNumOfFeatures()
  {
    return num_of_decisions_;
  }

  /** Serializes the fern.
   *
   * \param[out] stream the destination for the serialization
   */
  void
  serialize(::std::ostream& stream) const
  {
    // const int tmp_value = static_cast<int> (num_of_decisions_);
    // stream.write (reinterpret_cast<char*> (&tmp_value), sizeof (tmp_value));
    stream.write(reinterpret_cast<const char*>(&num_of_decisions_),
                 sizeof(num_of_decisions_));

    for (std::size_t feature_index = 0; feature_index < features_.size();
         ++feature_index) {
      features_[feature_index].serialize(stream);
    }

    for (std::size_t threshold_index = 0; threshold_index < thresholds_.size();
         ++threshold_index) {
      stream.write(reinterpret_cast<const char*>(&(thresholds_[threshold_index])),
                   sizeof(thresholds_[threshold_index]));
    }

    for (std::size_t node_index = 0; node_index < nodes_.size(); ++node_index) {
      nodes_[node_index].serialize(stream);
    }
  }

  /** Deserializes the fern.
   *
   * \param[in] stream the source for the deserialization
   */
  void
  deserialize(::std::istream& stream)
  {
    stream.read(reinterpret_cast<char*>(&num_of_decisions_), sizeof(num_of_decisions_));

    features_.resize(num_of_decisions_);
    thresholds_.resize(num_of_decisions_);
    nodes_.resize(0x1 << num_of_decisions_);

    for (std::size_t feature_index = 0; feature_index < features_.size();
         ++feature_index) {
      features_[feature_index].deserialize(stream);
    }

    for (std::size_t threshold_index = 0; threshold_index < thresholds_.size();
         ++threshold_index) {
      stream.read(reinterpret_cast<char*>(&(thresholds_[threshold_index])),
                  sizeof(thresholds_[threshold_index]));
    }

    for (std::size_t node_index = 0; node_index < nodes_.size(); ++node_index) {
      nodes_[node_index].deserialize(stream);
    }
  }

  /** Access operator for nodes.
   *
   * \param node_index the index of the node to access
   */
  inline NodeType&
  operator[](const std::size_t node_index)
  {
    return nodes_[node_index];
  }

  /** Access operator for nodes.
   *
   * \param node_index the index of the node to access
   */
  inline const NodeType&
  operator[](const std::size_t node_index) const
  {
    return nodes_[node_index];
  }

  /** Access operator for features.
   *
   * \param feature_index the index of the feature to access
   */
  inline FeatureType&
  accessFeature(const std::size_t feature_index)
  {
    return features_[feature_index];
  }

  /** Access operator for features.
   *
   * \param feature_index the index of the feature to access
   */
  inline const FeatureType&
  accessFeature(const std::size_t feature_index) const
  {
    return features_[feature_index];
  }

  /** Access operator for thresholds.
   *
   * \param threshold_index the index of the threshold to access
   */
  inline float&
  accessThreshold(const std::size_t threshold_index)
  {
    return thresholds_[threshold_index];
  }

  /** Access operator for thresholds.
   *
   * \param threshold_index the index of the threshold to access
   */
  inline const float&
  accessThreshold(const std::size_t threshold_index) const
  {
    return thresholds_[threshold_index];
  }

private:
  /** The number of decisions. */
  std::size_t num_of_decisions_;
  /** The list of Features used to make the decisions. */
  std::vector<FeatureType> features_;
  /** The list of thresholds used to make the decisions. */
  std::vector<float> thresholds_;
  /** The list of Nodes accessed by the Fern. */
  std::vector<NodeType> nodes_;
};

} // namespace pcl

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
#include <pcl/ml/dt/decision_tree.h>

#include <istream>
#include <ostream>

namespace pcl {

/** Class representing a decision forest. */
template <class NodeType>
class PCL_EXPORTS DecisionForest : public std::vector<pcl::DecisionTree<NodeType>> {

public:
  /** Constructor. */
  DecisionForest() = default;

  /** Destructor. */
  virtual ~DecisionForest() = default;

  /** Serializes the decision tree.
   *
   * \param[out] stream The destination for the serialization
   */
  void
  serialize(::std::ostream& stream) const
  {
    const int num_of_trees = static_cast<int>(this->size());
    stream.write(reinterpret_cast<const char*>(&num_of_trees), sizeof(num_of_trees));

    for (std::size_t tree_index = 0; tree_index < this->size(); ++tree_index) {
      (*this)[tree_index].serialize(stream);
    }

    // const int num_of_trees = static_cast<int> (trees_.size ());
    // stream.write (reinterpret_cast<const char*> (&num_of_trees), sizeof
    // (num_of_trees));

    // for (std::size_t tree_index = 0; tree_index < trees_.size (); ++tree_index)
    //{
    //  tree_[tree_index].serialize (stream);
    //}
  }

  /** Deserializes the decision tree.
   *
   * \param[in] stream The source for the deserialization
   */
  void
  deserialize(::std::istream& stream)
  {
    int num_of_trees;
    stream.read(reinterpret_cast<char*>(&num_of_trees), sizeof(num_of_trees));
    this->resize(num_of_trees);

    for (std::size_t tree_index = 0; tree_index < this->size(); ++tree_index) {
      (*this)[tree_index].deserialize(stream);
    }

    // int num_of_trees;
    // stream.read (reinterpret_cast<char*> (&num_of_trees), sizeof (num_of_trees));
    // trees_.resize (num_of_trees);

    // for (std::size_t tree_index = 0; tree_index < trees_.size (); ++tree_index)
    //{
    //  tree_[tree_index].deserialize (stream);
    //}
  }

private:
  /** The decision trees contained in the forest. */
  // std::vector<DecisionTree<NodeType> > trees_;
};

} // namespace pcl

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

namespace pcl
{

  /** \brief Class representing a decision tree. */
  template <class NodeType>
  class PCL_EXPORTS DecisionTree
  {
  
    public:

      /** \brief Constructor. */
      DecisionTree () : root_ () {}
      /** \brief Destructor. */
      virtual 
      ~DecisionTree () {}

      /** \brief Sets the root node of the tree.
        * \param[in] root The root node.
        */
      void
      setRoot (const NodeType & root)
      {
        root_ = root;
      }

      /** \brief Returns the root node of the tree. */
      NodeType &
      getRoot ()
      {
        return root_;
      }

      /** \brief Serializes the decision tree. 
        * \param[out] stream The destination for the serialization.
        */
      void 
      serialize (::std::ostream & stream) const
      {
        root_.serialize (stream);
      }

      /** \brief Deserializes the decision tree. 
        * \param[in] stream The source for the deserialization.
        */
      void deserialize (::std::istream & stream)
      {
        root_.deserialize (stream);
      }

    private:

      /** \brief The root node of the decision tree. */
      NodeType root_;

  };


}

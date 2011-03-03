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
 * $Id: reconstruction.h 35859 2011-02-08 22:44:43Z rusu $
 *
 */

#ifndef PCL_SURFACE_RECONSTRUCTION_H_
#define PCL_SURFACE_RECONSTRUCTION_H_

#include <pcl/pcl_base.h>
#include <pcl/PolygonMesh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ros/conversions.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace pcl
{
  /** \brief @b SurfaceReconstruction represents the base surface reconstruction class. 
    * \author Radu Bogdan Rusu
    */
  template <typename PointInT>
  class SurfaceReconstruction: public PCLBase<PointInT>
  {
    public:
      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

      typedef typename pcl::KdTree<PointInT> KdTree;
      typedef typename pcl::KdTree<PointInT>::Ptr KdTreePtr;

      typedef boost::function<int (int, double, std::vector<int> &, std::vector<float> &)> SearchMethod;

      /** \brief Constructor. */
      SurfaceReconstruction () : tree_() {}

      /** \brief Base method for surface reconstruction for all points given in
        * <setInputCloud (), setIndices ()> 
        * \param output the resultant reconstructed surface model
        */
      void reconstruct (pcl::PolygonMesh &output);

      /** \brief Provide a pointer to the search object.
        * \param tree a pointer to the spatial search object.
        */
      inline void
      setSearchMethod (const KdTreePtr &tree)
      {
        tree_ = tree;
        // Declare the search locator definition
        int (KdTree::*nearestKSearch)(int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) = &KdTree::nearestKSearch;
        search_method_ = boost::bind (nearestKSearch, boost::ref (tree_), _1, _2, _3, _4);
      }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr getSearchMethod () { return (tree_); }

    protected:
      /** \brief The search method template for indices. */
      SearchMethod search_method_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief Abstract surface reconstruction method. */
      virtual void performReconstruction (pcl::PolygonMesh &output) = 0;

      /** \brief Abstract class get name method. */
      virtual std::string getClassName () const { return (""); }
  };
}

#include "pcl/surface/impl/reconstruction.hpp"

#endif  // PCL_SURFACE_RECONSTRUCTION_H_


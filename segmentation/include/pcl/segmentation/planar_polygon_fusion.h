/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#pragma once

#include <Eigen/Core>
#include <vector>
#include <pcl/segmentation/planar_region.h>

namespace pcl
{
  /** \brief PlanarPolygonFusion takes a list of 2D planar polygons and
    * attempts to reduce them to a minimum set that best represents the scene,
    * based on various given comparators.
    */
  template <typename PointT>
  class PlanarPolygonFusion
  {
    public:
      /** \brief Constructor */
      PlanarPolygonFusion () : regions_ () {}
     
      /** \brief Destructor */
      virtual ~PlanarPolygonFusion () {}

      /** \brief Reset the state (clean the list of planar models). */
      void 
      reset ()
      {
        regions_.clear ();
      }
      
      /** \brief Set the list of 2D planar polygons to refine.
        * \param[in] input the list of 2D planar polygons to refine
        */
      void
      addInputPolygons (const std::vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT> > > &input)
      {
        int start = static_cast<int> (regions_.size ());
        regions_.resize (regions_.size () + input.size ());
        for(std::size_t i = 0; i < input.size (); i++)
          regions_[start+i] = input[i];
      }

    protected:
      /** \brief Internal list of planar states. */
      std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/planar_polygon_fusion.hpp>
#endif

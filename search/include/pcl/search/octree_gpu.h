/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Siddharth Choudhary (itzsid@gmail.com)
 */

#ifndef PCL_SEARCH_OCTREE_GPU_H_
#define PCL_SEARCH_OCTREE_GPU_H_

#include "pcl/search/search.h"
#include "pcl/octree/impl/octree_base.hpp"




namespace pcl
{

  namespace search
  {

  /** \brief @b search::OctreeGPU is a wrapper class around the octree functions written for GPU 
     */
    template <typename PointT>
    class OctreeGPU : public pcl::Search<PointT>
    {

    // acts as an interface to the octree GPU implementation

      public:
      OctreeGPU(){}
      ~OctreeGPU(){}

      typedef typename Search<PointT>::PointCloud PointCloud;
      typedef typename Search<PointT>::PointCloudConstPtr PointCloudConstPtr;

      /** \brief Provide a pointer to the input dataset.
      * \param cloud the const boost shared pointer to a PointCloud message
      */

      void setInputCloud( const PointCloudConstPtr& cloud);

      /** \brief Search for all the nearest neighbors of the query points in the given radiuses.
        * \param point the given query points
        * \param radiuses the radiuses of the sphere bounding all of point's neighbors
        * \param k_indices the resultant indices of the neighboring points
        * \param k_distances the resultant squared distances to the neighboring points
        * \param max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radiuses
      */
      int
      radiusSearch (std::vector<PointT>& point, 
                    std::vector< double>& radiuses, 
                    std::vector<std::vector<int> >& k_indices,    
                    std::vector<std::vector<float> >& k_distances, 
                    int max_nn) const;

    };

  }

}

#endif

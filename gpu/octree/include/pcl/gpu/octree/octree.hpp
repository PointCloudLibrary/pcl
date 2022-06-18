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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#ifndef _PCL_GPU_OCTREE_
#define _PCL_GPU_OCTREE_

#include <limits>
#include <vector>

#include <pcl/memory.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/octree/device_format.hpp>

namespace pcl
{
    namespace gpu
    {   
        /**
         * \brief   Octree implementation on GPU. It suppors parallel building and parallel batch search as well .       
         * \author  Anaoly Baksheev, Itseez, myname.mysurname@mycompany.com
         */

        class PCL_EXPORTS Octree
        {
        public:

            /** \brief Default constructor.*/             
            Octree();
           
            /** \brief Denstructor.*/             
            virtual ~Octree();

            /** \brief Types */
            using Ptr = shared_ptr<Octree>;
            using ConstPtr = shared_ptr<const Octree>;

            /** \brief Point typwe supported */
            using PointType = pcl::PointXYZ;

            /** \brief Point cloud supported */
            using PointCloud = DeviceArray<PointType>;
            
            /** \brief Point Batch query cloud type */
            using Queries = DeviceArray<PointType>;

            /** \brief Point Radiuses for batch query  */
            using Radiuses = DeviceArray<float>;            

            /** \brief Point Indices for batch query  */
            using Indices = DeviceArray<int>;    
            
            /** \brief Point Sqrt distances array type */
            using ResultSqrDists = DeviceArray<float>;
            
            const PointCloud*   cloud_;
            
            /** \brief Sets cloud for which octree is built */            
            void setCloud(const PointCloud& cloud_arg);

            /** \brief Performs parallel octree building */
			void build();

            /** \brief Returns true if tree has been built */
            bool isBuilt() const;

            /** \brief Downloads Octree from GPU to search using CPU function. It use useful for single (not-batch) search */
            void internalDownload();

            /** \brief Performs search of all points within given radius on CPU. It call \a internalDownload if necessary
              * \param[in] center center of sphere
              * \param[in] radius radious of sphere
              * \param[out] out indeces of points within give sphere
              * \param[in] max_nn maximum numver of results returned
              */
            void radiusSearchHost(const PointType& center, float radius, std::vector<int>& out,
                                  int max_nn = std::numeric_limits<int>::max());

            /** \brief Performs approximate nearest neighbor search on CPU. It call \a internalDownload if necessary
              * \param[in]  query 3D point for which neighbour is be fetched             
              * \param[out] out_index neighbour index
              * \param[out] sqr_dist square distance to the neighbour returned
              */
            void approxNearestSearchHost(const PointType& query, int& out_index, float& sqr_dist);

            /** \brief Performs batch radius search on GPU
              * \param[in] centers array of centers 
              * \param[in] radius radius for all queries
              * \param[in] max_results max number of returned points for each querey
              * \param[out] result results packed to single array
              */
            void radiusSearch(const Queries& centers, float radius, int max_results, NeighborIndices& result) const;

            /** \brief Performs batch radius search on GPU
              * \param[in] centers array of centers 
              * \param[in] radiuses array of radiuses
              * \param[in] max_results max number of returned points for each querey
              * \param[out] result results packed to single array
              */
            void radiusSearch(const Queries& centers, const Radiuses& radiuses, int max_results, NeighborIndices& result) const;

            /** \brief Performs batch radius search on GPU
              * \param[in] centers array of centers  
              * \param[in] indices indices for centers array (only for these points search is performed)
              * \param[in] radius radius for all queries
              * \param[in] max_results max number of returned points for each querey
              * \param[out] result results packed to single array
              */
            void radiusSearch(const Queries& centers, const Indices& indices, float radius, int max_results, NeighborIndices& result) const;

            /** \brief Batch approximate nearest search on GPU
              * \param[in] queries array of centers
              * \param[out] result array of results ( one index for each query ) 
              */
            PCL_DEPRECATED(1, 14, "use approxNearestSearch() which returns square distances instead")
            void approxNearestSearch(const Queries& queries, NeighborIndices& result) const;

            /** \brief Batch approximate nearest search on GPU
              * \param[in] queries array of centers
              * \param[out] result array of results ( one index for each query )
              * \param[out] sqr_distance corresponding square distances to results from query point
              */
            void approxNearestSearch(const Queries& queries, NeighborIndices& result, ResultSqrDists& sqr_distance) const;

            /** \brief Batch exact k-nearest search on GPU for k == 1 only!
              * \param[in] queries array of centers
              * \param[in] k number of neighbors (only k == 1 is supported)
              * \param[out] results array of results
              */
            void nearestKSearchBatch(const Queries& queries, int k, NeighborIndices& results) const;

            /** \brief Batch exact k-nearest search on GPU for k == 1 only!
              * \param[in] queries array of centers
              * \param[in] k number of neighbors (only k == 1 is supported)
              * \param[out] results array of results
              * \param[out] sqr_distances square distances to results
              */
            void nearestKSearchBatch(const Queries& queries, int k, NeighborIndices& results, ResultSqrDists& sqr_distances) const;

            /** \brief Desroys octree and release all resources */
            void clear();            
        private:
            void *impl;            
            bool built_;
        };        

      /** \brief Performs brute force radius search on GPU
        * \param[in] cloud cloud where to search
        * \param[in] query query point
        * \param[in] radius radius
        * \param[out] result indeces of points within give sphere
        * \param[in] buffer buffer for intermediate results. Keep reference to it between calls to eliminate internal allocations
        */             
        PCL_EXPORTS void bruteForceRadiusSearchGPU(const Octree::PointCloud& cloud, const Octree::PointType& query, float radius, DeviceArray<int>& result, DeviceArray<int>& buffer);
    }
}

#endif /* _PCL_GPU_OCTREE_ */

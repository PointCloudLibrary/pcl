/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#pragma once

#include <vector>

#include <pcl/memory.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/octree/device_format.hpp>
#include <pcl/pcl_exports.h>


namespace pcl
{
    namespace gpu
    {   
        namespace details{
        /**
         * \brief   Octree implementation on GPU. It suppors parallel building and parallel batch search as well .       
         * \author  Anaoly Baksheev, Itseez, myname.mysurname@mycompany.com
         */

        template <class T = pcl::PointXYZ>
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

            /** \brief Point type supported */
            using PointType = T;

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
            void radiusSearchHost(const PointType& center, float radius, std::vector<int>& out, int max_nn = INT_MAX);

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
        template <class T>
        PCL_EXPORTS void bruteForceRadiusSearchGPU(const typename Octree<T>::PointCloud& cloud, const typename Octree<T>::PointType& query, float radius, DeviceArray<int>& result, DeviceArray<int>& buffer);
    }
}
}
namespace pcl {
namespace gpu {
using Octree = pcl::gpu::details::Octree<pcl::PointXYZ>;

const auto bruteForceRadiusSearchGPU =
    pcl::gpu::details::bruteForceRadiusSearchGPU<pcl::PointXYZ>;

template <class PointT>
using OcTree [[deprectated("Will be replaced by Octree at PCL 1.15")]] =
    pcl::gpu::details::Octree<PointT>;

template <class PointT>
const auto BruteForceRadiusSearchGPU
    [[deprectated("Will be replaced by bruteForceRadiusSearchGPU at PCL 1.15")]] =
        pcl::gpu::details::bruteForceRadiusSearchGPU<PointT>;

} // namespace gpu
} // namespace pcl

#include <pcl/gpu/octree/impl/octree.hpp>

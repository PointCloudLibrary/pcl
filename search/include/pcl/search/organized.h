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
 * $Id$
 *
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/common/eigen.h>

#include <algorithm>
#include <vector>

namespace pcl
{
  namespace search
  {
    /** \brief OrganizedNeighbor is a class for optimized nearest neigbhor search in organized point clouds.
      * \author Radu B. Rusu, Julius Kammerl, Suat Gedikli, Koen Buys
      * \ingroup search
      */
    template<typename PointT>
    class OrganizedNeighbor : public pcl::search::Search<PointT>
    {

      public:
        // public typedefs
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;

        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using Ptr = shared_ptr<pcl::search::OrganizedNeighbor<PointT> >;
        using ConstPtr = shared_ptr<const pcl::search::OrganizedNeighbor<PointT> >;

        using pcl::search::Search<PointT>::indices_;
        using pcl::search::Search<PointT>::sorted_results_;
        using pcl::search::Search<PointT>::input_;

        /** \brief Constructor
          * \param[in] sorted_results whether the results should be return sorted in ascending order on the distances or not.
          *        This applies only for radius search, since knn always returns sorted resutls    
          * \param[in] eps the threshold for the mean-squared-error of the estimation of the projection matrix.
          *            if the MSE is above this value, the point cloud is considered as not from a projective device,
          *            thus organized neighbor search can not be applied on that cloud.
          * \param[in] pyramid_level the level of the down sampled point cloud to be used for projection matrix estimation
          */
        OrganizedNeighbor (bool sorted_results = false, float eps = 1e-4f, unsigned pyramid_level = 5)
          : Search<PointT> ("OrganizedNeighbor", sorted_results)
          , projection_matrix_ (Eigen::Matrix<float, 3, 4, Eigen::RowMajor>::Zero ())
          , KR_ (Eigen::Matrix<float, 3, 3, Eigen::RowMajor>::Zero ())
          , KR_KRT_ (Eigen::Matrix<float, 3, 3, Eigen::RowMajor>::Zero ())
          , eps_ (eps)
          , pyramid_level_ (pyramid_level)
        {
        }

        /** \brief Empty deconstructor. */
        ~OrganizedNeighbor () {}

        /** \brief Test whether this search-object is valid (input is organized AND from projective device)
          *        User should use this method after setting the input cloud, since setInput just prints an error 
          *        if input is not organized or a projection matrix could not be determined.
          * \return true if the input data is organized and from a projective device, false otherwise
          */
        bool 
        isValid () const
        {
          // determinant (KR) = determinant (K) * determinant (R) = determinant (K) = f_x * f_y.
          // If we expect at max an opening angle of 170degree in x-direction -> f_x = 2.0 * width / tan (85 degree);
          // 2 * tan (85 degree) ~ 22.86
          float min_f = 0.043744332f * static_cast<float>(input_->width);
          //std::cout << "isValid: " << determinant3x3Matrix<Eigen::Matrix3f> (KR_ / sqrt (KR_KRT_.coeff (8))) << " >= " << (min_f * min_f) << std::endl;
          return (determinant3x3Matrix<Eigen::Matrix3f> (KR_ / std::sqrt (KR_KRT_.coeff (8))) >= (min_f * min_f));
        }
        
        /** \brief Compute the camera matrix
          * \param[out] camera_matrix the resultant computed camera matrix 
          */
        void 
        computeCameraMatrix (Eigen::Matrix3f& camera_matrix) const;
        
        /** \brief Provide a pointer to the input data set, if user has focal length he must set it before calling this
          * \param[in] cloud the const boost shared pointer to a PointCloud message
          * \param[in] indices the const boost shared pointer to PointIndices
          */
        void
        setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr &indices = IndicesConstPtr ()) override
        {
          input_ = cloud;
          
          mask_.resize (input_->size ());
          input_ = cloud;
          indices_ = indices;

          if (indices_ && !indices_->empty())
          {
            mask_.assign (input_->size (), 0);
            for (const auto& idx : *indices_)
              mask_[idx] = 1;
          }
          else
            mask_.assign (input_->size (), 1);

          estimateProjectionMatrix ();
        }

        /** \brief Search for all neighbors of query point that are within a given radius.
          * \param[in] p_q the given query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
          * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
          * returned.
          * \return number of neighbors found in radius
          */
        int
        radiusSearch (const PointT &p_q,
                      double radius,
                      Indices &k_indices,
                      std::vector<float> &k_sqr_distances,
                      unsigned int max_nn = 0) const override;

        /** \brief estimated the projection matrix from the input cloud. */
        void 
        estimateProjectionMatrix ();

         /** \brief Search for the k-nearest neighbors for a given query point.
           * \note limiting the maximum search radius (with setMaxDistance) can lead to a significant improvement in search speed
           * \param[in] p_q the given query point (\ref setInputCloud must be given a-priori!)
           * \param[in] k the number of neighbors to search for (used only if horizontal and vertical window not given already!)
           * \param[out] k_indices the resultant point indices (must be resized to \a k beforehand!)
           * \param[out] k_sqr_distances \note this function does not return distances
           * \return number of neighbors found
           * @todo still need to implements this functionality
          */
        int
        nearestKSearch (const PointT &p_q,
                        int k,
                        Indices &k_indices,
                        std::vector<float> &k_sqr_distances) const override;

        /** \brief projects a point into the image
          * \param[in] p point in 3D World Coordinate Frame to be projected onto the image plane
          * \param[out] q the 2D projected point in pixel coordinates (u,v)
          * @return true if projection is valid, false otherwise
          */
        bool projectPoint (const PointT& p, pcl::PointXY& q) const;
        
      protected:

        struct Entry
        {
          Entry (index_t idx, float dist) : index (idx), distance (dist) {}
          Entry () : index (0), distance (0) {}
          index_t index;
          float distance;
          
          inline bool 
          operator < (const Entry& other) const
          {
            return (distance < other.distance);
          }
        };

        /** \brief test if point given by index is among the k NN in results to the query point.
          * \param[in] query query point
          * \param[in] k number of maximum nn interested in
          * \param[in,out] queue priority queue with k NN
          * \param[in] index index on point to be tested
          * \return whether the top element changed or not.
          */
        inline bool 
        testPoint (const PointT& query, unsigned k, std::vector<Entry>& queue, index_t index) const
        {
          const PointT& point = input_->points [index];
          if (mask_ [index] && std::isfinite (point.x))
          {
            //float squared_distance = (point.getVector3fMap () - query.getVector3fMap ()).squaredNorm ();
            float dist_x = point.x - query.x;
            float dist_y = point.y - query.y;
            float dist_z = point.z - query.z;
            float squared_distance = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
            const auto queue_size = queue.size ();
            const auto insert_into_queue = [&]{ queue.emplace (
                                                std::upper_bound (queue.begin(), queue.end(), squared_distance,
                                                [](float dist, const Entry& ent){ return dist<ent.distance; }),
                                                               index, squared_distance); };
            if (queue_size < k)
            {
              insert_into_queue ();
              return (queue_size + 1) == k;
            }
            if (queue.back ().distance > squared_distance)
            {
              queue.pop_back ();
              insert_into_queue ();
              return true; // top element has changed!
            }
          }
          return false;
        }

        inline void
        clipRange (int& begin, int &end, int min, int max) const
        {
          begin = std::max (std::min (begin, max), min);
          end   = std::min (std::max (end, min), max);
        }

        /** \brief Obtain a search box in 2D from a sphere with a radius in 3D
          * \param[in] point the query point (sphere center)
          * \param[in] squared_radius the squared sphere radius
          * \param[out] minX the min X box coordinate
          * \param[out] minY the min Y box coordinate
          * \param[out] maxX the max X box coordinate
          * \param[out] maxY the max Y box coordinate
          */
        void
        getProjectedRadiusSearchBox (const PointT& point, float squared_radius, unsigned& minX, unsigned& minY,
                                     unsigned& maxX, unsigned& maxY) const;


        /** \brief the projection matrix. Either set by user or calculated by the first / each input cloud */
        Eigen::Matrix<float, 3, 4, Eigen::RowMajor> projection_matrix_;

        /** \brief inveser of the left 3x3 projection matrix which is K * R (with K being the camera matrix and R the rotation matrix)*/
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> KR_;

        /** \brief inveser of the left 3x3 projection matrix which is K * R (with K being the camera matrix and R the rotation matrix)*/
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> KR_KRT_;

        /** \brief epsilon value for the MSE of the projection matrix estimation*/
        const float eps_;

        /** \brief using only a subsample of points to calculate the projection matrix. pyramid_level_ = use down sampled cloud given by pyramid_level_*/
        const unsigned pyramid_level_;
        
        /** \brief mask, indicating whether the point was in the indices list or not.*/
        std::vector<unsigned char> mask_;
      public:
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/search/impl/organized.hpp>
#endif

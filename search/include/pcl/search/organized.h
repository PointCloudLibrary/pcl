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
 * $Id: organized.h 4504 2012-02-17 00:47:06Z gedikli $
 *
 */

#ifndef PCL_SEARCH_ORGANIZED_NEIGHBOR_SEARCH_H_
#define PCL_SEARCH_ORGANIZED_NEIGHBOR_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/common/eigen.h>

#include <algorithm>
#include <queue>
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
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;

        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef boost::shared_ptr<pcl::search::OrganizedNeighbor<PointT> > Ptr;
        typedef boost::shared_ptr<const pcl::search::OrganizedNeighbor<PointT> > ConstPtr;

        using pcl::search::Search<PointT>::indices_;
        using pcl::search::Search<PointT>::sorted_results_;
        using pcl::search::Search<PointT>::input_;

        /** \brief OrganizedNeighbor constructor. */
        OrganizedNeighbor (bool sorted_results = false)
          : Search<PointT> ("OrganizedNeighbor", sorted_results)
          , projection_matrix_ (Eigen::Matrix<float, 3, 4, Eigen::RowMajor>::Zero ())
          , eps_ (1e-6)
        {
        }

        /** \brief Empty deconstructor. */
        virtual ~OrganizedNeighbor () {}

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
          float min_f = 0.043744332 * input_->width;
          //std::cout << "isValid: " << determinant3x3Matrix<Eigen::Matrix3f> (KR_ / sqrt (KR_KRT_.coeff (8))) << " >= " << (min_f * min_f) << std::endl;
          return (determinant3x3Matrix<Eigen::Matrix3f> (KR_ / sqrt (KR_KRT_.coeff (8))) >= (min_f * min_f));
        }
        
        void computeCameraMatrix (Eigen::Matrix3f& camera_matrix) const;
        /** \brief Provide a pointer to the input data set, if user has focal length he must set it before calling this
          * \param[in] cloud the const boost shared pointer to a PointCloud message
          * \param[in] indices the const boost shared pointer to PointIndices
          */
        virtual void
        setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr &indices = IndicesConstPtr ())
        {
          bool input_changed = false;
          if (input_ != cloud)
          {
            input_ = cloud;
            input_changed = true;
            mask_.resize (input_->size ());
          }

          if (indices_ != indices)
          {
            indices_ = indices;
            input_changed = true;
          }

          if (input_changed)
          {
            if (indices_.get () != NULL && indices_->size () != 0)
            {
              mask_.assign (input_->size (), false);
              for (std::vector<int>::const_iterator iIt = indices_->begin (); iIt != indices_->end (); ++iIt)
                mask_[*iIt] = true;
            }
            else
              mask_.assign (input_->size (), true);

            estimateProjectionMatrix ();
          }
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
                      const double radius,
                      std::vector<int> &k_indices,
                      std::vector<float> &k_sqr_distances,
                      unsigned int max_nn = 0) const;

        /** \brief estimated the projection matrix from the input cloud
         */
        void estimateProjectionMatrix ();

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
                        std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances) const;

      protected:

        struct Entry
        {
          Entry (int idx, float dist) : index (idx), distance (dist) {}
          Entry () : index (0), distance (0) {}
          unsigned index;
          float distance;
          
          inline bool 
          operator < (const Entry& other) const
          {
            return (distance < other.distance);
          }
        };

        /** \brief test if point given by index is among the k NN in results to the query point.
         *  \param query query point
         *  \param k number of maximum nn interested in
         *  \param queue priority queue with k NN
         *  \param index index on point to be tested
         *  \return wheter the top element changed or not.
         */
        inline bool testPoint (const PointT& query, unsigned k, std::priority_queue<Entry>& queue, unsigned index) const
        {
          const PointT& point = input_->points [index];
          if (mask_ [index] && pcl_isfinite (point.x))
          {
            float squared_distance = (point.getVector3fMap () - query.getVector3fMap ()).squaredNorm ();
            if (queue.size () < k)
              queue.push (Entry (index, squared_distance));
            else if (queue.top ().distance > squared_distance)
            {
              queue.pop ();
              queue.push (Entry (index, squared_distance));
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


        /** \brief copys upper or lower triangular part of the matrix to the other one */
        template <typename MatrixType> void
        makeSymmetric (MatrixType& matrix, bool use_upper_triangular = true) const;

        /** \brief the projection matrix. Either set by user or calculated by the first / each input cloud */
        Eigen::Matrix<float, 3, 4, Eigen::RowMajor> projection_matrix_;

        /** \brief inveser of the left 3x3 projection matrix which is K * R (with K being the camera matrix and R the rotation matrix)*/
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> KR_;

        /** \brief inveser of the left 3x3 projection matrix which is K * R (with K being the camera matrix and R the rotation matrix)*/
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> KR_KRT_;

        /** \brief epsilon value for the MSE of the projection matrix estimation*/
        float eps_;

        /** \brief mask, indicating whether the point was in the indices list or not.*/
        std::vector<bool> mask_;
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

#endif


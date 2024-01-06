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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <limits>
#include <queue>
#include <vector>

namespace pcl
{

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b OrganizedNeighborSearch class
   *  \note This class provides neighbor search routines for organized point clouds.
   *  \note
   *  \note typename: PointT: type of point used in pointcloud
   *  \author Julius Kammerl (julius@kammerl.de)
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    class OrganizedNeighborSearch

    {
    public:

      /** \brief OrganizedNeighborSearch constructor.
       * */
      OrganizedNeighborSearch ():
        radiusLookupTableWidth_(-1),
        radiusLookupTableHeight_(-1)
      {
        max_distance_ = std::numeric_limits<double>::max ();

        focalLength_ = 1.0f;
      }

      /** \brief Empty deconstructor. */
      virtual
      ~OrganizedNeighborSearch() = default;

      // public typedefs
      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;


      /** \brief Provide a pointer to the input data set.
       *  \param cloud_arg the const boost shared pointer to a PointCloud message
       */
      inline void
      setInputCloud (const PointCloudConstPtr &cloud_arg)
      {

        if (input_ != cloud_arg)
        {
          input_ = cloud_arg;

          estimateFocalLengthFromInputCloud ();
          generateRadiusLookupTable (input_->width, input_->height);
        }
      }

      /** \brief Search for all neighbors of query point that are within a given radius.
       * \param cloud_arg the point cloud data
       * \param index_arg the index in \a cloud representing the query point
       * \param radius_arg the radius of the sphere bounding all of p_q's neighbors
       * \param k_indices_arg the resultant indices of the neighboring points
       * \param k_sqr_distances_arg the resultant squared distances to the neighboring points
       * \param max_nn_arg if given, bounds the maximum returned neighbors to this value
       * \return number of neighbors found in radius
       */
      int
      radiusSearch (const PointCloudConstPtr &cloud_arg, int index_arg, double radius_arg,
                    std::vector<int> &k_indices_arg, std::vector<float> &k_sqr_distances_arg,
                    int max_nn_arg = std::numeric_limits<int>::max());

      /** \brief Search for all neighbors of query point that are within a given radius.
       * \param index_arg index representing the query point in the dataset given by \a setInputCloud.
       *        If indices were given in setInputCloud, index will be the position in the indices vector
       * \param radius_arg radius of the sphere bounding all of p_q's neighbors
       * \param k_indices_arg the resultant indices of the neighboring points
       * \param k_sqr_distances_arg the resultant squared distances to the neighboring points
       * \param max_nn_arg if given, bounds the maximum returned neighbors to this value
       * \return number of neighbors found in radius
       */
      int
      radiusSearch (int index_arg, const double radius_arg, std::vector<int> &k_indices_arg,
                    std::vector<float> &k_sqr_distances_arg, int max_nn_arg = std::numeric_limits<int>::max()) const;

      /** \brief Search for all neighbors of query point that are within a given radius.
       * \param p_q_arg the given query point
       * \param radius_arg the radius of the sphere bounding all of p_q's neighbors
       * \param k_indices_arg the resultant indices of the neighboring points
       * \param k_sqr_distances_arg the resultant squared distances to the neighboring points
       * \param max_nn_arg if given, bounds the maximum returned neighbors to this value
       * \return number of neighbors found in radius
       */
      int
      radiusSearch (const PointT &p_q_arg, const double radius_arg, std::vector<int> &k_indices_arg,
                    std::vector<float> &k_sqr_distances_arg, int max_nn_arg = std::numeric_limits<int>::max()) const;

      /** \brief Search for k-nearest neighbors at the query point.
       * \param cloud_arg the point cloud data
       * \param index_arg the index in \a cloud representing the query point
       * \param k_arg the number of neighbors to search for
       * \param k_indices_arg the resultant indices of the neighboring points (must be resized to \a k a priori!)
       * \param k_sqr_distances_arg the resultant squared distances to the neighboring points (must be resized to \a k
       * a priori!)
       * \return number of neighbors found
       */
      int
      nearestKSearch (const PointCloudConstPtr &cloud_arg, int index_arg, int k_arg, std::vector<int> &k_indices_arg,
                      std::vector<float> &k_sqr_distances_arg);

      /** \brief Search for k-nearest neighbors at query point
       * \param index_arg index representing the query point in the dataset given by \a setInputCloud.
       *        If indices were given in setInputCloud, index will be the position in the indices vector.
       * \param k_arg the number of neighbors to search for
       * \param k_indices_arg the resultant indices of the neighboring points (must be resized to \a k a priori!)
       * \param k_sqr_distances_arg the resultant squared distances to the neighboring points (must be resized to \a k
       * a priori!)
       * \return number of neighbors found
       */
      int
      nearestKSearch (int index_arg, int k_arg, std::vector<int> &k_indices_arg,
                      std::vector<float> &k_sqr_distances_arg);

      /** \brief Search for k-nearest neighbors at given query point.
       * @param p_q_arg the given query point
       * @param k_arg the number of neighbors to search for
       * @param k_indices_arg the resultant indices of the neighboring points (must be resized to k a priori!)
       * @param k_sqr_distances_arg  the resultant squared distances to the neighboring points (must be resized to k a priori!)
       * @return number of neighbors found
       */
      int
      nearestKSearch (const PointT &p_q_arg, int k_arg, std::vector<int> &k_indices_arg,
                      std::vector<float> &k_sqr_distances_arg);

      /** \brief Get the maximum allowed distance between the query point and its nearest neighbors. */
      inline double
      getMaxDistance () const
      {
        return (max_distance_);
      }

      /** \brief Set the maximum allowed distance between the query point and its nearest neighbors. */
      inline void
      setMaxDistance (double max_dist)
      {
        max_distance_ = max_dist;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Protected methods
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    protected:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief @b radiusSearchLoopkupEntry entry for radius search lookup vector
       *  \note This class defines entries for the radius search lookup vector
       *  \author Julius Kammerl (julius@kammerl.de)
       */
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      class radiusSearchLoopkupEntry
      {
      public:

        /** \brief Empty constructor  */
        radiusSearchLoopkupEntry ()
        {
        }

        /** \brief Define search point and calculate squared distance
         * @param x_shift shift in x dimension
         * @param y_shift shift in y dimension
         */
        void
        defineShiftedSearchPoint(int x_shift, int y_shift)
        {
          x_diff_ =x_shift;
          y_diff_ =y_shift;

          squared_distance_ = x_diff_ * x_diff_ + y_diff_ * y_diff_;
        }

        /** \brief Operator< for comparing radiusSearchLoopkupEntry instances with each other.  */
        bool
        operator< (const radiusSearchLoopkupEntry& rhs_arg) const
        {
          return (this->squared_distance_ < rhs_arg.squared_distance_);
        }

        // Public globals
        int x_diff_;
        int y_diff_;
        int squared_distance_;

      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief @b nearestNeighborCandidate entry for the nearest neighbor candidate queue
       *  \note This class defines entries for the nearest neighbor candidate queue
       *  \author Julius Kammerl (julius@kammerl.de)
       */
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      class nearestNeighborCandidate
      {
      public:

        /** \brief Empty constructor  */
        nearestNeighborCandidate ()
        {
        }

        /** \brief Operator< for comparing nearestNeighborCandidate instances with each other.  */
        bool
        operator< (const nearestNeighborCandidate& rhs_arg) const
        {
          return (this->squared_distance_ < rhs_arg.squared_distance_);
        }

        // Public globals
        int index_;
        double squared_distance_;

      };

      /** \brief Get point at index from input pointcloud dataset
       * \param index_arg index representing the point in the dataset given by \a setInputCloud
       * \return PointT from input pointcloud dataset
       */
      const PointT&
      getPointByIndex (const unsigned int index_arg) const;

      /** \brief Generate radius lookup table. It is used to subsequentially iterate over points
       *         which are close to the search point
       * \param width of organized point cloud
       * \param height of organized point cloud
       */
      void
      generateRadiusLookupTable (unsigned int width, unsigned int height);

      inline void
      pointPlaneProjection (const PointT& point, int& xpos, int& ypos) const
      {
        xpos = (int) pcl_round(point.x / (point.z * focalLength_));
        ypos = (int) pcl_round(point.y / (point.z * focalLength_));
      }

      void
      getProjectedRadiusSearchBox (const PointT& point_arg, double squared_radius_arg, int& minX_arg, int& minY_arg, int& maxX_arg, int& maxY_arg ) const;


      /** \brief Estimate focal length parameter that was used during point cloud generation
       */
      void
      estimateFocalLengthFromInputCloud ();

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Class getName method. */
      virtual std::string
      getName () const
      {
        return ("Organized_Neighbor_Search");
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Globals
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      /** \brief Pointer to input point cloud dataset. */
      PointCloudConstPtr input_;

      /** \brief Maximum allowed distance between the query point and its k-neighbors. */
      double max_distance_;

      /** \brief Global focal length parameter */
      double focalLength_;

      /** \brief Precalculated radius search lookup vector */
      std::vector<radiusSearchLoopkupEntry> radiusSearchLookup_;
      int radiusLookupTableWidth_;
      int radiusLookupTableHeight_;

    };

}

//#include "organized_neighbor_search.hpp"

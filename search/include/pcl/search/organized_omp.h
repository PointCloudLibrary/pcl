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
 * $Id: organized.h 3220 2011-11-21 15:50:53Z koenbuys $
 *
 */

#ifndef PCL_SEARCH_ORGANIZED_NEIGHBOR_SEARCH_OMP_H_
#define PCL_SEARCH_ORGANIZED_NEIGHBOR_SEARCH_OMP_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>

#include <algorithm>
#include <math.h>
#include <queue>
#include <vector>

namespace pcl
{
  namespace search
  {
    /** \brief @b OrgfanizedNeighbor is a class for optimized nearest neigbhor search in organized point clouds.
      *
      * The current API implements two different methods in the same corpus:
      * <ul>
      *   <li>a fast, simple but approximate, window-based nearest neighbor estimation method, similar to 2D</li>
      *   <li>a slower, but more accurate spherical projection method</li>
      * </ul>
      *
      * You can switch between the two methods using \ref setPrecision (true/false). By default the class uses
      * the faster approximate method.
      *
      * \author Radu B. Rusu, Julius Kammerl, Suat Gedikli, Koen Buys
      */
    template<typename PointT>
    class OrganizedNeighborOMP : public pcl::search::Search<PointT>
    {

      public:
        // public typedefs
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;

        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef boost::shared_ptr<pcl::search::OrganizedNeighborOMP<PointT> > Ptr;
        typedef boost::shared_ptr<const pcl::search::OrganizedNeighborOMP<PointT> > ConstPtr;

        /** \brief OrganizedNeighbor constructor. */
        OrganizedNeighborOMP () : threads_ (1)
        {
          max_distance_ = std::numeric_limits<double>::max ();
          oneOverFocalLength_ = 0.0f; //Indicate if it's not initialised
          horizontal_window_ = 0;
          vertical_window_ = 0;
          radiusLookupTableWidth_ =-1; 
          radiusLookupTableHeight_ =-1; 
          exactFocalLength_ = 0;      // we haven't estimated it yet or we haven't set it 
          precision_ = 0;
        }
        /** \brief OrganizedNeighbor constructor. 
          * \param[in] nr_threads the number of of hardware threads to use
          */
        OrganizedNeighborOMP (unsigned int nr_threads)
        {
          max_distance_ = std::numeric_limits<double>::max ();
          oneOverFocalLength_ = 0.0f; //Indicate if it's not initialised
          horizontal_window_ = 0;
          vertical_window_ = 0;
          radiusLookupTableWidth_ =-1; 
          radiusLookupTableHeight_ =-1; 
          exactFocalLength_ = 0;      // we haven't estimated it yet or we haven't set it 
          precision_ = 0;
          setNumberOfThreads (nr_threads);
        }

        /** \brief Initialize the scheduler and set the number of threads to use.
          * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
          */
        inline void 
        setNumberOfThreads (unsigned int nr_threads)
        { 
          if (nr_threads == 0)
            nr_threads = 1;
          threads_ = nr_threads; 
        }

        /** \brief Empty deconstructor. */
        ~OrganizedNeighborOMP () {}

        /** \brief Provide a pointer to the input data set, if user has focal length he must set it before calling this
         *  \param[in] cloud the const boost shared pointer to a PointCloud message
         */
        inline void
        setInputCloud (const PointCloudConstPtr &cloud)
        {
          if (input_ != cloud)
            input_ = cloud;

          if (precision_ == 1)
          {
            if (!exactFocalLength_)
            {
              estimateFocalLengthFromInputCloud (*cloud);
              generateRadiusLookupTable (cloud->width, cloud->height);
              exactFocalLength_ = 1;
            }
          }
          else
          {
            oneOverFocalLength_ = 1.0f;
          }
        }

        /** \brief Provide a pointer to the input data set, if user has focal length he must set it before calling this
         *  \param[in] cloud the const boost shared pointer to a PointCloud message
         *  \param[in] indices the const boost shared pointer to PointIndices
         */
        inline void
        setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices)
        {
          if (input_ != cloud)
            input_ = cloud;

          indices_ = indices;

          if (precision_ == 1)
          {
            if(!exactFocalLength_)
            {
              estimateFocalLengthFromInputCloud (*cloud);
              generateRadiusLookupTable (cloud->width, cloud->height);
              exactFocalLength_ = 1;
            }
          }
          else
          {
            oneOverFocalLength_ = 1.0f;
          }
        }

        /** \brief Get a pointer to the input dataset as passed by the user. 
        * \return the const boost shared pointer to a pointcloud
        */
        PointCloudConstPtr getInputCloud () { return input_; }

        /** \brief Get a pointer to the Indices
        * \return the boost shared pointer to the indices list
        */
        inline IndicesConstPtr const getIndices () { return (indices_); }

        /** \brief Set which of the two available nearest neighbor estimation methods should be used 
          * (approximate = 0 or exact = 1).
          * \param[in] precision set to 0 for the faster, approximate nearest neighbor search method, 1 otherwise
          */
        inline void setPrecision (int precision) { precision_ = precision; }

        /** \brief Obtain which of the nearest neighbor method is being used. */
        inline int getPrecision () { return (precision_); }

        /** \brief set the focal length, this should be done before calling set InputCloud */
        inline void setOneOverFocalLength (double oneOverFocalLength) 
        {
          oneOverFocalLength_ = oneOverFocalLength;
          exactFocalLength_ = 1;
        }

        /** \brief Get the 1/focallength, can be the set or calculated one */
        inline double getOneOverFocalLength () { return oneOverFocalLength_;}


        /** \brief Get the maximum allowed distance between the query point and its nearest neighbors. */
        inline double getMaxDistance () const { return (max_distance_); }

        /** \brief Set the maximum allowed distance between the query point and its nearest neighbors. */
        inline void setMaxDistance (double max_dist) { max_distance_ = max_dist; }

        /** \brief set the search window (horizontal, vertical) in pixels.
         * \param horizontal the horizontal window in pixel
         * \param vertical the vertical window in pixel
         */
        inline void
        setSearchWindow (int horizontal, int vertical)
        {
          horizontal_window_ = horizontal;
          vertical_window_ = vertical;
        }

        /** \brief Estimate the search window (horizontal, vertical) in pixels in order to get up to k-neighbors.
         * \param k the number of neighbors requested
         */
        void
        setSearchWindowAsK (int k);

        /** \brief Get the horizontal search window in pixels. */
        int getHorizontalSearchWindow () const { return (horizontal_window_); }

        /** \brief Get the vertical search window in pixels. */
        int getVerticalSearchWindow () const { return (vertical_window_); }

        /** \brief search for all the nearest neighbors of the query point in a given radius.
         * \param cloud the point cloud data
         * \param index the index in \a cloud representing the query point
         * \param radius the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        inline int
        radiusSearch (  const PointCloud& cloud, 
                        int index, 
                        double radius, 
                        std::vector<int>& k_indices,
                        std::vector<float>& k_distances, 
                        int max_nn = std::numeric_limits<int>::max())
        {
          PCL_ERROR("[pcl::search::OrganizedNeighborOMP::radiusSearch] This function is not supported by OrganizedNeighborOMP\n");
          return (0);
        }

        /** \brief search for all neighbors of query point that are within a given radius.
         * \param cloud the point cloud data
         * \param index the index in \a cloud representing the query point
         * \param radius the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_sqr_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        int
        radiusSearch (  const PointCloudConstPtr &cloud, 
                        int index, 
                        double radius,
                        std::vector<int> &k_indices, 
                        std::vector<float> &k_sqr_distances,
                        int max_nn = std::numeric_limits<int>::max());

        /** \brief search for all neighbors of query point that are within a given radius.
         * \param index index representing the query point in the dataset given by \a setInputCloud.
         *        If indices were given in setInputCloud, index will be the position in the indices vector
         * \param radius radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_sqr_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        int
        radiusSearch (  int index, 
                        const double radius, 
                        std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances, 
                        int max_nn = std::numeric_limits<int>::max()) const;

        /** \brief search for all neighbors of query point that are within a given radius.
         * \param p_q the given query point
         * \param radius the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_sqr_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        int
        radiusSearch (  const PointT &p_q, 
                        const double radius, 
                        std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances, 
                        int max_nn = std::numeric_limits<int>::max()) const;

        /** \brief Estimate the focal length parameter that was used during point cloud generation 
          * \param cloud the input point cloud dataset 
          * \return oneOverFocalLength
          */
        double
        estimateFocalLengthFromInputCloud (const pcl::PointCloud<PointT> &cloud);

        /** \brief Search for the k-nearest neighbors for a given query point.
          * \param[in] p_q the given query point
          * \param[in] k the number of neighbors to search for (used only if \ref horizontal and vertical window not given already!)
          * \param[out] k_indices the resultant point indices (must be resized to \a k beforehand!)
          * \param[out] k_distances \note this function does not return distances
          * \return number of neighbors found
          */
        int
        exactNearestKSearch ( const PointT &p_q,
                              int k, 
                              std::vector<int> &k_indices,
                              std::vector<float> &k_sqr_distances);

        /** \brief Search for the k-nearest neighbors for the given query point (zero-copy).
          *
          * \param[in] index the index representing the query point in the dataset given by \a setInputCloud 
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant point indices 
          * \param[out] k_distances the resultant point neighbor distances
          * \return number of neighbors found
          */
        inline int
        exactNearestKSearch ( int index, 
                              int k, 
                              std::vector<int> &k_indices, 
                              std::vector<float> &k_distances);

        /** \brief Search for the k-nearest neighbors for a given query point.
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] k the number of neighbors to search for (used only if \ref horizontal and vertical window not given already!)
          * \param[out] k_indices the resultant point indices (must be resized to \a k beforehand!)
          * \param[out] k_distances \note this function does not return distances
          * \return number of neighbors found
          */
        inline int
        exactNearestKSearch ( const pcl::PointCloud<PointT> &cloud, 
                              int index, 
                              int k, 
                              std::vector<int> &k_indices, 
                              std::vector<float> &k_distances);

         /** \brief Search for the k-nearest neighbors for a given query point.
          * \note limiting the maximum search radius (with setMaxDistance) can lead to a significant improvement in search speed
          * \param[in] p_q the given query point (\ref setInputCloud must be given a-priori!)
          * \param[in] k the number of neighbors to search for (used only if \ref horizontal and vertical window not given already!)
          * \param[out] k_indices the resultant point indices (must be resized to \a k beforehand!)
          * \param[out] k_distances \note this function does not return distances
          * \return number of neighbors found
          * @todo still need to implements this functionality
          */
        int
        nearestKSearch (  const PointT &p_q,
                          int k, 
                          std::vector<int> &k_indices,
                          std::vector<float> &k_sqr_distances)
        {
          PCL_ERROR ("[pcl::search::OrganizedNeighborOMP::approxNearestKSearch] Method not implemented!\n");
          return (0);
        }

        /** \brief Search for the k-nearest neighbors for the given query point (zero-copy).
          * \note limiting the maximum search radius (with setMaxDistance) can lead to a significant improvement in search speed
          *
          * \param[in] index the index representing the query point in the dataset (\ref setInputCloud must be given a-priori!)
          * \param[in] k the number of neighbors to search for (used only if \ref horizontal and vertical window not given already!)
          * \param[out] k_indices the resultant point indices (must be resized to \a k beforehand!)
          * \param[out] k_distances \note this function does not return distances
          * \return number of neighbors found
          */
        int
        nearestKSearch (int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances);

        /** \brief Search for the k-nearest neighbors for a given query point.
          * \note limiting the maximum search radius (with setMaxDistance) can lead to a significant improvement in search speed
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] k the number of neighbors to search for (used only if \ref horizontal and vertical window not given already!)
          * \param[out] k_indices the resultant point indices (must be resized to \a k beforehand!)
          * \param[out] k_distances \note this function does not return distances
          * \return number of neighbors found
          */
        int
        nearestKSearch (const pcl::PointCloud<PointT> &cloud, int index, int k, 
                              std::vector<int> &k_indices, std::vector<float> &k_distances);


      protected:

        /** \brief @b RadiusSearchLoopkupEntry entry for radius search lookup vector
          * \note This class defines entries for the radius search lookup vector
          * \author Julius Kammerl (julius@kammerl.de)
          */
        class RadiusSearchLoopkupEntry
        {
          public:
            /** \brief Empty constructor  */
            RadiusSearchLoopkupEntry () {}

            /** \brief Empty deconstructor  */
            virtual ~RadiusSearchLoopkupEntry () {}

            /** \brief Define search point and calculate squared distance
              * \param x_shift shift in x dimension
              * \param y_shift shift in y dimension
              */
            void
            defineShiftedSearchPoint (int x_shift, int y_shift)
            {
              x_diff_ = x_shift;
              y_diff_ = y_shift;

              squared_distance_ = x_diff_ * x_diff_ + y_diff_ * y_diff_;
            }

            /** \brief Operator< for comparing radiusSearchLoopkupEntry instances with each other.  */
            bool
            operator< (const RadiusSearchLoopkupEntry& rhs) const
            {
              return (this->squared_distance_ < rhs.squared_distance_);
            }

            // Public globals
            int x_diff_;int y_diff_;int squared_distance_;
        };

        /** \brief @b NearestNeighborCandidate entry for the nearest neighbor candidate queue
          * \note This class defines entries for the nearest neighbor candidate queue
          * \author Julius Kammerl (julius@kammerl.de)
         */
        class NearestNeighborCandidate
        {
          public:

          /** \brief Empty constructor  */
          NearestNeighborCandidate () {}

          /** \brief Empty deconstructor  */
          virtual ~NearestNeighborCandidate () {}

          /** \brief Operator< for comparing nearestNeighborCandidate instances with each other.  */
          bool
          operator< (const NearestNeighborCandidate& rhs) const
          {
            return (this->squared_distance_ < rhs.squared_distance_);
          }

          // Public globals
          int index_;
          double squared_distance_;
        };

        /** \brief Get point at index from input pointcloud dataset
          * \param index index representing the point in the dataset given by \a setInputCloud
          * \return PointT from input pointcloud dataset
          */
        const PointT&
        getPointByIndex (const unsigned int index) const;

        /** \brief Generate radius lookup table. It is used to subsequentially iterate over points
          *         which are close to the search point
          * \param width of organized point cloud
          * \param height of organized point cloud
          */
        void
        generateRadiusLookupTable (unsigned int width, unsigned int height);

        /** \brief Project a point to a plane and return it's X and Y coordinates.
          * \param[in] point the point to project 
          * \param[out] xpos the X position
          * \param[out] ypos the Y position
          */
        inline void
        pointPlaneProjection (const PointT& point, int& xpos, int& ypos) const
        {
          xpos = (int)pcl_round (point.x / (point.z * oneOverFocalLength_));
          ypos = (int)pcl_round (point.y / (point.z * oneOverFocalLength_));
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
        getProjectedRadiusSearchBox (const PointT& point, double squared_radius, int& minX, int& minY,
                                     int& maxX, int& maxY) const;


        /** \brief Class getName method. */
        virtual std::string getName () const { return ("Organized_Neighbor_Search"); }

        /** \brief The horizontal search window. */
        int horizontal_window_;int vertical_window_;int min_pts_;

        /** \brief Pointer to input point cloud dataset. */
        PointCloudConstPtr input_;
        /** \brief Pointer to input indices. */
        IndicesConstPtr indices_;

        /** \brief Maximum allowed distance between the query point and its k-neighbors. */
        double max_distance_;

        /** \brief Global focal length parameter */
        double oneOverFocalLength_;

        /** \brief Precalculated radius search lookup vector */
        std::vector<RadiusSearchLoopkupEntry> radiusSearchLookup_;

        int radiusLookupTableWidth_;
        int radiusLookupTableHeight_;

        /** \brief Set to 0 if the faster approximate method is to be used. 1 otherwise. */
        int precision_;

        /** \brief Indicates if the focallenght was already set or calculated */
        bool exactFocalLength_;

        /** \brief The number of threads the scheduler should use. */
        unsigned int threads_;
    };
  }
}

#endif


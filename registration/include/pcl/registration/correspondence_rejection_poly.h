/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_POLY_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_POLY_H_

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/point_cloud.h>

namespace pcl
{
  namespace registration
  {
    /** \brief CorrespondenceRejectorPoly 
      *
      * \author Anders Glent Buch
      * \ingroup registration
      * \tparam PointT the point type of the original point cloud from which the correspondences were obtained, must contain the fields x, y and z.
      */
    template <typename PointT>
    class PCL_EXPORTS CorrespondenceRejectorPoly: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

      public:
        typedef boost::shared_ptr<CorrespondenceRejectorPoly> Ptr;
        typedef boost::shared_ptr<const CorrespondenceRejectorPoly> ConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceRejectorPoly () 
          : data_container_ ()
          , cardinality_ (3)
          , similarity_threshold_ (0.75f)
          , iterations_ (10000)
        {
          rejection_name_ = "CorrespondenceRejectorPoly";
        }

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
          * \param[in] original_correspondences the set of initial correspondences given
          * \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
          */
        void 
        getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
                                     pcl::Correspondences& remaining_correspondences);

        /** \brief Provide a source point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] cloud a cloud containing XYZ data
          */
        inline void 
        setInputSource (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
        {
          if (!data_container_)
            data_container_.reset (new DataContainer<PointT>);
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputSource (cloud);
        }

        /** \brief Provide a source point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] cloud a cloud containing XYZ data
          */
        inline void 
        setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
        {
          PCL_WARN ("[pcl::registration::%s::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.\n", getClassName ().c_str ());
          if (!data_container_)
            data_container_.reset (new DataContainer<PointT>);
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputSource (cloud);
        }

        /** \brief Provide a target point cloud dataset (must contain XYZ
          * data!), used to compute the correspondence distance.  
          * \param[in] target a cloud containing XYZ data
          */
        inline void 
        setInputTarget (const typename pcl::PointCloud<PointT>::ConstPtr &target)
        {
          if (!data_container_)
            data_container_.reset (new DataContainer<PointT>);
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputTarget (target);
        }
        
        /**
         * \brief Set the polygon cardinality
         * \param cardinality polygon cardinality
         */
        inline void 
        setCardinality (int cardinality)
        {
          cardinality_ = cardinality;
        }
        
        /**
         * \brief Get the polygon cardinality
         * \return polygon cardinality
         */
        inline int 
        getCardinality ()
        {
          return (cardinality_);
        }
        
        /**
         * \brief Set the Euclidean similarity threshold between edge lengths
         * \param similarity similarity threshold
         */
        inline void 
        setSimilarityThreshold (float similarity_threshold)
        {
          similarity_threshold_ = similarity_threshold;
        }
        
        /**
         * \brief Get the Euclidean similarity threshold between edge lengths
         * \return similarity threshold
         */
        inline float 
        getSimilarityThreshold ()
        {
          return (similarity_threshold_);
        }
        
        /**
         * \brief Set the number of iterations
         * \param iterations number of iterations
         */
        inline void 
        setIterations (int iterations)
        {
          iterations_ = iterations;
        }
        
        /**
         * \brief Get the number of iterations
         * \return number of iterations
         */
        inline int 
        getIterations ()
        {
          return (iterations_);
        }

      protected:

        /** \brief Apply the rejection algorithm.
          * \param[out] correspondences the set of resultant correspondences.
          */
        inline void 
        applyRejection (pcl::Correspondences &correspondences)
        {
          getRemainingCorrespondences (*input_correspondences_, correspondences);
        }
        
        /**
         * \brief Get k unique random indices in range {0,...,n-1} (sampling without replacement)
         * \note No check is made to ensure that k <= n.
         * \param n upper index range, exclusive
         * \param k number of unique indices to sample
         * \return k unique random indices in range {0,...,n-1}
         */
        inline std::vector<int> 
        getUniqueRandomIndices (int n, int k)
        {
          // Marked sampled indices and sample counter
          std::vector<bool> sampled (n, false);
          int samples = 0;
          // Resulting unique indices
          std::vector<int> result;
          result.reserve (k);
          do
          {
            // Pick a random index in the range
            const int idx = (std::rand () % n);
            // If unique
            if (!sampled[idx])
            {
              // Mark as sampled and increment result counter
              sampled[idx] = true;
              ++samples;
              // Store
              result.push_back (idx);
            }
          }
          while (samples < k);
          
          return (result);
        }
        
        /**
         * Squared Euclidean distance between two points using the members x, y and z
         * \param p1 first point
         * \param p2 second point
         * \return squared Euclidean distance
         */
        inline float 
        computeSquaredDistance (const PointT& p1, const PointT& p2)
        {
          const float dx = p2.x - p1.x;
          const float dy = p2.y - p1.y;
          const float dz = p2.z - p1.z;
          
          return (dx*dx + dy*dy + dz*dz);
        }
        
        /**
         * \brief Edge length similarity thresholding
         * \param source source point cloud
         * \param target target point cloud
         * \param c1 first correspondence between source and target
         * \param c2 second correspondence between source and target
         * \param simsq squared similarity threshold in [0,1]
         * \return true if edge length ratio is larger than or equal to threshold
         */
        inline bool 
        thresholdEdgeLength (typename pcl::PointCloud<PointT>::ConstPtr source,
            typename pcl::PointCloud<PointT>::ConstPtr target,
            const pcl::Correspondence& c1,
            const pcl::Correspondence& c2,
            float simsq)
        {
          // Distance between source points
          const float dist_src = computeSquaredDistance ((*source)[c1.index_query], (*source)[c2.index_query]);
          // Distance between target points
          const float dist_tgt = computeSquaredDistance ((*target)[c1.index_match], (*target)[c2.index_match]);
          // Edge length similarity [0,1] where 1 is a perfect match
          const float edge_sim = (dist_src < dist_tgt ? dist_src / dist_tgt : dist_tgt / dist_src);
          
          return (edge_sim >= simsq);
        }
        
        /**
         * \brief Worker function for polygonal rejection using a single polygon
         * \param source source point cloud
         * \param target target point cloud
         * \param corr all correspondences
         * \param idx indices of sampled correspondences, must have a size of \ref cardinality_
         * \param simsq squared similarity threshold in [0,1]
         * \return true if all edge length ratios are larger than or equal to findThreshold
         */
        inline bool 
        thresholdPolygon (typename pcl::PointCloud<PointT>::ConstPtr source,
            typename pcl::PointCloud<PointT>::ConstPtr target,
            const pcl::Correspondences& corr,
            const std::vector<int>& idx,
            float simsq)
        {
          if (cardinality_ == 2) // Special case: when two points are considered, we only have one edge
          {
            return (thresholdEdgeLength (source, target, corr[ idx[0] ], corr[ idx[1] ], simsq));
          }
          else
          { // Otherwise check all edges
            for (int i = 0; i < cardinality_; ++i)
              if (!thresholdEdgeLength (source, target, corr[ idx[i] ], corr[ idx[(i+1)%cardinality_] ], simsq))
                return (false);
            
            return (true);
          }
        }
        
        /**
         * Compute a linear histogram. This function is equivalent to the MATLAB
         * function \b histc, with the edges set as follows: <b> lower:(upper-lower)/bins:upper </b>
         * \param data input samples
         * \param lower lower bound of input samples
         * \param upper upper bound of input samples
         * \param bins number of bins in output
         * \return linear histogram
         */
        std::vector<int> 
        computeHistogram (const std::vector<float>& data, float lower, float upper, int bins);
        
        /**
         * Find the optimal value for binary histogram thresholding using Otsu's method
         * \param histogram input histogram
         * \return threshold value according to Otsu's criterion
         */
        int 
        findThresholdOtsu (const std::vector<int>& histogram);

        typedef boost::shared_ptr<DataContainerInterface> DataContainerPtr;

        /** \brief A pointer to the DataContainer object containing the input and target point clouds */
        DataContainerPtr data_container_;
        
        /** \brief The polygon cardinality used during rejection */
        int cardinality_;
        
        /** \brief Lower edge length threshold in [0,1] used for verifying polygon similarities, where 1 is a perfect match */
        float similarity_threshold_;
        
        /** \brief Number of iterations to run */
        int iterations_;
    };
  }
}

#include <pcl/registration/impl/correspondence_rejection_poly.hpp>

#endif    // PCL_REGISTRATION_CORRESPONDENCE_REJECTION_POLY_H_

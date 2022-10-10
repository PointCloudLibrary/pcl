/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception.
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
#include <pcl/pcl_config.h>

namespace pcl
{
  namespace filters
  {
    /** Pyramid constructs a multi-scale representation of an organised point cloud.
      * It is an iterative smoothing subsampling algorithm.
      * The subsampling is fixed to 2. Two smoothing kernels may be used:
      * - [1/16 1/4 3/8 1/4 1/16] slower but produces finer result;
      * - [1/4 1/2 1/4] the more conventional binomial kernel which is faster.
      * We use a memory efficient algorithm so the convolving and subsampling are combined in a 
      * single step.
      *
      * \author Nizar Sallem
      */
    template <typename PointT>
    class Pyramid
    {
      public:
        using PointCloudPtr = typename PointCloud<PointT>::Ptr;
        using PointCloudConstPtr = typename PointCloud<PointT>::ConstPtr;
        using Ptr = shared_ptr< Pyramid<PointT> >;
        using ConstPtr = shared_ptr< const Pyramid<PointT> >;
 
        Pyramid (int levels = 4)
          : levels_ (levels)
          , large_ (false)
          , name_ ("Pyramid")
          , threshold_ (0.01)
          , threads_ (0)
        {
        }
      
        /** \brief Provide a pointer to the input dataset
          * \param cloud the const boost shared pointer to a PointCloud message
          */
        inline void 
        setInputCloud (const PointCloudConstPtr &cloud) { input_ = cloud; }

        /** \brief Get a pointer to the input point cloud dataset. */
        inline PointCloudConstPtr const 
        getInputCloud () { return (input_); }
      
        /** \brief Set the number of pyramid levels
          * \param levels desired number of pyramid levels
          */
        inline void
        setNumberOfLevels (int levels) { levels_ = levels; }
      
        /// \brief \return the number of pyramid levels
        inline int
        getNumberOfLevels () const { return (levels_); }

        /** \brief Initialize the scheduler and set the number of threads to use.
          * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic).
          */
        inline void
        setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }

        /** \brief Choose a larger smoothing kernel for enhanced smoothing.
          * \param large if true large smoothng kernel will be used.
          */
        inline void
        setLargeSmoothingKernel (bool large) { large_ = large; }
      
        /** Only points such us distance (center,point) < threshold are accounted for to prevent
          * ghost points.
          * Default value is 0.01, to disable set to std::numeric<float>::infinity ().
          * \param[in] threshold maximum allowed distance between center and neighbor.
          */
        inline void
        setDistanceThreshold (float threshold) { threshold_ = threshold; }

        /// \return the distance threshold
        inline float
        getDistanceThreshold () const { return (threshold_); }

        /** \brief compute the pyramid levels.
          * \param[out] output the constructed pyramid. It is resized to the number of levels.
          * \remark input_ is copied to output[0] for consistency reasons.
          */
        void
        compute (std::vector<PointCloudPtr>& output);

        inline const std::string&
        getClassName () const { return (name_); }
      
      private:

        /// \brief init computation
        bool 
        initCompute ();

        /** \brief nullify a point 
          * \param[in][out] p point to nullify
          */
        inline void
        nullify (PointT& p) const
        {
          p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
        }

        /// \brief The input point cloud dataset.
        PointCloudConstPtr input_;
        /// \brief number of pyramid levels
        int levels_;
        /// \brief use large smoothing kernel
        bool large_;
        /// \brief filter name
        std::string name_;
        /// \brief smoothing kernel
        Eigen::MatrixXf kernel_;
        /// Threshold distance between adjacent points
        float threshold_;
        /// \brief number of threads
        unsigned int threads_;

      public:
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

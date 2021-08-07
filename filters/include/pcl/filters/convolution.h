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

#include <pcl/point_cloud.h>

namespace pcl
{
  namespace filters
  {
    /** Convolution is a mathematical operation on two functions f and g,
      * producing a third function that is typically viewed as a modified
      * version of one of the original functions.
      * see http://en.wikipedia.org/wiki/Convolution.
      *
      * The class provides rows, column and separate convolving operations
      * of a point cloud.
      * Columns and separate convolution is only allowed on organised
      * point clouds.
      *
      * When convolving, computing the rows and cols elements at 1/2 kernel
      * width distance from the borders is not defined. We allow for 3
      * policies:
      * - Ignoring: elements at special locations are filled with zero
      * (default behaviour)
      * - Mirroring: the missing rows or columns are obtained through mirroring
      * - Duplicating: the missing rows or columns are obtained through
      * duplicating
      *
      * \author Nizar Sallem
      * \ingroup filters
      */

    template <typename PointIn, typename PointOut>
    class Convolution
    {
      public:
        using PointCloudIn = pcl::PointCloud<PointIn>;
        using PointCloudInPtr = typename PointCloudIn::Ptr;
        using PointCloudInConstPtr = typename PointCloudIn::ConstPtr;
        using PointCloudOut = pcl::PointCloud<PointOut>;
        using Ptr = shared_ptr< Convolution<PointIn, PointOut> >;
        using ConstPtr = shared_ptr< const Convolution<PointIn, PointOut> >;


        /// The borders policy available
        enum BORDERS_POLICY
        {
          BORDERS_POLICY_IGNORE = -1,
          BORDERS_POLICY_MIRROR = 0,
          BORDERS_POLICY_DUPLICATE = 1
        };
        /// Constructor
        Convolution ();
        /// Empty destructor
        ~Convolution () {}
        /** \brief Provide a pointer to the input dataset
          * \param cloud the const boost shared pointer to a PointCloud message
          * \remark Will perform a deep copy
          */
        inline void
        setInputCloud (const PointCloudInConstPtr& cloud) { input_ = cloud; }
        /** Set convolving kernel
          * \param[in] kernel convolving element
          */
        inline void
        setKernel (const Eigen::ArrayXf& kernel) { kernel_ = kernel; }
        /// Set the borders policy
        void
        setBordersPolicy (int policy) { borders_policy_ = policy; }
        /// Get the borders policy
        int
        getBordersPolicy () { return (borders_policy_); }
        /** \remark this is critical so please read it carefully.
          * In 3D the next point in (u,v) coordinate can be really far so a distance
          * threshold is used to keep us from ghost points.
          * The value you set here is strongly related to the sensor. A good value for
          * kinect data is 0.001. Default is std::numeric<float>::infinity ()
          * \param[in] threshold maximum allowed distance between 2 juxtaposed points
          */
        inline void
        setDistanceThreshold (const float& threshold) { distance_threshold_ = threshold; }
        /// \return the distance threshold
        inline const float &
        getDistanceThreshold () const { return (distance_threshold_); }
        /** \brief Initialize the scheduler and set the number of threads to use.
          * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
          */
        inline void
        setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }
        /** Convolve a float image rows by a given kernel.
          * \param[out] output the convolved cloud
          * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
          * output.cols () < input.cols () then output is resized to input sizes.
          */
        inline void
        convolveRows (PointCloudOut& output);
        /** Convolve a float image columns by a given kernel.
          * \param[out] output the convolved image
          * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
          * output.cols () < input.cols () then output is resized to input sizes.
          */
        inline void
        convolveCols (PointCloudOut& output);
        /** Convolve point cloud with an horizontal kernel along rows
          * then vertical kernel along columns : convolve separately.
          * \param[in] h_kernel kernel for convolving rows
          * \param[in] v_kernel kernel for convolving columns
          * \param[out] output the convolved cloud
          * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
          * output.cols () < input.cols () then output is resized to input sizes.
          */
        inline void
        convolve (const Eigen::ArrayXf& h_kernel, const Eigen::ArrayXf& v_kernel, PointCloudOut& output);
        /** Convolve point cloud with same kernel along rows and columns separately.
          * \param[out] output the convolved cloud
          * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
          * output.cols () < input.cols () then output is resized to input sizes.
          */
        inline void
        convolve (PointCloudOut& output);

      protected:
        /// \brief convolve rows and ignore borders
        void
        convolve_rows (PointCloudOut& output);
        /// \brief convolve cols and ignore borders
        void
        convolve_cols (PointCloudOut& output);
        /// \brief convolve rows and mirror borders
        void
        convolve_rows_mirror (PointCloudOut& output);
        /// \brief convolve cols and mirror borders
        void
        convolve_cols_mirror (PointCloudOut& output);
        /// \brief convolve rows and duplicate borders
        void
        convolve_rows_duplicate (PointCloudOut& output);
        /// \brief convolve cols and duplicate borders
        void
        convolve_cols_duplicate (PointCloudOut& output);
        /** init compute is an internal method called before computation
          * \param[in] output
          * \throw pcl::InitFailedException
          */
        void
        initCompute (PointCloudOut& output);
      private:
        /** \return the result of convolution of point at (\ai, \aj)
          * \note no test on finity is performed
          */
        inline PointOut
        convolveOneRowDense (int i, int j);
        /** \return the result of convolution of point at (\ai, \aj)
          * \note no test on finity is performed
          */
        inline PointOut
        convolveOneColDense (int i, int j);
        /** \return the result of convolution of point at (\ai, \aj)
          * \note only finite points within \a distance_threshold_ are accounted
          */
        inline PointOut
        convolveOneRowNonDense (int i, int j);
        /** \return the result of convolution of point at (\ai, \aj)
          * \note only finite points within \a distance_threshold_ are accounted
          */
        inline PointOut
        convolveOneColNonDense (int i, int j);

        /// Border policy
        int borders_policy_;
        /// Threshold distance between adjacent points
        float distance_threshold_;
        /// Pointer to the input cloud
        PointCloudInConstPtr input_;
        /// convolution kernel
        Eigen::ArrayXf kernel_;
        /// half kernel size
        int half_width_;
        /// kernel size - 1
        int kernel_width_;
      protected:
        /** \brief The number of threads the scheduler should use. */
        unsigned int threads_;

        void
        makeInfinite (PointOut& p)
        {
          p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
        }      
    };
  }
}

#include <pcl/filters/impl/convolution.hpp>

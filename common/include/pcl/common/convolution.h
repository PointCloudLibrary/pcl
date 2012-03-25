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
 * $Id: convolution.h 4255 2012-02-05 06:58:59Z rusu $
 *
 */

#ifndef PCL_COMMON_CONVOLUTION_H_
#define PCL_COMMON_CONVOLUTION_H_

#include <pcl/common/eigen.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/spring.h>
#include <pcl/exceptions.h>

namespace pcl
{
  namespace common
  {
    /** Class Convolution
      * Convolution is a mathematical operation on two functions f and g, 
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
      * - Zero padding convolution: elements at these locations are filled
      * with zero (default behaviour)
      * - Mirroring: the original point cloud is extending by 1/2 kernel 
      * width amount mirroring borders
      * - Duplicating: the original point cloud is extending by 1/2 kernel 
      * width amount using duplicating the top and bottom rows and/or left
      * and right columns
      * .
      * After convolving the original point cloud is shrinked to the 
      * original size.
      *
      * \author Nizar Sallem
      * \ingroup common
      */
    template <typename PointOperatorsType>
    class Convolution
    {
      public:
        typedef typename PointOperatorsType::PointIn PointIn;
        typedef typename PointOperatorsType::PointOut PointOut;
        typedef typename pcl::PointCloud<PointIn> PointCloudIn;
        typedef typename PointCloudIn::Ptr PointCloudInPtr;
        typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
        typedef typename pcl::PointCloud<PointOut> PointCloudOut;
        typedef typename PointCloudOut::Ptr PointCloudOutPtr;
        typedef typename pcl::PointCloudSpring<PointIn> PointCloudSpring;

        /// The borders policy available
        enum BORDERS_POLICY { IGNORE = -1, MIRROR, DUPLICATE };
        /// Constructor
        Convolution () : borders_policy_ (IGNORE), convolve_direction_ (-1) {}
        /// Set the borders policy
        void 
        setBordersPolicy (int policy) { borders_policy_ = policy; }
        /// Get the borders policy
        int getBordersPolicy () { return (borders_policy_); }
        /// Set the convolving direction
        void 
        setConvolveDirection (int direction) { convolve_direction_ = direction; }
        /// Get the convolving direction
        int getConvolveDirection () { return (convolve_direction_); }        
        
        /** Convolve point cloud with an horizontal kernel along rows 
          * then vertical kernel along columns : convolve separately.
          * \param[in] h_kernel kernel for convolving rows
          * \param[in] v_kernel kernel for convolving columns
          * \param[out] out the convolved cloud
          * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
          * output.cols () < input.cols () then output is resized to input sizes.
          */
        inline void
        convolve (const Eigen::ArrayXf& h_kernel, 
                  const Eigen::ArrayXf& v_kernel,
                  PointCloudOutPtr& out)
        {
          kernel_width_ = h_kernel.size ();
          convolve_direction_ = PointCloudSpring::BOTH;
          output_ = out;
          
          try
          {
            initCompute ();
            convolve_rows (h_kernel, output_);
            convolve_cols (v_kernel, output_);
            deinitCompute ();
          }
          catch (InitFailedException& e)
          {
            PCL_THROW_EXCEPTION (InitFailedException,
                                 "[pcl::common::Convolution::convolveCols] init failed " << e.what ());
          }          
        }
        /** Convolve a float image rows by a given kernel.
          * \param[in] kernel convolution kernel
          * \param[out] output the convolved cloud
          * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
          * output.cols () < input.cols () then output is resized to input sizes.
          */
        inline void
        convolveRows (const Eigen::ArrayXf& kernel, PointCloudOutPtr& output)
        {
          kernel_width_ = kernel.size ();
          convolve_direction_ = PointCloudSpring::HORIZONTAL;
          output_ = output;
          try
          {
            initCompute ();
            convolve_rows (kernel, output_);
            deinitCompute ();
          }
          catch (InitFailedException& e)
          {
            PCL_THROW_EXCEPTION (InitFailedException,
                                 "[pcl::common::Convolution::convolveRows] init failed " << e.what ());
          }
        }
        /** Convolve a float image columns by a given kernel.
          * \param[in] kernel convolution kernel
          * \param[out] output the convolved image
          * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
          * output.cols () < input.cols () then output is resized to input sizes.
          */
        inline void
        convolveCols (const Eigen::ArrayXf& kernel, PointCloudOutPtr& output)
        {
          kernel_width_ = kernel.size ();
          convolve_direction_ = PointCloudSpring::VERTICAL;
          output_ = output;
          
          try
          {
            initCompute ();
            convolve_cols (kernel, output_);
            deinitCompute ();
          }
          catch (InitFailedException& e)
          {
            PCL_THROW_EXCEPTION (InitFailedException,
                                 "[pcl::common::Convolution::convolveCols] init failed " << e.what ());
          }          
        }
        /** \brief Provide a pointer to the input dataset
          * \param cloud the const boost shared pointer to a PointCloud message
          * \remark Will perform a deep copy
          */
        inline void
        setInputCloud (const PointCloudInConstPtr& cloud)
        {
          input_.reset (new PointCloudIn (*cloud));
        }
        /** \brief Provide a pointer to the input dataset
          * \param cloud the boost shared pointer to a PointCloud message
          * \remark the point cloud will be transformed internally but 
          * then restored so you can safely use it.
          */
        inline void
        setInputCloud (const PointCloudInPtr& cloud)
        {
          input_ = cloud;
        }
        /// sum 2 PointIn to a PointOut
        inline PointOut 
        add (const PointIn& lhs, const PointIn& rhs)
        {
          return (operators_.add (lhs, rhs));
        }
        /// substract 2 PointIn to a PointOut
        inline PointOut
        minus (const PointIn& lhs, const PointIn& rhs)
        {
          return (operators_.minus (lhs, rhs));
        }
        /// multiply a scalar with a PointIn to a PointOut
        inline PointOut 
        dot (const float& scalar, const PointIn& p) 
        {
          return (operators_.dot (scalar, p));
        }
        /// multiply a PointIn with a scalar to a PointOut
        inline PointOut 
        dot (const PointIn& p, const float& scalar)
        {
          return (operators_.dot (scalar, p));
        }
        /// sum and assign a PointOut to another PointOut
        inline PointOut&
        plus_assign (PointOut& lhs, const PointOut& rhs)
        {
          return (operators_.plus_assign (lhs, rhs));
        }

      private:
        /** init compute is an internal method called before computation
          * \throw pcl::InitFailedException
          */
        void
        initCompute ();
        /** terminate computation shrinking the input point cloud if 
          * necessary.
          */
        inline void
        deinitCompute ()
        {
          if (borders_policy_ == MIRROR || borders_policy_ == DUPLICATE)
            spring_.shrink ();
        }
        /// convolve rows and ignore borders
        inline void
        convolve_rows (const Eigen::ArrayXf& kernel, PointCloudOutPtr& out);
        /// convolve cols and ignore borders
        inline void
        convolve_cols (const Eigen::ArrayXf& kernel, PointCloudOutPtr& out);
        /// Convolution Operator
        PointOperatorsType operators_;
        /// Border policy
        int borders_policy_;
        /// Convolving direction
        int convolve_direction_;
        /// Pointer to the output cloud
        PointCloudOutPtr output_;
        /// Pointer to the input cloud
        PointCloudInPtr input_;
        /// kernel size
        int kernel_width_;
        /// Half size of the kernel size
        int half_width_;
        /// Point cloud spring to expand and shrink the input cloud
        PointCloudSpring spring_;
    };
  }
}

#include <pcl/common/impl/convolution.hpp>

#endif

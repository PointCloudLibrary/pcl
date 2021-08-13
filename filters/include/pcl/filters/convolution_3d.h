/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#include <pcl/pcl_base.h>

namespace pcl
{
  namespace filters
  {
    /** \brief Class ConvolvingKernel base class for all convolving kernels
      * \ingroup filters
      */
    template<typename PointInT, typename PointOutT>
    class ConvolvingKernel
    {
      public:
        using Ptr = shared_ptr<ConvolvingKernel<PointInT, PointOutT> >;
        using ConstPtr = shared_ptr<const ConvolvingKernel<PointInT, PointOutT> >;
 
        using PointCloudInConstPtr = typename PointCloud<PointInT>::ConstPtr;

        /// \brief empty constructor
        ConvolvingKernel () {}

        /// \brief empty destructor
        virtual ~ConvolvingKernel () {}

        /** \brief Set input cloud
          * \param[in] input source point cloud
          */
        void
        setInputCloud (const PointCloudInConstPtr& input) { input_ = input; }

        /** \brief Convolve point at the center of this local information
          * \param[in] indices indices of the point in the source point cloud
          * \param[in] distances euclidean distance squared from the query point
          * \return the convolved point
          */
        virtual PointOutT
        operator() (const Indices& indices, const std::vector<float>& distances) = 0;

        /** \brief Must call this method before doing any computation
          * \note make sure to override this with at least
          * \code
          * bool initCompute ()
          * {
          *   return (true);
          * }
          * \endcode
          * in your kernel interface, else you are going nowhere!
          */
        virtual bool
        initCompute () { return false; }

        /** \brief Utility function that annihilates a point making it fail the \ref pcl::isFinite test
          * \param p point to annihilate
          */
        static void
        makeInfinite (PointOutT& p)
        {
          p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
        }

      protected:
        /// source cloud
        PointCloudInConstPtr input_;
    };

    /** \brief Gaussian kernel implementation interface
      * Use this as implementation reference
      * \ingroup filters
      */
    template<typename PointInT, typename PointOutT>
    class GaussianKernel : public ConvolvingKernel <PointInT, PointOutT>
    {
      public:
        using ConvolvingKernel<PointInT, PointOutT>::initCompute;
        using ConvolvingKernel<PointInT, PointOutT>::input_;
        using ConvolvingKernel<PointInT, PointOutT>::operator ();
        using ConvolvingKernel<PointInT, PointOutT>::makeInfinite;
        using Ptr = shared_ptr<GaussianKernel<PointInT, PointOutT> >;
        using ConstPtr = shared_ptr<GaussianKernel<PointInT, PointOutT> >;

        /** Default constructor */
        GaussianKernel ()
          : ConvolvingKernel <PointInT, PointOutT> ()
          , sigma_ (0)
          , threshold_ (std::numeric_limits<float>::infinity ())
        {}

        virtual ~GaussianKernel () {}

        /** Set the sigma parameter of the Gaussian
          * \param[in] sigma
          */
        inline void
        setSigma (float sigma) { sigma_ = sigma; }

        /** Set the distance threshold relative to a sigma factor i.e. points such as
          * ||pi - q|| > sigma_coefficient^2 * sigma^2 are not considered.
          */
        inline void
        setThresholdRelativeToSigma (float sigma_coefficient)
        {
          sigma_coefficient_.reset (sigma_coefficient);
        }

        /** Set the distance threshold such as pi, ||pi - q|| > threshold are not considered. */
        inline void
        setThreshold (float threshold) { threshold_ = threshold; }

        /** Must call this method before doing any computation */
        bool initCompute ();

        virtual PointOutT
        operator() (const Indices& indices, const std::vector<float>& distances);

      protected:
        float sigma_;
        float sigma_sqr_;
        float threshold_;
        boost::optional<float> sigma_coefficient_;
    };

    /** \brief Gaussian kernel implementation interface with RGB channel handling
      * Use this as implementation reference
      * \ingroup filters
      */
    template<typename PointInT, typename PointOutT>
    class GaussianKernelRGB : public GaussianKernel <PointInT, PointOutT>
    {
      public:
        using GaussianKernel<PointInT, PointOutT>::initCompute;
        using GaussianKernel<PointInT, PointOutT>::input_;
        using GaussianKernel<PointInT, PointOutT>::operator ();
        using GaussianKernel<PointInT, PointOutT>::makeInfinite;
        using GaussianKernel<PointInT, PointOutT>::sigma_sqr_;
        using GaussianKernel<PointInT, PointOutT>::threshold_;
        using Ptr = shared_ptr<GaussianKernelRGB<PointInT, PointOutT> >;
        using ConstPtr = shared_ptr<GaussianKernelRGB<PointInT, PointOutT> >;

        /** Default constructor */
        GaussianKernelRGB ()
          : GaussianKernel <PointInT, PointOutT> ()
        {}

        ~GaussianKernelRGB () {}

        PointOutT
        operator() (const Indices& indices, const std::vector<float>& distances);
    };

    /** Convolution3D handles the non organized case where width and height are unknown or if you
      * are only interested in convolving based on local neighborhood information.
      * The convolving kernel MUST be a radial symmetric and implement \ref ConvolvingKernel
      * interface.
      */
    template <typename PointIn, typename PointOut, typename KernelT>
    class Convolution3D : public pcl::PCLBase <PointIn>
    {
      public:
        using PointCloudIn = pcl::PointCloud<PointIn>;
        using PointCloudInConstPtr = typename PointCloudIn::ConstPtr;
        using KdTree = pcl::search::Search<PointIn>;
        using KdTreePtr = typename KdTree::Ptr;
        using PointCloudOut = pcl::PointCloud<PointOut>;
        using Ptr = shared_ptr<Convolution3D<PointIn, PointOut, KernelT> >;
        using ConstPtr = shared_ptr<Convolution3D<PointIn, PointOut, KernelT> >;

        using pcl::PCLBase<PointIn>::indices_;
        using pcl::PCLBase<PointIn>::input_;

        /** \brief Constructor */
        Convolution3D ();

        /** \brief Empty destructor */
        ~Convolution3D () {}

        /** \brief Initialize the scheduler and set the number of threads to use.
          * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
          */
        inline void
        setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }

        /** \brief Set convolving kernel
          * \param[in] kernel convolving element
          */
        inline void
        setKernel (const KernelT& kernel) { kernel_ = kernel; }

        /** \brief Provide a pointer to the input dataset that we need to estimate features at every point for.
          * \param cloud the const boost shared pointer to a PointCloud message
          */
        inline void
        setSearchSurface (const PointCloudInConstPtr &cloud) { surface_ = cloud; }

        /** \brief Get a pointer to the surface point cloud dataset. */
        inline PointCloudInConstPtr
        getSearchSurface () { return (surface_); }

        /** \brief Provide a pointer to the search object.
          * \param tree a pointer to the spatial search object.
          */
        inline void
        setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }

        /** \brief Get a pointer to the search method used. */
        inline KdTreePtr
        getSearchMethod () { return (tree_); }

        /** \brief Set the sphere radius that is to be used for determining the nearest neighbors
          * \param radius the sphere radius used as the maximum distance to consider a point a neighbor
          */
        inline void
        setRadiusSearch (double radius) { search_radius_ = radius; }

        /** \brief Get the sphere radius used for determining the neighbors. */
        inline double
        getRadiusSearch () { return (search_radius_); }

        /** Convolve point cloud.
          * \param[out] output the convolved cloud
          */
        void
        convolve (PointCloudOut& output);

      protected:
        /** \brief initialize computation */
        bool initCompute ();

        /** \brief An input point cloud describing the surface that is to be used for nearest neighbors estimation. */
        PointCloudInConstPtr surface_;

        /** \brief A pointer to the spatial search object. */
        KdTreePtr tree_;

        /** \brief The nearest neighbors search radius for each point. */
        double search_radius_;

        /** \brief number of threads */
        unsigned int threads_;

        /** \brief convlving kernel */
        KernelT kernel_;
    };
  }
}

#include <pcl/filters/impl/convolution_3d.hpp>

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef PCL_NDT_2D_H_
#define PCL_NDT_2D_H_

#include <pcl/registration/registration.h>

namespace pcl
{
  namespace ndt2d
  {
    /** \brief Class to store vector value and first and second derivatives
      * (grad vector and hessian matrix), so they can be returned easily from
      * functions
      */
    template<unsigned N=3, typename T=double>
    struct ValueAndDerivatives
    {
      ValueAndDerivatives () : hessian (), grad (), value () {}

      Eigen::Matrix<T, N, N> hessian;
      Eigen::Matrix<T, N, 1>    grad;
      T value;

      static ValueAndDerivatives<N,T>
      Zero ()
      {
        ValueAndDerivatives<N,T> r;
        r.hessian = Eigen::Matrix<T, N, N>::Zero ();
        r.grad    = Eigen::Matrix<T, N, 1>::Zero ();
        r.value   = 0;
        return r;
      }

      ValueAndDerivatives<N,T>&
      operator+= (ValueAndDerivatives<N,T> const& r)
      {
        hessian += r.hessian;
        grad    += r.grad;
        value   += r.value;
        return *this;
      }
    };

    /** \brief A normal distribution estimation class.
      *
      * First the indices of of the points from a point cloud that should be
      * modelled by the distribution are added with addIdx (...).
      *
      * Then estimateParams (...) uses the stored point indices to estimate the
      * parameters of a normal distribution, and discards the stored indices.
      *
      * Finally the distriubution, and its derivatives, may be evaluated at any
      * point using test (...).
      */
    template <typename PointT>
    class NormalDist
    {
      typedef pcl::PointCloud<PointT> PointCloud;

      public:
        NormalDist ()
          : min_n_ (3), n_ (0), pt_indices_ (), mean_ (), covar_inv_ ()
        {
        }

        /** \brief Store a point index to use later for estimating distribution parameters.
          * \param[in] i Point index to store
          */
        void
        addIdx (size_t i)
        {
          pt_indices_.push_back (i);
        }

        /** \brief Estimate the normal distribution parameters given the point indices provided. Memory of point indices is cleared.
          * \param[in] cloud                    Point cloud corresponding to indices passed to addIdx.
          * \param[in] min_covar_eigvalue_mult  Set the smallest eigenvalue to this times the largest.
          */
        void
        estimateParams (const PointCloud& cloud, double min_covar_eigvalue_mult = 0.001);

        /** \brief Return the 'score' (denormalised likelihood) and derivatives of score of the point p given this distribution.
          * \param[in] transformed_pt   Location to evaluate at.
          * \param[in] cos_theta        sin(theta) of the current rotation angle of rigid transformation: to avoid repeated evaluation
          * \param[in] sin_theta        cos(theta) of the current rotation angle of rigid transformation: to avoid repeated evaluation
          * estimateParams must have been called after at least three points were provided, or this will return zero.
          *
          */
        ValueAndDerivatives<3,double>
        test (const PointT& transformed_pt, const double& cos_theta, const double& sin_theta) const;

    protected:
        const size_t min_n_;
        size_t n_;
        std::vector<size_t> pt_indices_;
        Eigen::Vector2d mean_;
        Eigen::Matrix2d covar_inv_;
    };

    /** \brief Build a set of normal distributions modelling a 2D point cloud,
      * and provide the value and derivatives of the model at any point via the
      * test (...) function.
      */
    template <typename PointT>
    class NDTSingleGrid: public boost::noncopyable
    {
      typedef typename pcl::PointCloud<PointT> PointCloud;
      typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
      typedef typename pcl::ndt2d::NormalDist<PointT> NormalDist;

      public:
        NDTSingleGrid (PointCloudConstPtr cloud,
                       const Eigen::Vector2f& about,
                       const Eigen::Vector2f& extent,
                       const Eigen::Vector2f& step)
            : min_ (about - extent), max_ (min_ + 2*extent), step_ (step),
              cells_ ((max_[0]-min_[0]) / step_[0],
                      (max_[1]-min_[1]) / step_[1]),
              normal_distributions_ (cells_[0], cells_[1])
        {
          // sort through all points, assigning them to distributions:
          NormalDist* n;
          size_t used_points = 0;
          for (size_t i = 0; i < cloud->size (); i++)
          if ((n = normalDistForPoint (cloud->at (i))))
          {
            n->addIdx (i);
            used_points++;
          }

          PCL_DEBUG ("[pcl::NDTSingleGrid] NDT single grid %dx%d using %d/%d points\n", cells_[0], cells_[1], used_points, cloud->size ());

          // then bake the distributions such that they approximate the
          // points (and throw away memory of the points)
          for (int x = 0; x < cells_[0]; x++)
            for (int y = 0; y < cells_[1]; y++)
              normal_distributions_.coeffRef (x,y).estimateParams (*cloud);
        }

        /** \brief Return the 'score' (denormalised likelihood) and derivatives of score of the point p given this distribution.
          * \param[in] transformed_pt   Location to evaluate at.
          * \param[in] cos_theta        sin(theta) of the current rotation angle of rigid transformation: to avoid repeated evaluation
          * \param[in] sin_theta        cos(theta) of the current rotation angle of rigid transformation: to avoid repeated evaluation
          */
        ValueAndDerivatives<3,double>
        test (const PointT& transformed_pt, const double& cos_theta, const double& sin_theta) const
        {
          const NormalDist* n = normalDistForPoint (transformed_pt);
          // index is in grid, return score from the normal distribution from
          // the correct part of the grid:
          if (n)
            return n->test (transformed_pt, cos_theta, sin_theta);
          else
            return ValueAndDerivatives<3,double>::Zero ();
        }

      protected:
        /** \brief Return the normal distribution covering the location of point p
          * \param[in] p a point
          */
        NormalDist*
        normalDistForPoint (PointT const& p) const
        {
          // this would be neater in 3d...
          Eigen::Vector2f idxf;
          for (size_t i = 0; i < 2; i++)
            idxf[i] = (p.getVector3fMap ()[i] - min_[i]) / step_[i];
          Eigen::Vector2i idxi = idxf.cast<int> ();
          for (size_t i = 0; i < 2; i++)
            if (idxi[i] >= cells_[i] || idxi[i] < 0)
              return NULL;
          // const cast to avoid duplicating this function in const and
          // non-const variants...
          return const_cast<NormalDist*> (&normal_distributions_.coeffRef (idxi[0], idxi[1]));
        }

        Eigen::Vector2f min_;
        Eigen::Vector2f max_;
        Eigen::Vector2f step_;
        Eigen::Vector2i cells_;

        Eigen::Matrix<NormalDist, Eigen::Dynamic, Eigen::Dynamic> normal_distributions_;
    };

    /** \brief Build a Normal Distributions Transform of a 2D point cloud. This
      * consists of the sum of four overlapping models of the original points
      * with normal distributions.
      * The value and derivatives of the model at any point can be evaluated
      * with the test (...) function.
      */
    template <typename PointT>
    class NDT2D: public boost::noncopyable
    {
      typedef typename pcl::PointCloud<PointT> PointCloud;
      typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
      typedef NDTSingleGrid<PointT> SingleGrid;

      public:
        typedef boost::shared_ptr< NDT2D<PointT> > Ptr;
        typedef boost::shared_ptr< const NDT2D<PointT> > ConstPtr;

        /** \brief
          * \param[in] cloud the input point cloud
          * \param[in] about Centre of the grid for normal distributions model
          * \param[in] extent Extent of grid for normal distributions model
          * \param[in] step Size of region that each normal distribution will model
          */
        NDT2D (PointCloudConstPtr cloud,
             const Eigen::Vector2f& about,
             const Eigen::Vector2f& extent,
             const Eigen::Vector2f& step)
        {
          Eigen::Vector2f dx (step[0]/2, 0);
          Eigen::Vector2f dy (0, step[1]/2);
          single_grids_[0] = boost::make_shared<SingleGrid> (cloud, about,        extent, step);
          single_grids_[1] = boost::make_shared<SingleGrid> (cloud, about +dx,    extent, step);
          single_grids_[2] = boost::make_shared<SingleGrid> (cloud, about +dy,    extent, step);
          single_grids_[3] = boost::make_shared<SingleGrid> (cloud, about +dx+dy, extent, step);
        }

        /** \brief Return the 'score' (denormalised likelihood) and derivatives of score of the point p given this distribution.
          * \param[in] transformed_pt   Location to evaluate at.
          * \param[in] cos_theta        sin(theta) of the current rotation angle of rigid transformation: to avoid repeated evaluation
          * \param[in] sin_theta        cos(theta) of the current rotation angle of rigid transformation: to avoid repeated evaluation
          */
        ValueAndDerivatives<3,double>
        test (const PointT& transformed_pt, const double& cos_theta, const double& sin_theta) const
        {
          ValueAndDerivatives<3,double> r = ValueAndDerivatives<3,double>::Zero ();
          for (size_t i = 0; i < 4; i++)
              r += single_grids_[i]->test (transformed_pt, cos_theta, sin_theta);
          return r;
        }

      protected:
        boost::shared_ptr<SingleGrid> single_grids_[4];
    };

  } // namespace ndt2d

  /** \brief @b NormalDistributionsTransform2D provides an implementation of the
    * Normal Distributions Transform algorithm for scan matching.
    *
    * This implementation is intended to match the definition:
    * Peter Biber and Wolfgang Straßer. The normal distributions transform: A
    * new approach to laser scan matching. In Proceedings of the IEEE In-
    * ternational Conference on Intelligent Robots and Systems (IROS), pages
    * 2743–2748, Las Vegas, USA, October 2003.
    *
    * \author James Crosby
    */
  template <typename PointSource, typename PointTarget>
  class NormalDistributionsTransform2D : public Registration<PointSource, PointTarget>
  {
    typedef typename Registration<PointSource, PointTarget>::PointCloudSource PointCloudSource;
    typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
    typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

    typedef typename Registration<PointSource, PointTarget>::PointCloudTarget PointCloudTarget;
    typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
    typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

    typedef PointIndices::Ptr PointIndicesPtr;
    typedef PointIndices::ConstPtr PointIndicesConstPtr;

    public:
      typedef boost::shared_ptr< NormalDistributionsTransform2D<PointSource, PointTarget> > Ptr;
      typedef boost::shared_ptr< const NormalDistributionsTransform2D<PointSource, PointTarget> > ConstPtr;

      /** \brief Empty constructor. */
      NormalDistributionsTransform2D ()
        : Registration<PointSource,PointTarget> (),
          grid_centre_ (0,0), grid_step_ (1,1), grid_extent_ (20,20), newton_lambda_ (1,1,1)
      {
        reg_name_ = "NormalDistributionsTransform2D";
      }
      
      /** \brief Empty destructor */
      virtual ~NormalDistributionsTransform2D () {}

      /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to).
        * \param[in] cloud the input point cloud target
        */
      inline void
      setInputTarget (const PointCloudTargetConstPtr &cloud)
      {
        Registration<PointSource, PointTarget>::setInputTarget (cloud);
        init ();
      }
 
      /** \brief centre of the ndt grid (target coordinate system)
        * \param centre value to set
        */
      virtual void
      setGridCentre (const Eigen::Vector2f& centre) { grid_centre_ = centre; init(); }

      /** \brief Grid spacing (step) of the NDT grid
        * \param[in] step value to set
        */
      virtual void
      setGridStep (const Eigen::Vector2f& step) { grid_step_ = step; init(); }

      /** \brief NDT Grid extent (in either direction from the grid centre)
        * \param[in] extent value to set
        */
      virtual void
      setGridExtent (const Eigen::Vector2f& extent) { grid_extent_ = extent; init(); }

      /** \brief NDT Newton optimisation step size parameter
        * \param[in] lambda step size: 1 is simple newton optimisation, smaller values may improve convergence
        */
       virtual void
       setOptimizationStepSize (const double& lambda) { newton_lambda_ = Eigen::Vector3d (lambda, lambda, lambda); }

      /** \brief NDT Newton optimisation step size parameter
        * \param[in] lambda step size: (1,1,1) is simple newton optimisation,
        * smaller values may improve convergence, or elements may be set to
        * zero to prevent optimisation over some parameters
        *
        * This overload allows control of updates to the individual (x, y,
        * theta) free parameters in the optimisation. If, for example, theta is
        * believed to be close to the correct value a small value of lambda[2]
        * should be used.
        */
       virtual void
       setOptimizationStepSize (const Eigen::Vector3d& lambda) { newton_lambda_ = lambda; }

    protected:
      /** \brief Rigid transformation computation method with initial guess.
        * \param[out] output the transformed input point cloud dataset using the rigid transformation found
        * \param[in] guess the initial guess of the transformation to compute
        */
      virtual void 
      computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess);

      /**
       * Initialize the NDT grid cells
       */
      void inline
      init()
      {
        if (target_)
          target_ndt_ = typename ndt2d::NDT2D<PointTarget>::Ptr(new ndt2d::NDT2D<PointTarget>(target_, grid_centre_, grid_extent_, grid_step_));
      }

      using Registration<PointSource, PointTarget>::reg_name_;
      using Registration<PointSource, PointTarget>::target_;
      using Registration<PointSource, PointTarget>::converged_;
      using Registration<PointSource, PointTarget>::nr_iterations_;
      using Registration<PointSource, PointTarget>::max_iterations_;
      using Registration<PointSource, PointTarget>::transformation_epsilon_;
      using Registration<PointSource, PointTarget>::transformation_rotation_epsilon_;
      using Registration<PointSource, PointTarget>::transformation_;
      using Registration<PointSource, PointTarget>::previous_transformation_;
      using Registration<PointSource, PointTarget>::final_transformation_;
      using Registration<PointSource, PointTarget>::update_visualizer_;
      using Registration<PointSource, PointTarget>::indices_;

      Eigen::Vector2f grid_centre_;
      Eigen::Vector2f grid_step_;
      Eigen::Vector2f grid_extent_;
      Eigen::Vector3d newton_lambda_;
      typename ndt2d::NDT2D<PointTarget>::Ptr target_ndt_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace pcl

#include <pcl/registration/impl/ndt_2d.hpp>

#endif // ndef PCL_NDT_2D_H_


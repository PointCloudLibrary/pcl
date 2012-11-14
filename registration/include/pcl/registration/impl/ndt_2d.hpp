/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
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
#ifndef PCL_NDT_2D_IMPL_H_
#define PCL_NDT_2D_IMPL_H_
#include <cmath>

#include <pcl/registration/eigen.h>
#include <pcl/registration/boost.h>

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
        estimateParams (const PointCloud& cloud, double min_covar_eigvalue_mult = 0.001)
        {
          Eigen::Vector2d sx  = Eigen::Vector2d::Zero ();
          Eigen::Matrix2d sxx = Eigen::Matrix2d::Zero ();
          
          std::vector<size_t>::const_iterator i;
          for (i = pt_indices_.begin (); i != pt_indices_.end (); i++)
          {
            Eigen::Vector2d p (cloud[*i]. x, cloud[*i]. y);
            sx  += p;
            sxx += p * p.transpose ();
          }
          
          n_ = pt_indices_.size ();

          if (n_ >= min_n_)
          {
            mean_ = sx / static_cast<double> (n_);
            // Using maximum likelihood estimation as in the original paper
            Eigen::Matrix2d covar = (sxx - 2 * (sx * mean_.transpose ())) / static_cast<double> (n_) + mean_ * mean_.transpose ();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver (covar);
            if (solver.eigenvalues ()[0] < min_covar_eigvalue_mult * solver.eigenvalues ()[1])
            {
              PCL_DEBUG ("[pcl::NormalDist::estimateParams] NDT normal fit: adjusting eigenvalue %f\n", solver.eigenvalues ()[0]);
              Eigen::Matrix2d l = solver.eigenvalues ().asDiagonal ();
              Eigen::Matrix2d q = solver.eigenvectors ();
              // set minimum smallest eigenvalue:
              l (0,0) = l (1,1) * min_covar_eigvalue_mult;
              covar = q * l * q.transpose ();
            }
            covar_inv_ = covar.inverse ();
          }

          pt_indices_.clear ();
        }

        /** \brief Return the 'score' (denormalised likelihood) and derivatives of score of the point p given this distribution.
          * \param[in] transformed_pt   Location to evaluate at.
          * \param[in] cos_theta        sin(theta) of the current rotation angle of rigid transformation: to avoid repeated evaluation
          * \param[in] sin_theta        cos(theta) of the current rotation angle of rigid transformation: to avoid repeated evaluation
          * estimateParams must have been called after at least three points were provided, or this will return zero.
          *
          */
        ValueAndDerivatives<3,double>
        test (const PointT& transformed_pt, const double& cos_theta, const double& sin_theta) const
        {
          if (n_ < min_n_)
            return ValueAndDerivatives<3,double>::Zero ();
          
          ValueAndDerivatives<3,double> r;
          const double x = transformed_pt.x;
          const double y = transformed_pt.y;
          const Eigen::Vector2d p_xy (transformed_pt.x, transformed_pt.y);
          const Eigen::Vector2d q = p_xy - mean_;
          const Eigen::RowVector2d qt_cvi (q.transpose () * covar_inv_);
          const double exp_qt_cvi_q = std::exp (-0.5 * double (qt_cvi * q));
          r.value = -exp_qt_cvi_q;

          Eigen::Matrix<double, 2, 3> jacobian;
          jacobian <<
            1, 0, -(x * sin_theta + y*cos_theta),
            0, 1,   x * cos_theta - y*sin_theta;
          
          for (size_t i = 0; i < 3; i++)
            r.grad[i] = double (qt_cvi * jacobian.col (i)) * exp_qt_cvi_q;
          
          // second derivative only for i == j == 2:
          const Eigen::Vector2d d2q_didj (
              y * sin_theta - x*cos_theta,
            -(x * sin_theta + y*cos_theta)
          );

          for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 3; j++)
              r.hessian (i,j) = -exp_qt_cvi_q * (
                double (-qt_cvi*jacobian.col (i)) * double (-qt_cvi*jacobian.col (j)) +
                (-qt_cvi * ((i==2 && j==2)? d2q_didj : Eigen::Vector2d::Zero ())) +
                (-jacobian.col (j).transpose () * covar_inv_ * jacobian.col (i))
              );
          
          return r;
        }

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
} // namespace pcl


namespace Eigen
{
  /* This NumTraits specialisation is necessary because NormalDist is used as
   * the element type of an Eigen Matrix.
   */
  template<typename PointT> struct NumTraits<pcl::ndt2d::NormalDist<PointT> >
  {
    typedef double Real;
    static Real dummy_precision () { return 1.0; }
    enum {
      IsComplex = 0,
      IsInteger = 0,
      IsSigned = 0,
      RequireInitialization = 1,
      ReadCost = 1,
      AddCost = 1,
      MulCost = 1
    };
  };
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::NormalDistributionsTransform2D<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess)
{
  PointCloudSource intm_cloud = output;

  if (guess != Eigen::Matrix4f::Identity ())
  {
    transformation_ = guess;
    transformPointCloud (output, intm_cloud, transformation_);
  } 

  // build Normal Distribution Transform of target cloud:
  ndt2d::NDT2D<PointTarget> target_ndt (target_, grid_centre_, grid_extent_, grid_step_);
  
  // can't seem to use .block<> () member function on transformation_
  // directly... gcc bug? 
  Eigen::Matrix4f& transformation = transformation_;


  // work with x translation, y translation and z rotation: extending to 3D
  // would be some tricky maths, but not impossible.
  const Eigen::Matrix3f initial_rot (transformation.block<3,3> (0,0));
  const Eigen::Vector3f rot_x (initial_rot*Eigen::Vector3f::UnitX ());
  const double z_rotation = std::atan2 (rot_x[1], rot_x[0]);

  Eigen::Vector3d xytheta_transformation (
    transformation (0,3),
    transformation (1,3),
    z_rotation
  );

  while (!converged_)
  {
    const double cos_theta = std::cos (xytheta_transformation[2]);
    const double sin_theta = std::sin (xytheta_transformation[2]);
    previous_transformation_ = transformation;    

    ndt2d::ValueAndDerivatives<3, double> score = ndt2d::ValueAndDerivatives<3, double>::Zero ();
    for (size_t i = 0; i < intm_cloud.size (); i++)
      score += target_ndt.test (intm_cloud[i], cos_theta, sin_theta);
    
    PCL_DEBUG ("[pcl::NormalDistributionsTransform2D::computeTransformation] NDT score %f (x=%f,y=%f,r=%f)\n",
      float (score.value), xytheta_transformation[0], xytheta_transformation[1], xytheta_transformation[2]
    );

    if (score.value != 0)
    {
      // test for positive definiteness, and adjust to ensure it if necessary:
      Eigen::EigenSolver<Eigen::Matrix3d> solver;
      solver.compute (score.hessian, false);
      double min_eigenvalue = 0;
      for (int i = 0; i <3; i++)
        if (solver.eigenvalues ()[i].real () < min_eigenvalue)
            min_eigenvalue = solver.eigenvalues ()[i].real ();

      // ensure "safe" positive definiteness: this is a detail missing
      // from the original paper
      if (min_eigenvalue < 0)
      {
        double lambda = 1.1 * min_eigenvalue - 1;
        score.hessian += Eigen::Vector3d (-lambda, -lambda, -lambda).asDiagonal ();
        solver.compute (score.hessian, false);
        PCL_DEBUG ("[pcl::NormalDistributionsTransform2D::computeTransformation] adjust hessian: %f: new eigenvalues:%f %f %f\n",
            float (lambda),
            solver.eigenvalues ()[0].real (),
            solver.eigenvalues ()[1].real (),
            solver.eigenvalues ()[2].real ()
        );
      }
      assert (solver.eigenvalues ()[0].real () >= 0 &&
              solver.eigenvalues ()[1].real () >= 0 &&
              solver.eigenvalues ()[2].real () >= 0);
      
      Eigen::Vector3d delta_transformation (-score.hessian.inverse () * score.grad);
      Eigen::Vector3d new_transformation = xytheta_transformation + newton_lambda_.cwiseProduct (delta_transformation);

      xytheta_transformation = new_transformation;
      
      // update transformation matrix from x, y, theta:
      transformation.block<3,3> (0,0).matrix () = Eigen::Matrix3f (Eigen::AngleAxisf (static_cast<float> (xytheta_transformation[2]), Eigen::Vector3f::UnitZ ()));
      transformation.block<3,1> (0,3).matrix () = Eigen::Vector3f (static_cast<float> (xytheta_transformation[0]), static_cast<float> (xytheta_transformation[1]), 0.0f);

      //std::cout << "new transformation:\n" << transformation << std::endl;
    }
    else
    {
      PCL_ERROR ("[pcl::NormalDistributionsTransform2D::computeTransformation] no overlap: try increasing the size or reducing the step of the grid\n");
      break;
    }
    
    transformPointCloud (output, intm_cloud, transformation);

    nr_iterations_++;
    
    if (update_visualizer_ != 0)
      update_visualizer_ (output, *indices_, *target_, *indices_);

    //std::cout << "eps=" << fabs ((transformation - previous_transformation_).sum ()) << std::endl;

    if (nr_iterations_ > max_iterations_ ||
       (transformation - previous_transformation_).array ().abs ().sum () < transformation_epsilon_)
      converged_ = true;
  }
  final_transformation_ = transformation;
  output = intm_cloud;
}

#endif    // PCL_NDT_2D_IMPL_H_
 

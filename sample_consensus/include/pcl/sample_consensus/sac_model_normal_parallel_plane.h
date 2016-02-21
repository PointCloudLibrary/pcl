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

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_NORMALPARALLELPLANE_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_NORMALPARALLELPLANE_H_

#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/model_types.h>

namespace pcl
{
  /** \brief SampleConsensusModelNormalParallelPlane defines a model for 3D
    * plane segmentation using additional surface normal constraints. Basically
    * this means that checking for inliers will not only involve a "distance to
    * model" criterion, but also an additional "maximum angular deviation"
    * between the plane's normal and the inlier points normals. In addition,
    * the plane normal must lie parallel to an user-specified axis.
    *
    * The model coefficients are defined as:
    *   - \b a : the X coordinate of the plane's normal (normalized)
    *   - \b b : the Y coordinate of the plane's normal (normalized)
    *   - \b c : the Z coordinate of the plane's normal (normalized)
    *   - \b d : the fourth <a href="http://mathworld.wolfram.com/HessianNormalForm.html">Hessian component</a> of the plane's equation
    *
    * To set the influence of the surface normals in the inlier estimation
    * process, set the normal weight (0.0-1.0), e.g.:
    * \code
    * SampleConsensusModelNormalPlane<pcl::PointXYZ, pcl::Normal> sac_model;
    * ...
    * sac_model.setNormalDistanceWeight (0.1);
    * ...
    * \endcode
    *
    * In addition, the user can specify more constraints, such as:
    * 
    *   - an axis along which we need to search for a plane perpendicular to (\ref setAxis);
    *   - an angle \a tolerance threshold between the plane's normal and the above given axis (\ref setEpsAngle);
    *   - a distance we expect the plane to be from the origin (\ref setDistanceFromOrigin);
    *   - a distance \a tolerance as the maximum allowed deviation from the above given distance from the origin (\ref setEpsDist).
    *
    * \note Please remember that you need to specify an angle > 0 in order to activate the axis-angle constraint!
    *
    * \author Radu B. Rusu and Jared Glover and Nico Blodow
    * \ingroup sample_consensus
    */
  template <typename PointT, typename PointNT>
  class SampleConsensusModelNormalParallelPlane : public SampleConsensusModelNormalPlane<PointT, PointNT>
  {
    public:
      using SampleConsensusModel<PointT>::model_name_;
      using SampleConsensusModel<PointT>::input_;
      using SampleConsensusModel<PointT>::indices_;
      using SampleConsensusModelFromNormals<PointT, PointNT>::normals_;
      using SampleConsensusModelFromNormals<PointT, PointNT>::normal_distance_weight_;
      using SampleConsensusModel<PointT>::error_sqr_dists_;

      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef typename SampleConsensusModelFromNormals<PointT, PointNT>::PointCloudNPtr PointCloudNPtr;
      typedef typename SampleConsensusModelFromNormals<PointT, PointNT>::PointCloudNConstPtr PointCloudNConstPtr;

      typedef boost::shared_ptr<SampleConsensusModelNormalParallelPlane> Ptr;

      /** \brief Constructor for base SampleConsensusModelNormalParallelPlane.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelNormalParallelPlane (const PointCloudConstPtr &cloud,
                                               bool random = false) 
        : SampleConsensusModelNormalPlane<PointT, PointNT> (cloud, random)
        , axis_ (Eigen::Vector4f::Zero ())
        , distance_from_origin_ (0)
        , eps_angle_ (-1.0)
        , cos_angle_ (-1.0)
        , eps_dist_ (0.0)
      {
        model_name_ = "SampleConsensusModelNormalParallelPlane";
        sample_size_ = 3;
        model_size_ = 4;
      }

      /** \brief Constructor for base SampleConsensusModelNormalParallelPlane.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelNormalParallelPlane (const PointCloudConstPtr &cloud, 
                                               const std::vector<int> &indices,
                                               bool random = false) 
        : SampleConsensusModelNormalPlane<PointT, PointNT> (cloud, indices, random)
        , axis_ (Eigen::Vector4f::Zero ())
        , distance_from_origin_ (0)
        , eps_angle_ (-1.0)
        , cos_angle_ (-1.0)
        , eps_dist_ (0.0)
      {
        model_name_ = "SampleConsensusModelNormalParallelPlane";
        sample_size_ = 3;
        model_size_ = 4;
      }
      
      /** \brief Empty destructor */
      virtual ~SampleConsensusModelNormalParallelPlane () {}

      /** \brief Set the axis along which we need to search for a plane perpendicular to.
        * \param[in] ax the axis along which we need to search for a plane perpendicular to
        */
      inline void
      setAxis (const Eigen::Vector3f &ax) { axis_.head<3> () = ax; axis_.normalize ();}

      /** \brief Get the axis along which we need to search for a plane perpendicular to. */
      inline Eigen::Vector3f
      getAxis () { return (axis_.head<3> ()); }

      /** \brief Set the angle epsilon (delta) threshold.
        * \param[in] ea the maximum allowed deviation from 90 degrees between the plane normal and the given axis.
        * \note You need to specify an angle > 0 in order to activate the axis-angle constraint!
        */
      inline void
      setEpsAngle (const double ea) { eps_angle_ = ea; cos_angle_ = fabs (cos (ea));}

      /** \brief Get the angle epsilon (delta) threshold. */
      inline double
      getEpsAngle () { return (eps_angle_); }

      /** \brief Set the distance we expect the plane to be from the origin
        * \param[in] d distance from the template plane to the origin
        */
      inline void
      setDistanceFromOrigin (const double d) { distance_from_origin_ = d; }

      /** \brief Get the distance of the plane from the origin. */
      inline double
      getDistanceFromOrigin () { return (distance_from_origin_); }

      /** \brief Set the distance epsilon (delta) threshold.
        * \param[in] delta the maximum allowed deviation from the template distance from the origin
        */
      inline void
      setEpsDist (const double delta) { eps_dist_ = delta; }

      /** \brief Get the distance epsilon (delta) threshold. */
      inline double
      getEpsDist () { return (eps_dist_); }

      /** \brief Return an unique id for this model (SACMODEL_NORMAL_PARALLEL_PLANE). */
      inline pcl::SacModel
      getModelType () const { return (SACMODEL_NORMAL_PARALLEL_PLANE); }

    	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
      using SampleConsensusModel<PointT>::sample_size_;
      using SampleConsensusModel<PointT>::model_size_;

      /** \brief Check whether a model is valid given the user constraints.
        * \param[in] model_coefficients the set of model coefficients
        */
      virtual bool
      isModelValid (const Eigen::VectorXf &model_coefficients);

   private:
      /** \brief The axis along which we need to search for a plane perpendicular to. */
      Eigen::Vector4f axis_;

      /** \brief The distance from the template plane to the origin. */
      double distance_from_origin_;

      /** \brief The maximum allowed difference between the plane normal and the given axis.  */
      double eps_angle_;

      /** \brief The cosine of the angle*/
      double cos_angle_;
      /** \brief The maximum allowed deviation from the template distance from the origin. */
      double eps_dist_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>
#endif

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_NORMALPARALLELPLANE_H_

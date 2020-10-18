/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PCL_SEGMENTATION_IMPL_SAC_SEGMENTATION_H_
#define PCL_SEGMENTATION_IMPL_SAC_SEGMENTATION_H_

#include <pcl/segmentation/sac_segmentation.h>

// Sample Consensus methods
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/rmsac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/prosac.h>

// Sample Consensus models
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/sample_consensus/sac_model_stick.h>

#include <pcl/memory.h>  // for static_pointer_cast

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SACSegmentation<PointT>::segment (PointIndices &inliers, ModelCoefficients &model_coefficients)
{
  // Copy the header information
  inliers.header = model_coefficients.header = input_->header;

  if (!initCompute ()) 
  {
    inliers.indices.clear (); model_coefficients.values.clear ();
    return;
  }

  // Initialize the Sample Consensus model and set its parameters
  if (!initSACModel (model_type_))
  {
    PCL_ERROR ("[pcl::%s::segment] Error initializing the SAC model!\n", getClassName ().c_str ());
    deinitCompute ();
    inliers.indices.clear (); model_coefficients.values.clear ();
    return;
  }
  // Initialize the Sample Consensus method and set its parameters
  initSAC (method_type_);

  if (!sac_->computeModel (0))
  {
    PCL_ERROR ("[pcl::%s::segment] Error segmenting the model! No solution found.\n", getClassName ().c_str ());
    deinitCompute ();
    inliers.indices.clear (); model_coefficients.values.clear ();
    return;
  }

  // Get the model inliers
  sac_->getInliers (inliers.indices);

  // Get the model coefficients
  Eigen::VectorXf coeff (model_->getModelSize ());
  sac_->getModelCoefficients (coeff);

  // If the user needs optimized coefficients
  if (optimize_coefficients_)
  {
    Eigen::VectorXf coeff_refined (model_->getModelSize ());
    model_->optimizeModelCoefficients (inliers.indices, coeff, coeff_refined);
    model_coefficients.values.resize (coeff_refined.size ());
    memcpy (&model_coefficients.values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));
    // Refine inliers
    model_->selectWithinDistance (coeff_refined, threshold_, inliers.indices);
  }
  else
  {
    model_coefficients.values.resize (coeff.size ());
    memcpy (&model_coefficients.values[0], &coeff[0], coeff.size () * sizeof (float));
  }

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SACSegmentation<PointT>::initSACModel (const int model_type)
{
  if (model_)
    model_.reset ();

  // Build the model
  switch (model_type)
  {
    case SACMODEL_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_PLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelPlane<PointT> (input_, *indices_, random_));
      break;
    }
    case SACMODEL_LINE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_LINE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelLine<PointT> (input_, *indices_, random_));
      break;
    }
    case SACMODEL_STICK:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_STICK\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelStick<PointT> (input_, *indices_));
      double min_radius, max_radius;
      model_->getRadiusLimits (min_radius, max_radius);
      if (radius_min_ != min_radius && radius_max_ != max_radius)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting radius limits to %f/%f\n", getClassName ().c_str (), radius_min_, radius_max_);
        model_->setRadiusLimits (radius_min_, radius_max_);
      }
      break;
    }
    case SACMODEL_CIRCLE2D:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_CIRCLE2D\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelCircle2D<PointT> (input_, *indices_, random_));
      typename SampleConsensusModelCircle2D<PointT>::Ptr model_circle = static_pointer_cast<SampleConsensusModelCircle2D<PointT> > (model_);
      double min_radius, max_radius;
      model_circle->getRadiusLimits (min_radius, max_radius);
      if (radius_min_ != min_radius && radius_max_ != max_radius)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting radius limits to %f/%f\n", getClassName ().c_str (), radius_min_, radius_max_);
        model_circle->setRadiusLimits (radius_min_, radius_max_);
      }
      break;
    }
    case SACMODEL_CIRCLE3D:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_CIRCLE3D\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelCircle3D<PointT> (input_, *indices_));
      typename SampleConsensusModelCircle3D<PointT>::Ptr model_circle3d = static_pointer_cast<SampleConsensusModelCircle3D<PointT> > (model_);
      double min_radius, max_radius;
      model_circle3d->getRadiusLimits (min_radius, max_radius);
      if (radius_min_ != min_radius && radius_max_ != max_radius)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting radius limits to %f/%f\n", getClassName ().c_str (), radius_min_, radius_max_);
        model_circle3d->setRadiusLimits (radius_min_, radius_max_);
      }
      break;
    }
    case SACMODEL_SPHERE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_SPHERE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelSphere<PointT> (input_, *indices_, random_));
      typename SampleConsensusModelSphere<PointT>::Ptr model_sphere = static_pointer_cast<SampleConsensusModelSphere<PointT> > (model_);
      double min_radius, max_radius;
      model_sphere->getRadiusLimits (min_radius, max_radius);
      if (radius_min_ != min_radius && radius_max_ != max_radius)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting radius limits to %f/%f\n", getClassName ().c_str (), radius_min_, radius_max_);
        model_sphere->setRadiusLimits (radius_min_, radius_max_);
      }
      break;
    }
    case SACMODEL_PARALLEL_LINE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_PARALLEL_LINE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelParallelLine<PointT> (input_, *indices_, random_));
      typename SampleConsensusModelParallelLine<PointT>::Ptr model_parallel = static_pointer_cast<SampleConsensusModelParallelLine<PointT> > (model_);
      if (axis_ != Eigen::Vector3f::Zero () && model_parallel->getAxis () != axis_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the axis to %f, %f, %f\n", getClassName ().c_str (), axis_[0], axis_[1], axis_[2]);
        model_parallel->setAxis (axis_);
      }
      if (eps_angle_ != 0.0 && model_parallel->getEpsAngle () != eps_angle_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the epsilon angle to %f (%f degrees)\n", getClassName ().c_str (), eps_angle_, eps_angle_ * 180.0 / M_PI);
        model_parallel->setEpsAngle (eps_angle_);
      }
      break;
    }
    case SACMODEL_PERPENDICULAR_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_PERPENDICULAR_PLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelPerpendicularPlane<PointT> (input_, *indices_, random_));
      typename SampleConsensusModelPerpendicularPlane<PointT>::Ptr model_perpendicular = static_pointer_cast<SampleConsensusModelPerpendicularPlane<PointT> > (model_);
      if (axis_ != Eigen::Vector3f::Zero () && model_perpendicular->getAxis () != axis_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the axis to %f, %f, %f\n", getClassName ().c_str (), axis_[0], axis_[1], axis_[2]);
        model_perpendicular->setAxis (axis_);
      }
      if (eps_angle_ != 0.0 && model_perpendicular->getEpsAngle () != eps_angle_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the epsilon angle to %f (%f degrees)\n", getClassName ().c_str (), eps_angle_, eps_angle_ * 180.0 / M_PI);
        model_perpendicular->setEpsAngle (eps_angle_);
      }
      break;
    }
    case SACMODEL_PARALLEL_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_PARALLEL_PLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelParallelPlane<PointT> (input_, *indices_, random_));
      typename SampleConsensusModelParallelPlane<PointT>::Ptr model_parallel = static_pointer_cast<SampleConsensusModelParallelPlane<PointT> > (model_);
      if (axis_ != Eigen::Vector3f::Zero () && model_parallel->getAxis () != axis_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the axis to %f, %f, %f\n", getClassName ().c_str (), axis_[0], axis_[1], axis_[2]);
        model_parallel->setAxis (axis_);
      }
      if (eps_angle_ != 0.0 && model_parallel->getEpsAngle () != eps_angle_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the epsilon angle to %f (%f degrees)\n", getClassName ().c_str (), eps_angle_, eps_angle_ * 180.0 / M_PI);
        model_parallel->setEpsAngle (eps_angle_);
      }
      break;
    }
    default:
    {
      PCL_ERROR ("[pcl::%s::initSACModel] No valid model given!\n", getClassName ().c_str ());
      return (false);
    }
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SACSegmentation<PointT>::initSAC (const int method_type)
{
  if (sac_)
    sac_.reset ();
  // Build the sample consensus method
  switch (method_type)
  {
    case SAC_RANSAC:
    default:
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Using a method of type: SAC_RANSAC with a model threshold of %f\n", getClassName ().c_str (), threshold_);
      sac_.reset (new RandomSampleConsensus<PointT> (model_, threshold_));
      break;
    }
    case SAC_LMEDS:
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Using a method of type: SAC_LMEDS with a model threshold of %f\n", getClassName ().c_str (), threshold_);
      sac_.reset (new LeastMedianSquares<PointT> (model_, threshold_));
      break;
    }
    case SAC_MSAC:
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Using a method of type: SAC_MSAC with a model threshold of %f\n", getClassName ().c_str (), threshold_);
      sac_.reset (new MEstimatorSampleConsensus<PointT> (model_, threshold_));
      break;
    }
    case SAC_RRANSAC:
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Using a method of type: SAC_RRANSAC with a model threshold of %f\n", getClassName ().c_str (), threshold_);
      sac_.reset (new RandomizedRandomSampleConsensus<PointT> (model_, threshold_));
      break;
    }
    case SAC_RMSAC:
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Using a method of type: SAC_RMSAC with a model threshold of %f\n", getClassName ().c_str (), threshold_);
      sac_.reset (new RandomizedMEstimatorSampleConsensus<PointT> (model_, threshold_));
      break;
    }
    case SAC_MLESAC:
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Using a method of type: SAC_MLESAC with a model threshold of %f\n", getClassName ().c_str (), threshold_);
      sac_.reset (new MaximumLikelihoodSampleConsensus<PointT> (model_, threshold_));
      break;
    }
    case SAC_PROSAC:
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Using a method of type: SAC_PROSAC with a model threshold of %f\n", getClassName ().c_str (), threshold_);
      sac_.reset (new ProgressiveSampleConsensus<PointT> (model_, threshold_));
      break;
    }
  }
  // Set the Sample Consensus parameters if they are given/changed
  if (sac_->getProbability () != probability_)
  {
    PCL_DEBUG ("[pcl::%s::initSAC] Setting the desired probability to %f\n", getClassName ().c_str (), probability_);
    sac_->setProbability (probability_);
  }
  if (max_iterations_ != -1 && sac_->getMaxIterations () != max_iterations_)
  {
    PCL_DEBUG ("[pcl::%s::initSAC] Setting the maximum number of iterations to %d\n", getClassName ().c_str (), max_iterations_);
    sac_->setMaxIterations (max_iterations_);
  }
  if (samples_radius_ > 0.)
  {
    PCL_DEBUG ("[pcl::%s::initSAC] Setting the maximum sample radius to %f\n", getClassName ().c_str (), samples_radius_);
    // Set maximum distance for radius search during random sampling
    model_->setSamplesMaxDist (samples_radius_, samples_radius_search_);
  }
  if (sac_->getNumberOfThreads () != threads_)
  {
    PCL_DEBUG ("[pcl::%s::initSAC] Setting the number of threads to %i\n", getClassName ().c_str (), threads_);
    sac_->setNumberOfThreads (threads_);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SACSegmentationFromNormals<PointT, PointNT>::initSACModel (const int model_type)
{
  if (!input_ || !normals_)
  {
    PCL_ERROR ("[pcl::%s::initSACModel] Input data (XYZ or normals) not given! Cannot continue.\n", getClassName ().c_str ());
    return (false);
  }
  // Check if input is synced with the normals
  if (input_->size () != normals_->size ())
  {
    PCL_ERROR ("[pcl::%s::initSACModel] The number of points in the input point cloud differs than the number of points in the normals!\n", getClassName ().c_str ());
    return (false);
  }

  if (model_)
    model_.reset ();

  // Build the model
  switch (model_type)
  {
    case SACMODEL_CYLINDER:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_CYLINDER\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelCylinder<PointT, PointNT > (input_, *indices_, random_));
      typename SampleConsensusModelCylinder<PointT, PointNT>::Ptr model_cylinder = static_pointer_cast<SampleConsensusModelCylinder<PointT, PointNT> > (model_);

      // Set the input normals
      model_cylinder->setInputNormals (normals_);
      double min_radius, max_radius;
      model_cylinder->getRadiusLimits (min_radius, max_radius);
      if (radius_min_ != min_radius && radius_max_ != max_radius)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting radius limits to %f/%f\n", getClassName ().c_str (), radius_min_, radius_max_);
        model_cylinder->setRadiusLimits (radius_min_, radius_max_);
      }
      if (distance_weight_ != model_cylinder->getNormalDistanceWeight ())
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting normal distance weight to %f\n", getClassName ().c_str (), distance_weight_);
        model_cylinder->setNormalDistanceWeight (distance_weight_);
      }
      if (axis_ != Eigen::Vector3f::Zero () && model_cylinder->getAxis () != axis_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the axis to %f, %f, %f\n", getClassName ().c_str (), axis_[0], axis_[1], axis_[2]);
        model_cylinder->setAxis (axis_);
      }
      if (eps_angle_ != 0.0 && model_cylinder->getEpsAngle () != eps_angle_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the epsilon angle to %f (%f degrees)\n", getClassName ().c_str (), eps_angle_, eps_angle_ * 180.0 / M_PI);
        model_cylinder->setEpsAngle (eps_angle_);
      }
      break;
    }
    case SACMODEL_NORMAL_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_NORMAL_PLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelNormalPlane<PointT, PointNT> (input_, *indices_, random_));
      typename SampleConsensusModelNormalPlane<PointT, PointNT>::Ptr model_normals = static_pointer_cast<SampleConsensusModelNormalPlane<PointT, PointNT> > (model_);
      // Set the input normals
      model_normals->setInputNormals (normals_);
      if (distance_weight_ != model_normals->getNormalDistanceWeight ())
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting normal distance weight to %f\n", getClassName ().c_str (), distance_weight_);
        model_normals->setNormalDistanceWeight (distance_weight_);
      }
      break;
    }
    case SACMODEL_NORMAL_PARALLEL_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_NORMAL_PARALLEL_PLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelNormalParallelPlane<PointT, PointNT> (input_, *indices_, random_));
      typename SampleConsensusModelNormalParallelPlane<PointT, PointNT>::Ptr model_normals = static_pointer_cast<SampleConsensusModelNormalParallelPlane<PointT, PointNT> > (model_);
      // Set the input normals
      model_normals->setInputNormals (normals_);
      if (distance_weight_ != model_normals->getNormalDistanceWeight ())
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting normal distance weight to %f\n", getClassName ().c_str (), distance_weight_);
        model_normals->setNormalDistanceWeight (distance_weight_);
      }
      if (distance_from_origin_ != model_normals->getDistanceFromOrigin ())
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the distance to origin to %f\n", getClassName ().c_str (), distance_from_origin_);
        model_normals->setDistanceFromOrigin (distance_from_origin_);
      }
      if (axis_ != Eigen::Vector3f::Zero () && model_normals->getAxis () != axis_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the axis to %f, %f, %f\n", getClassName ().c_str (), axis_[0], axis_[1], axis_[2]);
        model_normals->setAxis (axis_);
      }
      if (eps_angle_ != 0.0 && model_normals->getEpsAngle () != eps_angle_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the epsilon angle to %f (%f degrees)\n", getClassName ().c_str (), eps_angle_, eps_angle_ * 180.0 / M_PI);
        model_normals->setEpsAngle (eps_angle_);
      }
      break;
    }
    case SACMODEL_CONE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_CONE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelCone<PointT, PointNT> (input_, *indices_, random_));
      typename SampleConsensusModelCone<PointT, PointNT>::Ptr model_cone = static_pointer_cast<SampleConsensusModelCone<PointT, PointNT> > (model_);

      // Set the input normals
      model_cone->setInputNormals (normals_);
      double min_angle, max_angle;
      model_cone->getMinMaxOpeningAngle(min_angle, max_angle);
      if (min_angle_ != min_angle && max_angle_ != max_angle)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting minimum and maximum opening angle to %f and %f \n", getClassName ().c_str (), min_angle_, max_angle_);
        model_cone->setMinMaxOpeningAngle (min_angle_, max_angle_);
      }

      if (distance_weight_ != model_cone->getNormalDistanceWeight ())
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting normal distance weight to %f\n", getClassName ().c_str (), distance_weight_);
        model_cone->setNormalDistanceWeight (distance_weight_);
      }
      if (axis_ != Eigen::Vector3f::Zero () && model_cone->getAxis () != axis_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the axis to %f, %f, %f\n", getClassName ().c_str (), axis_[0], axis_[1], axis_[2]);
        model_cone->setAxis (axis_);
      }
      if (eps_angle_ != 0.0 && model_cone->getEpsAngle () != eps_angle_)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting the epsilon angle to %f (%f degrees)\n", getClassName ().c_str (), eps_angle_, eps_angle_ * 180.0 / M_PI);
        model_cone->setEpsAngle (eps_angle_);
      }
      break;
    }
    case SACMODEL_NORMAL_SPHERE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_NORMAL_SPHERE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelNormalSphere<PointT, PointNT> (input_, *indices_, random_));
      typename SampleConsensusModelNormalSphere<PointT, PointNT>::Ptr model_normals_sphere = static_pointer_cast<SampleConsensusModelNormalSphere<PointT, PointNT> > (model_);
      // Set the input normals
      model_normals_sphere->setInputNormals (normals_);
      double min_radius, max_radius;
      model_normals_sphere->getRadiusLimits (min_radius, max_radius);
      if (radius_min_ != min_radius && radius_max_ != max_radius)
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting radius limits to %f/%f\n", getClassName ().c_str (), radius_min_, radius_max_);
        model_normals_sphere->setRadiusLimits (radius_min_, radius_max_);
      }

      if (distance_weight_ != model_normals_sphere->getNormalDistanceWeight ())
      {
        PCL_DEBUG ("[pcl::%s::initSACModel] Setting normal distance weight to %f\n", getClassName ().c_str (), distance_weight_);
        model_normals_sphere->setNormalDistanceWeight (distance_weight_);
      }
      break;
    }
    // If nothing else, try SACSegmentation
    default:
    {
      return (pcl::SACSegmentation<PointT>::initSACModel (model_type));
    }
  }

  return (true);
}

#define PCL_INSTANTIATE_SACSegmentation(T) template class PCL_EXPORTS pcl::SACSegmentation<T>;
#define PCL_INSTANTIATE_SACSegmentationFromNormals(T,NT) template class PCL_EXPORTS pcl::SACSegmentationFromNormals<T,NT>;

#endif        // PCL_SEGMENTATION_IMPL_SAC_SEGMENTATION_H_


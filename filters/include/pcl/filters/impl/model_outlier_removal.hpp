/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl/filters/model_outlier_removal.h>
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::ModelOutlierRemoval<PointT>::initSACModel (pcl::SacModel model_type)
{
  // Build the model
  switch (model_type)
  {
    case SACMODEL_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelPLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelPlane<PointT> (input_));
      break;
    }
    case SACMODEL_LINE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelLINE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelLine<PointT> (input_));
      break;
    }
    case SACMODEL_CIRCLE2D:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelCIRCLE2D\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelCircle2D<PointT> (input_));
      break;
    }
    case SACMODEL_SPHERE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelSPHERE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelSphere<PointT> (input_));
      break;
    }
    case SACMODEL_PARALLEL_LINE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelPARALLEL_LINE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelParallelLine<PointT> (input_));
      break;
    }
    case SACMODEL_PERPENDICULAR_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelPERPENDICULAR_PLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelPerpendicularPlane<PointT> (input_));
      break;
    }
    case SACMODEL_CYLINDER:
    {
      PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelCYLINDER\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelCylinder<PointT, pcl::Normal> (input_));
      break;
    }
    case SACMODEL_NORMAL_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelNORMAL_PLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelNormalPlane<PointT, pcl::Normal> (input_));
      break;
    }
    case SACMODEL_CONE:
    {
      PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelCONE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelCone<PointT, pcl::Normal> (input_));
      break;
    }
    case SACMODEL_NORMAL_SPHERE:
    {
      PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelNORMAL_SPHERE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelNormalSphere<PointT, pcl::Normal> (input_));
      break;
    }
    case SACMODEL_NORMAL_PARALLEL_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelNORMAL_PARALLEL_PLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelNormalParallelPlane<PointT, pcl::Normal> (input_));
      break;
    }
    case SACMODEL_PARALLEL_PLANE:
    {
      PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelPARALLEL_PLANE\n", getClassName ().c_str ());
      model_.reset (new SampleConsensusModelParallelPlane<PointT> (input_));
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ModelOutlierRemoval<PointT>::applyFilterIndices (Indices &indices)
{
  //The arrays to be used
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator
  //is the filtersetup correct?
  bool valid_setup = true;

  valid_setup &= initSACModel (model_type_);

  using SACModelFromNormals = SampleConsensusModelFromNormals<PointT, pcl::Normal>;
  // Returns NULL if cast isn't possible
  auto *model_from_normals = dynamic_cast<SACModelFromNormals *> (& (*model_));

  if (model_from_normals)
  {
    if (!cloud_normals_)
    {
      valid_setup = false;
      PCL_ERROR ("[pcl::ModelOutlierRemoval::applyFilterIndices]: no normals cloud set.\n");
    }
    else
    {
      model_from_normals->setNormalDistanceWeight (normals_distance_weight_);
      model_from_normals->setInputNormals (cloud_normals_);
    }
  }

  //if the filter setup is invalid filter for nan and return;
  if (!valid_setup)
  {
    for (const auto& iii : (*indices_)) // iii = input indices iterator
    {
      // Non-finite entries are always passed to removed indices
      if (!isFinite ((*input_)[iii]))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = iii;
        continue;
      }
      indices[oii++] = iii;
    }
    return;
  }
  // check distance of pointcloud to model
  std::vector<double> distances;
  //TODO: get signed distances !
  model_->setIndices(indices_); // added to reduce computation and arrange distances with indices
  model_->getDistancesToModel (model_coefficients_, distances);

  bool thresh_result;

  // Filter for non-finite entries and the specified field limits
  for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii)  // iii = input indices iterator
  {
    // Non-finite entries are always passed to removed indices
    if (!isFinite ((*input_)[ (*indices_)[iii]]))
    {
      if (extract_removed_indices_)
        (*removed_indices_)[rii++] = (*indices_)[iii];
      continue;
    }

    // use threshold function to separate outliers from inliers:
    thresh_result = threshold_function_ (distances[iii]);

    // in normal mode: define outliers as false thresh_result
    if (!negative_ && !thresh_result)
    {
      if (extract_removed_indices_)
        (*removed_indices_)[rii++] = (*indices_)[iii];
      continue;
    }

    // in negative_ mode: define outliers as true thresh_result
    if (negative_ && thresh_result)
    {
      if (extract_removed_indices_)
        (*removed_indices_)[rii++] = (*indices_)[iii];
      continue;
    }

    // Otherwise it was a normal point for output (inlier)
    indices[oii++] = (*indices_)[iii];

  }

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);

}

#define PCL_INSTANTIATE_ModelOutlierRemoval(T) template class PCL_EXPORTS pcl::ModelOutlierRemoval<T>;


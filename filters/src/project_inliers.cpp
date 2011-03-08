/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: project_inliers.cpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/filters/impl/project_inliers.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ProjectInliers<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &output)
{
  if (indices_->empty ())
  {
    ROS_WARN ("[pcl::%s::applyFilter] No indices given or empty indices!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  //Eigen::Map<Eigen::VectorXf, Eigen::Aligned> model_coefficients (&model_->values[0], model_->values.size ());
  // More expensive than a map but safer (32bit architectures seem to complain)
  Eigen::VectorXf model_coefficients (model_->values.size ());
  for (size_t i = 0; i < model_->values.size (); ++i)
    model_coefficients[i] = model_->values[i];

  // Construct the model and project
  if (!initSACModel (model_type_))
  {
    ROS_ERROR ("[pcl::%s::segment] Error initializing the SAC model!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> cloud_out;

  if (!copy_all_fields_)
    sacmodel_->projectPoints (*indices_, model_coefficients, cloud_out, false);
  else
    sacmodel_->projectPoints (*indices_, model_coefficients, cloud_out, true);

  if (copy_all_data_)
  {
    output.height       = input_->height;
    output.width        = input_->width;
    output.is_bigendian = input_->is_bigendian;
    output.point_step   = input_->point_step;
    output.row_step     = input_->row_step;
    output.data         = input_->data;
    output.is_dense     = input_->is_dense;

    // Get the distance field index
    int x_idx = pcl::getFieldIndex (output, "x");
    int y_idx = pcl::getFieldIndex (output, "y");
    int z_idx = pcl::getFieldIndex (output, "z");

    // Copy the projected points
    for (size_t i = 0; i < indices_->size (); ++i)
    {
      memcpy (&output.data[(*indices_)[i] * output.point_step + output.fields[x_idx].offset], &cloud_out.points[(*indices_)[i]].x, sizeof (float));
      memcpy (&output.data[(*indices_)[i] * output.point_step + output.fields[y_idx].offset], &cloud_out.points[(*indices_)[i]].y, sizeof (float));
      memcpy (&output.data[(*indices_)[i] * output.point_step + output.fields[z_idx].offset], &cloud_out.points[(*indices_)[i]].z, sizeof (float));
    }
  }
  else
  {
    if (!copy_all_fields_)
    {
      pcl::toROSMsg<pcl::PointXYZ> (cloud_out, output);
    }
    else
    {
      // Copy everything
      output.height       = 1;
      output.width        = indices_->size ();
      output.point_step   = input_->point_step;
      output.data.resize (output.width * output.point_step);
      output.is_bigendian = input_->is_bigendian;
      output.row_step     = output.point_step * output.width;
      // All projections should return valid data, so is_dense = true
      output.is_dense     = true;

      // Get the distance field index
      int x_idx = pcl::getFieldIndex (output, "x");
      int y_idx = pcl::getFieldIndex (output, "y");
      int z_idx = pcl::getFieldIndex (output, "z");
      // Copy the projected points
      for (size_t i = 0; i < indices_->size (); ++i)
      {
        memcpy (&output.data[i * output.point_step], &input_->data[(*indices_)[i] * input_->point_step], output.point_step);
        memcpy (&output.data[i * output.point_step + output.fields[x_idx].offset], &cloud_out.points[(*indices_)[i]].x, sizeof (float));
        memcpy (&output.data[i * output.point_step + output.fields[y_idx].offset], &cloud_out.points[(*indices_)[i]].y, sizeof (float));
        memcpy (&output.data[i * output.point_step + output.fields[z_idx].offset], &cloud_out.points[(*indices_)[i]].z, sizeof (float));
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ProjectInliers<sensor_msgs::PointCloud2>::initSACModel (int model_type)
{
  // Convert the input data
  PointCloud<PointXYZ> cloud;
  fromROSMsg (*input_, cloud);
  PointCloud<PointXYZ>::Ptr cloud_ptr = cloud.makeShared ();

  // Build the model
  switch (model_type)
  {
    case SACMODEL_PLANE:
    {
      //ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_PLANE", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelPlane<pcl::PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_LINE:
    {
      //ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_LINE", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelLine<pcl::PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_CIRCLE2D:
    {
      //ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_CIRCLE2D", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelCircle2D<pcl::PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_SPHERE:
    {
      //ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_SPHERE", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelSphere<pcl::PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_PARALLEL_LINE:
    {
      //ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_PARALLEL_LINE", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelParallelLine<pcl::PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_PERPENDICULAR_PLANE:
    {
      //ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_PERPENDICULAR_PLANE", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_CYLINDER:
    {
      //ROS_DEBUG ("[pcl::%s::segment] Using a model of type: SACMODEL_CYLINDER", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelCylinder<pcl::PointXYZ, Normal> (cloud_ptr));
      break;
    }
    case SACMODEL_NORMAL_PLANE:
    {
      //ROS_DEBUG ("[pcl::%s::segment] Using a model of type: SACMODEL_NORMAL_PLANE", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelNormalPlane<pcl::PointXYZ, Normal> (cloud_ptr));
      break;
    }
    case SACMODEL_NORMAL_PARALLEL_PLANE:
    {
      //ROS_DEBUG ("[pcl::%s::segment] Using a model of type: SACMODEL_NORMAL_PARALLEL_PLANE", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelNormalParallelPlane<pcl::PointXYZ, Normal> (cloud_ptr));
      break;
    }
    case SACMODEL_PARALLEL_PLANE:
    {
      //ROS_DEBUG ("[pcl::%s::segment] Using a model of type: SACMODEL_PARALLEL_PLANE", getClassName ().c_str ());
      sacmodel_.reset (new SampleConsensusModelParallelPlane<pcl::PointXYZ> (cloud_ptr));
      break;
    }
    default:
    {
      ROS_ERROR ("[pcl::%s::initSACModel] No valid model given!", getClassName ().c_str ());
      return (false);
    }
  }
  return (true);
}

// Instantiations of specific point types
PCL_INSTANTIATE(ProjectInliers, PCL_XYZ_POINT_TYPES);


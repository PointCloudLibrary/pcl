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
 *
 */

#include <pcl/filters/impl/model_outlier_removal.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ModelOutlierRemoval<pcl::PCLPointCloud2>::initSACModel (int model_type)
{
  // Convert the input data
  PointCloud<PointXYZ> cloud;
  fromPCLPointCloud2 (*input_, cloud);
  PointCloud<PointXYZ>::Ptr cloud_ptr = cloud.makeShared ();

   model_from_normals = NULL;
  // Build the model
  switch (model_type)
  {
    case SACMODEL_PLANE:
    {
      //PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelPLANE\n", getClassName ().c_str ());
      model.reset (new SampleConsensusModelPlane<PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_LINE:
    {
      //PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelLINE\n", getClassName ().c_str ());
      model.reset (new SampleConsensusModelLine<PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_CIRCLE2D:
    {
      //PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelCIRCLE2D\n", getClassName ().c_str ());
      model.reset (new SampleConsensusModelCircle2D<PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_SPHERE:
    {
      //PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelSPHERE\n", getClassName ().c_str ());
      model.reset (new SampleConsensusModelSphere<PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_PARALLEL_LINE:
    {
      //PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelPARALLEL_LINE\n", getClassName ().c_str ());
      model.reset (new SampleConsensusModelParallelLine<PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_PERPENDICULAR_PLANE:
    {
      //PCL_DEBUG ("[pcl::%s::initSACModel] Using a model of type: modelPERPENDICULAR_PLANE\n", getClassName ().c_str ());
      model.reset (new SampleConsensusModelPerpendicularPlane<PointXYZ> (cloud_ptr));
      break;
    }
    case SACMODEL_CYLINDER:
    {
      //PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelCYLINDER\n", getClassName ().c_str ());
      SampleConsensusModelCylinder<PointXYZ, pcl::Normal> *ptr =
        new SampleConsensusModelCylinder<PointXYZ, pcl::Normal> (cloud_ptr);
      model_from_normals = ptr;
      model.reset (ptr);
      break;
    }
    case SACMODEL_NORMAL_PLANE:
    {
      //PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelNORMAL_PLANE\n", getClassName ().c_str ());
      SampleConsensusModelNormalPlane<PointXYZ, pcl::Normal> *ptr =
        new SampleConsensusModelNormalPlane<PointXYZ, pcl::Normal> (cloud_ptr);
      model_from_normals = ptr;
      model.reset (ptr);
      break;
    }
    case SACMODEL_CONE:
    {
      //PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelCONE\n", getClassName ().c_str ());
      SampleConsensusModelCone<PointXYZ, pcl::Normal> *ptr =
        new SampleConsensusModelCone<PointXYZ, pcl::Normal> (cloud_ptr);
      model_from_normals = ptr;
      model.reset (ptr);
      break;
    }
    case SACMODEL_NORMAL_SPHERE:
    {
      //PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelNORMAL_SPHERE\n", getClassName ().c_str ());
      SampleConsensusModelNormalSphere<PointXYZ, pcl::Normal> *ptr =
        new SampleConsensusModelNormalSphere<PointXYZ, pcl::Normal> (cloud_ptr);
      model_from_normals = ptr;
      model.reset (ptr);
      break;
    }
    case SACMODEL_NORMAL_PARALLEL_PLANE:
    {
      //PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelNORMAL_PARALLEL_PLANE\n", getClassName ().c_str ());
      SampleConsensusModelNormalParallelPlane<PointXYZ, pcl::Normal> *ptr 
        = new SampleConsensusModelNormalParallelPlane<PointXYZ, pcl::Normal> (cloud_ptr);
      model_from_normals = ptr;
      model.reset (ptr);
      break;
    }
    case SACMODEL_PARALLEL_PLANE:
    {
      //PCL_DEBUG ("[pcl::%s::segment] Using a model of type: modelPARALLEL_PLANE\n", getClassName ().c_str ());
      model.reset (new SampleConsensusModelParallelPlane<PointXYZ> (cloud_ptr));
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



//////////////////////////////////////////////////////////////////////////
void
pcl::ModelOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (PointCloud2 &output)
{
  if (!input_)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Input dataset not given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  // If fields x/y/z are not present, we cannot filter
  if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  bool valid_setup = initSACModel (model_type_);
 
  if( model_from_normals )
  {
    if( !cloud_normals )
    {
      valid_setup = false;
      PCL_ERROR ("[pcl::ModelOutlierRemoval::applyFilterIndices]: no normals cloud set.\n");
    } else {
      model_from_normals->setNormalDistanceWeight( normals_distance_weight );
      model_from_normals->setInputNormals(cloud_normals);
    }
  }

  if (!valid_setup)
  {
    // Silly - if no filtering is actually done, and we want to keep the data organized, 
    // just copy everything. Any optimizations that can be done here???
    output = *input_;
    return;
  }

  int nr_points = input_->width * input_->height;
  // get distances:
  std::vector<double> distances;
  model->getDistancesToModel (model_coefficients, distances);


  // Check if we're going to keep the organized structure of the cloud or not
  if (keep_organized_)
  {
    output.width = input_->width;
    output.height = input_->height;
    // Check what the user value is: if !finite, set is_dense to false, true otherwise
    if (!pcl_isfinite (user_filter_value_))
      output.is_dense = false;
    else
      output.is_dense = input_->is_dense;
  }
  else
  {
    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height = 1; // filtering breaks the organized structure
    // Because we're doing explit checks for isfinite, is_dense = true
    output.is_dense = true;
  }
  output.row_step = input_->row_step;
  output.point_step = input_->point_step;
  output.is_bigendian = input_->is_bigendian;
  output.data.resize (input_->data.size ());

  removed_indices_->resize (input_->data.size ());

  int nr_p = 0;
  int nr_removed_p = 0;
  // Create the first xyz_offset
  Eigen::Array4i xyz_offset (input_->fields[x_idx_].offset, input_->fields[y_idx_].offset,
                             input_->fields[z_idx_].offset, 0);

  Eigen::Vector4f pt = Eigen::Vector4f::Zero ();

  // @todo fixme
  if (input_->fields[x_idx_].datatype != pcl::PCLPointField::FLOAT32 || 
      input_->fields[y_idx_].datatype != pcl::PCLPointField::FLOAT32 ||
      input_->fields[z_idx_].datatype != pcl::PCLPointField::FLOAT32
     )
  {
    PCL_ERROR ("[pcl::%s::downsample] Distance filtering requested, but distances are not float/double in the dataset! Only FLOAT32/FLOAT64 distances are supported right now.\n", getClassName ().c_str ());
    output.width = output.height = 0;	
    output.data.clear ();
    return;
  }
  float badpt = std::numeric_limits<float>::quiet_NaN ();
  // Check whether we need to store filtered valued in place
  bool thresh_result = false;
  if (keep_organized_)
  {
    // Go over all points
    for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step)
    {
      // Copy all the fields
      memcpy (&output.data[cp * output.point_step], &input_->data[cp * output.point_step], output.point_step);


      // use threshold to seperate outliers (remove list) from inliers
      thresh_result = threshold_function ( distances[cp] );
      if (negative_ )
      {
        // Use a threshold for cutting out points which inside the interval
        if (thresh_result)
        {
          // Unoptimized memcpys: assume fields x, y, z are in random order
          memcpy (&output.data[xyz_offset[0]], &badpt, sizeof(float));
          memcpy (&output.data[xyz_offset[1]], &badpt, sizeof(float));
          memcpy (&output.data[xyz_offset[2]], &badpt, sizeof(float));
          continue;
        }
        else
        {
          if (extract_removed_indices_)
          {
            (*removed_indices_)[nr_removed_p] = cp;
            nr_removed_p++;
          }
        }
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if (!thresh_result)
        {
          // Unoptimized memcpys: assume fields x, y, z are in random order
          memcpy (&output.data[xyz_offset[0]], &badpt, sizeof(float));
          memcpy (&output.data[xyz_offset[1]], &badpt, sizeof(float));
          memcpy (&output.data[xyz_offset[2]], &badpt, sizeof(float));
          continue;
        }
        else
        {
          if (extract_removed_indices_)
          {
            (*removed_indices_)[nr_removed_p] = cp;
            nr_removed_p++;
          }
        }
      }
    }
  }
  // Remove filtered points
  else
  {
    // Go over all points
    for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step)
    {

     // use threshold to seperate outliers (remove list) from inliers
     thresh_result = threshold_function ( distances[cp] );
     if (negative_)
      {
        // Use a threshold for cutting out points which inside the interval
        if (thresh_result)
        {
          if (extract_removed_indices_)
          {
            (*removed_indices_)[nr_removed_p] = cp;
            nr_removed_p++;
          }
          continue;
        }
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if (thresh_result)
        {
          if (extract_removed_indices_)
          {
            (*removed_indices_)[nr_removed_p] = cp;
            nr_removed_p++;
          }
          continue;
        }
      }

      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof(float));
      memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof(float));
      memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof(float));

      // Check if the point is invalid
      if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2]))
      {
        if (extract_removed_indices_)
        {
          (*removed_indices_)[nr_removed_p] = cp;
          nr_removed_p++;
        }
        continue;
      }

      // Copy all the fields
      memcpy (&output.data[nr_p * output.point_step], &input_->data[cp * output.point_step], output.point_step);
      nr_p++;
    }
    output.width = nr_p;
  } // !keep_organized_
  // No distance filtering, process all data. No need to check for is_organized here as we did it above

  output.row_step = output.point_step * output.width;
  output.data.resize (output.width * output.height * output.point_step);

  removed_indices_->resize (nr_removed_p);

}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(ModelOutlierRemoval, PCL_XYZ_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE


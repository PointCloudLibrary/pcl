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
 *  Author: Raphael Favier, Technical University Eindhoven, (r.mysurname <aT> tue.nl)
 */

#ifndef PCL_WORLD_MODEL_IMPL_HPP_
#define PCL_WORLD_MODEL_IMPL_HPP_

#include <pcl/gpu/kinfu_large_scale/world_model.h>

template <typename PointT>
void 
pcl::kinfuLS::WorldModel<PointT>::addSlice ( PointCloudPtr new_cloud)
{
  PCL_DEBUG ("Adding new cloud. Current world contains %d points.\n", world_->points.size ());

  PCL_DEBUG ("New slice contains %d points.\n", new_cloud->points.size ());

  *world_ += *new_cloud;

  PCL_DEBUG ("World now contains  %d points.\n", world_->points.size ());
}


template <typename PointT>
void 
pcl::kinfuLS::WorldModel<PointT>::getExistingData(const double previous_origin_x, const double previous_origin_y, const double previous_origin_z, const double offset_x, const double offset_y, const double offset_z, const double volume_x, const double volume_y, const double volume_z, pcl::PointCloud<PointT> &existing_slice)
{
  double newOriginX = previous_origin_x + offset_x; 
  double newOriginY = previous_origin_y + offset_y; 
  double newOriginZ = previous_origin_z + offset_z;
  double newLimitX = newOriginX + volume_x; 
  double newLimitY = newOriginY + volume_y; 
  double newLimitZ = newOriginZ + volume_z;
	
  // filter points in the space of the new cube
  PointCloudPtr newCube (new pcl::PointCloud<PointT>);
  // condition
  ConditionAndPtr range_condAND (new pcl::ConditionAnd<PointT> ());
  range_condAND->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE, newOriginX)));
  range_condAND->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, newLimitX)));
  range_condAND->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GE, newOriginY)));
  range_condAND->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, newLimitY)));
  range_condAND->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GE, newOriginZ)));
  range_condAND->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, newLimitZ))); 
  
  // build the filter
  pcl::ConditionalRemoval<PointT> condremAND (true);
  condremAND.setCondition (range_condAND);
  condremAND.setInputCloud (world_);
  condremAND.setKeepOrganized (false);
  
  // apply filter
  condremAND.filter (*newCube);
	
  // filter points that belong to the new slice
  ConditionOrPtr range_condOR (new pcl::ConditionOr<PointT> ());
  
  if(offset_x >= 0)
	range_condOR->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE,  previous_origin_x + volume_x - 1.0 )));
  else
	range_condOR->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT,  previous_origin_x )));
	
  if(offset_y >= 0)
	range_condOR->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GE,  previous_origin_y + volume_y - 1.0 )));
  else
	range_condOR->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT,  previous_origin_y )));
	
  if(offset_z >= 0)
	range_condOR->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GE,  previous_origin_z + volume_z - 1.0 )));
  else
	range_condOR->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT,  previous_origin_z )));
  
  // build the filter
  pcl::ConditionalRemoval<PointT> condrem (true);
  condrem.setCondition (range_condOR);
  condrem.setInputCloud (newCube);
  condrem.setKeepOrganized (false);
  // apply filter
  condrem.filter (existing_slice);  
 
  if(existing_slice.points.size () != 0)
  {
	//transform the slice in new cube coordinates
	Eigen::Affine3f transformation; 
	transformation.translation ()[0] = newOriginX;
	transformation.translation ()[1] = newOriginY;
	transformation.translation ()[2] = newOriginZ;
		
	transformation.linear ().setIdentity ();

	transformPointCloud (existing_slice, existing_slice, transformation.inverse ());
	
  }
}


template <typename PointT>
void
pcl::kinfuLS::WorldModel<PointT>::getWorldAsCubes (const double size, std::vector<typename WorldModel<PointT>::PointCloudPtr> &cubes, std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &transforms, double overlap)
{
  
  if(world_->points.size () == 0)
  {
	PCL_INFO("The world is empty, returning nothing\n");
	return;
  }

  PCL_INFO ("Getting world as cubes. World contains %d points.\n", world_->points.size ());

  // remove nans from world cloud
  world_->is_dense = false;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud ( *world_, *world_, indices);
	
  PCL_INFO ("World contains %d points after nan removal.\n", world_->points.size ());
  

  // check cube size value
  double cubeSide = size;
  if (cubeSide <= 0.0f)
  {
	PCL_ERROR ("Size of the cube must be positive and non null (%f given). Setting it to 3.0 meters.\n", cubeSide);
	cubeSide = 512.0f;
  }

  std::cout << "cube size is set to " << cubeSide << std::endl;

  // check overlap value
  double step_increment = 1.0f - overlap;
  if (overlap < 0.0)
  {
	PCL_ERROR ("Overlap ratio must be positive or null (%f given). Setting it to 0.0 procent.\n", overlap);
	step_increment = 1.0f;
  }
  if (overlap > 1.0)
  {
	PCL_ERROR ("Overlap ratio must be less or equal to 1.0 (%f given). Setting it to 10 procent.\n", overlap);
	step_increment = 0.1f;
  }

  
  // get world's bounding values on XYZ
  PointT min, max;
  pcl::getMinMax3D(*world_, min, max);

  PCL_INFO ("Bounding box for the world: \n\t [%f - %f] \n\t [%f - %f] \n\t [%f - %f] \n", min.x, max.x, min.y, max.y, min.z, max.z);

  PointT origin = min;
  
  // clear returned vectors
  cubes.clear();
  transforms.clear();

  // iterate with box filter
  while (origin.x < max.x)
  {
	origin.y = min.y;
	while (origin.y < max.y)
	{
	  origin.z = min.z;
	  while (origin.z < max.z)
	  {
		// extract cube here
		PCL_INFO ("Extracting cube at: [%f, %f, %f].\n",  origin.x,  origin.y,  origin.z);

		// pointcloud for current cube.
		PointCloudPtr box (new pcl::PointCloud<PointT>);


		// set conditional filter
		ConditionAndPtr range_cond (new pcl::ConditionAnd<PointT> ());
		range_cond->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE, origin.x)));
		range_cond->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, origin.x + cubeSide)));
		range_cond->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GE, origin.y)));
		range_cond->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, origin.y + cubeSide)));
		range_cond->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GE, origin.z)));
		range_cond->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, origin.z + cubeSide)));

		// build the filter
		pcl::ConditionalRemoval<PointT> condrem;
		condrem.setCondition (range_cond);
		condrem.setInputCloud (world_);
		condrem.setKeepOrganized(false);
		// apply filter
		condrem.filter (*box);

		// also push transform along with points.
		if(box->points.size() > 0)
		{
		  Eigen::Vector3f transform;
		  transform[0] = origin.x, transform[1] = origin.y, transform[2] = origin.z;
		  transforms.push_back(transform);
		  cubes.push_back(box);        
		}
		else
		{
		  PCL_INFO ("Extracted cube was empty, skiping this one.\n");
		}
		origin.z += cubeSide * step_increment;
	  }
	  origin.y += cubeSide * step_increment;
	}
	origin.x += cubeSide * step_increment;
  }


 /* for(int c = 0 ; c < cubes.size() ; ++c)
  {
	std::stringstream name;
	name << "cloud" << c+1 << ".pcd";
	pcl::io::savePCDFileASCII(name.str(), *(cubes[c]));
	
  }*/

  std::cout << "returning " << cubes.size() << " cubes" << std::endl;

}

template <typename PointT>
inline void 
pcl::kinfuLS::WorldModel<PointT>::setIndicesAsNans (PointCloudPtr cloud, IndicesConstPtr indices)
{
  std::vector<pcl::PCLPointField> fields;
  pcl::for_each_type<FieldList> (pcl::detail::FieldAdder<PointT> (fields));
  float my_nan = std::numeric_limits<float>::quiet_NaN ();
  
  for (int rii = 0; rii < static_cast<int> (indices->size ()); ++rii)  // rii = removed indices iterator
  {
	uint8_t* pt_data = reinterpret_cast<uint8_t*> (&cloud->points[(*indices)[rii]]);
	for (int fi = 0; fi < static_cast<int> (fields.size ()); ++fi)  // fi = field iterator
	  memcpy (pt_data + fields[fi].offset, &my_nan, sizeof (float));
  }
}


template <typename PointT>
void 
pcl::kinfuLS::WorldModel<PointT>::setSliceAsNans (const double origin_x, const double origin_y, const double origin_z, const double offset_x, const double offset_y, const double offset_z, const int size_x, const int size_y, const int size_z)
{ 
  // PCL_DEBUG ("IN SETSLICE AS NANS\n");
  
  PointCloudPtr slice (new pcl::PointCloud<PointT>);
  
  // prepare filter limits on all dimensions  
  double previous_origin_x = origin_x;
  double previous_limit_x = origin_x + size_x - 1;
  double new_origin_x = origin_x + offset_x;
  double new_limit_x = previous_limit_x + offset_x;

  double previous_origin_y = origin_y;
  double previous_limit_y = origin_y + size_y - 1;
  double new_origin_y = origin_y + offset_y;
  double new_limit_y = previous_limit_y + offset_y;  
 
  double previous_origin_z = origin_z;
  double previous_limit_z = origin_z + size_z - 1;
  double new_origin_z = origin_z + offset_z;
  double new_limit_z = previous_limit_z + offset_z; 
   
  // get points of slice on X (we actually set a negative filter and set the ouliers (so, our slice points) to nan)
  double lower_limit_x, upper_limit_x;
  if(offset_x >=0)
  {
	lower_limit_x = previous_origin_x;
	upper_limit_x = new_origin_x;
  }
  else
  {
	lower_limit_x = new_limit_x;
	upper_limit_x = previous_limit_x;    
  }
  
  // PCL_DEBUG ("Limit X: [%f - %f]\n", lower_limit_x, upper_limit_x);
  
  ConditionOrPtr range_cond_OR_x (new pcl::ConditionOr<PointT> ());
  range_cond_OR_x->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE,  upper_limit_x ))); // filtered dimension
  range_cond_OR_x->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT,  lower_limit_x ))); // filtered dimension
	
  range_cond_OR_x->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GE,  previous_limit_y)));
  range_cond_OR_x->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT,  previous_origin_y )));
	
  range_cond_OR_x->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GE,  previous_limit_z)));
  range_cond_OR_x->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT,  previous_origin_z )));

  pcl::ConditionalRemoval<PointT> condrem_x (true);
  condrem_x.setCondition (range_cond_OR_x);
  condrem_x.setInputCloud (world_);
  condrem_x.setKeepOrganized (false);
  // apply filter
  condrem_x.filter (*slice);  
  IndicesConstPtr indices_x = condrem_x.getRemovedIndices ();
  
  //set outliers (so our slice points) to nan
  setIndicesAsNans(world_, indices_x);
  
  // PCL_DEBUG("%d points set to nan on X\n", indices_x->size ());
  
  // get points of slice on Y (we actually set a negative filter and set the ouliers (so, our slice points) to nan)
  double lower_limit_y, upper_limit_y;
  if(offset_y >=0)
  {
	lower_limit_y = previous_origin_y;
	upper_limit_y = new_origin_y;
  }
  else
  {
	lower_limit_y = new_limit_y;
	upper_limit_y = previous_limit_y;    
  }
  
  // PCL_DEBUG ("Limit Y: [%f - %f]\n", lower_limit_y, upper_limit_y);
  
  ConditionOrPtr range_cond_OR_y (new pcl::ConditionOr<PointT> ());
  range_cond_OR_y->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE,  previous_limit_x )));
  range_cond_OR_y->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT,  previous_origin_x )));
	
  range_cond_OR_y->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GE,  upper_limit_y))); // filtered dimension
  range_cond_OR_y->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT,  lower_limit_y ))); // filtered dimension
	
  range_cond_OR_y->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GE,  previous_limit_z)));
  range_cond_OR_y->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT,  previous_origin_z )));

  pcl::ConditionalRemoval<PointT> condrem_y (true);
  condrem_y.setCondition (range_cond_OR_y);
  condrem_y.setInputCloud (world_);
  condrem_y.setKeepOrganized (false);
  // apply filter
  condrem_y.filter (*slice);  
  IndicesConstPtr indices_y = condrem_y.getRemovedIndices ();
  
  //set outliers (so our slice points) to nan
  setIndicesAsNans(world_, indices_y);
  // PCL_DEBUG ("%d points set to nan on Y\n", indices_y->size ());
  
  // get points of slice on Z (we actually set a negative filter and set the ouliers (so, our slice points) to nan)
  double lower_limit_z, upper_limit_z;
  if(offset_z >=0)
  {
	lower_limit_z = previous_origin_z;
	upper_limit_z = new_origin_z;
  }
  else
  {
	lower_limit_z = new_limit_z;
	upper_limit_z = previous_limit_z;    
  }
  
  // PCL_DEBUG ("Limit Z: [%f - %f]\n", lower_limit_z, upper_limit_z);
  
  ConditionOrPtr range_cond_OR_z (new pcl::ConditionOr<PointT> ());
  range_cond_OR_z->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE,  previous_limit_x )));
  range_cond_OR_z->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT,  previous_origin_x )));
	
  range_cond_OR_z->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GE,  previous_limit_y)));
  range_cond_OR_z->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT,  previous_origin_y )));
	
  range_cond_OR_z->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GE,  upper_limit_z))); // filtered dimension
  range_cond_OR_z->addComparison (FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT,  lower_limit_z ))); // filtered dimension

  pcl::ConditionalRemoval<PointT> condrem_z (true);
  condrem_z.setCondition (range_cond_OR_z);
  condrem_z.setInputCloud (world_);
  condrem_z.setKeepOrganized (false);
  // apply filter
  condrem_z.filter (*slice);  
  IndicesConstPtr indices_z = condrem_z.getRemovedIndices ();
  
  //set outliers (so our slice points) to nan
  setIndicesAsNans(world_, indices_z);
  // PCL_DEBUG("%d points set to nan on Z\n", indices_z->size ());
  
  
}

#define PCL_INSTANTIATE_WorldModel(T) template class PCL_EXPORTS pcl::kinfuLS::WorldModel<T>;

#endif // PCL_WORLD_MODEL_IMPL_HPP_

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

#ifndef PCL_WORLD_MODEL_H_
#define PCL_WORLD_MODEL_H_

#include <pcl/common/impl/common.hpp>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/gpu/kinfu_large_scale/tsdf_buffer.h>
//#include <boost/graph/buffer_concepts.hpp>


namespace pcl
{
  namespace kinfuLS
  {
    /** \brief WorldModel maintains a 3D point cloud that can be queried and updated via helper functions.\n
      * The world is represented as a point cloud.\n
      * When new points are added to the world, we replace old ones by the newest ones.
      * This is acheived by setting old points to nan (for speed)
      * \author Raphael Favier
      */
    template <typename PointT>
    class WorldModel
    {
      public:

        typedef boost::shared_ptr<WorldModel<PointT> > Ptr;
        typedef boost::shared_ptr<const WorldModel<PointT> > ConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef typename pcl::ConditionAnd<PointT>::Ptr ConditionAndPtr;
        typedef typename pcl::ConditionOr<PointT>::Ptr ConditionOrPtr;
        typedef typename pcl::FieldComparison<PointT>::ConstPtr FieldComparisonConstPtr;
        
        typedef typename pcl::traits::fieldList<PointT>::type FieldList;

        /** \brief Default constructor for the WorldModel.
          */
        WorldModel() : 
          world_ (new PointCloud)
        {
          world_->is_dense = false;
        }
        
        /** \brief Clear the world.
          */
        void reset()
        {
          if(world_->points.size () != 0)
          {
            PCL_WARN("Clearing world model\n");
            world_->points.clear ();
          }
        }

        /** \brief Append a new point cloud (slice) to the world.
          * \param[in] new_cloud the point cloud to add to the world
          */
        void addSlice (const PointCloudPtr new_cloud);


        /** \brief Retreive existing data from the world model, after a shift
          * \param[in] previous_origin_x global origin of the cube on X axis, before the shift
          * \param[in] previous_origin_y global origin of the cube on Y axis, before the shift
          * \param[in] previous_origin_z global origin of the cube on Z axis, before the shift
          * \param[in] offset_x shift on X, in indices
          * \param[in] offset_y shift on Y, in indices
          * \param[in] offset_z shift on Z, in indices
          * \param[in] volume_x size of the cube, X axis, in indices
          * \param[in] volume_y size of the cube, Y axis, in indices
          * \param[in] volume_z size of the cube, Z axis, in indices
          * \param[out] existing_slice the extracted point cloud representing the slice
          */
        void getExistingData(const double previous_origin_x, const double previous_origin_y, const double previous_origin_z,
                            const double offset_x, const double offset_y, const double offset_z,
                            const double volume_x, const double volume_y, const double volume_z, pcl::PointCloud<PointT> &existing_slice);
        
        /** \brief Give nan values to the slice of the world 
          * \param[in] origin_x global origin of the cube on X axis, before the shift
          * \param[in] origin_y global origin of the cube on Y axis, before the shift
          * \param[in] origin_z global origin of the cube on Z axis, before the shift
          * \param[in] offset_x shift on X, in indices
          * \param[in] offset_y shift on Y, in indices
          * \param[in] offset_z shift on Z, in indices
          * \param[in] size_x size of the cube, X axis, in indices
          * \param[in] size_y size of the cube, Y axis, in indices
          * \param[in] size_z size of the cube, Z axis, in indices
          */                    
        void setSliceAsNans (const double origin_x, const double origin_y, const double origin_z,
                            const double offset_x, const double offset_y, const double offset_z,
                            const int size_x, const int size_y, const int size_z);            

        /** \brief Remove points with nan values from the world.
          */
        void cleanWorldFromNans () 
        { 
          world_->is_dense = false;
          std::vector<int> indices; 
          pcl::removeNaNFromPointCloud (*world_, *world_, indices);
        }

        /** \brief Returns the world as a point cloud.
          */
        PointCloudPtr getWorld () 
        { 
          return (world_); 
        }
        
        /** \brief Returns the number of points contained in the world.
          */      
        size_t getWorldSize () 
        { 
          return (world_->points.size () );
        }

        /** \brief Returns the world as two vectors of cubes of size "size" (pointclouds) and transforms
          * \param[in] size the size of a 3D cube.
          * \param[out] cubes a vector of point clouds representing each cube (in their original world coordinates). 
          * \param[out] transforms a vector containing the xyz position of each cube in world coordinates.
          * \param[in] overlap optional overlap (in percent) between each cube (usefull to create overlapped meshes).
          */
        void getWorldAsCubes (double size, std::vector<PointCloudPtr> &cubes, std::vector<Eigen::Vector3f> &transforms, double overlap = 0.0);
        
        
      private:

        /** \brief cloud containing our world */
        PointCloudPtr world_;

        /** \brief set the points which index is in the indices vector to nan 
          * \param[in] cloud the cloud that contains the point to be set to nan
          * \param[in] indices the vector of indices to set to nan
          */
        inline void setIndicesAsNans (PointCloudPtr cloud, IndicesConstPtr indices);
        
    };
  }
}

#endif // PCL_WORLD_MODEL_H_

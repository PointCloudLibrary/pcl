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

#ifndef PCL_RECOGNITION_MODEL_LIBRARY_H_
#define PCL_RECOGNITION_MODEL_LIBRARY_H_

#include "auxiliary.h"
#include <pcl/recognition/ransac_based/voxel_structure.h>
#include <pcl/recognition/ransac_based/orr_octree.h>
#include <pcl/common/random.h>
#include <pcl/pcl_exports.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ctime>
#include <string>
#include <list>
#include <set>
#include <map>

namespace pcl
{
  namespace recognition
  {
    class PCL_EXPORTS ModelLibrary
    {
      public:
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloudIn;
        typedef pcl::PointCloud<pcl::Normal> PointCloudN;

        /** \brief Stores some information about the model. */
        class Model
        {
          public:
            Model (const PointCloudIn& points, const PointCloudN& normals, float voxel_size, const std::string& object_name,
                   float frac_of_points_for_registration, void* user_data = NULL)
            : obj_name_(object_name),
              user_data_ (user_data)
            {
              octree_.build (points, voxel_size, &normals);

              const std::vector<ORROctree::Node*>& full_leaves = octree_.getFullLeaves ();
              if ( full_leaves.empty () )
                return;

              // Initialize
              std::vector<ORROctree::Node*>::const_iterator it = full_leaves.begin ();
              const float *p = (*it)->getData ()->getPoint ();
              aux::copy3 (p, octree_center_of_mass_);
              bounds_of_octree_points_[0] = bounds_of_octree_points_[1] = p[0];
              bounds_of_octree_points_[2] = bounds_of_octree_points_[3] = p[1];
              bounds_of_octree_points_[4] = bounds_of_octree_points_[5] = p[2];

              // Compute both the bounds and the center of mass of the octree points
              for ( ++it ; it != full_leaves.end () ; ++it )
              {
                aux::add3 (octree_center_of_mass_, (*it)->getData ()->getPoint ());
                aux::expandBoundingBoxToContainPoint (bounds_of_octree_points_, (*it)->getData ()->getPoint ());
              }

              int num_octree_points = static_cast<int> (full_leaves.size ());
              // Finalize the center of mass computation
              aux::mult3 (octree_center_of_mass_, 1.0f/static_cast<float> (num_octree_points));

              int num_points_for_registration = static_cast<int> (static_cast<float> (num_octree_points)*frac_of_points_for_registration);
              points_for_registration_.resize (static_cast<size_t> (num_points_for_registration));

              // Prepare for random point sampling
              std::vector<int> ids (num_octree_points);
              for ( int i = 0 ; i < num_octree_points ; ++i )
                ids[i] = i;

              // The random generator
              pcl::common::UniformGenerator<int> randgen (0, num_octree_points - 1, static_cast<uint32_t> (time (NULL)));

              // Randomly sample some points from the octree
              for ( int i = 0 ; i < num_points_for_registration ; ++i )
              {
                // Choose a random position within the array of ids
                randgen.setParameters (0, static_cast<int> (ids.size ()) - 1);
                int rand_pos = randgen.run ();

                // Copy the randomly selected octree point
                aux::copy3 (octree_.getFullLeaves ()[ids[rand_pos]]->getData ()->getPoint (), points_for_registration_[i]);

                // Delete the selected id
                ids.erase (ids.begin() + rand_pos);
              }
            }

            virtual ~Model ()
            {
            }

            inline const std::string&
            getObjectName () const
            {
              return (obj_name_);
            }

            inline const ORROctree&
            getOctree () const
            {
              return (octree_);
            }

            inline void*
            getUserData () const
            {
              return (user_data_);
            }

            inline const float*
            getOctreeCenterOfMass () const
            {
              return (octree_center_of_mass_);
            }

            inline const float*
            getBoundsOfOctreePoints () const
            {
              return (bounds_of_octree_points_);
            }

            inline const PointCloudIn&
            getPointsForRegistration () const
            {
              return (points_for_registration_);
            }

          protected:
            const std::string obj_name_;
            ORROctree octree_;
            float octree_center_of_mass_[3];
            float bounds_of_octree_points_[6];
            PointCloudIn points_for_registration_;
            void* user_data_;
        };

        typedef std::list<std::pair<const ORROctree::Node::Data*, const ORROctree::Node::Data*> > node_data_pair_list;
        typedef std::map<const Model*, node_data_pair_list> HashTableCell;
        typedef VoxelStructure<HashTableCell, float> HashTable;

      public:
        /** \brief This class is used by 'ObjRecRANSAC' to maintain the object models to be recognized. Normally, you do not need to use
          * this class directly. */
        ModelLibrary (float pair_width, float voxel_size, float max_coplanarity_angle = 3.0f*AUX_DEG_TO_RADIANS/*3 degrees*/);
        virtual ~ModelLibrary ()
        {
          this->clear();
        }

        /** \brief Removes all models from the library and clears the hash table. */
        void
        removeAllModels ();

        /** \brief This is a threshold. The larger the value the more point pairs will be considered as co-planar and will
          * be ignored in the off-line model pre-processing and in the online recognition phases. This makes sense only if
          * "ignore co-planar points" is on. Call this method before calling addModel. */
        inline void
        setMaxCoplanarityAngleDegrees (float max_coplanarity_angle_degrees)
        {
          max_coplanarity_angle_ = max_coplanarity_angle_degrees*AUX_DEG_TO_RADIANS;
        }

        /** \brief Call this method in order NOT to add co-planar point pairs to the hash table. The default behavior
          * is ignoring co-planar points on. */
        inline void
        ignoreCoplanarPointPairsOn ()
        {
          ignore_coplanar_opps_ = true;
        }

        /** \brief Call this method in order to add all point pairs (co-planar as well) to the hash table. The default
          * behavior is ignoring co-planar points on. */
        inline void
        ignoreCoplanarPointPairsOff ()
        {
          ignore_coplanar_opps_ = false;
        }

        /** \brief Adds a model to the hash table.
          *
          * \param[in] points represents the model to be added.
          * \param[in] normals are the normals at the model points.
          * \param[in] object_name is the unique name of the object to be added.
          * \param[in] frac_of_points_for_registration is the number of points used for fast ICP registration prior to hypothesis testing
          * \param[in] user_data is a pointer to some data (can be NULL)
          *
          * Returns true if model successfully added and false otherwise (e.g., if object_name is not unique). */
        bool
        addModel (const PointCloudIn& points, const PointCloudN& normals, const std::string& object_name,
                  float frac_of_points_for_registration, void* user_data = NULL);

        /** \brief Returns the hash table built by this instance. */
        inline const HashTable&
        getHashTable () const
        {
          return (hash_table_);
        }

        inline const Model*
        getModel (const std::string& name) const
        {
          std::map<std::string,Model*>::const_iterator it = models_.find (name);
          if ( it != models_.end () )
            return (it->second);

          return (NULL);
        }

        inline const std::map<std::string,Model*>&
        getModels () const
        {
          return (models_);
        }

      protected:
        /** \brief Removes all models from the library and destroys the hash table. This method should be called upon destroying this object. */
        void
        clear ();

        /** \brief Returns true if the oriented point pair was added to the hash table and false otherwise. */
        bool
        addToHashTable (Model* model, const ORROctree::Node::Data* data1, const ORROctree::Node::Data* data2);

      protected:
        float pair_width_;
        float voxel_size_;
        float max_coplanarity_angle_;
        bool ignore_coplanar_opps_;

        std::map<std::string,Model*> models_;
        HashTable hash_table_;
        int num_of_cells_[3];
    };
  } // namespace recognition
} // namespace pcl

#endif // PCL_RECOGNITION_MODEL_LIBRARY_H_

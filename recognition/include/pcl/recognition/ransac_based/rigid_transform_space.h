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

/*
 * rigid_transform_space.h
 *
 *  Created on: Feb 15, 2013
 *      Author: papazov
 */

#ifndef PCL_RECOGNITION_RIGID_TRANSFORM_SPACE_H_
#define PCL_RECOGNITION_RIGID_TRANSFORM_SPACE_H_

#include "orr_octree.h"
#include "model_library.h"
#include <pcl/pcl_exports.h>
#include <list>
#include <map>

#define ROT_SPACE_TEST

namespace pcl
{
  namespace recognition
  {
    class PCL_EXPORTS Cell
    {
      public:
        class Entry
        {
          public:
            Entry ()
            : num_transforms_ (0)
            {
              aux::set3 (axis_angle_, 0.0f);
              aux::set3 (translation_, 0.0f);
            }

            Entry (const Entry& src)
            : num_transforms_ (src.num_transforms_)
            {
              aux::copy3 (src.axis_angle_, this->axis_angle_);
              aux::copy3 (src.translation_, this->translation_);
            }

            inline void
            addRigidTransform (const float axis_angle[3], const float translation[3])
            {
              aux::add3 (this->axis_angle_, axis_angle);
              aux::add3 (this->translation_, translation);
              ++num_transforms_;
            }

            inline void
            computeAverageRigidTransform (float *rigid_transform = NULL)
            {
              if ( num_transforms_ >= 2 )
              {
                float factor = 1.0f/static_cast<float> (num_transforms_);
                aux::mult3 (axis_angle_, factor);
                aux::mult3 (translation_, factor);
                num_transforms_ = 1;
              }

              if ( rigid_transform )
              {
                // Save the rotation (in matrix form)
                aux::axisAngleToRotationMatrix (axis_angle_, rigid_transform);
                // Save the translation
                aux::copy3 (translation_, rigid_transform + 9);
              }
            }

            inline const float*
            getAxisAngle () const
            {
              return (axis_angle_);
            }

            inline const float*
            getTranslation () const
            {
              return (translation_);
            }

          public:
            float axis_angle_[3], translation_[3];
            int num_transforms_;
        };// class Entry

      public:
        Cell (){}
        virtual ~Cell ()
        {
          model_to_entry_.clear ();
        }

        inline int*
        getPositionId ()
        {
          return (pos_id_);
        }

        inline int*
        getRotationId ()
        {
          return (rot_id_);
        }

        inline std::map<const ModelLibrary::Model*,Entry>&
        getEntries ()
        {
          return (model_to_entry_);
        }

        inline void
        addRigidTransform (const ModelLibrary::Model* model, const float axis_angle[3], const float translation[3])
        {
          model_to_entry_[model].addRigidTransform (axis_angle, translation);
        }

      protected:
        std::map<const ModelLibrary::Model*,Entry> model_to_entry_;
        int pos_id_[3], rot_id_[3];
    }; // class Cell

    /** \brief This is a class for a discrete representation of the rotation space based on the axis-angle representation.
      * This class is not supposed to be very general. That's why it is dependent on the class ModelLibrary.
      *
      * \author Chavdar Papazov
      * \ingroup recognition
      */
    template<class Data>
    class PCL_EXPORTS RotationSpace
    {
      public:
        /** \brief We use the axis-angle representation for rotations. The axis is encoded in the vector
          * and the angle is its magnitude. This is represented in an octree with bounds [-pi, pi]^3. */
        RotationSpace (float discretization)
        {
          float min = -(AUX_PI_FLOAT + 0.000000001f), max = AUX_PI_FLOAT + 0.000000001f;
          float bounds[6] = {min, max, min, max, min, max};

          // Build the voxel structure
          octree_.build (bounds, discretization);
          aux::set3 (pos_id_, -1);
        }

        virtual ~RotationSpace ()
        {
          for ( std::list<Cell*>::iterator it = full_cells_.begin () ; it != full_cells_.end () ; ++it )
            delete *it;
          full_cells_.clear ();
        }

        inline int*
        getPositionId ()
        {
          return (pos_id_);
        }

        inline std::list<Cell*>&
        getFullCells ()
        {
          return (full_cells_);
        }

        inline void
        setOctreeLeaf (ORROctree::Node* leaf)
        {
          positional_leaf_ = leaf;
        }

        inline ORROctree::Node*
        getOctreeLeaf () const
        {
          return (positional_leaf_);
        }

        inline void
        setData (const Data& data)
        {
          data_ = data;
        }

        inline const Data&
        getData () const
        {
          return (data_);
        }

        inline bool
        addRigidTransform (const ModelLibrary::Model* model, const float axis_angle[3], const float translation[3])
        {
          ORROctree::Node* rot_leaf = octree_.createLeaf (axis_angle[0], axis_angle[1], axis_angle[2]);

          if ( !rot_leaf )
          {
            const float *b = octree_.getBounds ();
            printf ("WARNING in 'RotationSpace::%s()': the provided axis-angle input (%f, %f, %f) is "
                    "out of the rotation space bounds ([%f, %f], [%f, %f], [%f, %f]).\n",
                    __func__, axis_angle[0], axis_angle[1], axis_angle[2], b[0], b[1], b[2], b[3], b[4], b[5]);
            return (false);
          }

          Cell* cell;

          if ( !rot_leaf->getData ()->getUserData () )
          {
            cell = new Cell ();
            rot_leaf->getData ()->setUserData (cell);
            // Save the ids
            aux::copy3(pos_id_, cell->getPositionId ());
            rot_leaf->getData ()->get3dId (cell->getRotationId ());
            // Save the cell
            full_cells_.push_back (cell);
          }
          else
            cell = static_cast<Cell*> (rot_leaf->getData ()->getUserData ());

          // Add the rigid transform to the cell
          cell->addRigidTransform (model, axis_angle, translation);

          return (true);
        }

        inline void
        getNeighbors(std::list<RotationSpace<Data>* >& neighs) const
        {
        }

      protected:
        ORROctree octree_;
        std::list<Cell*> full_cells_;
        int pos_id_[3];
        ORROctree::Node* positional_leaf_;
        Data data_;
    };// class RotationSpace

    template<class Data>
    class PCL_EXPORTS RigidTransformSpace
    {
      public:
        RigidTransformSpace ()
        : num_occupied_rotation_spaces_ (0)
        {}

        virtual ~RigidTransformSpace ()
        {
          this->clear ();
        }

        inline void
        build (const float* pos_bounds, float translation_cell_size, float rotation_cell_size)
        {
          this->clear ();

          translation_cell_size_  = translation_cell_size;
          rotation_cell_size_  = rotation_cell_size;

          pos_octree_.build (pos_bounds, translation_cell_size);
        }

        inline void
        clear ()
        {
          for ( typename std::list<RotationSpace<Data>*>::iterator it = rotation_space_list_.begin () ; it != rotation_space_list_.end () ; ++it )
            delete *it;
          rotation_space_list_.clear ();
          pos_octree_.clear ();
          num_occupied_rotation_spaces_ = 0;
        }

        inline std::list<RotationSpace<Data>*>&
        getRotationSpaces ()
        {
          return (rotation_space_list_);
        }

        inline const std::list<RotationSpace<Data>*>&
        getRotationSpaces () const
        {
          return (rotation_space_list_);
        }

        inline int
        getNumberOfOccupiedRotationSpaces ()
        {
          return (num_occupied_rotation_spaces_);
        }

        inline bool
        getPositionCellBounds (const int id[3], float bounds[6]) const
        {
          const ORROctree::Node* leaf = pos_octree_.getLeaf (id);

          if ( !leaf )
            return (false);

          leaf->getBounds (bounds);

          return (true);
        }

        inline bool
        addRigidTransform (const ModelLibrary::Model* model, const float position[3], const float rigid_transform[12])
        {
          // Get the leaf 'position' ends up in
          ORROctree::Node* leaf = pos_octree_.createLeaf (position[0], position[1], position[2]);

          if ( !leaf )
          {
            printf ("WARNING in 'RigidTransformSpace::%s()': the input position (%f, %f, %f) is out of bounds.\n",
                    __func__, position[0], position[1], position[2]);
            return (false);
          }

          RotationSpace<Data>* rot_space;

          // Shall we create a new rotation space instance
          if ( !leaf->getData ()->getUserData () )
          {
            rot_space = new RotationSpace<Data> (rotation_cell_size_);
            leaf->getData ()->setUserData (rot_space);
            leaf->getData ()->get3dId (rot_space->getPositionId ());
            rot_space->setOctreeLeaf (leaf);
            rotation_space_list_.push_back (rot_space);
            ++num_occupied_rotation_spaces_;
          }
          else // get the existing rotation space
            rot_space = static_cast<RotationSpace<Data>*> (leaf->getData ()->getUserData ());

          float rot_angle, axis_angle[3];
          // Extract the axis-angle representation from the rotation matrix
          aux::rotationMatrixToAxisAngle (rigid_transform, axis_angle, rot_angle);
          // Multiply the axis by the angle to get the final representation
          aux::mult3 (axis_angle, rot_angle);

          // Now, add the rigid transform to the space representation
          rot_space->addRigidTransform (model, axis_angle, rigid_transform + 9);

          return (true);
        }

      protected:
        ORROctree pos_octree_;
        float translation_cell_size_;
        float rotation_cell_size_;
        int num_occupied_rotation_spaces_;
        std::list<RotationSpace<Data>*> rotation_space_list_;
    }; // class RigidTransformSpace
  } // namespace recognition
} // namespace pcl

#endif /* PCL_RECOGNITION_RIGID_TRANSFORM_SPACE_H_ */

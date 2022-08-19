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

#pragma once

#include "simple_octree.h"
#include "model_library.h"
#include <pcl/pcl_exports.h>
#include <list>
#include <map>

namespace pcl
{
  namespace recognition
  {
    class RotationSpaceCell
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

            const Entry& operator = (const Entry& src)
            {
              num_transforms_ = src.num_transforms_;
              aux::copy3 (src.axis_angle_, this->axis_angle_);
              aux::copy3 (src.translation_, this->translation_);

              return *this;
            }

            inline const Entry&
            addRigidTransform (const float axis_angle[3], const float translation[3])
            {
              aux::add3 (this->axis_angle_, axis_angle);
              aux::add3 (this->translation_, translation);
              ++num_transforms_;

              return *this;
            }

            inline void
            computeAverageRigidTransform (float *rigid_transform = nullptr)
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

            inline int
            getNumberOfTransforms () const
            {
              return (num_transforms_);
            }

          protected:
            float axis_angle_[3], translation_[3];
            int num_transforms_;
        };// class Entry

      public:
        RotationSpaceCell () = default;
        virtual ~RotationSpaceCell ()
        {
          model_to_entry_.clear ();
        }

        inline std::map<const ModelLibrary::Model*,Entry>&
        getEntries ()
        {
          return (model_to_entry_);
        }

        inline const RotationSpaceCell::Entry*
        getEntry (const ModelLibrary::Model* model) const
        {
          auto res = model_to_entry_.find (model);

          if ( res != model_to_entry_.end () )
            return (&res->second);

          return (nullptr);
        }

        inline const RotationSpaceCell::Entry&
        addRigidTransform (const ModelLibrary::Model* model, const float axis_angle[3], const float translation[3])
        {
          return model_to_entry_[model].addRigidTransform (axis_angle, translation);
        }

      protected:
        std::map<const ModelLibrary::Model*,Entry> model_to_entry_;
    }; // class RotationSpaceCell

    class RotationSpaceCellCreator
    {
      public:
        RotationSpaceCellCreator () = default;
        virtual ~RotationSpaceCellCreator () = default;

        RotationSpaceCell* create (const SimpleOctree<RotationSpaceCell, RotationSpaceCellCreator, float>::Node* )
        {
          return (new RotationSpaceCell ());
        }
    };

    using CellOctree = SimpleOctree<RotationSpaceCell, RotationSpaceCellCreator, float>;

    /** \brief This is a class for a discrete representation of the rotation space based on the axis-angle representation.
      * This class is not supposed to be very general. That's why it is dependent on the class ModelLibrary.
      *
      * \author Chavdar Papazov
      * \ingroup recognition
      */
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
          octree_.build (bounds, discretization, &cell_creator_);
        }

        virtual ~RotationSpace ()
        {
          octree_.clear ();
        }

        inline void
        setCenter (const float* c)
        {
          center_[0] = c[0];
          center_[1] = c[1];
          center_[2] = c[2];
        }

        inline const float*
        getCenter () const { return center_;}

        inline bool
        getTransformWithMostVotes (const ModelLibrary::Model* model, float rigid_transform[12]) const
        {
          RotationSpaceCell::Entry with_most_votes;
          const std::vector<CellOctree::Node*>& full_leaves = octree_.getFullLeaves ();
          int max_num_transforms = 0;

          // For each full leaf
          for (const auto &full_leaf : full_leaves)
          {
            // Is there an entry for 'model' in the current cell
            const RotationSpaceCell::Entry *entry = full_leaf->getData ().getEntry (model);
            if ( !entry )
              continue;

            int num_transforms = entry->getNumberOfTransforms ();
            const std::set<CellOctree::Node*>& neighs = full_leaf->getNeighbors ();

            // For each neighbor
            for (const auto &neigh : neighs)
            {
              const RotationSpaceCell::Entry *neigh_entry = neigh->getData ().getEntry (model);
              if ( !neigh_entry )
                continue;

              num_transforms += neigh_entry->getNumberOfTransforms ();
            }

            if ( num_transforms > max_num_transforms )
            {
              with_most_votes = *entry;
              max_num_transforms = num_transforms;
            }
          }

          if ( !max_num_transforms )
            return false;

          with_most_votes.computeAverageRigidTransform (rigid_transform);

          return true;
        }

        inline bool
        addRigidTransform (const ModelLibrary::Model* model, const float axis_angle[3], const float translation[3])
        {
          CellOctree::Node* cell = octree_.createLeaf (axis_angle[0], axis_angle[1], axis_angle[2]);

          if ( !cell )
          {
            const float *b = octree_.getBounds ();
            printf ("WARNING in 'RotationSpace::%s()': the provided axis-angle input (%f, %f, %f) is "
                    "out of the rotation space bounds ([%f, %f], [%f, %f], [%f, %f]).\n",
                    __func__, axis_angle[0], axis_angle[1], axis_angle[2], b[0], b[1], b[2], b[3], b[4], b[5]);
            return (false);
          }

          // Add the rigid transform to the cell
          cell->getData ().addRigidTransform (model, axis_angle, translation);

          return (true);
        }

      protected:
        CellOctree octree_;
        RotationSpaceCellCreator cell_creator_;
        float center_[3];
    };// class RotationSpace

    class RotationSpaceCreator
    {
      public:
        RotationSpaceCreator()
        : counter_ (0)
        {}

        virtual ~RotationSpaceCreator() = default;

        RotationSpace* create(const SimpleOctree<RotationSpace, RotationSpaceCreator, float>::Node* leaf)
        {
          auto *rot_space = new RotationSpace (discretization_);
          rot_space->setCenter (leaf->getCenter ());
          rotation_spaces_.push_back (rot_space);

          ++counter_;

          return (rot_space);
        }

        void
        setDiscretization (float value){ discretization_ = value;}

        int
        getNumberOfRotationSpaces () const { return (counter_);}

        const std::list<RotationSpace*>&
        getRotationSpaces () const { return (rotation_spaces_);}

        std::list<RotationSpace*>&
        getRotationSpaces (){ return (rotation_spaces_);}

        void
        reset ()
        {
          counter_ = 0;
          rotation_spaces_.clear ();
        }

      protected:
        float discretization_;
        int counter_;
        std::list<RotationSpace*> rotation_spaces_;
    };

    using RotationSpaceOctree = SimpleOctree<RotationSpace, RotationSpaceCreator, float>;

    class PCL_EXPORTS RigidTransformSpace
    {
      public:
        RigidTransformSpace () = default;
        virtual ~RigidTransformSpace (){ this->clear ();}

        inline void
        build (const float* pos_bounds, float translation_cell_size, float rotation_cell_size)
        {
          this->clear ();

          rotation_space_creator_.setDiscretization (rotation_cell_size);

          pos_octree_.build (pos_bounds, translation_cell_size, &rotation_space_creator_);
        }

        inline void
        clear ()
        {
          pos_octree_.clear ();
          rotation_space_creator_.reset ();
        }

        inline std::list<RotationSpace*>&
        getRotationSpaces ()
        {
          return (rotation_space_creator_.getRotationSpaces ());
        }

        inline const std::list<RotationSpace*>&
        getRotationSpaces () const
        {
          return (rotation_space_creator_.getRotationSpaces ());
        }

        inline int
        getNumberOfOccupiedRotationSpaces ()
        {
          return (rotation_space_creator_.getNumberOfRotationSpaces ());
        }

        inline bool
        addRigidTransform (const ModelLibrary::Model* model, const float position[3], const float rigid_transform[12])
        {
          // Get the leaf 'position' ends up in
          RotationSpaceOctree::Node* leaf = pos_octree_.createLeaf (position[0], position[1], position[2]);

          if ( !leaf )
          {
            printf ("WARNING in 'RigidTransformSpace::%s()': the input position (%f, %f, %f) is out of bounds.\n",
                    __func__, position[0], position[1], position[2]);
            return (false);
          }

          float rot_angle, axis_angle[3];
          // Extract the axis-angle representation from the rotation matrix
          aux::rotationMatrixToAxisAngle (rigid_transform, axis_angle, rot_angle);
          // Multiply the axis by the angle to get the final representation
          aux::mult3 (axis_angle, rot_angle);

          // Now, add the rigid transform to the rotation space
          leaf->getData ().addRigidTransform (model, axis_angle, rigid_transform + 9);

          return (true);
        }

      protected:
        RotationSpaceOctree pos_octree_;
        RotationSpaceCreator rotation_space_creator_;
    }; // class RigidTransformSpace
  } // namespace recognition
} // namespace pcl

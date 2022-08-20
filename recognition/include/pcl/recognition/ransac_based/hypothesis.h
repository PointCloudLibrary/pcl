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
 * hypothesis.h
 *
 *  Created on: Mar 12, 2013
 *      Author: papazov
 */

#pragma once

#include <pcl/recognition/ransac_based/model_library.h>
#include <pcl/recognition/ransac_based/auxiliary.h>

namespace pcl
{
  namespace recognition
  {
    class HypothesisBase
    {
      public:
        HypothesisBase (const ModelLibrary::Model* obj_model)
        : obj_model_ (obj_model)
        {}

        HypothesisBase (const ModelLibrary::Model* obj_model, const float* rigid_transform)
        : obj_model_ (obj_model)
        {
          std::copy(rigid_transform, rigid_transform + 12, rigid_transform_);
        }

        virtual  ~HypothesisBase () = default;

        void
        setModel (const ModelLibrary::Model* model)
        {
          obj_model_ = model;
        }

      public:
        float rigid_transform_[12];
        const ModelLibrary::Model* obj_model_;
    };

    class Hypothesis: public HypothesisBase
    {
      public:
        Hypothesis (const ModelLibrary::Model* obj_model = nullptr)
         : HypothesisBase (obj_model),
           match_confidence_ (-1.0f),
           linear_id_ (-1)
        {
        }

        Hypothesis (const Hypothesis& src)
        : HypothesisBase (src.obj_model_, src.rigid_transform_),
          match_confidence_  (src.match_confidence_),
          explained_pixels_ (src.explained_pixels_)
        {
        }

        ~Hypothesis () override = default;

        const Hypothesis&
        operator =(const Hypothesis& src)
        {
          std::copy(src.rigid_transform_, src.rigid_transform_ + 12, this->rigid_transform_);
          this->obj_model_  = src.obj_model_;
          this->match_confidence_  = src.match_confidence_;
          this->explained_pixels_ = src.explained_pixels_;

          return *this;
        }

        void
        setLinearId (int id)
        {
          linear_id_ = id;
        }

        int
        getLinearId () const
        {
          return (linear_id_);
        }

        void
        computeBounds (float bounds[6]) const
        {
          const float *b = obj_model_->getBoundsOfOctreePoints ();
          float p[3];

          // Initialize 'bounds'
          aux::transform (rigid_transform_, b[0], b[2], b[4], p);
          bounds[0] = bounds[1] = p[0];
          bounds[2] = bounds[3] = p[1];
          bounds[4] = bounds[5] = p[2];

          // Expand 'bounds' to contain the other 7 points of the octree bounding box
          aux::transform (rigid_transform_, b[0], b[2], b[5], p); aux::expandBoundingBoxToContainPoint (bounds, p);
          aux::transform (rigid_transform_, b[0], b[3], b[4], p); aux::expandBoundingBoxToContainPoint (bounds, p);
          aux::transform (rigid_transform_, b[0], b[3], b[5], p); aux::expandBoundingBoxToContainPoint (bounds, p);
          aux::transform (rigid_transform_, b[1], b[2], b[4], p); aux::expandBoundingBoxToContainPoint (bounds, p);
          aux::transform (rigid_transform_, b[1], b[2], b[5], p); aux::expandBoundingBoxToContainPoint (bounds, p);
          aux::transform (rigid_transform_, b[1], b[3], b[4], p); aux::expandBoundingBoxToContainPoint (bounds, p);
          aux::transform (rigid_transform_, b[1], b[3], b[5], p); aux::expandBoundingBoxToContainPoint (bounds, p);
        }

        void
        computeCenterOfMass (float center_of_mass[3]) const
        {
          aux::transform (rigid_transform_, obj_model_->getOctreeCenterOfMass (), center_of_mass);
        }

      public:
        float match_confidence_;
        std::set<int> explained_pixels_;
        int linear_id_;
    };
  } // namespace recognition
} // namespace pcl

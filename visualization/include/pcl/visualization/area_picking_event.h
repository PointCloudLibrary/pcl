/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#ifndef PCL_VISUALIZATION_AREA_PICKING_EVENT_H_
#define PCL_VISUALIZATION_AREA_PICKING_EVENT_H_

#include <pcl/pcl_macros.h>
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkSmartPointer.h>
namespace pcl
{
  namespace visualization
  {
    /** /brief Class representing 3D area picking events. */
    class PCL_EXPORTS AreaPickingEvent
    {
      public:
        AreaPickingEvent (int nb_points, const std::vector< std::vector<int> >& indices, vtkActorCollection* actorsCollection)
          : nb_points_ (nb_points)
          , indices_ (indices)
          ,actors_ (actorsCollection)
        {}

        /** \brief For situations where a whole are is selected, return the points indices.
          * \param[out] indices indices of the points under the area selected by user.
          * \return true, if the area selected by the user contains points, false otherwise
          */
        inline bool
        getPointsIndices (std::vector<int>& indices) const
        {
          if (nb_points_ <= 0)
            return (false);
          for(int i=0; i < indices_.size(); i++)
          indices.insert(indices.end(), indices_.at(i).begin(), indices_.at(i).end());
          return (true);
        }
        /** \brief For situations where a whole are is selected, return the actors of selected clouds.
          * \param[out] actors vtkActorCollection of all actors selected by area picking event.
          * \return true, if the area selected by the user contains points or contains actor, false otherwise
          */
        inline bool
        getActors (vtkActorCollection* actors) const
        {
          if (nb_points_ <= 0)
            return (false);
          actors = actors_;
          return (true);
        }

        /** \brief For situations where a whole are is selected, return the indices of selected actor.
          * \param[in] actor vtkActor of cloud.
          * \param[out] indices vector of indices for given cloud.
          * \return true, if the area selected by the user contains points or contains actor, false otherwise
          */
        inline bool
        getActorsIndices (vtkSmartPointer<vtkActor> actor, std::vector<int>& indices) const
        {
          if (nb_points_ <= 0 || actors_->GetNumberOfItems() <=0)
            return (false);
            actors_->InitTraversal();
            for(vtkIdType i = 0; i< actors_->GetNumberOfItems(); i++)
            {
              vtkActor* actor_ = actors_->GetNextActor();
              if(actor_ == actor)
                indices = indices_.at(i);
            }
          return (true);
        }
      private:
        int nb_points_;
        std::vector< std::vector<int> > indices_;
        vtkActorCollection* actors_;
    };
  } //namespace visualization
} //namespace pcl

#endif  /* PCL_VISUALIZATION_AREA_PICKING_EVENT_H_ */

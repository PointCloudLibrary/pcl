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
#ifndef PCL_MODELER_POINTS_ACTOR_ITEM_H_
#define PCL_MODELER_POINTS_ACTOR_ITEM_H_

#include <pcl/apps/modeler/channel_actor_item.h>
#include <pcl/visualization/point_cloud_handlers.h>

class vtkIdTypeArray;

namespace pcl
{
  namespace modeler
  {
    class PointsActorItem : public ChannelActorItem
    {
      public:
        PointsActorItem(QTreeWidgetItem* parent,
                        const boost::shared_ptr<CloudMesh>& cloud_mesh,
                        const vtkSmartPointer<vtkRenderWindow>& render_window);
        ~PointsActorItem ();

        virtual std::string
        getItemName() const {return "Points Actor Item";}

      protected:
        virtual void
        initImpl();

        virtual void
        updateImpl();

        virtual void
        prepareContextMenu(QMenu* menu) const;

        virtual void
        prepareProperties(ParameterDialog* parameter_dialog);

        virtual void
        setProperties();

      private:

    };
  }
}

#endif // PCL_MODELER_POINTS_ACTOR_ITEM_H_
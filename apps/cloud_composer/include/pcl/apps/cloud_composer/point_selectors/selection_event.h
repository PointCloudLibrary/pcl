/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright  (c) 2012, Jeremie Papon.
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkDataSetMapper.h>
#include <vtkRenderer.h>

namespace pcl
{
  namespace cloud_composer
  {
    class RectangularFrustumSelector;  
    
    class SelectionEvent
    {
      
      public:
        SelectionEvent (vtkSmartPointer <vtkPolyData> selected_points, vtkSmartPointer<vtkActor> selected_actor, vtkSmartPointer<vtkDataSetMapper> selected_mapper, QMap < QString, vtkPolyData* > id_selected_map, vtkRenderer* renderer) 
        : selected_points_ (std::move(selected_points)) 
        , selected_actor_ (std::move(selected_actor))
        , selected_mapper_ (std::move(selected_mapper))
        , id_selected_data_map_ (std::move(id_selected_map))
        , renderer_ (renderer) 
        {}
        
        ~SelectionEvent ();
        
        inline vtkIdType
        getNumPoints () const { return selected_points_->GetNumberOfPoints (); }
        
        vtkSmartPointer <vtkPolyData>
        getPolyData () const { return selected_points_; }
        
        vtkSmartPointer <vtkDataSetMapper>
        getMapper () const { return selected_mapper_; }
        
        vtkSmartPointer <vtkActor>
        getActor () const { return selected_actor_; }
        
        void
        findIndicesInItem (CloudItem* cloud_item, const pcl::PointIndices::Ptr& indices);
        
      private:
      
        vtkSmartPointer <vtkPolyData> selected_points_;
        vtkSmartPointer<vtkActor> selected_actor_;
        vtkSmartPointer<vtkDataSetMapper> selected_mapper_;
        QMap < QString, vtkPolyData* > id_selected_data_map_;
        vtkRenderer* renderer_;
       
    };
    
  }
  
}

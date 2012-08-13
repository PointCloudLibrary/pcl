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

#ifndef MANIPULATION_EVENT_H_
#define MANIPULATION_EVENT_H_

#include <pcl/visualization/vtk.h>
#include <pcl/apps/cloud_composer/items/cloud_item.h>
#include <pcl/apps/cloud_composer/qt.h>

namespace pcl
{
  namespace cloud_composer
  {
      
    
    class PCL_EXPORTS ManipulationEvent
    {
      
      public:
        ManipulationEvent () 
        {}
        
        ~ManipulationEvent ();
        
        void
        addManipulation (QString id, vtkSmartPointer<vtkMatrix4x4> start, vtkSmartPointer<vtkMatrix4x4> end);
        
        inline QMap <QString, vtkSmartPointer<vtkMatrix4x4> >
        getStartMap () const { return id_start_map_;}
        
        inline QMap <QString, vtkSmartPointer<vtkMatrix4x4> >
        getEndMap () const { return id_end_map_;}
        
        
      private:
        QMap <QString, vtkSmartPointer<vtkMatrix4x4> > id_start_map_;
        QMap <QString, vtkSmartPointer<vtkMatrix4x4> > id_end_map_;
       
    };
    
  }
  
}

#endif // MANIPULATION_EVENT_H_
        
        